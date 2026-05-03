#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState, CameraInfo
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import qos_profile_sensor_data
import tkinter as tk
import threading
import math
import os

try:
    from gazebo_msgs.msg import ModelStates, EntityState
    GAZEBO_READY = True
except ImportError:
    GAZEBO_READY = False

try:
    from linkattacher_msgs.srv import AttachLink, DetachLink
    LINK_ATTACHER_READY = True
except ImportError:
    LINK_ATTACHER_READY = False

class VisualJogger(Node):
    def __init__(self):
        super().__init__('visual_jogger')
        self.cartesian_publisher = self.create_publisher(Pose, '/target_position', 10)
        
        self.arm_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_publisher = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        self.cam_frame_id = 'custom_stereo_right'
        self.info_sub = self.create_subscription(CameraInfo, '/custom_stereo/stereo/right/camera_info', self.info_callback, qos_profile_sensor_data)
        
        if GAZEBO_READY:
            self.model_sub = self.create_subscription(ModelStates, '/gazebo/model_states', self.model_callback, 10)
            self.entity_pub = self.create_publisher(EntityState, '/gazebo/set_entity_state', 10)
            
        if LINK_ATTACHER_READY:
            self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
            self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
            
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.2, self.timer_callback)
        
        self.step_cm = 1.0
        self.step_deg = 5.0
        
        self.real_x_cm = 40.0
        self.real_y_cm = 0.0
        self.real_z_cm = 40.0
        self.real_roll_deg = 0.0
        self.real_pitch_deg = 0.0
        self.real_yaw_deg = 0.0
        
        self.cmd_x = 40.0
        self.cmd_y = 0.0
        self.cmd_z = 40.0
        self.cmd_roll = -180.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0
        
        self.gazebo_red = "Gazebo Red:      Waiting for data..."
        self.gazebo_green = "Gazebo Green:    Waiting for data..."
        
        self.mode = "joint"
        self.updating_sliders = False
        self.attached_color = None
        
        self.angle_text_var = None
        self.real_loc_var = None
        self.attach_status_var = None
        
        self.live_red_var = None
        self.live_green_var = None
        self.cap_red_var = None
        self.cap_green_var = None
        
        self.debug_cam_to_apple = None
        self.debug_base_to_tcp = None
        self.debug_base_to_cam = None
        self.debug_world_to_tcp = None
        self.debug_world_to_cam = None
        self.debug_world_to_apple = None
        self.debug_base_to_apple = None
        self.debug_real_apple = None
        
        self.live_red_target = None
        self.live_green_target = None
        self.captured_red_target = None
        self.captured_green_target = None
        
        self.sliders = []
        self.entry_vars = []
        self.gripper_slider = None
        self.latest_arm_angles = [0.0] * 6
        
        self.arm_joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        self.gripper_joint_names = [
            'gripper_to_left_finger', 'gripper_to_right_finger'
        ]

    def info_callback(self, msg):
        self.cam_frame_id = msg.header.frame_id

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def model_callback(self, msg):
        try:
            if 'red_apple' in msg.name:
                idx = msg.name.index('red_apple')
                pos = msg.pose[idx].position
                self.gazebo_red = f"{'Gazebo Red:':<16} X: {pos.x*100:6.1f}   Y: {pos.y*100:6.1f}   Z: {pos.z*100:6.1f}"
            if 'green_apple' in msg.name:
                idx = msg.name.index('green_apple')
                pos = msg.pose[idx].position
                self.gazebo_green = f"{'Gazebo Green:':<16} X: {pos.x*100:6.1f}   Y: {pos.y*100:6.1f}   Z: {pos.z*100:6.1f}"
        except:
            pass

    def joint_callback(self, msg):
        if self.angle_text_var is None or len(self.sliders) < 6 or self.gripper_slider is None:
            return
        try:
            angles = []
            for i in range(6):
                index = msg.name.index(self.arm_joint_names[i])
                angles.append(msg.position[index])
            self.latest_arm_angles = angles
            self.angle_text_var.set(
                f"Base: {angles[0]:6.2f} Shoulder: {angles[1]:6.2f} Elbow: {angles[2]:6.2f}\n"
                f"Wrist 1: {angles[3]:6.2f} Wrist 2: {angles[4]:6.2f} Wrist 3: {angles[5]:6.2f}"
            )
            if self.mode == "cartesian":
                self.updating_sliders = True
                for i in range(6):
                    degree_val = math.degrees(angles[i])
                    self.sliders[i].set(degree_val)
                    if len(self.entry_vars) == 6:
                        self.entry_vars[i].set(f"{degree_val:.1f}")
                if 'gripper_to_left_finger' in msg.name:
                    g_idx = msg.name.index('gripper_to_left_finger')
                    g_pos = msg.position[g_idx]
                    g_mm = 100.0 - (g_pos * 2000.0)
                    g_mm = max(0.0, min(100.0, g_mm))
                    self.gripper_slider.set(g_mm)
                self.updating_sliders = False
        except ValueError:
            pass

    def get_tf_string(self, prefix, target, source):
        try:
            t = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            x = t.transform.translation.x * 100.0
            y = t.transform.translation.y * 100.0
            z = t.transform.translation.z * 100.0
            return f"{prefix:<16} X: {x:6.1f}   Y: {y:6.1f}   Z: {z:6.1f}"
        except:
            return f"{prefix:<16} Searching network..."

    def timer_callback(self):
        if self.real_loc_var is not None:
            try:
                t = self.tf_buffer.lookup_transform('world', 'tcp_link', rclpy.time.Time())
                self.real_x_cm = t.transform.translation.x * 100.0
                self.real_y_cm = t.transform.translation.y * 100.0
                self.real_z_cm = t.transform.translation.z * 100.0
                q = t.transform.rotation
                r, p, y = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
                self.real_roll_deg = math.degrees(r)
                self.real_pitch_deg = math.degrees(p)
                self.real_yaw_deg = math.degrees(y)
                self.real_loc_var.set(
                    f"TCP X: {self.real_x_cm:6.1f} Y: {self.real_y_cm:6.1f} Z: {self.real_z_cm:6.1f} cm\n"
                    f"Roll: {self.real_roll_deg:6.1f} Pitch: {self.real_pitch_deg:6.1f} Yaw: {self.real_yaw_deg:6.1f}"
                )
            except:
                pass

        if self.debug_world_to_tcp is not None:
            self.debug_base_to_tcp.set(self.get_tf_string("Base to TCP:", 'base_link', 'tcp_link'))
            self.debug_world_to_tcp.set(self.get_tf_string("World to TCP:", 'world', 'tcp_link'))
            
            self.debug_base_to_cam.set(self.get_tf_string("Base to Cam:", 'base_link', self.cam_frame_id))
            self.debug_world_to_cam.set(self.get_tf_string("World to Cam:", 'world', self.cam_frame_id))
            
            cam_red = self.get_tf_string("Cam to Red:", self.cam_frame_id, 'apple_red')
            cam_green = self.get_tf_string("Cam to Green:", self.cam_frame_id, 'apple_green')
            self.debug_cam_to_apple.set(f"{cam_red}\n{cam_green}")
            
            world_red = self.get_tf_string("World to Red:", 'world', 'apple_red')
            world_green = self.get_tf_string("World to Green:", 'world', 'apple_green')
            self.debug_world_to_apple.set(f"{world_red}\n{world_green}")
            
            base_red = self.get_tf_string("Base to Red:", 'base_link', 'apple_red')
            base_green = self.get_tf_string("Base to Green:", 'base_link', 'apple_green')
            self.debug_base_to_apple.set(f"{base_red}\n{base_green}")
            
            if GAZEBO_READY:
                self.debug_real_apple.set(f"{self.gazebo_red}\n{self.gazebo_green}")

        if self.live_red_var is not None and self.live_green_var is not None:
            for color in ['red', 'green']:
                try:
                    tw = self.tf_buffer.lookup_transform('world', f'apple_{color}', rclpy.time.Time())
                    wx = tw.transform.translation.x * 100.0
                    wy = tw.transform.translation.y * 100.0
                    wz = tw.transform.translation.z * 100.0

                    if color == 'red':
                        self.live_red_var.set(f"Live Red\nX: {wx:6.1f}  Y: {wy:6.1f}  Z: {wz:6.1f}")
                        self.live_red_target = [wx, wy, wz]
                    else:
                        self.live_green_var.set(f"Live Green\nX: {wx:6.1f}  Y: {wy:6.1f}  Z: {wz:6.1f}")
                        self.live_green_target = [wx, wy, wz]
                except:
                    pass

    def capture_positions(self):
        if self.live_red_target:
            self.captured_red_target = list(self.live_red_target)
            self.cap_red_var.set(f"Captured Red\nX: {self.captured_red_target[0]:6.1f}  Y: {self.captured_red_target[1]:6.1f}  Z: {self.captured_red_target[2]:6.1f}")
        if self.live_green_target:
            self.captured_green_target = list(self.live_green_target)
            self.cap_green_var.set(f"Captured Green\nX: {self.captured_green_target[0]:6.1f}  Y: {self.captured_green_target[1]:6.1f}  Z: {self.captured_green_target[2]:6.1f}")

    def move_cartesian(self, axis, direction):
        if self.mode == "joint":
            self.cmd_x = self.real_x_cm
            self.cmd_y = self.real_y_cm
            self.cmd_z = self.real_z_cm
            self.cmd_roll = self.real_roll_deg
            self.cmd_pitch = self.real_pitch_deg
            self.cmd_yaw = self.real_yaw_deg
            
        self.mode = "cartesian"

        if axis == 'x': self.cmd_x += self.step_cm * direction
        elif axis == 'y': self.cmd_y += self.step_cm * direction
        elif axis == 'z':
            self.cmd_z += self.step_cm * direction
            if self.cmd_z < 42.0:
                self.cmd_z = 42.0
        elif axis == 'roll': self.cmd_roll += self.step_deg * direction
        elif axis == 'pitch': self.cmd_pitch += self.step_deg * direction
        elif axis == 'yaw': self.cmd_yaw += self.step_deg * direction
            
        msg = Pose()
        msg.position.x = self.cmd_x / 100.0
        msg.position.y = self.cmd_y / 100.0
        msg.position.z = self.cmd_z / 100.0
        qx, qy, qz, qw = self.euler_to_quaternion(
            math.radians(self.cmd_roll), math.radians(self.cmd_pitch), math.radians(self.cmd_yaw))
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        self.cartesian_publisher.publish(msg)

    def execute_apple_action(self, color, action):
        target = self.captured_red_target if color == 'red' else self.captured_green_target
        if target is None: return

        self.mode = "cartesian"
        self.cmd_x = target[0]
        self.cmd_y = target[1]
        
        if action == 'hover':
            self.cmd_z = 53.5
        elif action == 'pick':
            self.cmd_z = 44.5
            
        self.cmd_roll = -180.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0

        msg = Pose()
        msg.position.x = self.cmd_x / 100.0
        msg.position.y = self.cmd_y / 100.0
        msg.position.z = self.cmd_z / 100.0
        qx, qy, qz, qw = self.euler_to_quaternion(
            math.radians(self.cmd_roll), math.radians(self.cmd_pitch), math.radians(self.cmd_yaw))
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        self.cartesian_publisher.publish(msg)

    def execute_center_action(self, action):
        self.mode = "cartesian"
        self.cmd_x = 50.0
        self.cmd_y = 0.0
        
        if action == 'hover':
            self.cmd_z = 53.5
        elif action == 'pick':
            self.cmd_z = 44.5
            
        self.cmd_roll = -180.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0

        msg = Pose()
        msg.position.x = self.cmd_x / 100.0
        msg.position.y = self.cmd_y / 100.0
        msg.position.z = self.cmd_z / 100.0
        qx, qy, qz, qw = self.euler_to_quaternion(
            math.radians(self.cmd_roll), math.radians(self.cmd_pitch), math.radians(self.cmd_yaw))
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        self.cartesian_publisher.publish(msg)

    def execute_close(self, color):
        target_mm = 60.0
        self.updating_sliders = True
        self.gripper_slider.set(target_mm)
        self.updating_sliders = False
        self.send_gripper_gap(target_mm)
        self.attached_color = color
        
        if LINK_ATTACHER_READY:
            if self.attach_status_var:
                self.attach_status_var.set(f"Link Attacher: Attempting to grab {color} apple...")
            
            if not self.attach_client.wait_for_service(timeout_sec=1.0):
                if self.attach_status_var:
                    self.attach_status_var.set("Link Attacher Error: /ATTACHLINK service is missing.")
                return

            req = AttachLink.Request()
            req.model1_name = 'collaborative_arm'
            req.link1_name = 'wrist_3_link'  # Changed from gripper_base
            req.model2_name = f"{color}_apple"
            req.link2_name = 'link'
            
            future = self.attach_client.call_async(req)
            future.add_done_callback(self.attach_done)

    def attach_done(self, future):
        if self.attach_status_var:
            try:
                res = future.result()
                if res.success:
                    self.attach_status_var.set(f"Link Attacher Success: {res.message}")
                else:
                    self.attach_status_var.set(f"Link Attacher Failed: {res.message}")
            except Exception as e:
                self.attach_status_var.set(f"Link Attacher Exception: {str(e)}")

    def execute_open(self):
        target_mm = 100.0
        self.updating_sliders = True
        self.gripper_slider.set(target_mm)
        self.updating_sliders = False
        self.send_gripper_gap(target_mm)
        
        if LINK_ATTACHER_READY and self.attached_color:
            if self.attach_status_var:
                self.attach_status_var.set("Link Attacher: Releasing object...")
                
            if not self.detach_client.wait_for_service(timeout_sec=1.0):
                if self.attach_status_var:
                    self.attach_status_var.set("Link Attacher Error: /DETACHLINK service is missing.")
                return

            req = DetachLink.Request()
            req.model1_name = 'collaborative_arm'
            req.link1_name = 'wrist_3_link'  # Changed from gripper_base
            req.model2_name = f"{self.attached_color}_apple"
            req.link2_name = 'link'
            
            future = self.detach_client.call_async(req)
            future.add_done_callback(self.detach_done)
            
        self.attached_color = None

    def detach_done(self, future):
        if self.attach_status_var:
            try:
                res = future.result()
                if res.success:
                    self.attach_status_var.set(f"Link Attacher Success: {res.message}")
                else:
                    self.attach_status_var.set(f"Link Attacher Failed: {res.message}")
            except Exception as e:
                self.attach_status_var.set(f"Link Attacher Exception: {str(e)}")

    def reset_gazebo_world(self):
        if self.attached_color:
            self.execute_open()
            threading.Timer(1.5, self._perform_reset).start()
        else:
            self._perform_reset()

    def _perform_reset(self):
        os.system('ros2 service call /reset_world std_srvs/srv/Empty')
        self.live_red_target = None
        self.live_green_target = None
        self.captured_red_target = None
        self.captured_green_target = None
        self.cap_red_var.set("Captured Red\nWaiting...")
        self.cap_green_var.set("Captured Green\nWaiting...")
        if self.attach_status_var:
            self.attach_status_var.set("Link Attacher: World Reset.")

    def send_arm_angles(self, angles_rad):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = angles_rad
        point.time_from_start.sec = 1
        traj_msg.points.append(point)
        self.arm_publisher.publish(traj_msg)

    def send_gripper_gap(self, gap_mm):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.gripper_joint_names
        point = JointTrajectoryPoint()
        finger_pos = (100.0 - float(gap_mm)) / 2000.0
        finger_pos = max(0.0, min(0.05, finger_pos))
        point.positions = [finger_pos, finger_pos]
        point.time_from_start.sec = 1
        traj_msg.points.append(point)
        self.gripper_publisher.publish(traj_msg)

    def slider_moved(self, value):
        if self.updating_sliders or self.mode != "joint": return
        angles_rad = []
        for i in range(6):
            val = self.sliders[i].get()
            if len(self.entry_vars) == 6:
                self.entry_vars[i].set(f"{val:.1f}")
            angles_rad.append(math.radians(val))
        self.send_arm_angles(angles_rad)

    def entry_changed(self, event, idx):
        self.mode = "joint"
        self.updating_sliders = True
        try:
            val = float(self.entry_vars[idx].get())
            self.sliders[idx].set(val)
            angles_rad = []
            for i in range(6):
                angles_rad.append(math.radians(self.sliders[i].get()))
            self.send_arm_angles(angles_rad)
        except ValueError:
            pass
        self.updating_sliders = False

    def gripper_moved(self, value):
        if self.updating_sliders: return
        self.send_gripper_gap(self.gripper_slider.get())

    def set_mode_joint(self, event):
        self.mode = "joint"

    def go_home(self):
        self.mode = "joint"
        self.updating_sliders = True
        home_angles_deg = [0.0, -90.0, -90.0, -90.0, 90.0, 0.0]
        home_angles_rad = [0.0, -1.5708, -1.5708, -1.5708, 1.5708, 0.0]
        for i in range(6):
            self.sliders[i].set(home_angles_deg[i])
            if len(self.entry_vars) == 6:
                self.entry_vars[i].set(f"{home_angles_deg[i]:.1f}")
        self.updating_sliders = False
        self.send_arm_angles(home_angles_rad)

    def go_search_state(self):
        self.mode = "joint"
        self.updating_sliders = True
        search_angles_deg = [-180.0, -90.0, -90.0, -90.0, 90.0, 0.0]
        search_angles_rad = [-3.14159, -1.5708, -1.5708, -1.5708, 1.5708, 0.0]
        for i in range(6):
            self.sliders[i].set(search_angles_deg[i])
            if len(self.entry_vars) == 6:
                self.entry_vars[i].set(f"{search_angles_deg[i]:.1f}")
        self.updating_sliders = False
        self.send_arm_angles(search_angles_rad)

def run_interface(ros_node):
    window = tk.Tk()
    window.title("Advanced Arm Control Panel - Debug Mode")
    window.geometry("2000x700")

    main_frame = tk.Frame(window)
    main_frame.pack(fill="both", expand=True, padx=10, pady=10)

    control_frame = tk.Frame(main_frame)
    control_frame.grid(row=0, column=0, sticky="n")

    tk.Label(control_frame, text="Cartesian (cm)", font=("Arial", 12, "bold")).grid(row=0, column=0, columnspan=3, pady=2)
    tk.Label(control_frame, text="Manual Joint Control", font=("Arial", 12, "bold")).grid(row=0, column=3, columnspan=3, pady=2)

    def click_cartesian(axis, direction): ros_node.move_cartesian(axis, direction)

    tk.Button(control_frame, text="X -", command=lambda: click_cartesian('x', -1), height=1, width=5).grid(row=1, column=0, padx=5)
    tk.Button(control_frame, text="X +", command=lambda: click_cartesian('x', 1), height=1, width=5).grid(row=1, column=2, padx=5)
    tk.Button(control_frame, text="Y -", command=lambda: click_cartesian('y', -1), height=1, width=5).grid(row=2, column=0, pady=2)
    tk.Button(control_frame, text="Y +", command=lambda: click_cartesian('y', 1), height=1, width=5).grid(row=2, column=2, pady=2)
    tk.Button(control_frame, text="Z -", command=lambda: click_cartesian('z', -1), height=1, width=5).grid(row=3, column=0, pady=2)
    tk.Button(control_frame, text="Z +", command=lambda: click_cartesian('z', 1), height=1, width=5).grid(row=3, column=2, pady=2)
    tk.Button(control_frame, text="Roll -", command=lambda: click_cartesian('roll', -1), height=1, width=5).grid(row=4, column=0, pady=2)
    tk.Button(control_frame, text="Roll +", command=lambda: click_cartesian('roll', 1), height=1, width=5).grid(row=4, column=2, pady=2)
    tk.Button(control_frame, text="Pitch -", command=lambda: click_cartesian('pitch', -1), height=1, width=5).grid(row=5, column=0, pady=2)
    tk.Button(control_frame, text="Pitch +", command=lambda: click_cartesian('pitch', 1), height=1, width=5).grid(row=5, column=2, pady=2)
    tk.Button(control_frame, text="Yaw -", command=lambda: click_cartesian('yaw', -1), height=1, width=5).grid(row=6, column=0, pady=2)
    tk.Button(control_frame, text="Yaw +", command=lambda: click_cartesian('yaw', 1), height=1, width=5).grid(row=6, column=2, pady=2)

    slider_frame = tk.Frame(control_frame)
    slider_frame.grid(row=1, column=3, rowspan=6, columnspan=3, padx=10)

    labels = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
    for i in range(6):
        tk.Label(slider_frame, text=labels[i]).grid(row=i, column=0, sticky="e")
        s = tk.Scale(slider_frame, from_=-180, to=180, orient="horizontal", length=180, showvalue=0, command=ros_node.slider_moved)
        s.bind("<Button-1>", ros_node.set_mode_joint)
        s.grid(row=i, column=1)
        ros_node.sliders.append(s)
        var = tk.StringVar(value="0.0")
        ros_node.entry_vars.append(var)
        e = tk.Entry(slider_frame, textvariable=var, width=5)
        e.bind("<Return>", lambda event, idx=i: ros_node.entry_changed(event, idx))
        e.bind("<FocusOut>", lambda event, idx=i: ros_node.entry_changed(event, idx))
        e.bind("<Button-1>", ros_node.set_mode_joint)
        e.grid(row=i, column=2, padx=5)

    btn_frame = tk.Frame(slider_frame)
    btn_frame.grid(row=6, column=0, columnspan=3, pady=10)
    tk.Button(btn_frame, text="GO HOME", command=ros_node.go_home, bg="yellow").pack(side="left", padx=5)
    tk.Button(btn_frame, text="SEARCH STATE", command=ros_node.go_search_state, bg="lightblue").pack(side="left", padx=5)

    tk.Label(control_frame, text="Gripper Gap (mm)", font=("Arial", 10)).grid(row=7, column=3, columnspan=3)
    gripper_slider = tk.Scale(control_frame, from_=0, to=100, orient="horizontal", length=180, command=ros_node.gripper_moved)
    gripper_slider.bind("<Button-1>", ros_node.set_mode_joint)
    gripper_slider.set(100)
    gripper_slider.grid(row=8, column=3, columnspan=3)
    ros_node.gripper_slider = gripper_slider

    status_frame = tk.Frame(control_frame)
    status_frame.grid(row=9, column=0, columnspan=6, pady=20)

    real_text = tk.StringVar()
    real_text.set("Listening to real position...")
    ros_node.real_loc_var = real_text
    angle_text = tk.StringVar()
    angle_text.set("Waiting for joint angles...")
    ros_node.angle_text_var = angle_text

    tk.Label(status_frame, text="Real Position (World to TCP):", font=("Arial", 10, "bold")).pack()
    tk.Label(status_frame, textvariable=real_text, font=("monospace", 11, "bold"), fg="red").pack()
    tk.Label(status_frame, textvariable=angle_text, font=("monospace", 10), fg="green").pack()

    action_frame = tk.Frame(main_frame)
    action_frame.grid(row=0, column=1, padx=10, sticky="n")

    tk.Label(action_frame, text="Vision Targets & Actions", font=("Arial", 14, "bold")).pack(pady=5)
    
    live_frame = tk.Frame(action_frame)
    live_frame.pack(pady=10)
    ros_node.live_red_var = tk.StringVar(value="Live Red\nWaiting...")
    tk.Label(live_frame, textvariable=ros_node.live_red_var, font=("monospace", 10), fg="red", width=35).pack(side="left")
    ros_node.live_green_var = tk.StringVar(value="Live Green\nWaiting...")
    tk.Label(live_frame, textvariable=ros_node.live_green_var, font=("monospace", 10), fg="green", width=35).pack(side="left")

    tk.Button(action_frame, text="CAPTURE TARGETS", command=ros_node.capture_positions, bg="purple", fg="white", font=("Arial", 12, "bold"), width=20, height=2).pack(pady=10)

    cap_frame = tk.Frame(action_frame)
    cap_frame.pack(pady=10)
    
    red_col = tk.Frame(cap_frame)
    red_col.pack(side="left", padx=10)
    ros_node.cap_red_var = tk.StringVar(value="Captured Red\nWaiting...")
    tk.Label(red_col, textvariable=ros_node.cap_red_var, font=("monospace", 10), fg="darkred", width=35).pack(pady=5)
    tk.Button(red_col, text="Hover Red (+10cm)", command=lambda: ros_node.execute_apple_action('red', 'hover'), bg="pink", width=22).pack(pady=5)
    tk.Button(red_col, text="Pick Red Target", command=lambda: ros_node.execute_apple_action('red', 'pick'), bg="lightcoral", width=22).pack(pady=5)
    
    red_btn_frame = tk.Frame(red_col)
    red_btn_frame.pack(pady=5)
    tk.Button(red_btn_frame, text="Close", command=lambda: ros_node.execute_close('red'), bg="tomato", width=10).pack(side="left", padx=2)
    tk.Button(red_btn_frame, text="Open", command=ros_node.execute_open, bg="lightgray", width=10).pack(side="left", padx=2)

    green_col = tk.Frame(cap_frame)
    green_col.pack(side="left", padx=10)
    ros_node.cap_green_var = tk.StringVar(value="Captured Green\nWaiting...")
    tk.Label(green_col, textvariable=ros_node.cap_green_var, font=("monospace", 10), fg="darkgreen", width=35).pack(pady=5)
    tk.Button(green_col, text="Hover Green (+10cm)", command=lambda: ros_node.execute_apple_action('green', 'hover'), bg="lightgreen", width=22).pack(pady=5)
    tk.Button(green_col, text="Pick Green Target", command=lambda: ros_node.execute_apple_action('green', 'pick'), bg="palegreen", width=22).pack(pady=5)
    
    green_btn_frame = tk.Frame(green_col)
    green_btn_frame.pack(pady=5)
    tk.Button(green_btn_frame, text="Close", command=lambda: ros_node.execute_close('green'), bg="limegreen", width=10).pack(side="left", padx=2)
    tk.Button(green_btn_frame, text="Open", command=ros_node.execute_open, bg="lightgray", width=10).pack(side="left", padx=2)

    center_col = tk.Frame(cap_frame)
    center_col.pack(side="left", padx=10)
    tk.Label(center_col, text="Center Swap Area\nX: 50.0  Y: 0.0", font=("monospace", 10), fg="blue", width=35).pack(pady=5)
    tk.Button(center_col, text="Hover Center (+10cm)", command=lambda: ros_node.execute_center_action('hover'), bg="lightblue", width=22).pack(pady=5)
    tk.Button(center_col, text="Place/Pick Center", command=lambda: ros_node.execute_center_action('pick'), bg="skyblue", width=22).pack(pady=5)
    
    center_btn_frame = tk.Frame(center_col)
    center_btn_frame.pack(pady=5)
    tk.Button(center_btn_frame, text="Release Apple", command=ros_node.execute_open, bg="lightgray", width=15).pack(side="left", padx=2)

    tk.Button(action_frame, text="Reset Gazebo Cubes", command=ros_node.reset_gazebo_world, bg="orange", width=20).pack(pady=10)

    ros_node.attach_status_var = tk.StringVar(value="Link Attacher Status: Waiting for user action...")
    tk.Label(action_frame, textvariable=ros_node.attach_status_var, font=("Arial", 11, "bold"), fg="blue").pack(pady=5)

    debug_frame = tk.Frame(main_frame, highlightbackground="black", highlightthickness=1)
    debug_frame.grid(row=0, column=2, sticky="n")

    tk.Label(debug_frame, text="Transformation Matrices", font=("Arial", 12, "bold")).pack(pady=5)
    ros_node.debug_cam_to_apple = tk.StringVar(value="Cam to Apple: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_cam_to_apple, font=("monospace", 10), justify="left").pack(pady=2, anchor="w")
    ros_node.debug_base_to_tcp = tk.StringVar(value="Base to TCP: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_base_to_tcp, font=("monospace", 10), justify="left").pack(pady=2, anchor="w")
    ros_node.debug_base_to_cam = tk.StringVar(value="Base to Cam: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_base_to_cam, font=("monospace", 10), justify="left").pack(pady=2, anchor="w")
    ros_node.debug_world_to_tcp = tk.StringVar(value="World to TCP: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_world_to_tcp, font=("monospace", 10), justify="left").pack(pady=2, anchor="w")
    ros_node.debug_world_to_cam = tk.StringVar(value="World to Cam: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_world_to_cam, font=("monospace", 10), justify="left").pack(pady=2, anchor="w")
    ros_node.debug_base_to_apple = tk.StringVar(value="Base to Apple: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_base_to_apple, font=("monospace", 10), justify="left").pack(pady=2, anchor="w")
    ros_node.debug_world_to_apple = tk.StringVar(value="World to Apple: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_world_to_apple, font=("monospace", 10), justify="left").pack(pady=2, anchor="w")

    tk.Label(debug_frame, text="Ground Truth Comparisons", font=("Arial", 12, "bold")).pack(pady=(15,5))
    ros_node.debug_real_apple = tk.StringVar(value="Gazebo Ground Truth: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_real_apple, font=("monospace", 10), fg="blue", justify="left").pack(pady=2, anchor="w")

    ros_node.go_home()
    window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = VisualJogger()
    gui_thread = threading.Thread(target=run_interface, args=(node,))
    gui_thread.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()