import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Image
from cv_bridge import CvBridge
import cv2
import subprocess
import os
import signal

class DualCamJoyController(Node):
    def __init__(self):
        super().__init__('joy_cam_controller')
        
        # Subscriptions
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        # Using 10 depth; if lag occurs, consider qos_profile_sensor_data
        self.cam1_sub = self.create_subscription(Image, '/front_camera/image_raw', self.cam1_cb, 10)
        self.cam2_sub = self.create_subscription(Image, '/rear_camera/image_raw', self.cam2_cb, 10)
        
        self.bridge = CvBridge()
        self.frame1 = None
        self.frame2 = None
        
        # Recording States
        self.recording_processes = {1: None, 2: None}
        self.video_paths = {1: "", 2: ""}
        
        # Button Mapping (Verified for your 8-axis controller)
        self.BTN_LB = 6   # Index 6
        self.BTN_RB = 7   # Index 7
        self.AXIS_LT = 4  # Index 4
        self.AXIS_RT = 5  # Index 5
        
        self.lt_pressed = False
        self.rt_pressed = False
        self.lb_last_state = 0
        self.rb_last_state = 0

        self.save_dir = os.path.expanduser("~/forensic/ROBOT/Tests/Videos/")
        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info(f"Controller Ready. Files saving to: {self.save_dir}")

    def cam1_cb(self, msg):
        try:
            self.frame1 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Cam1 Bridge Error: {e}")

    def cam2_cb(self, msg):
        try:
            self.frame2 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Cam2 Bridge Error: {e}")

    def joy_callback(self, msg):
        # Safety check for axis length
        if len(msg.axes) < 6: return

        # --- CAMERA 1 (LB Toggle Rec | LT Snapshot) ---
        if msg.buttons[self.BTN_LB] == 1 and self.lb_last_state == 0:
            self.toggle_recording(1)
        self.lb_last_state = msg.buttons[self.BTN_LB]

        # LT Trigger (usually axis 4)
        if msg.axes[self.AXIS_LT] < -0.7:
            if not self.lt_pressed:
                self.take_snapshot(1)
                self.lt_pressed = True
        elif msg.axes[self.AXIS_LT] > 0.0:
            self.lt_pressed = False

        # --- CAMERA 2 (RB Toggle Rec | RT Snapshot) ---
        if msg.buttons[self.BTN_RB] == 1 and self.rb_last_state == 0:
            self.toggle_recording(2)
        self.rb_last_state = msg.buttons[self.BTN_RB]

        # RT Trigger (usually axis 5)
        if msg.axes[self.AXIS_RT] < -0.7:
            if not self.rt_pressed:
                self.take_snapshot(2)
                self.rt_pressed = True
        elif msg.axes[self.AXIS_RT] > 0.0:
            self.rt_pressed = False

    def take_snapshot(self, cam_num):
        frame = self.frame1 if cam_num == 1 else self.frame2
        if frame is not None:
            timestamp = self.get_clock().now().to_msg().sec
            name = f"snap_cam{cam_num}_{timestamp}.jpg"
            full_path = os.path.join(self.save_dir, name)
            cv2.imwrite(full_path, frame)
            self.get_logger().info(f"[recorder_node][Snapshot saved: {full_path}]")
        else:
            self.get_logger().warn(f"[recorder_node][Cannot snapshot Cam{cam_num}: No frame received yet.]")

    def toggle_recording(self, cam_num):
        # STOP
        if self.recording_processes[cam_num] is not None:
            self.get_logger().info(f"[recorder_node][Stopping Bag for Cam {cam_num}...]")
            # Bags need SIGINT to close the database index correctly
            self.recording_processes[cam_num].send_signal(signal.SIGINT)
            try:
                self.recording_processes[cam_num].wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"Bag {cam_num} didn't close gracefully, killing...")
                self.recording_processes[cam_num].kill()
            
            self.recording_processes[cam_num] = None
            return

        # START
        topic = "/front_camera/image_raw" if cam_num == 1 else "/rear_camera/image_raw"
        timestamp = self.get_clock().now().to_msg().sec
        # Ros2 bag creates a directory, not a single file
        bag_dir = os.path.join(self.save_dir, f"bag_cam{cam_num}_{timestamp}")

        cmd = [
            "ros2", "bag", "record",
            "-o", bag_dir,
            topic,
            "--storage", "mcap" # Highly recommended for image data
        ]
        
        try:
            self.recording_processes[cam_num] = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            self.get_logger().info(f"[recorder_node][Recording Bag Cam {cam_num} to: {bag_dir}]")
        except Exception as e:
            self.get_logger().error(f"[recorder_node][Bag recording failed: {e}]")

def main():
    rclpy.init()
    node = DualCamJoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup any running recorders on exit
        for p in node.recording_processes.values():
            if p: p.kill()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()