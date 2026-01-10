#!/usr/bin/env python3
"""
QR code mission test script
------------------------------
Author: Jess Mathews
GitHub: https://github.com/jessmathews

Function:
Aligns centre of camera frame and centre of qr code using cmd_vel commands with a proportional controller
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped
from geographic_msgs.msg import GeoPoseStamped

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool,CommandTOL
from cv_bridge import CvBridge
import cv2
import time
from qrdet import QRDetector

class QRTest(Node):
    def __init__(self):
        super().__init__("mission_test_node")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.detector = QRDetector(model_size="n")
        self.bridge = CvBridge()
        
        # Publishers
        self.vel_pub = self.create_publisher(TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        self.global_setpoint_pub = self.create_publisher(GeoPoseStamped,"/mavros/setpoint_position/global",10)


        # Subscribers
        self.state_sub = self.create_subscription(State, "/mavros/state", self.state_callback, qos_profile)
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, qos_profile)
        self.pose_sub = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.pose_callback, qos_profile)
        
        # Service Clients
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        
        
        print("Waiting for services...")
        self.set_mode_client.wait_for_service(timeout_sec=10.0)
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.takeoff_client.wait_for_service(timeout_sec=10.0)
        print("Services ready!\n")
        
        # Parameters
        self.kp = 0.05
        self.max_vel = 1.0
        self.land_threshold = 15.0  
        self.required_stable_time = 2.0 
        self.descend_speed = -0.3     # m/s
        self.ascend_speed  =  0.4     # m/s
        self.z_hold_speed  =  0.0

        self.min_land_altitude  = 1.5     # trigger land below this
        self.max_search_alt = 6.0    # don't climb forever

                
        # State variables
        self.centered_start_time = None
        self.landing_phase = False 
        self.final_land_triggered = False
        self.qr_detection = False 
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.prev_time = None
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.err_x = 0.0
        self.err_y = 0.0
        self.mission_complete = False



        self.get_logger().info("Mission Test Node Started.")
            
    
    def state_callback(self, msg):
        self.current_state = msg


    def arm(self) -> bool:
        """Arm the vehicle"""
        self.get_logger().info("Arming throttle...")
        req = CommandBool.Request()
        req.value = True

        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info("Armed\n")
            return True
        else:
            print("Failed to arm\n")
            return False
    
    def takeoff(self,altitude:float) -> bool:
        """Takeoff to specific altitude"""
        self.get_logger().info(f"Taking off to {altitude}m...")

        req = CommandTOL.Request()

        req.altitude = altitude
        
        future = self.takeoff_client.call_async(req)

        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)


        if future.result() and future.result().success:
            self.get_logger().info(f"Takeoff command sent\n")
            return True
        else:
            self.get_logger().error("Takeoff command failed\n")
            return False
          
    def wait_for_altitude(self,target_alt:float,tolerance:float = 0.2) -> bool:
        """
        Wait for altitude to be reached
        
        :param target_alt: Target altitude to be reached
        """
        print(f"Waiting to reach {target_alt}m...")

        while rclpy.ok():
            rclpy.spin_once(self,timeout_sec=0.1)
            current_alt = self.current_pose.pose.position.z
            print(f"Altitude: {current_alt:.2f}m / {target_alt}m ",end="\r")

            if abs(current_alt-target_alt) < tolerance:
                self.get_logger().info(f"Reached Altitude {current_alt:.2f}m ")
                return True
            
            time.sleep(0.2)


        

    def goto_gps(self, latitude: float, longitude: float, altitude: float, duration: float = 15.0) -> bool:
        """
        Fly to a GPS coordinate using MAVROS global position setpoints
        :param latitude: Latitude coordinates
        :param longitude: Longitude coordinates
        :param altitude: ASML altitude (in meters)
        :duration: time to publish coordinates

        """

        self.get_logger().info(f"Going to GPS -> lat={latitude}, lon={longitude}, alt={altitude}")

        target = GeoPoseStamped()

        # REQUIRED
        target.header.frame_id = ""

        target.pose.position.latitude = float(latitude)
        target.pose.position.longitude = float(longitude)
        target.pose.position.altitude = float(altitude)

        # Neutral orientation (yaw = 0)
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 1.0

        rate_hz = 10.0   # MUST be >= 2 Hz
        period = 1.0 / rate_hz

        start_time = self.get_clock().now()

        while rclpy.ok():
            now = self.get_clock().now()
            elapsed = (now - start_time).nanoseconds * 1e-9

            if elapsed > duration:
                break

            target.header.stamp = now.to_msg()
            self.global_setpoint_pub.publish(target)

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        self.get_logger().info("GPS setpoint streaming completed")
        return True


    def pose_callback(self, msg):
        # Update current altitude from Local Position (Z)
        self.current_pose = msg

    def set_mode(self,mode):
        """Set flight mode"""
        if self.mission_complete:
            return
        self.get_logger().info(f"Setting {mode} mode")
        req = SetMode.Request()
        req.custom_mode = mode

        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self,future,timeout_sec=5.0)

        if future.result() and  future.result().mode_sent:
            self.get_logger().info(f"{mode} set\n")
            return True
        else:
            self.get_logger().error("Failed to set mode")
            return False



    def trigger_final_land(self):
        if self.final_land_triggered:
            return
        self.set_mode("LAND")
        self.final_land_triggered = True
        self.mission_complete = True
    
    def image_callback(self, msg):
        if self.final_land_triggered:
            return 
        if not self.qr_detection:
            return
        # print(type(msg))
        #  IMAGE 
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.resize(cv_img, (320, 320))
        h, w = img.shape[:2]
        cx_img, cy_img = w // 2, h // 2
        # convert to grayscale only if necessary qrdet is said to work on "BGR color profiles better" and change is_bgr to False
        # img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(image=img, is_bgr=True)

        #  VELOCITY MSG 
        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = self.get_clock().now().to_msg()
        vel_cmd.header.frame_id = ""

        current_alt = self.current_pose.pose.position.z

        #  PARAMETERS 
        kp_xy = 0.005
        kd_xy = 0.020

        xy_deadband = 10          # pixels
        max_xy_vel = 0.8

        descend_speed = -0.3
        

        #  DEFAULT 
        vel_cmd.twist.linear.x = 0.0
        vel_cmd.twist.linear.y = 0.0
        vel_cmd.twist.linear.z = 0.0

        now = time.time()
        if self.prev_time is None:
            self.prev_time = now
            return

        dt = max(now - self.prev_time, 1e-3)
        self.prev_time = now

        #  QR DETECTED 
        if detections:
            box = detections[0]["bbox_xyxy"]
            cx = (box[0] + box[2]) / 2
            cy = (box[1] + box[3]) / 2

            self.err_x = cx - cx_img
            self.err_y = cy - cy_img

            # Deadband
            if abs(self.err_x) < xy_deadband:
                self.err_x = 0.0
            if abs(self.err_y) < xy_deadband:
                self.err_y = 0.0

            # Derivative
            derr_x = (self.err_x - self.prev_err_x) / dt
            derr_y = (self.err_y - self.prev_err_y) / dt

            self.prev_err_x = self.err_x
            self.prev_err_y = self.err_y

            # PD control
            vx = -(kp_xy * self.err_x + kd_xy * derr_x)
            vy =  (kp_xy * self.err_y + kd_xy * derr_y)

            # QR size scaling
            qr_area = (box[2] - box[0]) * (box[3] - box[1])
            scale = max(0.2, min(1.0, 15000.0 / max(qr_area, 1.0)))
            vx *= scale
            vy *= scale

            # Velocity limit
            vel_cmd.twist.linear.x = max(min(vx, max_xy_vel), -max_xy_vel)
            vel_cmd.twist.linear.y = max(min(vy, max_xy_vel), -max_xy_vel)

            qr_centered = (
                abs(self.err_x) < self.land_threshold and
                abs(self.err_y) < self.land_threshold
            )

            #  Z CONTROL 
            if qr_centered and current_alt <= self.min_land_altitude:
                self.get_logger().info("QR centered & low altitude → LAND")
                self.qr_detection = False
                self.trigger_final_land()
                return

            if qr_centered:
                vel_cmd.twist.linear.z = descend_speed
                # Freeze XY during descent
                vel_cmd.twist.linear.x = 0.0
                vel_cmd.twist.linear.y = 0.0

            else:
                vel_cmd.twist.linear.z = 0.0

        #  QR LOST 
        else:
            if self.final_land_triggered:
                return
            if not self.qr_detection:
                return
  
            self.get_logger().info("Entering RECOVERY!")
            # Derivative
            # derr_x = (self.err_x - self.prev_err_x) / dt
            # derr_y = (self.err_y - self.prev_err_y) / dt

            # self.prev_err_x = self.err_x
            # self.prev_err_y = self.err_y

            # P control
            vx = -(kp_xy * self.err_x)
            vy =  (kp_xy * self.err_y)

            # Start moving towards last seen position
            print("moving to qr!",end="\r")
            vel_cmd.twist.linear.x = max(min(vx, max_xy_vel), -max_xy_vel)
            vel_cmd.twist.linear.y = max(min(vy, max_xy_vel), -max_xy_vel)



            """
            # RECOVER QR IF LOST
            self.prev_err_x = 0.0
            self.prev_err_y = 0.0
            print("QR LOST!!")

            if current_alt < max_search_alt:
                vel_cmd.twist.linear.z = ascend_speed
            else:
                vel_cmd.twist.linear.z = 0.0
            """

        #  PUBLISH 
        self.vel_pub.publish(vel_cmd)

        #  DEBUG (optional) 
        print(f"ALT={current_alt:.3f} QR={'YES' if detections else 'NO'} "
              f"V=({vel_cmd.twist.linear.x:.2f},"
              f"{vel_cmd.twist.linear.y:.2f},"
              f"{vel_cmd.twist.linear.z:.2f})",end='\r')
    
    def run_mission(self):
        """Mission Execution"""
        print("="*70)
        print("MISSION: TAKEOFF to 5m -> NAVIGATE GPS -> DETECT QR -> LAND on QR")
        print("="*70)
        print()
        
        print("Waiting for FCU connection...")
        while not self.current_state.connected and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        print("Connected to FCU\n")
        print("starting timer...")
        start_time = time.time()

        print("[1] Set GUIDED mode")
        if not self.set_mode("GUIDED"):
            return
        time.sleep(1)

        print("[2] ARM throttle")
        if not self.arm():
            return
        time.sleep(2)

        
        print("[3] Takeoff to 5m")
        if not self.takeoff(5.0):
            return
        
        self.wait_for_altitude(5.0)
        time.sleep(1)
        print()
        
        print("[4] Navigate to GPS Coordinates 8.544100,76.904188,5")
        if not self.goto_gps(latitude=8.544100,longitude=76.904188,altitude=5.0):

            return
        self.qr_detection = True
        
        print()
        print("[5] Land on QR")
        print("starting qr precision landing")

        while not self.mission_complete and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f"Mission duration: {((time.time() - start_time)/60.0):.2f} minutes")
        # self.image_callback(self)




def main():
    rclpy.init()
    node = QRTest()
    try:
        # rclpy.spin(node)
        node.run_mission()
        start_time = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()