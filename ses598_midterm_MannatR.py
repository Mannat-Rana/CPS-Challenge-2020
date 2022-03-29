
"""
testing offboard positon control with a simple takeoff script
"""
import time
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
import math
import numpy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler


class OffbPosCtl:
    curr_drone_pose = PoseStamped()
    waypointIndex = 0
    distThreshold = 1
    sim_ctr = 1

    des_pose = PoseStamped()
    isReadyToFly = False
    # location
    orientation = quaternion_from_euler(0, 0, 3.14 / 2 + 3.14 / 8)
    locations = numpy.matrix([
                              [60.5, -12.5, 21, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [61, -13, 21, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [61.5, -12.5, 21, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [61, -12.5, 21, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [60.5, -12.5, 21, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [60.25, -12, 21, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [59.75, -11.5, 21, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [59, -11.75, 21, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [60, -12, 21, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [40.75, 4, 15, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [13, -65.5, 5, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [13, -65.5, 0, orientation[0], orientation[1], orientation[2], orientation[3]]
                              ])

    def mavrosTopicStringRoot(self, uavID=0):
        mav_topic_string = '/mavros/'
        return mav_topic_string

    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        drone_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.drone_pose_cb)
        rover_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.rover_pose_cb)
        state_sub = rospy.Subscriber('/mavros/state', State, callback=self.drone_state_cb)
        self.image_sub = rospy.Subscriber('/uav_camera_down/image_raw', Image, callback=self.image_cb)
        self.bridge = CvBridge()
        self.probe_location_img = [None, None]
        self.vel_command = Twist()
        self.probe_visible = False
        self.data_mulled = False
        self.rock_mapped = False
        self.mission_complete = False


        attach = rospy.Publisher('/attach', String, queue_size=10)
        NUM_UAV = 2
        mode_proxy = [None for i in range(NUM_UAV)]
        arm_proxy = [None for i in range(NUM_UAV)]
        self.current_image = None

        # Comm for drones
        for uavID in range(0, NUM_UAV):
            mode_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
            arm_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/cmd/arming', CommandBool)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_drone_pose)
        shape = self.locations.shape
        print("Going to loop around rock for mapping")
        while not rospy.is_shutdown():
            #print self.sim_ctr, shape[0], self.waypointIndex
            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = mode_proxy[uavID](1, 'OFFBOARD')
                except rospy.ServiceException, e:
                    print ("mavros/set_mode service call failed: %s" % e)

            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                rospy.wait_for_service(self.mavrosTopicStringRoot(uavID) + '/cmd/arming')

            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = arm_proxy[uavID](True)
                except rospy.ServiceException, e:
                    print ("mavros1/set_mode service call failed: %s" % e)

            if self.waypointIndex is shape[0]:
                self.waypointIndex = shape[0] - 1
                self.mission_complete = True
                rospy.wait_for_service('/mavros/cmd/land')
                landService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
                landService(altitude=0, latitude=self.des_pose.pose.position.x, longitude=self.des_pose.pose.position.y, min_pitch=0, yaw=0)
                print("Finished landing...Mission Complete")

            if self.waypointIndex == 9 and (not self.rock_mapped):
                self.rock_mapped = True
                print("Finished mapping rock, moving to probe")


            if self.isReadyToFly:
                des = self.set_desired_pose().position
                azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                     self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                az_quat = quaternion_from_euler(0, 0, azimuth)

                curr = self.curr_drone_pose.pose.position
                dist = math.sqrt(
                    (curr.x - des.x) * (curr.x - des.x) + (curr.y - des.y) * (curr.y - des.y) + (curr.z - des.z) * (
                            curr.z - des.z))
                if (dist < self.distThreshold) and (not self.mission_complete):
                    self.waypointIndex += 1

                if (self.probe_visible) and (not self.data_mulled) and (self.rock_mapped):
                    self.vis_servo_probe()

            self.pose_pub.publish(self.des_pose)
            rate.sleep()

    def vis_servo_probe(self):
        print("Found the probe, now staying centered around it with visual servoing while mulling data...")
        y_not_center = True
        x_not_center = True
        center_counter = 0
        while (center_counter < 5000) and (not rospy.is_shutdown()) and (self.probe_visible):
            x_offset = self.probe_location_img[0] - 320
            y_offset = self.probe_location_img[1] - 240
            #print(x_offset, y_offset)
            if abs(x_offset) > 40:
                x_not_center = True
                if x_offset < 0:
                    self.des_pose.pose.position.x -= 0.000001
                else:
                    self.des_pose.pose.position.x += 0.000001
            else:
                x_not_center = False

            if abs(y_offset) > 40:
                y_not_center = True
                if y_offset < 0:
                    self.des_pose.pose.position.y += 0.000001
                else:
                    self.des_pose.pose.position.y -= 0.000001
            else:
                y_not_center = False
            self.pose_pub.publish(self.des_pose)
            if (not x_not_center) and (not y_not_center):
                center_counter += 1
                #print(center_counter)

        if self.probe_visible:
            print("Finished mulling, leaving probe to go land")
            self.data_mulled = True


    def image_cb(self, img):
        self.current_image = self.bridge.imgmsg_to_cv2(img, "passthrough")
        hsv = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)
        red_lower = numpy.array([110, 100, 50], numpy.uint8)
        red_upper = numpy.array([130, 255, 255], numpy.uint8)
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        kernal = numpy.ones((5, 5), "uint8")

        # For red color
        red_mask = cv2.dilate(red_mask, kernal)
        res_red = cv2.bitwise_and(self.current_image, self.current_image,
                                  mask=red_mask)


        # Creating contour to track red color
        _, contours, _ = cv2.findContours(red_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 300):
                x, y, w, h = cv2.boundingRect(contour)
                self.current_image = cv2.rectangle(self.current_image, (x, y),
                                           (x + w, y + h),
                                           (0, 0, 255), 2)

                cv2.putText(self.current_image, "Probe", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 255))
                x_center = x + (w/2)
                y_center = y + (h/2)
                self.probe_location_img = [x_center, y_center]
                self.probe_visible = True
                cv2.circle(self.current_image, (x_center, y_center), 3, (0, 0, 255))
            else:
                self.probe_visible = False
        cv2.imshow("Image Window", self.current_image)
        cv2.waitKey(3)

    def set_desired_pose(self):
        self.des_pose.pose.position.x = self.locations[self.waypointIndex, 0]
        self.des_pose.pose.position.y = self.locations[self.waypointIndex, 1]
        self.des_pose.pose.position.z = self.locations[self.waypointIndex, 2]
        self.des_pose.pose.orientation.x = self.locations[self.waypointIndex, 3]
        self.des_pose.pose.orientation.y = self.locations[self.waypointIndex, 4]
        self.des_pose.pose.orientation.z = self.locations[self.waypointIndex, 5]
        self.des_pose.pose.orientation.w = self.locations[self.waypointIndex, 6]
        if self.locations[self.waypointIndex, :].sum() == 0:
            self.des_pose.pose.position.x = self.curr_rover_pose.pose.position.x
            self.des_pose.pose.position.y = self.curr_rover_pose.pose.position.y
            self.des_pose.pose.position.z = max(self.curr_rover_pose.pose.position.z, 10)
            orientation = quaternion_from_euler(0, 0, 3.14 / 2)
            self.des_pose.pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        return self.des_pose.pose

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def drone_pose_cb(self, msg):
        self.curr_drone_pose = msg

    def rover_pose_cb(self, msg):
        self.curr_rover_pose = msg

    def drone_state_cb(self, msg):
        #print msg.mode
        if (msg.mode == 'OFFBOARD'):
            self.isReadyToFly = True
            #print "readyToFly"


if __name__ == "__main__":
    OffbPosCtl()