#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self, arg_alt):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = arg_alt, latitude = float('nan'), longitude = float('nan'), min_pitch=0.0, yaw=0.0)
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoTakeoffMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.TAKEOFF')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autotakeoff Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e

    def change_mav_frame(self):
        rospy.wait_for_service('/mavros/setpoint_velocity/mav_frame')
        try:
            frame = rospy.ServiceProxy('/mavros/setpoint_velocity/mav_frame', mavros_msgs.srv.SetMavFrame)
            resp = frame(8)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed")


class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        self.vel = TwistStamped()

        self.vel.twist.linear.y = 0
        self.vel.twist.linear.z = 0
        self.vel.twist.linear.x = 0
        self.vel.twist.angular.z = 0
        self.vel.twist.angular.x = 0
        self.vel.twist.angular.y = 0
        # Instantiate a setpoints message
        self.sp = PositionTarget()

        self.g = GlobalPositionTarget()
        self.g.altitude = 0
        self.g.latitude = 0
        self.g.longitude = 0
        self.g.type_mask=4088
        self.g.coordinate_frame=6
        # # set the flag to use position setpoints and yaw angle
        # self.sp.type_mask = int('010111111000', 2)
        # # LOCAL_NED
        # self.sp.coordinate_frame = 1

        # # We will fly at a fixed altitude for now
        # # Altitude setpoint, [meters]
        # self.ALT_SP = 3.0
        # # update the setpoint message with the required altitude
        # self.sp.position.z = self.ALT_SP
        # # Step size for position update
        # self.STEP_SIZE = 2.0
        # # Fence. We will assume a square fence for now
        # self.FENCE_LIMIT = 5.0

        # # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 3.0)

        # # initial values for setpoints
        # self.sp.position.x = 0.0
        # self.sp.position.y = 0.0

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

    # Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y

    def x_dir(self):
        self.sp.position.x = self.local_pos.x + 5
        self.sp.position.y = self.local_pos.y

    def neg_x_dir(self):
        self.sp.position.x = self.local_pos.x - 5
        self.sp.position.y = self.local_pos.y

    def y_dir(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y + 5

    def neg_y_dir(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y - 5


# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    g_pub = rospy.Publisher(
            '/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)

    #modes.change_mav_frame()

    # Make sure the drone is armed
    # while not cnt.state.armed:
    #     modes.setArm()
    #     rate.sleep()

    # set in takeoff mode and takeoff to default altitude (3 m)
    # modes.setOffboardMode()
    #modes.setTakeoff()
    # rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect


    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
    # activate OFFBOARD mode
    #modes.setAutoTakeoffMode()
    modes.setTakeoff(12)

    k=0
    while k<10:
        #sp_pub.publish(cnt.sp)
        #vel_pub.publish(cnt.vel)
        g_pub.publish(cnt.g)
        rate.sleep()
        k = k + 1
    
    while cnt.state.mode != "OFFBOARD":
        if cnt.local_pos.z > 2.2:
            rospy.loginfo("OFFBOARD")
            modes.setOffboardMode()
            break


    # ROS main loop
    while not rospy.is_shutdown():
        #cnt.updateSp()
        #sp_pub.publish(cnt.sp)

        speed = 4
        radius = 2
        # cnt.vel.twist.linear.x = speed
        # cnt.vel.twist.angular.z = - speed/radius
        # vel_pub.publish(cnt.vel)
        cnt.g.altitude = 20
        cnt.g.latitude = 47.397664
        cnt.g.longitude = 8.545923
        g_pub.publish(cnt.g)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
