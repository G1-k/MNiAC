#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
from geometry_msgs.msg import TwistStamped

#global variable
latitude =0.0
longitude=0.0


def Takeoff():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='GUIDED')
    except rospy.ServiceException, e:
        print ("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e)

    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print ("Service arm call failed: %s"%e)

    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
        # Set Altitude Here
        takeoffService(altitude = 5, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print ("Service takeoff call failed: %s"%e)
    

def Land():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        
        isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException, e:
        print ("Service land call failed: %s. The vehicle cannot land "%e)

def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
    #print ("longitude: %.7f" %longitude)
    #print ("latitude: %.7f" %latitude)

def Square_path():
    square_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    square = TwistStamped()

    user_input = raw_input("Enter square size in mts: "); # 
    side_length = float(user_input)
    flag_x = 1
    flag_y = 1
    for x in range(2,6):
    	print 'x=', (x)
    	print '4%x=', (4%x)
	if 4%x == 0:
		square.twist.linear.x = side_length
		flag_x= -1
	else:
		square.twist.linear.y = side_length
		flag_y= -1

	square_pub.publish(square)
	rospy.sleep(5)

	square.twist.linear.x=0;
	square.twist.linear.y=0;
	square_pub.publish(square);
	rospy.sleep(2);
	if flag_x == -1 and flag_y == -1:
		side_length *= -1
		flag_x = 1
		flag_y = 1



def stop(): 

    stop_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    stop = TwistStamped()
    stop.twist.linear.x=0.0
    stop.twist.linear.y=0.0
    stop.twist.angular.z=0.0 
    stop_pub.publish(stop)


    
def Loop():
    x='3'
    while ((not rospy.is_shutdown())):
        print("1 - Takeoff")
        print("2 - Land")
        print("3 - Square Path")
        print("4 - Stop the drone")
        x = raw_input("Enter your input: ");
        if (x=='1'):
            Takeoff()
        elif(x=='2'):
            Land()
        elif(x=='3'):
            Square_path()
        elif(x=='4')
            stop()
        else: 
            print ("Invalid input")
        
        
    
if __name__ == '__main__':
    rospy.init_node('Square_path', anonymous=True)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    Loop()
