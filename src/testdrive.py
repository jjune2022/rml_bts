# ros
import rospy
from erp_com.msg import Status, Cmd
from geometry_msgs.msg import PointStamped


class ERPtestdrive:
    def __init__(self):
        rospy.Subscriber("/current_pos", PointStamped, self.testdrive)
        rospy.Subscriber("/erp42_status", Status, self.update)
        self.msg_pub = rospy.Publisher("/erp42_command", Cmd, queue_size=10)
        
    def update(self, msg):
        # int8 control_mode
        # bool e_stop
        # uint8 gear
        # uint8 speed
        # int32 steer
        # uint8 brake
        # int32 encoder
        self.gear = msg.gear
        self.e_stop = msg.e_stop
        self.encoder = msg.encoder
        self.speed = msg.speed
        self.brake = msg.brake
        self.steer = msg.steer
        
        
        msg = Cmd()
        msg.e_stop = 0
        msg.gear = 0
        msg.brake = 100
        msg.speed = 0
        steering = 0
        msg.steer = steering / 71
        print(self.encoder)
        if self.e_stop == 0: # if emergency stop is not working
            if self.gear == 0: # if gear is on front
                if self.encoder <= -100: # while car drives about 1.2m
                    msg.brake = 0
                    msg.speed = 20
                    # msg.steer = 1000
        self.msg_pub.publish(msg)

                
    def testdrive(self):
        msg = Cmd()
        # initial values
        # bool e_stop
        # uint8 gear
        # uint8 speed
        # int32 steer
        # uint8 brake

        msg.e_stop = 0
        msg.gear = 0
        msg.brake = 100
        msg.speed = 0
        msg.steer = 0

        if self.e_stop == 0: # if emergency stop is not working
            if self.gear == 0: # if gear is on front
                if self.encoder <= 100: # while car drives about 1.2m
                    msg.brake = 0
                    msg.speed = 6
                    msg.steer = 0
                    # convert deg or rad to integer btw -2000 ~ 2000

        self.msg_pub.publish(msg)

        

if __name__ == "__main__":
    rospy.init_node("testdrive", anonymous=False)
    node = ERPtestdrive()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()