#!/usr/bin/python3


import rospy
from std_msgs.msg import String, Int32
from pynput.keyboard import Key, Listener


class TeleopPublisher:
    def __init__(self):
        rospy.init_node('keyboard_teleop_controller')
        self.speed = 100.0
        self.pub = rospy.Publisher('teleop_speed', int32, queue_size=10)

        with Listener(on_press=self.key_callback) as listener:
            rospy.spin()

    def key_callback(self, key):

        try:
            key_str = key.char
        except:
            key_str = str(key)
        

        if key_str == 'Key.up':
            if(self.speed < 100):
                self.speed += 1
            else:
                pass
        if key_str =='Key.down':
            if(self.speed > 0):
                self.speed -= 1
            else:
                pass

        #rospy.loginfo(f"Key pressed: {key_str}")
        print("SPEED: " + str(self.speed))
        self.pub.publish(str(self.speed))

if __name__ == '__main__':
    try:
        node = TeleopPublisher()
    except rospy.ROSInterruptException:
        pass