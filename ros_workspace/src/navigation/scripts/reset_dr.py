import rospy
from navigation.msg import reset
import time

should_exit = False

class ResetNode:
    def __init__(self):
        self.should_exit = False
    def reset_handler(self, data):
        self.should_exit = True
    def run(self):
        self.should_exit = False
        rospy.init_node('reset_dr', anonymous=True)
        rospy.Subscriber('reset_bno', reset, self.reset_handler)
        reset_publisher = rospy.Publisher('reset_bno', reset, queue_size=10)
        rst_msg = reset()
        while not rospy.is_shutdown():
            reset_publisher.publish(rst_msg)
            if self.should_exit:
                print("Sent reset")
                break
            time.sleep(.5)
        print("exiting")
    

if __name__ == '__main__':
    try:
        node = ResetNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
