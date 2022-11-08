import rospy
import actionlib

from std_msgs.msg import String
from pxrf.msg import TakeMeasurementAction
from std_srvs.srv import SetBool


class TakeMeasurementServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('take_measurement', TakeMeasurementAction, self.execute, False)
        self.cmd_pub = rospy.Publisher('pxrf_cmd', String, queue_size=1)
        self.deploy_pxrf = rospy.ServiceProxy('deploy_sensor', SetBool)
        self.server.start()

    def execute(self, goal):
        print('sampling...')
        sample_time = float(rospy.get_param('~sample_time', 2.0))
        self.deploy_pxrf(True)
        self.cmd_pub.publish(String('start'))
        rospy.sleep(sample_time)
        self.cmd_pub.publish(String('stop'))
        self.deploy_pxrf(False)
        print('Sample Complete!')
        self.server.set_succeeded()


if __name__ == "__main__":
    rospy.init_node('pxrf_manager')
    server = TakeMeasurementServer()
    rospy.spin()

    