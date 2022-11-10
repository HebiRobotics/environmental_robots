import rospy
import actionlib

from std_msgs.msg import String
from pxrf.msg import TakeMeasurementAction
from std_srvs.srv import SetBool


class TakeMeasurementServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('take_measurement', TakeMeasurementAction, self.execute, False)
        self.cmd_pub = rospy.Publisher('pxrf_cmd', String, queue_size=1)
        self.cmd_sub = rospy.Subscriber('pxrf_result', String, self.pxrf_cb)
        self.deploy_pxrf = rospy.ServiceProxy('deploy_sensor', SetBool)
        self.pxrf_response = 'NoResponse'
        self.should_stop = False
        self.server.start()

    def pxrf_cb(self, msg):
        self.pxrf_response = msg.data

    def stop_sampling(self, evt):
        self.should_stop = True

    def execute(self, goal):
        self.pxrf_response = 'NoResponse'
        self.should_stop = False

        print('sampling...')
        sample_time = float(rospy.get_param('~sample_time', 2.0))

        self.deploy_pxrf(True)

        self.cmd_pub.publish(String('start'))

        rospy.Timer(sample_time, self.stop_sampling, oneshot=True)
        while self.pxrf_response != '201' and not self.should_stop:
            rospy.sleep(0.1)

        if self.pxrf_response != '201':
            self.cmd_pub.publish(String('stop'))

        self.deploy_pxrf(False)

        print('Sample Complete!')
        self.server.set_succeeded(self.pxrf_response)


if __name__ == "__main__":
    rospy.init_node('pxrf_manager')
    server = TakeMeasurementServer()
    rospy.spin()

    