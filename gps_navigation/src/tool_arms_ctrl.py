import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolRequest

import numpy as np
import hebi


if __name__ == '__main__':
    rospy.init_node('tool_arms_ctrl')

    lookup = hebi.Lookup()
    rospy.sleep(2)

    family = 'HEBI'
    names = ['J1_rake']
    tool_arm = lookup.get_group_from_names(family, names)
    while tool_arm is None:
        tool_arm = lookup.get_group_from_names(family, names)

    tool_arm_cmd = hebi.GroupCommand(tool_arm.size)

    tool_arm_fbk = tool_arm.get_next_feedback()

    tool_angle_down = 0.0
    tool_angle_up = 0.0


    family = 'HEBI'
    names = ['J1_shoulder', 'J2_elbow']
    sensor_arm = lookup.get_group_from_names(family, names)
    while sensor_arm is None:
        sensor_arm = lookup.get_group_from_names(family, names)

    sensor_arm_cmd = hebi.GroupCommand(sensor_arm.size)

    sensor_arm_fbk = sensor_arm.get_next_feedback()

    sensor_angles_down = [0.0, 0.0]
    sensor_angles_up = [0.0, 0.0]

    t = rospy.get_time()
    tool_arm_trajectory = hebi.trajectory.create_trajectory([t, t+3], [tool_arm_fbk.position[0], tool_angle_up])

    positions = np.empty((2,2), dtype=np.float64)
    positions[0, :] = sensor_arm_fbk.position
    positions[1, :] = sensor_angles_up
    sensor_arm_trajectory = hebi.trajectory.create_trajectory([t, t+3], positions)


    def tool_arm_cb(request: SetBoolRequest):
        global tool_arm_trajectory
        t = rospy.get_time()
        times = [t, t+1]
        curr_pos = tool_arm_fbk.position[0]

        if request.data:
            end_pos = tool_angle_up
        else:
            end_pos = tool_angle_down

        tool_arm_trajectory = hebi.trajectory.create_trajectory(times, [curr_pos, end_pos])
        return []

    
    def sensor_arm_cb(request):
        global sensor_arm_trajectory
        t = rospy.get_time()
        times = [t, t+2]
        positions = np.empty((2, 2), dtype=np.float64)

        if request.data:
            positions[0, :] = sensor_arm_fbk.position
            positions[1, :] = sensor_angles_down
        else:
            positions[0, :] = sensor_arm_fbk.position
            positions[1, :] = sensor_angles_up

        sensor_arm_trajectory = hebi.trajectory.create_trajectory(times, positions)
        return []

    rospy.Service('deploy_tool', SetBool, tool_arm_cb)
    rospy.Service('deploy_sensor', SetBool, sensor_arm_cb)

    while not rospy.is_shutdown():
        t = rospy.get_time()
        sensor_arm.get_next_feedback(reuse_fbk=sensor_arm_fbk)
        tool_arm.get_next_feedback(reuse_fbk=tool_arm_fbk)

        p, v, a = sensor_arm_trajectory.get_state(t)

        rospy.spin()