import rospy
import actionlib
import wholebody_keyboard_ctrl.msg as wbmsg


def wholebody_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('leg_movement_action', wbmsg.LegMovementAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = wbmsg.LegMovementGoal()

    goal.goal_pos_fl_x = 0.45
    goal.goal_pos_fl_y = 0.15
    goal.goal_pos_fl_z = -0.5
    goal.wheel_rotation_enabled_fl = False

    goal.goal_pos_fr_x = 0.4
    goal.goal_pos_fr_y = -0.4
    goal.goal_pos_fr_z = -0.5
    goal.wheel_rotation_enabled_fr = False

    goal.goal_pos_bl_x = -0.4
    goal.goal_pos_bl_y =  0.4
    goal.goal_pos_bl_z = -0.5
    goal.wheel_rotation_enabled_bl = False

    goal.goal_pos_br_x = -0.1
    goal.goal_pos_br_y = -0.4
    goal.goal_pos_br_z = -0.5
    goal.wheel_rotation_enabled_br = False

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':

       try:
           # Initializes a rospy node so that the SimpleActionClient can
           # publish and subscribe over ROS.
           rospy.init_node('fibonacci_client_py')
           result = wholebody_client()

       except rospy.ROSInterruptException:
           print("program interrupted before completion")
