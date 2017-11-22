import rospy

from AbstractAction import AbstractAction
from ActionManager import ActionManager
from pnp_msgs.srv import PNPStartStateActionSaver, PNPStopStateActionSaver, PNPConditionValue

class recordDemonstration(AbstractAction):

    _demonstration_filename = None

    def _start_action(self):
        # Get the action for which we want a demonstration taking the last interrupted goal
        service_proxy = rospy.ServiceProxy("/PNPConditionValue", PNPConditionValue)

        condition = "InterruptedGoal"

        # get the last interrupted goal
        interrupted_goal = service_proxy(condition).value.split("_")

        goal_action = interrupted_goal[0]
        goal_params = interrupted_goal[1:]

        goal_str = goal_action + "_" + "_".join(goal_params)
        print "Recording ", goal_str, "..."

        # call the starting service
        starting_sp = rospy.ServiceProxy("start_state_action_saver", PNPStartStateActionSaver)

        self._demonstration_filename = "demonstration_" + goal_str + "_" + str(rospy.Time.now().to_nsec()) + ".bag"
        response = starting_sp(self._demonstration_filename)

        if response:
            rospy.loginfo("Started recording demonstration in " + self._demonstration_filename)
        else:
            rospy.logwarn("Error starting recording demonstration")

    def _stop_action(self):
        # call the stopping service
        if self._demonstration_filename:
            stopping_sp = rospy.ServiceProxy("stop_state_action_saver", PNPStopStateActionSaver)

            response = stopping_sp(self._demonstration_filename)

            if response:
                rospy.loginfo("Stopped recording demonstration in " + self._demonstration_filename)
            else:
                rospy.logwarn("Error stopping recording demonstration")

    @classmethod
    def is_goal_reached(cls, _):
        # Get the action for which we want a demonstration taking the last interrupted goal
        service_proxy = rospy.ServiceProxy("/PNPConditionValue", PNPConditionValue)

        condition = "InterruptedGoal"

        # get the last interrupted goal
        interrupted_goal = service_proxy(condition).value.split("_")

        goal_action = interrupted_goal[0]
        goal_params = interrupted_goal[1:]

        # Call the ActionManager static method
        if goal_action:
            return ActionManager.is_goal_reached(goal_action, goal_params)
        return False
