import rospy
from AbstractServiceCondition import AbstractServiceCondition
from strands_navigation_msgs.srv import EstimateTravelTime
from topological_navigation.msg import GotoNodeActionGoal
from std_msgs.msg import String

class estimatedTimeout(AbstractServiceCondition):

    _service_name = "/topological_navigation/travel_time_estimator"

    _service_type = EstimateTravelTime

    def __init__(self):

        rospy.Subscriber("/closest_node", String, self._current_node_callback)

        rospy.Subscriber("/topological_navigation/goal", GotoNodeActionGoal, self._current_goal_callback)
        self.current_goal = None

        super(estimatedTimeout, self).__init__()

    def _current_node_callback(self, data):
        self.current_node = data.data

    def _current_goal_callback(self, data):
        self.current_goal = data

    def evaluate(self, params):
        target_node = str(params[0])

        estimate_timeout = self.service_proxy(self.current_node, target_node).travel_time

        starting_time = self.current_goal.goal_id.stamp
        now = rospy.get_rostime()

        time_passed = now - starting_time

        print "Evaluating timeout", starting_time, now, time_passed

        if (time_passed.to_nsec() - estimate_timeout.to_nsec()) > 0:
            return True
        return False
