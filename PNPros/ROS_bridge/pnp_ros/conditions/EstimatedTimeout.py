import rospy
from AbstractServiceCondition import AbstractServiceCondition
from strands_navigation_msgs.srv import EstimateTravelTime
from topological_navigation.msg import GotoNodeActionGoal
from std_msgs.msg import String

class EstimatedTimeout(AbstractServiceCondition):

    _service_name = "/topological_navigation/travel_time_estimator"

    _service_type = EstimateTravelTime

    _ADDITIVE_CONSTANT_TIMEOUT = rospy.Duration.from_sec(20)

    def __init__(self):

        rospy.Subscriber("/current_node", String, self._current_node_callback)

        rospy.Subscriber("/topological_navigation/goal", GotoNodeActionGoal, self._current_goal_callback)

        self.current_goal = None
        self.current_node = None

        super(EstimatedTimeout, self).__init__()

    def _current_node_callback(self, data):
        self.current_node = data.data

    def _current_goal_callback(self, data):
        self.current_goal = data

    def evaluate(self, params):
        target_node = str(params[0])

        # Return False if we are not in a topological node bcs in this case we don't have any estimate
        if self.current_node and self.current_node != "none":
            estimate_timeout = self.service_proxy(self.current_node, target_node).travel_time

            starting_time = self.current_goal.goal_id.stamp
            now = rospy.get_rostime()

            time_passed = now - starting_time

            # check timeout expiration
            if time_passed.to_nsec() > estimate_timeout.to_nsec() + self._ADDITIVE_CONSTANT_TIMEOUT.to_nsec():
                # update the listeners now
                for listener in self._updates_listeners:
                    listener.receive_update(self.__name__, target_node)

                return True

        return False
