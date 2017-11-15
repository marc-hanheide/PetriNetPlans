import rospy
from AbstractTopicCondition import AbstractTopicCondition
from topological_navigation.msg import GotoNodeActionGoal

class currentGoal(AbstractTopicCondition):

    _topic_name = "/topological_navigation/goal"

    _topic_type = GotoNodeActionGoal

    def evaluate(self, params):
        node = str(params[0])

        if self.last_data:
            current_goal = self.last_data.goal.target

            return node == current_goal
        else:
            rospy.logwarn("currentGoal condition value not yet initialized")
            return True
