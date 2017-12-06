from AbstractTopicCondition import AbstractTopicCondition
from nav_msgs.msg import Odometry

class Odom(AbstractTopicCondition):

    _topic_name = "/odom"

    _topic_type = Odometry

    def _get_value_from_data(self, data):

        return str(data.pose)

    def evaluate(self, params):

        return str(params) == self.last_value
