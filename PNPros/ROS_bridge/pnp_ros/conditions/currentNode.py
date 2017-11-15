from AbstractTopicCondition import AbstractTopicCondition
from std_msgs.msg import String

class currentNode(AbstractTopicCondition):

    _topic_name = "/current_node"

    _topic_type = String

    def evaluate(self, params):
        node = str(params[0])

        return node == self.last_data.data
