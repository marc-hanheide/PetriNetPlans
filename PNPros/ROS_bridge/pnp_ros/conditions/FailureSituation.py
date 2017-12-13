
from AbstractTopicCondition import AbstractTopicCondition
from pnp_msgs.msg import ActionFailure

class FailureSituation(AbstractTopicCondition):

    _topic_name = "/failure_signal"

    _topic_type = ActionFailure

    def _get_value_from_data(self, data):
        if data.truth_value:
            return "true"
        else:
            return "false"

    def evaluate(self, params):
        if self.last_value is None or self.last_value == "false":
            return False
        return True
