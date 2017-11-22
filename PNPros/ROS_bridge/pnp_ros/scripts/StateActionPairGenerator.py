import os
import rospy
from pnp_msgs.srv import PNPStartStateActionSaver, PNPStartStateActionSaverResponse,\
                         PNPStopStateActionSaver, PNPStopStateActionSaverResponse
from AbstractTopicCondition import ConditionListener

class StateActionPairGenerator(ConditionListener):

    def __init__(self, action_manager, condition_manager):
        self._action_manager = action_manager
        self._condition_manager = condition_manager

        self._executed_actions_history = []
        self._conditions_history = []
        self._saving_bags = {}

        # Initialize the state action saver services
        self._start_sa_service_provider = rospy.Service(
            "start_state_action_saver",
            PNPStartStateActionSaver,
            self._start_state_action_saver_cb
        )
        self._stop_sa_service_provider = rospy.Service(
            "stop_state_action_saver",
            PNPStopStateActionSaver,
            self._stop_state_action_saver_cb
        )

        self._condition_manager.register_condition_listener(self)

    def _start_state_action_saver_cb(self, req):
        bag_name = req.bag_name

        if not bag_name in self._saving_bags.keys():
            # create new bag
            filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)),\
                    "../demonstrations", bag_name)
            bag = open(filepath, "w")

            # insert bag in dict
            self._saving_bags.update({
                bag_name : bag
            })

            return PNPStartStateActionSaverResponse(True)
        else:
            rospy.logwarn("Saver in bag " + bag_name + " already running")
            return PNPStartStateActionSaverResponse(False)

    def _stop_state_action_saver_cb(self, req):
        bag_name = req.bag_name

        if bag_name in self._saving_bags.keys():
            # close the bag
            self._saving_bags[bag_name].close()

            # remove the bag from the dict
            del self._saving_bags[bag_name]

            rospy.loginfo("Closed bag " + bag_name)
            return PNPStopStateActionSaverResponse(True)
        else:
            rospy.logwarn("No running saver in bag " + bag_name + " found")
            return PNPStopStateActionSaverResponse(False)

    def receive_update(self, condition_instance):
        condition_name = condition_instance.get_name()
        condition_value = condition_instance.get_value()

        # Save the current conditions
        current_state = self._condition_manager.get_conditions_dump()
        self._conditions_history.append(current_state)

        # Get executable actions
        action_list = self._action_manager.get_actions()

        # Check the actions that reached the goal after this change
        action_candidates = []
        for action in action_list:
            if self._action_manager.is_goal_reached(action, condition_value.split("_")):
                action_candidates.append(action + "_" + condition_value)

        # Check that the action goal was not already reached at the prev step
        executed_actions = action_candidates[:]
        if len(self._executed_actions_history) > 0:
            for i, action in enumerate(action_candidates):
                if action in self._executed_actions_history[-1]:
                    del executed_actions[i]

        # Save the actions executed in current step
        if len(executed_actions):
            self._executed_actions_history.append(executed_actions)
        else:
            self._executed_actions_history.append(["NO_ACTION"])

        assert(len(self._conditions_history)==len(self._executed_actions_history))

        # Save state actions in all open bags
        for bag in self._saving_bags.values():
            for state_el in current_state:
                bag.write(state_el + " ")
            bag.write("\t")
            for ex_action in executed_actions:
                bag.write(ex_action + " ")
            bag.write("\n")
