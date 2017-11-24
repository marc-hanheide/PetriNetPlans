import os
import rospy
from AbstractTopicCondition import ConditionListener
from pnp_msgs.srv import PNPStartStateActionSaver, PNPStartStateActionSaverResponse,\
                         PNPStopStateActionSaver, PNPStopStateActionSaverResponse
# TODO have a service message for allof them

class StateActionPairGenerator(ConditionListener):

    def __init__(self, action_manager, condition_manager):
        self._action_manager = action_manager
        self._condition_manager = condition_manager

        self._actions_history = {}
        self._conditions_history = {}
        self._saving_bags = {}

        # Initialize the state action saver services
        self._start_sa_service_provider = rospy.Service("start_state_action_saver",
            PNPStartStateActionSaver, self._start_state_action_saver_cb)
        self._stop_sa_service_provider = rospy.Service("stop_state_action_saver",
            PNPStopStateActionSaver, self._stop_state_action_saver_cb)
        self._check_running_service_provider = rospy.Service("running_state_action_saver",
            PNPStartStateActionSaver, self._check_running_cb)

        self._condition_manager.register_condition_listener(self)

    def _check_running_cb(self, req):
        goal = req.goal

        if goal in self._saving_bags.keys():
            return PNPStartStateActionSaverResponse(1)
        else:
            return PNPStartStateActionSaverResponse(0)

    def _start_state_action_saver_cb(self, req):
        goal = req.goal

        if not goal in self._saving_bags.keys():
            bag_name = goal + "_" + str(rospy.Time.now().to_nsec()) + ".txt"
            # create new bag
            filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)),\
                    "../demonstrations", bag_name)
            bag = open(filepath, "w")

            # insert bag in dict
            self._saving_bags.update({
                goal : bag
            })
            self._actions_history.update({
                goal : []
            })
            self._conditions_history.update({
                goal : []
            })

            rospy.loginfo("Started recording demonstration in " + filepath)

            return PNPStartStateActionSaverResponse(1)
        else:
            rospy.logwarn("Saver for " + goal + " already running")
            return PNPStartStateActionSaverResponse(0)

    def _stop_state_action_saver_cb(self, req):
        goal = req.goal

        if goal in self._saving_bags.keys():

            # Save state actions in all open bags
            bag = self._saving_bags[goal]
            for timestep in range(len(self._conditions_history[goal])):
                for state_el in self._conditions_history[goal][timestep]:
                    bag.write(state_el + " ")
                bag.write("\t")
                for ex_action in self._actions_history[goal][timestep]:
                    bag.write(ex_action + " ")
                bag.write("\n")

            # close the bag
            self._saving_bags[goal].close()

            # remove the bag from the dict
            del self._saving_bags[goal]
            del self._conditions_history[goal]
            del self._actions_history[goal]

            rospy.loginfo("Closed bag " + goal)
            return PNPStopStateActionSaverResponse(1)
        else:
            rospy.logwarn("No running saver in bag " + goal + " found")
            return PNPStopStateActionSaverResponse(0)

    def receive_update(self, condition_instance):
        condition_name = condition_instance.get_name()
        condition_value = condition_instance.get_value()

        # Get the current conditions
        current_state = self._condition_manager.get_conditions_dump()

        # Get executable actions
        action_list = self._action_manager.get_actions()

        # Check the actions that reached the goal after this change
        action_candidates = []
        if condition_value.lower() != "none":
            for action in action_list:
                if self._action_manager.is_goal_reached(action, condition_value.split("_")):
                    action_candidates.append(action + "_".join(["", condition_value]))

        for bag_name in self._saving_bags.keys():
            # Save the condition
            self._conditions_history[bag_name].append(current_state)
            # spread the action to the previous timesteps without actions
            executed_actions = action_candidates[:]
            for timestep in range(len(self._actions_history[bag_name])-1, -1, -1):
                if len(self._actions_history[bag_name][timestep]) == 0:
                    self._actions_history[bag_name][timestep] = executed_actions
                else:
                    break

            # Add empty container of actions for the current step
            self._actions_history[bag_name].append([])

            assert(len(self._conditions_history[bag_name])==len(self._actions_history[bag_name]))
