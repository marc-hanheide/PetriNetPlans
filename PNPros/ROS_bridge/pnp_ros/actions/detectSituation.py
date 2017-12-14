import os
import rospy
import pickle
from AbstractAction import AbstractAction
from std_msgs.msg import Float64MultiArray

class detectSituation(AbstractAction):

    def _start_action(self):
        # Read the last scan window
        scan_window = rospy.wait_for_message("/scan_history", Float64MultiArray, timeout=2)

        # NOTE now I'm just collecting all the demonstrations in the same model
        #      I'm hence considering just one failure situation

        # Check if a model recognizes the situation
        model = None
        # TODO

        if model is not None:
            # Take the corresponding recovery model
            pass
        else:
            # Collect the samples
            dataPath = '%s/catkin_ws/data/detect_trajectories/' % os.path.expanduser("~")
            files = os.listdir(dataPath)
            det_trajectories = []
            for filename in files:
                traj = pickle.load(filename)
                det_trajectories.append(traj)
            det_trajectories.append(scan_window.data)

            # save the current trajectory
            filename = os.path.join(dataPath, str(rospy.Time.now().to_nsec()) + ".traj")
            pickle.dump(filename, scan_window.data)






    def _stop_action(self):
        pass

    @classmethod
    def is_goal_reached(cls, params):
        return True
