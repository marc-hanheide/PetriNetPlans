import os
import rospy
import pickle
import numpy as np
import GPy
from AbstractAction import AbstractAction
from std_msgs.msg import Float64MultiArray, String

class saveTrajectory(AbstractAction):

    def _start_action(self):
        scan_window = rospy.wait_for_message("scan_history", Float64MultiArray, timeout=2)

        # Save the new trajectory
        folder = '%s/catkin_ws/data/detect_trajectories'  % os.path.expanduser("~")
        if scan_window:
            if len(self.params) > 0 and self.params[0] == "neg":
                filename = '%s/neg_%s.traj' % (folder, rospy.Time.now().to_nsec())
            else:
                filename = '%s/%s.traj' % (folder, rospy.Time.now().to_nsec())

            pickle.dump(scan_window.data, open(filename, "wb"))
            rospy.loginfo("Saving failed trajectory in %s" % filename)
        else:
            rospy.logwarn("Scan window not received")

        ## Train the model with the new trajectory
        X = []
        negX = []
        for filename in os.listdir(folder):
            filepath = folder + "/" + filename
            if filename.startswith("neg"):
                negX.append(pickle.load(open(filepath, "rb")))
            else:
                X.append(pickle.load(open(filepath, "rb")))
        Y = [1] * len(X) + [0] * len(negX); Y = np.array(Y); Y.shape = (len(Y), 1)
        X += negX; X = np.array(X)
        print "INPUT DATA SIZE: ", X.shape
        print "OUTPUT DATA SIZE: ", Y.shape

        # define model
        model = GPy.models.GPClassification(X, Y)

        # optimize model
        model.optimize()
        print model

        ## save model
        folder = '%s/catkin_ws/data/GPmodels'  % os.path.expanduser("~")
        np.save('%s/detector_model_params.npy' % folder, model.param_array)
        np.save('%s/detector_model_X.npy' % folder, X)
        np.save('%s/detector_model_Y.npy' % folder, Y)

        # Signal the model update event
        sign_pub = rospy.Publisher("failure_model_updated", String, queue_size=10)
        sign_pub.publish(String(""))

        # Signal that the action is done
        self.params.append("done")

    def _stop_action(self):
        pass

    @classmethod
    def is_goal_reached(cls, params):
        if len(params) > 0 and params[-1] == "done":
            return True

        return False
