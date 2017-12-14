#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import GPy
import rospy
import random
import pickle
import numpy as np
from std_msgs.msg import Float64MultiArray, String
from pnp_msgs.msg import ActionFailure

current_scan_window = []
model = None

def receive_scan_history(data):
    global current_scan_window
    current_scan_window = data.data[:]

def load_model(data):
    global model

    folder = '%s/catkin_ws/data/GPmodels'  % os.path.expanduser("~")
    try:
        model_params = np.load('%s/detector_model_params.npy' % folder)
        X = np.load('%s/detector_model_X.npy' % folder)
        Y = np.load('%s/detector_model_Y.npy' % folder)
    except IOError:
        pass
    else:
        print "X", X.shape, "Y", Y.shape
        model = GPy.models.GPClassification(X, Y, initialize=False)
        model.update_model(False) # do not call the underlying expensive algebra on load
        model.initialize_parameter() # Initialize the parameters (connect the parameters up)
        model[:] = model_params # Load the parameters
        model.update_model(True) # Call the algebra only once
        print model
        #model.optimize()


if __name__ == "__main__":
    global current_scan_window
    global model

    load_model("")

    # init node
    rospy.init_node("failure_detector")

    # scan history Subscriber
    rospy.Subscriber("scan_history", Float64MultiArray, receive_scan_history)

    # failure model updated
    rospy.Subscriber("failure_model_updated", String, load_model)

    # failure Publisher
    pub = rospy.Publisher("failure_signal", ActionFailure, queue_size=10)

    rate = rospy.Rate(5) #hz

    while not rospy.is_shutdown():
        # Check failure with learned model
        if model is None:
            pass
            #rospy.loginfo('%s/detector_model.gp' % folder + " not found")
        elif len(current_scan_window) > 0:
            failure = False

            # Make prediction

            Xtest = np.array(current_scan_window); Xtest.shape = (1, len(current_scan_window))
            print "Xtest", Xtest.shape
            (Yp, var) = model.predict(Xtest)
            print Yp
            if Yp > 0.75:
                failure = True
                rospy.loginfo("FAILURE AUTO-DETECTED, acc: " + str(Yp) + " var: " + str(var))
        #    elif Yp < 0.5 and random.random() < 0.02:
        #        # Save a negative trajectory with 10% probability
        #        folder = '%s/catkin_ws/data/detect_trajectories'  % os.path.expanduser("~")
        #        filename = '%s/neg_%s.traj' % (folder, rospy.Time.now().to_nsec())

        #        pickle.dump(current_scan_window, open(filename, "wb"))
        #        rospy.loginfo("!!! Saving negative failed trajectory in %s" % filename)

            if failure:
                msg = ActionFailure()
                msg.stamp = rospy.Time.now()
                msg.cause = "autodetected"
                pub.publish(msg)

        rate.sleep()


    rospy.spin()
