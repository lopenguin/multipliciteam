#!/usr/bin/env python3
#
#   arm_trajectory_generator.py
#
#   Multipliciteam Final Project - Arm Trajectory Generator/Controller
#
#   Visualize the 7DOF, handle motion from one point to another while avoiding
#   singularities and following the secondary centering task
#
#   Publish:   /joint_states   sensor_msgs/JointState
#
import rospy
import numpy as np

from sensor_msgs.msg     import JointState
from urdf_parser_py.urdf import Robot
from desired_T           import Catch, Ball
from asteroid_generator  import AsteroidHandler

# Import the kinematics stuff:
from kinematics import Kinematics, p_from_T, R_from_T, Rx, Ry, Rz
# We could also import the whole thing ("import kinematics"),
# but then we'd have to write "kinematics.p_from_T()" ...

# Import the Spline stuff:
from splines import  CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5


#
#  Generator Class
#
class Generator:
    # Initialize.
    def __init__(self):


    '''
    Called every 5 ms! Forces update of arm position commands and asteroid info.
    '''
    def update(self, t, dt):
        self.update_arm(t, dt)
        self.update_asteroid(t, dt)


    '''
    Generates new asteroids and updates asteroid predicted trajectory.
    '''
    def update_asteroid(self, t, dt):


    '''
    Updates the arm position.
    '''
    def update_arm(self, t, dt):


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare/initialize this node.
    rospy.init_node('redundant')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    generator = Generator()

    # Prepare a servo loop at 100Hz.
    rate  = 200
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))


    # Run the servo loop until shutdown (killed or ctrl-C'ed).
    t = 0
    while not rospy.is_shutdown():

        # Update the controller.
        generator.update(t, dt)

        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.
        servo.sleep()

        # Update the time.
        t += dt
