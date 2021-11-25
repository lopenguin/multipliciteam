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
from asteroid_generator  import AsteroidHandler, Asteroid

# Import the kinematics stuff:
from kinematics import Kinematics, p_from_T, R_from_T, Rx, Ry, Rz, axisangle_from_R, R_from_axisangle
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
        # Create a publisher to send the joint commands.  Add some time
        # for the subscriber to connect.  This isn't necessary, but means
        # we don't start sending messages until someone is listening.
        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        rospy.sleep(0.25)

        # Grab the robot's URDF from the parameter server.
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = Kinematics(robot, 'world', 'tool0')

        # asteroid handling
        self.asteroid_handler = AsteroidHandler()
        self.arm_length = 1.0 # replace with fkin later

        # other variables
        self.catching_asteroid = False
        self.segments = []
        self.segment_index = 0
        self.last_pos = np.array([1.0, 1.0, 1.0]).reshape([3,1]) # updated every time the arm moves!
        self.last_vel = np.array([0.0, 0.0, 0.0]).reshape([3,1])
        self.last_xidr = np.array([1.0, 0.0, 0.0]).reshape([3,1])
        self.last_wx = 0.0

    '''
    Called every 5 ms! Forces update of arm position commands and asteroid info.
    '''
    def update(self, t, dt):
        self.update_arm(t, dt)
        self.update_asteroid(t, dt)


    '''
    Handles the creation of new asteroids and path to intercept asteroid.
    '''
    def update_asteroid(self, t, dt):
        # only change path when not catching asteroid
        if (not self.catching_asteroid):
            # clear out segments and segment index
            self.segments = []
            self.segment_index = 0

            # generate an asteroid to catch. Future implementation may just
            # select an already created asteroid here.
            self.asteroid = Asteroid(self.asteroid_handler, self.arm_length, t)
            # self.asteroid = Asteroid(self.asteroid_handler, self.arm_length, t, self.get_next_asteroid())

            # Spline to the position and speed of the first reachable intersection point
            # TODO: select the first "reachable" asteroid using a spline with
            # maximum q_dot_dot and q_dot implemented.
            t_target = asteroid.get_intercept_times()[0]
            # current positions
            pc = self.last_pos
            vc = self.last_vel
            Rxc = self.last_xdir # vector of x_tip direction
            wxc = self.last_wx
            # targets
            pd = asteroid.get_position(t_target)
            vd = asteroid.get_velocity(t_target)
            Rxd = -asteroid.get_direction()
            wxd = 0.0

            self.segments.append(\
                QSplineParam(t_target - t, pc, vc, Rxc, wxc, pd, vd, Rxd, wxd))

            # velocity match according to a critically damped spring!
            self.segments.append(\
                CritDampParam('''Tyler stuff goes here'''))

    '''
    Updates the arm position.
    '''
    def update_arm(self, t, dt):
        if (self.catching_asteroid):



    # Path. s from 0 to 1 is motion, s at 1 is holding.
    def pd(self, s):
        p0 = self.segments[self.segment_index].get_p0()
        pf = self.segments[self.segment_index].get_pf()
        return (1-s) * p0 + (s) * pf

    def vd(self, s, sdot):
        p0 = self.segments[self.segment_index].get_p0()
        pf = self.segments[self.segment_index].get_pf()
        return (pf - p0)*sdot

    def Rd(self, s):
        R0 = self.segments[self.segment_index].get_R0()
        Rf = self.segments[self.segment_index].get_Rf()

        # Retrieve rotation matrix that we are applying to move from initial
        # to final orientation
        R = np.transpose(R0) @ Rf # TODO check if this ordering is correct
        
        # retrieve the 3x1 axis and scalar angle to rotate about that axis
        (axis, angle) = axisangle_from_R(R)
        ex = axis[0,0]
        ey = axis[1,0]
        ez = axis[2,0]

        # change desired angle based on path variable to move smoothly through
        theta = angle * s

        # Rotating about arbitrary axis
        return R_from_axisangle(axis, theta)

    def wd(self, s, sdot): # TODO factor in desired orientation
        R0 = self.segments[self.segment_index].get_R0()
        Rf = self.segments[self.segment_index].get_Rf()

        # Retrieve rotation matrix that we are applying to move from initial
        # to final orientation
        R = np.transpose(R0) @ Rf # TODO check if this ordering is correct, remove code repetition
        
        # retrieve the 3x1 axis and scalar angle to rotate about that axis
        (axis, angle) = axisangle_from_R(R)
        
        return axis * sdot

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
