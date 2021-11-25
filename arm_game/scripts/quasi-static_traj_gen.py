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

        # segment stuff
        self.segments = []
        self.segment_index = 0
        self.t0 = 0.0

        # starting guess and last values
        self.last_q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape([7,1])
        self.lam = 1.0

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
            # self.segments.append(\
            #     CritDampParam('''Tyler stuff goes here'''))
            self.segments.append(Hold5Param(1.0, pd, Rxd)); # for quasi-static

    '''
    Updates the arm position.
    '''
    def update_arm(self, t, dt):
        if (self.catching_asteroid):
            dur = self.segments[self.segment_index].duration()
            if (t - self.t0 >= dur):
                self.t0 = (self.t0 + dur)
                self.segment_index += 1

                if (self.segment_index >= len(self.segments)):
                    self.catching_asteroid = False

            (s, sdot) = self.segments[self.segment_index].evaluate(t - self.t0)

            (T, Jp) = self.kin.fkin(self.last_q)
            p = p_from_T(T)
            R = R_from_T(T)
            Jp_inv = np.linalg.pinv(Jp)

            # error terms
            ep = self.ep(self.pd(s), p)
            eR = self.eR(self.Rd(s), R) # gives a 3 x 1 vector

            # Compute velocity
            prdot = self.vd(s,sdot) + self.lam * ep
            wrdot = self.wd(s,sdot) + self.lam * eR
            qdot = Jp_inv @ np.vstack((prdot, wrdot))

            # discretely integrate
            q = (self.last_q + dt * qdot)

            # save info
            self.last_q = q
            self.last_p = p #self.pd(s)
            self.last_R = R #self.Rd(s)
            self.last_v = self.vd(s, sdot)
            self.last_w = self.wd(s, sdot)

            # Create and send the command message.  Note the names have to
            # match the joint names in the URDF.  And their number must be
            # the number of position/velocity elements.
            # cmdmsg = JointState()
            # cmdmsg.name         = ['joint_a1', 'joint_a2', 'joint_a3','joint_a4','joint_a5','joint_a6','joint_a7']
            # cmdmsg.position     = q
            # cmdmsg.velocity     = qdot
            # cmdmsg.header.stamp = rospy.Time.now()
            # self.pub.publish(cmdmsg)

    # Path. s from 0 to 1 is motion, s at 1 is holding.
    def pd(self, s):
        p0 = self.segments[self.segment_index].get_p0()
        pf = self.segments[self.segment_index].get_pf()
        return (1-s) * p0 + (s) * pf

    def vd(self, s, sdot):
        p0 = self.segments[self.segment_index].get_p0()
        pf = self.segments[self.segment_index].get_pf()
        return (pf - p0)*sdot

    def Rd(self, s): # TODO factor in desired orientation
        return Rx(np.pi/2) @ Ry(np.pi)

    def wd(self, s, sdot): # TODO factor in desired orientation
        return np.array([0.0, 0.0, 0.0]).reshape((3,1))

    # Error functions
    def ep(self, pd, pa):
        return (pd - pa)
    def eR(self, Rd, Ra):
        return 0.5*(np.cross(Ra[:,0:1], Rd[:,0:1], axis=0))

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
