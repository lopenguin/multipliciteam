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
from std_msgs.msg        import Float64
from urdf_parser_py.urdf import Robot

# import stuff to make asteroids!
from asteroid_generator  import AsteroidHandler, Asteroid

# Import the kinematics stuff:
from kinematics import Kinematics, p_from_T, R_from_T, Rx, Ry, Rz
# We could also import the whole thing ("import kinematics"),
# but then we'd have to write "kinematics.p_from_T()" ...

# Import the Spline stuff:
from splines import  QSplinePR, QHoldPR, QuinticSpline


#
#  Generator Class
#
class Generator:
    # Initialize.
    def __init__(self):
        # The Gazebo controllers treat each joint seperately.  We thus
        # need a seperate publisher for each joint under the topic
        # "/BOTNAME/CONTROLLER/command"...
        self.N    = 7
        self.pubs = []
        for i in range(self.N):
            topic = "/iiwa7/j" + str(i+1) + "_setposition/command"
            self.pubs.append(rospy.Publisher(topic, Float64, queue_size=10))

        # # We used to add a short delay to allow the connection to form
        # # before we start sending anything.  However, if we start
        # # Gazebo "paused", this already provides time for the system
        # # to set up, before the clock starts.
        # rospy.sleep(0.25)

        # Find the simulation's starting position.  This will block,
        # but that's appropriate if we don't want to start until we
        # have this information.  Of course, the simulation starts at
        # zero, so we can simply use that information too.
        print("Waiting for joint state message")
        msg = rospy.wait_for_message('/iiwa7/joint_states', JointState);
        print("Message received!")
        theta0 = np.array(msg.position).reshape((-1,1))
        rospy.loginfo("Gazebo's starting position: %s", str(theta0.T))

        # Grab the robot's URDF from the parameter server.
        robot = Robot.from_parameter_server()

        # Instantiate the Kinematics
        self.kin = Kinematics(robot, 'world', 'iiwa7_bucket') # TODO: bucket?

        # asteroid handling
        self.asteroid_handler = AsteroidHandler()
        (T_curr, J_curr) = self.kin.fkin(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape([7,1]))
        self.arm_length = np.linalg.norm(p_from_T(T_curr)) - 0.1
        print(self.arm_length)

        # other variables
        self.catching_asteroid = False
        self.asteroid = Asteroid(self.asteroid_handler, self.arm_length, 0.0)

        # segment stuff
        self.segments = []
        self.segment_index = 0
        self.t0 = 0.0

        # starting guess and last values
        self.last_q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape([7,1])
        self.lam = 1.0

        self.last_pos = np.array([1.0, 1.0, 1.0]).reshape([3,1]) # updated every time the arm moves!
        self.last_vel = np.array([0.0, 0.0, 0.0]).reshape([3,1])
        self.last_xdir = np.array([1.0, 0.0, 0.0]).reshape([3,1])
        self.last_wx = np.array([0.0, 0.0, 0.0]).reshape([3,1])

        # Also reset the trajectory, starting at the beginning.
        self.reset()

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
            print("Spawning asteroid!")
            self.asteroid.remove()
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
            t_target = self.asteroid.get_intercept_times(t)[0]
            t_target = float(t_target)
            # current positions
            pc = self.last_pos
            vc = self.last_vel
            Rxc = self.last_xdir # vector of x_tip direction
            wxc = self.last_wx
            # targets
            pd = self.asteroid.get_position(t_target)
            vd = self.asteroid.get_velocity(t_target)
            Rxd = -self.asteroid.get_direction()
            wxd = np.array([0.0, 0.0, 0.0]).reshape([3,1])

            self.segments.append(\
                QSplinePR(t_target - t, pc, vc, Rxc, pd, vd, Rxd))

            # velocity match according to a critically damped spring!
            # self.segments.append(\
            #     CritDampParam('''Tyler stuff goes here'''))
            self.segments.append(QHoldPR(1.0, pd, Rxd)); # for quasi-static

            self.catching_asteroid = True

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
                return

            (T, Jp) = self.kin.fkin(self.last_q)
            p = p_from_T(T)
            R = R_from_T(T)
            # weighted pseudoinverse
            gam = 0.1;
            J_winv = Jp.T @ np.linalg.inv(Jp @ Jp.T + gam*gam*np.eye(6));

            # desired terms
            (pd, vd) = self.segments[self.segment_index].evaluate_p(t - self.t0)
            (Rd, wd) = self.segments[self.segment_index].evaluate_R(t - self.t0)

            # error terms
            ep = self.ep(pd, p)
            eR = self.eR(Rd, R) # gives a 3 x 1 vector

            # Compute velocity
            prdot = vd + self.lam * ep
            wrdot = wd + self.lam * eR
            qdot = J_winv @ np.vstack((prdot, wrdot))

            # discretely integrate
            q = (self.last_q + dt * qdot)

            # save info
            self.last_q = q
            self.last_p = p #self.pd(s)
            self.last_R = R #self.Rd(s)
            self.last_v = vd
            self.last_w = wd

            # Create and send the command message.  Note the names have to
            # match the joint names in the URDF.  And their number must be
            # the number of position/velocity elements.
            # cmdmsg = JointState()
            # cmdmsg.name         = ['joint_a1', 'joint_a2', 'joint_a3','joint_a4','joint_a5','joint_a6','joint_a7']
            # cmdmsg.position     = q
            # cmdmsg.velocity     = qdot
            # cmdmsg.header.stamp = rospy.Time.now()
            # self.pub.publish(cmdmsg)
            for i in range(self.N):
                self.pubs[i].publish(Float64(q[i]))

    # Error functions
    def ep(self, pd, pa):
        return (pd - pa)
    def eR(self, Rd, Ra):
        return 0.5*(np.cross(Ra[:,1:2], Rd[:,1:2], axis=0))

    # Reset.  If the simulation resets, also restart the trajectory.
    def reset(self):
        self.asteroid.remove()
        # Just reset the start time to zero and create a new asteroid
        self.t0    = 0.0
        self.catching_asteroid = False


        self.last_q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape([7,1])
        self.last_pos = np.array([1.0, 1.0, 1.0]).reshape([3,1]) # updated every time the arm moves!
        self.last_vel = np.array([0.0, 0.0, 0.0]).reshape([3,1])
        self.last_xdir = np.array([1.0, 0.0, 0.0]).reshape([3,1])
        self.last_wx = np.array([0.0, 0.0, 0.0]).reshape([3,1])

#
#  Main Code
#
if __name__ == "__main__":
    print("Starting quasi-static demo...")
    # Prepare/initialize this node.
    rospy.init_node('trajectory')

    # Instantiate the trajectory generator object, encapsulating all
    # the computation and local variables.
    generator = Generator()

    # Prepare a servo loop at 200Hz.
    rate  = 100;
    servo = rospy.Rate(rate)
    dt    = servo.sleep_dur.to_sec()
    rospy.loginfo("Running the servo loop with dt of %f seconds (%fHz)" %
                  (dt, rate))


    # Run the servo loop until shutdown (killed or ctrl-C'ed).  This
    # relies on rospy.Time, which is set by the simulation.  Therefore
    # slower-than-realtime simulations propagate correctly.
    starttime = rospy.Time.now()
    lasttime  = starttime
    while not rospy.is_shutdown():

        # Current time (since start)
        servotime = rospy.Time.now()
        t  = (servotime - starttime).to_sec()
        dt = (servotime - lasttime).to_sec()
        lasttime = servotime

        # Update the controller.
        generator.update(t, dt)

        # Wait for the next turn.  The timing is determined by the
        # above definition of servo.  Note, if you reset the
        # simulation, the time jumps back to zero and triggers an
        # exception.  If desired, we can simple reset the time here to
        # and start all over again.
        try:
            servo.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            # Reset the time counters, as well as the trajectory
            # generator object.
            rospy.loginfo("Resetting...")
            generator.reset()
            starttime = rospy.Time.now()
            lasttime  = starttime
