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
import math

from sensor_msgs.msg     import JointState
from std_msgs.msg        import Float64
from urdf_parser_py.urdf import Robot

# import stuff to make asteroids!
from asteroid_generator  import AsteroidHandler, Asteroid

# Import the kinematics stuff:
from kinematics import Kinematics, p_from_T, R_from_T, Rx, Ry, Rz, R_from_axisangle
# We could also import the whole thing ("import kinematics"),
# but then we'd have to write "kinematics.p_from_T()" ...

# Import the Spline stuff:
from splines import  QSplinePR, QHoldPR, QuinticSpline, CritDampPR, QSplinePOnly


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
        self.arm_length = np.linalg.norm(p_from_T(T_curr))

        # other variables
        self.asteroid = Asteroid(self.asteroid_handler, self.arm_length, 0.0)

        # segment stuff
        self.segments = []
        self.segment_index = 0
        self.t0 = 0.0

        # starting guess and last values
        self.last_q = np.array([0.0, np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape([7,1])
        self.lam = 7.0

        self.last_pos = np.array([1.0, 1.0, 1.0]).reshape([3,1]) # updated every time the arm moves!
        self.last_vel = np.array([0.0, 0.0, 0.0]).reshape([3,1])
        self.last_R = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        self.last_wx = np.array([0.0, 0.0, 0.0]).reshape([3,1])
        self.asteroid_direction = self.asteroid.get_direction()

        # Also reset the trajectory, starting at the beginning.
        # self.reset()

    '''
    Called every 5 ms! Forces update of arm position commands and asteroid info.
    '''
    def update(self, t, dt):
        self.update_arm(t, dt)
        # self.update_asteroid(t, dt)


    '''
    Handles the creation of new asteroids and path to intercept asteroid.
    '''
    def update_asteroid(self, t, dt):
        # only change path when not catching asteroid
        print("Spawning asteroid!")
        self.asteroid.remove()
        # clear out segments and segment index
        self.segments = []
        self.segment_index = 0

        # generate an asteroid to catch. Future implementation may just
        # select an already created asteroid here.
        self.asteroid = Asteroid(self.asteroid_handler, self.arm_length, t)

        # Spline to the position and speed of the first reachable intersection point
        # TODO: select the first "reachable" asteroid using a spline with
        # maximum q_dot_dot and q_dot implemented.
        intercept_times = self.asteroid.get_intercept_times(t)
        t_target = intercept_times[int((np.random.random()/2 + 0.1)*len(intercept_times))] # Todo: update
        # t_target = intercept_times[int(0.5*len(intercept_times))] # Todo: update
        t_target = float(t_target)
        # current positions
        pc = self.last_pos
        vc = self.last_vel
        Rxc = self.last_R
        # targets
        pd = self.asteroid.get_position(t_target)
        vd = np.array([0.0,0.0,0.0]).reshape([3,1]) #self.asteroid.get_velocity(t_target)
        self.asteroid_direction = self.asteroid.get_direction()
        Rxd = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

        self.segments.append(\
            QSplinePOnly((t_target - t) / 2, pc, vc, pd, vd))

        # velocity match according to a critically damped spring!
        # self.segments.append(\
        #     CritDampParam('''Tyler stuff goes here'''))
        # self.segments.append(QHoldPR(1.0, pd, Rxd)); # for quasi-static
        # self.segments.append(CritDampPR(1.0, pd, vd, Rxd)); # for dynamic

    '''
    Updates the arm position.
    '''
    def update_arm(self, t, dt):
        if (self.segment_index >= len(self.segments)):
            self.update_asteroid(t, dt)

        dur = self.segments[self.segment_index].duration()
        if (t - self.t0 >= dur):
            print(self.segment_index)
            self.t0 = (self.t0 + dur)
            # only add a segment to change orientation if we just did a position only segment!
            if (self.segments[self.segment_index].get_type() == "position_only"):
                vd = np.array([0.0,0.0,0.0]).reshape([3,1])
                Rf = self.Rf(-self.asteroid_direction, self.last_R)
                self.segments.append(QSplinePR(dur, self.last_pos, vd, self.last_R, \
                                                    self.last_pos, vd, Rf))
                # add a hold, just for fun.
                self.segments.append(QHoldPR(1.0, self.last_pos, Rf))

            self.segment_index += 1

        if (self.segment_index >= len(self.segments)):
            self.update_asteroid(t, dt)
        segment = self.segments[self.segment_index]

        (T, Jp) = self.kin.fkin(self.last_q)
        p = p_from_T(T)
        R = R_from_T(T)
        # weighted pseudoinverse
        J_inv = np.array([])
        if (segment.get_type() == "both"):
            gam = 0.05
            J_inv = Jp.T @ np.linalg.inv(Jp @ Jp.T + gam*gam*np.eye(6));
        elif (segment.get_type() == "position_only"):
            gam = 0.05
            Jp = Jp[0:3, 0:7]
            J_inv = Jp.T @ np.linalg.inv(Jp @ Jp.T + gam*gam*np.eye(3));
        else:
            print("I don't recognize you!")
            while (True):
                pass
        # J_inv = np.linalg.pinv(Jp)

        # desired terms
        (pd, vd) = segment.evaluate_p(t - self.t0)
        (Rd, wd) = segment.evaluate_R(t - self.t0)

        # error terms
        ep = self.ep(pd, p)
        # print(ep)
        eR = self.eR(Rd, R) # gives a 3 x 1 vector

        # Compute velocity
        prdot = vd + self.lam * ep
        wrdot = wd + self.lam * eR

        qdot = np.array([])
        if (segment.get_type() == "both"):
            qdot = J_inv @ np.vstack((prdot, wrdot))
        elif (segment.get_type() == "position_only"):
            qdot = J_inv @ prdot

        # discretely integrate
        q = (self.last_q + dt * qdot)

        # save info
        self.last_pos = p # updated every time the arm moves!
        self.last_vel = vd
        self.last_R = R
        self.last_q = q

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

    # Desired orientation calculation based on current orientation, R (np array)
    def Rf(self, eyd, R): # aligns y axis to path of incoming asteroid, eyd (np array)
        eyc = R[0:3,1:2] # current y axis
        axis = np.cross(eyc, eyd, axis=0).reshape([3]) # axis to rotate eyc about to meet eyd
        norm = np.linalg.norm(axis) # gives sin of angle between them

        # assume eyc and eyd to both be normed
        cos = (eyc.T @ eyd)[0] # gives cosine of angle between them
        angle = math.atan2(norm, cos) # potentially more robust way of determining angle
        axis /= norm

        return R_from_axisangle(axis, angle) @ R


    # Error functions
    def ep(self, pd, pa):
        return (pd - pa)
    def eR(self, Rd, Ra):
        return 0.5*(np.cross(Ra[:,0:1], Rd[:,0:1], axis=0) +
                    np.cross(Ra[:,1:2], Rd[:,1:2], axis=0) +
                    np.cross(Ra[:,2:3], Rd[:,2:3], axis=0))
    def eX(self, Xd, Ra):
        return 0.5*(np.cross(Ra[:,1:2], Xd, axis=0))

    # Reset.  If the simulation resets, also restart the trajectory.
    def reset(self):
        self.asteroid_handler.delete_all_asteroids()
        # Just reset the start time to zero and create a new asteroid
        self.t0    = 0.0

        self.last_q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape([7,1])
        self.last_pos = np.array([1.0, 1.0, 1.0]).reshape([3,1]) # updated every time the arm moves!
        self.last_vel = np.array([0.0, 0.0, 0.0]).reshape([3,1])
        # self.last_R = np.array([1.0, 0.0, 0.0]).reshape([3,1])
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
