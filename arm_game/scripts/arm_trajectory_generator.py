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
from desired_t           import Catch

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
        self.kin = Kinematics(robot, 'world', 'tip')

        # Initialize the storage of the last joint values (column vector). 
        # The joints are generally centered at 0 radians.
        self.lasttheta = \
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape((-1,1))

        # Create point to hold start/end position of each motion (prev goal and new goal)
        self.p_start = self.lasttheta 
        self.p_end   = self.lasttheta

        # Create the splines for the path variable. 
        # The first Goto will be replaced by a motion segment to catch a ball
        # in a given time, or it will be ignored if the ball can't be caught -
        # the hold segment will be run instead.
        self.segments = (Goto(0.0, 1.0, 1.0, 'Path'), # s from 0 to 1 is motion
                         Hold(1.0,      1.0, 'Path')) # s at 1 is holding

        # Initialize the current segment index and starting time t0.
        self.index = 0
        self.t0    = 0.0

        self.get_next_target() # Updates segments, p_start, and p_end

        

        # Initialize/save parameters.
        self.lam = 20 # aggressiveness of convergence to goal
        self.gamma = 1 # weight of pseudo-inverse

    # Path. s from 0 to 1 is motion, s at 1 is holding.
    def pd(self, s):
        if   (s < 1.0):
            return (1-s) * self.p_start + (s) * self.p_end
        else:
            return self.p_end

    def vd(self, s, sdot):
        if   (s < 1.0):
            return (self.p_end - self.p_start) * sdot
        else:
            return 0

    def Rd(self, s): # TODO factor in desired orientation
        return Rx(np.pi/2 * s/3) @ Ry(np.pi)

    def wd(self, s, sdot): # TODO factor in desired orientation
        return np.array([np.pi/6, 0.0, 0.0]).reshape((3,1)) * sdot

    # Error functions
    def ep(self, pd, pa):
        return (pd - pa)
    def eR(self, Rd, Ra):
        return 0.5*(np.cross(Ra[:,0:1], Rd[:,0:1], axis=0) +
                    np.cross(Ra[:,1:2], Rd[:,1:2], axis=0) +
                    np.cross(Ra[:,2:3], Rd[:,2:3], axis=0))

    # Called once we have finished a 1s position hold and are ready to catch
    # another asteroid
    def get_next_target(self, t):
        (p, x, t_move) = Catch.get_catch_pos(t)
        if (t_move <= 1):
            # If t_move<1 second, we can't move there fast enough, so we stay 
            # put for t_move seconds.
            self.index = 1 # Go right to the 2nd segment, which is hold.
                           # Holds at p_end, which is end of previous segment
                           # and thus the current position.
            self.segments[1] = Hold(1.0,      t_move,       'Path')
        else:
            # Run the path in t-1 seconds, and then hold for the last second.
            # TODO this is not necessarily ideal if t is, say, 1.1
            self.segments[0] = Goto(0.0, 1.0, t_move - 1.0, 'Path')
            self.segments[1] = Hold(1.0,      1.0,          'Path')
            self.p_start = self.p_end
            self.p_end   = p

            #TODO update desired orientation



    # Update is called every 10ms!
    def update(self, t, dt):
        # If the current segment is done, shift to the next.
        dur = self.segments[self.index].duration()
        if (t - self.t0 >= dur):
            self.t0    = (self.t0    + dur)                    
            self.index = (self.index + 1) % len(self.segments)  # cyclic
            # Segments wrap around if we finished our previous task
            # Last segment was either a 1s hold or a wait of some time <1s 
            # if an asteroid was coming in too fast to catch.
            if (self.index == 0):
                self.get_next_target(t)

        # Check whether we are done with all segments.
        if (self.index >= len(self.segments)):
            rospy.signal_shutdown("Done with motion")
            return


        # Determine the desired tip position/rotation/velocities (task
        # information) for the current time.  Start grabbing the
        # current path variable (from the spline segment).  Then use
        # the above functions to convert to p/R/v/w:
        (s, sdot) = self.segments[self.index].evaluate(t - self.t0)
        pd = self.pd(s)
        Rd = self.Rd(s)
        vd = self.vd(s,sdot)
        wd = self.wd(s,sdot)


        # Then start at the last cycle's joint values.
        theta = self.lasttheta

        # Compute the forward kinematics (using last cycle's theta),
        # extracting the position and orientation.
        (T,J) = self.kin.fkin(theta)
        p     = p_from_T(T)
        R     = R_from_T(T)

        # Stack the linear and rotation reference velocities (summing
        # the desired velocities and scaled errors)
        xrdot = np.vstack((vd + self.lam * self.ep(pd, p),
                           wd + self.lam * self.eR(Rd, R)))


        # Solve the inverse Jacobian with centering secondary task.
        # The center of each joint angle's range is 0 radians.
        center    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        center    = np.pi * np.array(center).reshape((-1,1))
        thetadot2 = - 10.0 * (theta - center)

        # Calculate weighted pseudo-inverse of Jacobian to avoid singularities
        Jpinv    = np.transpose(J) @ 
                   np.linalg.inv(J @ np.transpose(J) + self.gamma ** 2 * np.eye(6))
        thetadot = thetadot2 + Jpinv @ (xrdot - J@thetadot2)

        # Take a step, using Euler integration to advance the joints.
        theta = theta + dt * thetadot


        # Save the joint values (to be used next cycle).
        self.lasttheta = theta

        # Collect and send the JointState message (with the current time).
        cmdmsg = JointState()
        cmdmsg.name         = ['joint_a1', 'joint_a2', 'joint_a3',
                               'joint_a4', 'joing_a5', 'joint_a6', 'joint_a7']
        cmdmsg.position     = theta
        cmdmsg.velocity     = thetadot
        cmdmsg.header.stamp = rospy.Time.now()
        self.pub.publish(cmdmsg)


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
    rate  = 100
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
