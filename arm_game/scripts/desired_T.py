'''
desired_T.py

Gives a position, orientation, and time to intercept an asteroid.
'''
import numpy as np
from kinematics import T_from_Rp
from asteroid_generator import AsteroidHandler

import rospy
import rospkg
from gazebo_msgs.srv import GetPhysicsProperties

# interface class to handle asteroids.
class Ball:
    # input the get_asteroid_state() output from our asteroid
    def __init__(self, asteroid, start_t):
        self.update_asteroid(asteroid, start_t);

    def update_asteroid(self, new_asteroid, new_t):
        self.asteroid = new_asteroid;
        self.start_t = new_t;

        self.p0 = np.array([self.asteroid.pose.position.x, self.asteroid.position.y, \
                            self.asteroid.positon.z]).reshape([1,3]);
        self.v0 = np.array([self.asteroid.twist.linear.x, self.asteroid.twist.linear.y, \
                            self.asteroid.twist.linear.z]).reshape([1,3]);

        # grav = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties);
        self.grav = np.array([0, 0, -9.8]).reshape([1,3])

    def get_p(self, t):
        dt = t - self.start_t;

        p = self.p0 + self.v0*dt + 0.5*self.grav*dt*dt;
        return p;

    def get_v(self, t):
        dt = t - self.start_t;
        
        v = self.v0 + self.grav*dt;
        return v;



# Unique to each asteroid
class Catch:
    def __init__(self, ball, arm_length, t_start):
        self.ball = ball;
        self.arm_length = arm_length;
        self.t_start = t_start;

        self.dt = 0.05;
        self.good_p = np.array();  # ball pos
        self.good_t = np.array();  # time of position

        self.setup_catch_pos();

    def update_ball(self, new_ball):
        self.ball = new_ball;
        self.setup_catch_pos();

    def setup_catch_pos(self):
        # Compute a range of times when ball's path intersects the task space.
        #    - task space: estimate as the sphere enclosed by the fully extended arm.
        #    - assume the ball starts at the zero position.
        # Save the good times
        t = self.t_start
        last_ball_dist = float("inf");
        while True:
            ball_p = ball.get_p(t);
            # check if in task space
            ball_dist = np.norm(ball_p);
            if (ball_dist < self.arm_length):
                self.good_p.append(ball_p);
                self.good_t.append(ball_t);
            # check if moving away/parallel to task space
            elif (last_ball_dist <= ball_dist):
                break;
            elif (t > 15):
                # timeout
                break;

            t = t + self.dt;
            last_ball_dist = ball_dist;


    def get_catch_pos(self, t_cur):
        # Rule out good positions before our current time, then pick the latest
        # position and return it as well as time to get there.
        # returns: (p, x, t) -> (position (p), x-axis orientation (x), travel time (t))

        # check if last position is past current time
        if (self.good_t[-1] <= t_cur):
            return (np.zeros(1,3), np.zeros(1,3), -1);

        p = self.good_p[-1];
        t = self.good_t[-1] - t_cur;

        # x axis should point in the opposite direction of the ball normal
        # vector (velocity). Don't really care much about the other two.
        n = self.ball.get_v() / np.norm(self.ball_get_v());
        x = -n;

        return (p, x, t);
