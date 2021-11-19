'''
desired_T.py

Gives a position, orientation, and time to intercept an asteroid.
'''
import numpy as np
from kinematics import T_from_Rp

# placeholder ball class: generates a ball that moves at constant velocity
class Ball:
    def __init__(self, p0, v0):
        self.p0 = p0;
        self.v0 = v0;

    def get_p(t):
        return self.p0 + t*self.v0;

    def get_v(t):
        return self.v0;



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
