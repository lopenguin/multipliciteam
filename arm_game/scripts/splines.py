#!/usr/bin/env python3
#
#   splines.py
#
#   TO IMPORT, ADD TO YOUR CODE:
#   from splines import CubicSpline, Goto, Hold, Stay, QuinticSpline, Goto5
#
#
#   This library provides the cubic and quintic spline trajectory
#   segment classes.  I seperated from the main code, just so I don't
#   have to copy/paste it again and again....
#
#   The classes are:
#
#      seg = CubicSpline(p0, v0, pf, vf, T, space='Joint')
#      seg = Goto(       p0,     pf,     T, space='Joint')
#      seg = Hold(       p,              T, space='Joint')
#      seg = Stay(       p,                 space='Joint')
#
#      seg = QuinticSpline(p0, v0, a0, pf, vf, af, T, space='Joint')
#      seg = Goto5(        p0,         pf,         T, space='Joint')
#
#   Each spline object then provides a
#
#      (pos, vel) = seg.evaluate(t)
#      T          = seg.duration()
#      space      = seg.space()
#
#   That is, each segment can compute the position and velocity for
#   the specified time.  Note it can also report which "space" it
#   wants, but inthe end it is the user's responsibility to
#   interpret/use the pos/vel information accordingly.
#
#   The segments also assume a t=0 start time.  That is, when calling
#   evaluate(), please first subtract the start time of the segment!
#
#   The p0,pf,v0,vf,a0,af may be NumPy arrays.
#
import math
from kinematics import axisangle_from_R, R_from_axisangle
import numpy as np


# general segment object (interface class)
class Segment:
    def __init__(self, T, space='Joint'):
        self.T = T
        self.space = space
        pass

    def evaluate(self, t):
        pass

    def get_p0(self):
        (p0, v0) = self.evaluate(0.0)
        return p0

    def get_v0(self):
        (p0, v0) = self.evaluate(0.0)
        return v0

    def get_pf(self):
        (pf, vf) = self.evaluate(self.T)
        return pf

    def get_vf(self):
        (pf, vf) = self.evaluate(self.T)
        return vf

    def space(self):
        return self.usespace

    def duration(self):
        return self.T


#
#  Cubic Segment Objects
#
#  These compute/store the 4 spline parameters, along with the
#  duration and given space.  Note the space is purely for information
#  and doesn't change the computation.
#
class CubicSpline(Segment):
    # Initialize.
    def __init__(self, p0, v0, pf, vf, T, space='Joint'):
        Segment.__init__(self, T, space)
        # Precompute the spline parameters.
        self.a = p0
        self.b = v0
        self.c = 3 * (pf - p0) / T ** 2 - vf / T - 2 * v0 / T
        self.d = -2 * (pf - p0) / T ** 3 + vf / T ** 2 + v0 / T ** 2

    # Compute the position/velocity for a given time (w.r.t. t=0 start).
    def evaluate(self, t):
        # Compute and return the position and velocity.
        p = self.a + self.b * t + self.c * t ** 2 + self.d * t ** 3
        v = self.b + 2 * self.c * t + 3 * self.d * t ** 2
        return (p, v)


class Goto(CubicSpline):
    # Use zero initial/final velocities (of same size as positions).
    def __init__(self, p0, pf, T, space='Joint'):
        CubicSpline.__init__(self, p0, 0 * p0, pf, 0 * pf, T, space)


class Hold(Goto):
    # Use the same initial and final positions.
    def __init__(self, p, T, space='Joint'):
        Goto.__init__(self, p, p, T, space)


class Stay(Hold):
    # Use an infinite time (stay forever).
    def __init__(self, p, space='Joint'):
        Hold.__init__(self, p, math.inf, space);


#
#  Quintic Segment Objects
#
#  These compute/store the 6 spline parameters, along with the
#  duration and given space.  Note the space is purely for information
#  and doesn't change the computation.
#
class QuinticSpline(Segment):
    # Initialize.
    def __init__(self, p0, v0, a0, pf, vf, af, T, space='Joint'):
        Segment.__init__(self, T, space)
        # Precompute the six spline parameters.
        self.a = p0
        self.b = v0
        self.c = a0
        self.d = -10 * p0 / T ** 3 - 6 * v0 / T ** 2 - 3 * a0 / T + 10 * pf / T ** 3 - 4 * vf / T ** 2 + 0.5 * af / T
        self.e = 15 * p0 / T ** 4 + 8 * v0 / T ** 3 + 3 * a0 / T ** 2 - 15 * pf / T ** 4 + 7 * vf / T ** 3 - 1 * af / T ** 2
        self.f = -6 * p0 / T ** 5 - 3 * v0 / T ** 4 - 1 * a0 / T ** 3 + 6 * pf / T ** 5 - 3 * vf / T ** 4 + 0.5 * af / T ** 3

    # Compute the position/velocity for a given time (w.r.t. t=0 start).
    def evaluate(self, t):
        # Compute and return the position and velocity.
        p = self.a + self.b * t + self.c * t ** 2 + self.d * t ** 3 + self.e * t ** 4 + self.f * t ** 5
        v = self.b + 2 * self.c * t + 3 * self.d * t ** 2 + 4 * self.e * t ** 3 + 5 * self.f * t ** 4
        return (p, v)


class Goto5(QuinticSpline):
    # Use zero initial/final velocities/accelerations (same size as positions).
    def __init__(self, p0, pf, T, space='Joint'):
        QuinticSpline.__init__(self, p0, 0 * p0, 0 * p0, pf, 0 * pf, 0 * pf, T, space)


'''
Special segment object! (Interface class)
'''


class SegmentPR:
    def __init__(self, T, space='Joint'):
        self.T = T
        self.space = space
        pass

    def evaluate_p(self, t):
        pass

    def evaluate_R(self, t):
        pass

    def get_p0(self):
        (p0, v0) = self.evaluate_p(0.0)
        return p0

    def get_v0(self):
        (p0, v0) = self.evaluate_p(0.0)
        return v0

    def get_pf(self):
        (pf, vf) = self.evaluate_p(self.T)
        return pf

    def get_vf(self):
        (pf, vf) = self.evaluate_p(self.T)
        return vf

    def get_R0(self):
        (R0, w0) = self.evaluate_R(0.0)
        return R0

    def get_w0(self):
        (R0, w0) = self.evaluate_R(0.0)
        return w0

    def get_Rf(self):
        (Rf, wf) = self.evaluate_R(self.T)
        return Rf

    def get_wf(self):
        (Rf, wf) = self.evaluate_R(self.T)
        return wf

    def space(self):
        return self.usespace

    def duration(self):
        return self.T


'''
Special quintic spline.

Call evaluate_p() to get (p, v).
Call evaluate_R() to get (R, w) where w0 = wf = [0.0, 0.0, 0.0].T

'''


class QSplinePR(SegmentPR):
    def __init__(self, T, p0, v0, R0, pf, vf, Rf):
        SegmentPR.__init__(self, T)

        self.p_spline = QuinticSpline(p0, v0, 0 * v0, pf, vf, 0 * vf, T)
        # ^ this spline is causing problems. For some reason it is moving stuff! WHYYYY

        self.R_spline = QuinticSpline(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, T)

        self.R0 = R0
        self.Rf = Rf
        R_tot = (self.R0).T @ self.Rf  # TODO check order!
        (self.axis, self.tot_angle) = axisangle_from_R(R_tot)

    def evaluate_p(self, t):
        print(self.p_spline.evaluate(t))
        return self.p_spline.evaluate(t)

    def evaluate_R(self, t):
        (s, sdot) = self.R_spline.evaluate(t)

        angle = self.tot_angle * s
        if (np.all(self.R0 == self.Rf)):
            return (self.R0, np.array([0.0,0.0,0.0]).reshape([3, 1]))
        R = R_from_axisangle(self.axis, angle)
        w = np.reshape(self.axis * sdot, [3,1])
        return (R,w)


'''
Hold subclass for quintic spline
'''


class QHoldPR(QSplinePR):
    def __init__(self, T, p, R):
        QSplinePR.__init__(self, T, p, 0*p, R, p, 0*p, R)

'''
Critically dampen a velocity from initial velocity to final velocity. Maintain rotation

Call evaluate_p() to get (p, v).
Call evaluate_R() to get (R, w). R is constant, w=0

'''


class CritDampPR(SegmentPR):
    def __init__(self, T, p0, v0, R0):
        self.T = T

        self.R_spline = QuinticSpline(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, self.T)

        self.p0 = p0
        self.v0 = v0
        self.R0 = R0
        self.Rf = R0
        self.gamma

        R_tot = (self.R0).T @ self.Rf  # TODO check order!
        (self.axis, self.tot_angle) = axisangle_from_R(R_tot)

    def evaluate_p(self, t):
        x = self.p0 + self.v0 * t * math.exp(-self.gamma * t / 2)
        xdot = self.v0 + self.v0 * math.exp(-self.gamma * t / 2)
        return (x, xdot)

    def evaluate_R(self, t):
        (s, sdot) = self.R_spline.evaluate(t)
        angle = self.tot_angle * s
        R = R_from_axisangle(self.axis, angle)
        w = self.axis * sdot
        return (R, w)
