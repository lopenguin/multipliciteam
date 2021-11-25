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
        self.c =  3*(pf-p0)/T**2 - vf/T    - 2*v0/T
        self.d = -2*(pf-p0)/T**3 + vf/T**2 +   v0/T**2

    # Compute the position/velocity for a given time (w.r.t. t=0 start).
    def evaluate(self, t):
        # Compute and return the position and velocity.
        p = self.a + self.b * t +   self.c * t**2 +   self.d * t**3
        v =          self.b     + 2*self.c * t    + 3*self.d * t**2
        return (p,v)

class Goto(CubicSpline):
    # Use zero initial/final velocities (of same size as positions).
    def __init__(self, p0, pf, T, space='Joint'):
        CubicSpline.__init__(self, p0, 0*p0, pf, 0*pf, T, space)

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
        Segment.__init__(T, space)
        # Precompute the six spline parameters.
        self.a = p0
        self.b = v0
        self.c = a0
        self.d = -10*p0/T**3 - 6*v0/T**2 - 3*a0/T    + 10*pf/T**3 - 4*vf/T**2 + 0.5*af/T
        self.e =  15*p0/T**4 + 8*v0/T**3 + 3*a0/T**2 - 15*pf/T**4 + 7*vf/T**3 -   1*af/T**2
        self.f =  -6*p0/T**5 - 3*v0/T**4 - 1*a0/T**3 +  6*pf/T**5 - 3*vf/T**4 + 0.5*af/T**3

    # Compute the position/velocity for a given time (w.r.t. t=0 start).
    def evaluate(self, t):
        # Compute and return the position and velocity.
        p = self.a + self.b * t +   self.c * t**2 +   self.d * t**3 +   self.e * t**4 +   self.f * t**5
        v =          self.b     + 2*self.c * t    + 3*self.d * t**2 + 4*self.e * t**3 + 5*self.f * t**4
        return (p,v)

class Goto5(QuinticSpline):
    # Use zero initial/final velocities/accelerations (same size as positions).
    def __init__(self, p0, pf, T, space='Joint'):
        QuinticSpline.__init__(self, p0, 0*p0, 0*p0, pf, 0*pf, 0*pf, T, space)

'''
Special quintic spline. Returns a parameterized s from 0.0 -> 1.0
with zero final and initial acceleration. p0, v0, R0, w0, pf, vf, Rf, w0 are
simply stored and can be accessed with get_p0()...

Note: R0 and w0 are not restricted to being matricies and vectors. R0 can be
the vector x direction and w0 can just be wx, if desired.
'''
class QSplineParam(QuinticSpline):
    def __init__(self, T, p0, v0, R0, w0, pf, vf, Rf, wf):
        # scale sdot0, sdotf by v0 and vf
        sdot0 = v0 / (pf - p0)
        sdotf = vf / (pf - p0)

        QuinticSpline.__init__(self, 0.0, sdot0, 0.0, 1.0, sdotf, 0.0, T)

        self.p0 = p0
        self.v0 = v0
        self.R0 = R0
        self.w0 = w0
        self.pf = pf
        self.vf = vf
        self.Rf = Rf
        self.wf = wf


    def get_p0(self):
        return self.p0
    def get_v0(self):
        return self.v0
    def get_R0(self):
        return self.R0
    def get_w0(self):
        return self.w0
    def get_pf(self):
        return self.pf
    def get_vf(self):
        return self.vf
    def get_Rf(self):
        return self.Rf
    def get_wf(self):
        return self.wf

# Class hold at a point and orientation.
class Hold5Param(QSplineParam):
    def __init(self, T, p, R):
        QSplineParam.__init__(self, T, p, 0*p, R, 0*p, p, 0*p, R, 0*p)
