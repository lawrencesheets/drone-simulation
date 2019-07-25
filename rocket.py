import      numpy       as np
from        vpython     import *
from        constants   import *




class Rocket(object):

    def __init__(self):

        self.pos        = vector(0, 1000, 0)
        self.axis       = vector(0, 37, 0)
        self.radius     = 3
        self.mass       = 377
        self.C_d        = 0.8
        self.velocity   = vector(0, 0, 0)
        self.render     = cylinder(pos=self.pos, axis=self.axis, radius=self.radius)
        self.Xa         = 6.0

    def update(self, wind):

        acceleration    = vector(0, gravity, 0)
        velocity        = vector(self.velocity) + wind.velocity
        drag            = vector(velocity) * -1
        drag.mag        = self.calcDrag(velocity)/self.mass
        self.velocity   += acceleration * dt
        self.velocity   += drag * dt
        self.pos        += self.velocity * dt
        self.render.pos = self.pos


    def calcDrag(self, v):

        return self.C_d * 0.5 * self.Xa * rho * mag(v)**2
