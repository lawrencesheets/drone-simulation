import      numpy       as np
from        vpython     import *
from        constants   import *




class Rocket(object):

    def __init__(self):

        self.pos        = vector(0, 500, 0)     # [m]
        self.axis       = vector(0, 12, 0)      # [m]
        self.radius     = 1                     # [m]
        self.mass       = 377                   # [kg]
        self.C_d        = 0.7
        self.velocity   = vector(0, 0, 0)
        self.render     = cylinder(pos=self.pos, axis=self.axis, radius=self.radius)
        self.Xa         = 47.5                  # [m**2]

    def update(self, wind, parachute):

        acceleration    = vector(0, gravity, 0)
        velocity        = vector(self.velocity) + wind.velocity
        drag            = vector(velocity) * -1
        drag.mag        = self.calcDrag(velocity)/self.mass
        self.velocity   += acceleration * dt
        self.velocity   += drag * dt
        self.pos        += self.velocity * dt
        self.render.pos = self.pos

        for v in parachute.vertices:
            v.pos       += self.velocity * dt


    def calcDrag(self, v):

        return self.C_d * 0.5 * self.Xa * rho * mag(v)**2
