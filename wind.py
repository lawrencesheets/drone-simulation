import      numpy       as np
from        vpython     import *
from        constants   import *




class Wind(object):

    def __init__(self):

        self.velocity   = x_hat


    def rotate(self, angle):

        self.velocity   += rotate(x_hat, angle=radians(angle), axis=y_hat)


    def scale(self, mag):

        self.velocity.mag = abs(mag)
