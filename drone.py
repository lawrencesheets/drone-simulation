import      numpy       as np
from        vpython     import *
from        constants   import *




class Drone(object):

    def __init__(self, target, index):

        self.target     = target
        self.index      = index
        self.pos        = vector(np.random.randint(-500, 500), np.random.randint(0, 200), np.random.randint(-500, 500))
        self.velocity   = vector(0, 0, 0)
        self.radius     = 2
        self.axis       = vector(0, 1, 0)
        self.render     = box(pos=self.pos, size=vector(5,1,5))
        self.render.rotate(angle=radians(45), axis=y_hat)
        self.propellers = [
            cylinder(pos=self.pos+(x_hat*6), axis=self.axis*0.25, radius=1),
            cylinder(pos=self.pos-(x_hat*6), axis=self.axis*0.25, radius=1),
            cylinder(pos=self.pos+(z_hat*6), axis=self.axis*0.25, radius=1),
            cylinder(pos=self.pos-(z_hat*6), axis=self.axis*0.25, radius=1),
        ]
        self.mass       = 50
        self.orientation = y_hat
        self.throttle   = 1.0
        self.force      = (250 + self.mass) * abs(gravity)
        self.altitude   = self.pos.y
        self.KP         = 0.1
        self.KI         = 0.0
        self.KD         = 10.0
        self.integral   = 0.0
        self.prevError  = target.pos.y - self.altitude

        self.x_integral = vector(0,0,0)
        self.x_KP       = 1.0
        self.x_KI       = 0.0
        self.x_KD       = 500.0
        self.x_prevError = self.pos - target.pos
        self.x_prevError.y = 0

        # arrows that show the force vector
        self.thrustVector = arrow(pos=self.pos-vector(0,self.radius/2,0)-self.orientation*self.force*self.throttle*0.1, axis=self.orientation*self.force*self.throttle*0.1, shaftwidth=0.5, color=color.red)


    def update(self):

        self.calcThrottle()
        self.calcOrientation()
        thrust          = self.orientation * ((self.force * self.throttle) / self.mass)
        acceleration    = vector(0, gravity, 0)
        self.velocity   += acceleration * dt
        self.velocity   += thrust * dt
        self.pos        += self.velocity * dt
        self.render.pos = self.pos
        self.altitude   = self.pos.y

        # update force vector arrow
        self.thrustVector.pos = self.pos-vector(0,self.radius/2,0)-self.orientation*self.force*self.throttle*0.1
        self.thrustVector.axis = self.orientation*self.force*self.throttle*0.1

        # update body
        self.propellers[0].pos = self.pos+(x_hat*6)
        self.propellers[1].pos = self.pos-(x_hat*6)
        self.propellers[2].pos = self.pos+(z_hat*6)
        self.propellers[3].pos = self.pos-(z_hat*6)



    def calcThrottle(self):

        error           = self.target.pos.y - self.altitude
        self.integral   += error
        derivative      = error - self.prevError
        self.prevError  = error
        self.throttle   = self.KP * error + self.KI * self.integral + self.KD * derivative

        if self.throttle < 0.0:
            self.throttle = 0.0
        if self.throttle > 1.0:
            self.throttle = 1.0


    def calcOrientation(self):

        error           = self.pos - self.target.pos
        error.y         = 0
        self.x_integral += error
        derivative      = error - self.x_prevError
        self.x_prevError = error
        angle           = self.x_KP * error + self.x_KI * self.x_integral + self.x_KD * derivative
        angle.mag       = 45 if mag(angle) > 45 else angle.mag
        self.orientation = rotate(y_hat, angle=radians(mag(angle)), axis=rotate(angle, angle=radians(-90), axis=y_hat ))
