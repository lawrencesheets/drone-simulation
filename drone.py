import      copy
from        math        import pi
from        vpython     import *
import      numpy       as np
from        sine        import Sine


scene                   = canvas(height=800, width=1200)
scene.camera.pos        = vector(0, 100, 0)

dt                      = 0.01
gravity                 = -9.80665  # [m/s]
rho                     = 1.225     # [kg/m**3]

x_hat                   = vector(1,0,0)
y_hat                   = vector(0,1,0)
z_hat                   = vector(0,0,1)

ground                  = box(pos=vector(0,0,0), size=vector(5000,1,5000), texture="http://cdn.towall.net/l/water-sea.jpg" )


target                  = sphere(pos=vector(0, 100, 0), radius=1, color=color.yellow)
target1                 = sphere(pos=target.pos+x_hat*100, radius=1, color=color.yellow)
target2                 = sphere(pos=target.pos+z_hat*100, radius=1, color=color.yellow)
target3                 = sphere(pos=target.pos-x_hat*100, radius=1, color=color.yellow)
target4                 = sphere(pos=target.pos-z_hat*100, radius=1, color=color.yellow)
target1.index           = 0
target2.index           = 1
target3.index           = 2
target4.index           = 3
targets                 = [target1, target2, target3, target4]

wind                    = vector(1-np.random.random()*2, 0, 1-np.random.random()*2)
frequencies             = [ Sine(np.random.random()*0.05, np.random.randint(5,20), np.random.random()*2*pi) for x in range(5) ]
points                  = []

for x in range(10000):
    y = 0

    for freq in frequencies:
        y += freq.y(x)

    points.append(y)



def toggle_visibility(array):
    for x in array:
        x.radius = 0

toggle_visibility([target] + targets)


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
        # self.thrustVector = arrow(pos=self.pos-vector(0,self.radius/2,0)-self.orientation*self.force*self.throttle*0.1, axis=self.orientation*self.force*self.throttle*0.1, shaftwidth=0.5, color=color.red)


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
        # self.thrustVector.pos = self.pos-vector(0,self.radius/2,0)-self.orientation*self.force*self.throttle*0.1
        # self.thrustVector.axis = self.orientation*self.force*self.throttle*0.1

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



class Wind(object):

    def __init__(self):

        self.velocity   = x_hat


    def rotate(self, angle):

        self.velocity   = rotate(x_hat, angle=radians(angle), axis=y_hat)


    def scale(self, mag):

        self.velocity.mag = abs(mag)




drones                  = [Drone(target,x) for x in range(4)]
tether                  = 10

distances               = []
temp_drones             = copy.copy(drones)
temp_targets            = copy.copy(targets)
temp_index              = []

for x in range(len(targets)):
    for i in range(len(temp_drones)-1,-1,-1):
        for j in range(len(temp_targets)-1,-1,-1):
            dist        = mag(temp_drones[i].pos - temp_targets[j].pos)
            distances.append(dist)
            temp_index.append([temp_drones[i],temp_targets[j]])

    closest             = distances.index(min(distances))
    drones[temp_index[closest][0].index].target = targets[temp_index[closest][1].index]

    temp_drones.remove(temp_index[closest][0])
    temp_targets.remove(temp_index[closest][1])
    temp_index          = []
    distances           = []

drones.sort(key=lambda x: x.target.index, reverse=True)

a                       = vertex(pos=drones[0].pos-(y_hat*tether), color=color.gray(0.5), opacity=0.5)
b                       = vertex(pos=drones[1].pos-(y_hat*tether), color=color.gray(0.5), opacity=0.5)
c                       = vertex(pos=drones[2].pos-(y_hat*tether), color=color.gray(0.5), opacity=0.5)
d                       = vertex(pos=drones[3].pos-(y_hat*tether), color=color.gray(0.5), opacity=0.5)

net                     = quad( v0=a, v1=b, v2=c, v3=d)

rocket                  = Rocket()
trajectory              = cylinder(pos=rocket.pos, axis=vector(0,-rocket.pos.y,0), color=color.black, radius=0.25, opacity=0.25)
falling                 = True
elapsed                 = 0
angle                   = 0

wind                    = Wind()



while True:

    rate(1/dt)

    for drone in drones:
        drone.update()

    net.v0.pos          = drones[0].pos-(y_hat*tether)
    net.v1.pos          = drones[1].pos-(y_hat*tether)
    net.v2.pos          = drones[2].pos-(y_hat*tether)
    net.v3.pos          = drones[3].pos-(y_hat*tether)

    # adjust wind
    wind.rotate(points[elapsed])
    wind.scale(points[len(points)-1-elapsed])

    if falling:
        rocket.update(wind)
    else:
        rocket.pos.y    = sum([d.pos.y for d in drones]) / len(drones) - tether
        if angle < 180:
            rocket.render.rotate(angle=radians(0.5), axis=vector(-wind.velocity.z, 0, wind.velocity.x))
            angle       += 1
        rocket.render.pos = rocket.pos

    trajectory.pos      = rocket.pos
    trajectory.axis.y   = -rocket.pos.y

    target.pos.x        = rocket.pos.x
    target.pos.z        = rocket.pos.z

    target1.pos         = target.pos+x_hat*100
    target2.pos         = target.pos+z_hat*100
    target3.pos         = target.pos-x_hat*100
    target4.pos         = target.pos-z_hat*100

    scene.center        = target.pos

    if rocket.pos.y < target.pos.y-tether and falling:

        # rocket momentum = mass times velocity
        momentum = rocket.mass * mag(rocket.velocity)
        mass = rocket.mass

        for drone in drones:

            momentum += drone.mass * mag(drone.velocity)
            mass += drone.mass
            drone.mass  += rocket.mass / len(drones)

        velocity = momentum / mass
        rv = norm(rocket.velocity)
        rv.mag = velocity

        for drone in drones:

            drone.velocity += rv

        rocket.mass     = 0
        falling         = False

    elapsed             += 1


# end
