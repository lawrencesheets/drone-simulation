import      copy
from        math        import pi
from        vpython     import *
import      numpy       as np
from        constants   import *
from        drone       import Drone
from        rocket      import Rocket
from        wind        import Wind
from        sine        import Sine
from        parachute   import Parachute



scene                   = canvas(height=800, width=1200)
scene.camera.pos        = vector(0, 100, 0)

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

def toggle_visibility(array):
    for x in array:
        x.radius = 0

toggle_visibility([target] + targets)

for x in range(10000):
    y = 0
    for freq in frequencies:
        y += freq.y(x)
    points.append(y)

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
parachute               = Parachute(rocket)
wind                    = Wind()

# trajectory              = cylinder(pos=rocket.pos, axis=vector(0,-rocket.pos.y,0), color=color.black, radius=0.25, opacity=0.25)
falling                 = True
elapsed                 = 0
angle                   = 0


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
        rocket.update(wind, parachute)
    else:
        rocket.pos.y    = sum([d.pos.y for d in drones]) / len(drones) - tether
        if angle < 180:
            rocket.render.rotate(angle=radians(0.5), axis=vector(-wind.velocity.z, 0, wind.velocity.x))
            angle       += 1
        rocket.render.pos = rocket.pos

    # trajectory.pos      = rocket.pos
    # trajectory.axis.y   = -rocket.pos.y

    target.pos.x        = rocket.pos.x
    target.pos.z        = rocket.pos.z

    target1.pos         = target.pos+x_hat*100
    target2.pos         = target.pos+z_hat*100
    target3.pos         = target.pos-x_hat*100
    target4.pos         = target.pos-z_hat*100

    # scene.center        = target.pos

    if rocket.pos.y < target.pos.y-tether and falling:

        # hide parachute
        for t in parachute.triangles:
            t.visible   = False

        # rocket momentum = mass times velocity
        momentum        = rocket.mass * mag(rocket.velocity)
        mass            = rocket.mass

        for drone in drones:

            momentum    += drone.mass * mag(drone.velocity)
            mass        += drone.mass
            drone.mass  += rocket.mass / len(drones)

        velocity        = momentum / mass
        r_vel           = norm(rocket.velocity)
        r_vel.mag       = velocity

        for drone in drones:
            drone.velocity += r_vel

        rocket.mass     = 0
        falling         = False

    elapsed             += 1


# end
