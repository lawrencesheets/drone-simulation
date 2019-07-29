import      math
import      numpy       as np
from        vpython     import *
from        constants   import *



class Parachute(object):

    def __init__(self, parent=None):
        self.vertices = []
        self.triangles = []
        count = 0
        for i in range(5):
            for j in self.frange(0.0,2*math.pi,math.pi*0.1):
                self.vertices.append(vertex(pos=vector(sin(j) * (10 - (2*i)), sin(i/4*(math.pi/2))*5, cos(j) * (10 - (2*i))) ))
                if parent:
                    self.vertices[count].pos += parent.pos + parent.axis*3
                if count%2 == 0:
                    self.vertices[count].color = color.blue
                count += 1

        for i in range(len(self.vertices)-20):
            t1 = triangle(v0=self.vertices[i], v1=self.vertices[i+1], v2=self.vertices[i+20])
            t2 = triangle(v0=self.vertices[i], v1=self.vertices[i+19], v2=self.vertices[i+20])
            self.triangles.append(t1)
            self.triangles.append(t2)


    def frange(self, start, stop, step):
        i = start
        while i < stop:
            yield i
            i += step


#
#
# scene                   = canvas(height=800, width=1200)
# p = Parachute()
