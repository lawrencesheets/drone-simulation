from math import sin




class Sine(object):

    def __init__(self, freq, amp, phase):

        self.freq = freq
        self.amp = amp
        self.phase = phase

    def y(self, x):

        return self.amp * sin(self.freq * (x + self.phase))
