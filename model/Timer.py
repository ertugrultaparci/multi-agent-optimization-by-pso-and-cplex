import time
import math

class ContextTimer(object):
    def __init__(self, msg):
        self.msg = msg
        self.start = 0

    def __enter__(self):
        self.start = time.time()
        print('--> begin {0}'.format(self.msg))
        return self  # return value is value of with ()

    def __exit__(self, *args):
        elapsed = time.time() - self.start
        self.msecs = math.ceil(1000* elapsed)
        print('<-- end {0},  time: {1:.0f} ms'.format(self.msg, self.msecs))