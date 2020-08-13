#
# A convenient, gym-like wrapper for pybullet
# https://github.com/liusida/PyBulletWrapper
#
# Author of this file:
#   2020 Sida Liu (learner.sida.liu@gmail.com)
# License:
#   MIT
# Description:
#   This wrapper is the base of all other wrappers, it takes some Python tricks create a class interface for a module.
#   Gym-like wrappers are easy to use.
#   In this way, everyone can make their own wrappers, and add useful functionalities to pybullet (without modifing the C source code).
# Support Version:
#   pybullet 2.8.5 (installed via `pip install pybullet`)
#   Python 3.6
# Reference:
#   (1) PyBullet Quickstart Guide
#   (2) pybullet.c

class BaseWrapperPyBullet(object):
    def __init__(self, p):
        self.p = p

    # Basic mechanism: calling __getattribute__() will not trigger __getattr__(), so we override __getattr__, and calling __getattribute__ accordingly.
    def __getattr__(self, name):
        # Check for special member variable `p` and methods implemented in this class
        if name == 'p' or name in type(self).__dict__:
            return self.__getattribute__(self, name)
        # Otherwise, call pybullet's function
        return getattr(self.p, name)


if __name__ == "__main__":
    import pybullet
    import time
    p = BaseWrapperPyBullet(pybullet)
    p.connect(p.GUI)
    while True:
        p.stepSimulation()
        time.sleep(0.1)
