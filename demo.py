# Author: Sida Liu 2020 (learner.sida.liu@gmail.com)
# This is a demo of using wrappers

import pybullet as p
from pybullet_wrapper.base import BaseWrapperPyBullet
from pybullet_wrapper.handy import HandyPyBullet

p = BaseWrapperPyBullet(p)
p.start()
while True:
    p.stepSimulation()
    p.sleep()
