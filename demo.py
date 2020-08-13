# Author: Sida Liu 2020 (learner.sida.liu@gmail.com)
# This is a demo of using wrappers

import pybullet as p

# This tricky import brings back the IntelliSense if this repo exists as a subfolder of a workspace.
if __name__ == '__main__':
    from pybullet_wrapper.base import BaseWrapperPyBullet
    from pybullet_wrapper.handy import HandyPyBullet
else:
    from .pybullet_wrapper.base import BaseWrapperPyBullet
    from .pybullet_wrapper.handy import HandyPyBullet

p = BaseWrapperPyBullet(p)
p = HandyPyBullet(p)
p.start(withPanels=True)
while True:
    p.stepSimulation()
    p.sleep()
