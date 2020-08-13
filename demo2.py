# Author: Sida Liu 2020 (learner.sida.liu@gmail.com)
# This is a more complicated demo of using wrappers

import pybullet as p
from pybullet_wrapper.base import BaseWrapperPyBullet
from pybullet_wrapper.handy import HandyPyBullet
from pybullet_wrapper.pretty_float import PrettyFloatPyBullet

p = BaseWrapperPyBullet(p)
p = HandyPyBullet(p)
p = PrettyFloatPyBullet(p)

# Note: here we use start() function from `HandyPyBullet`
p.start()
planeId = p.loadURDF("plane.urdf")
r2d2Id = p.loadURDF("r2d2.urdf", [0, 0, 2])
r2d2Id2 = p.loadURDF("r2d2.urdf", [0, 2, 2])

# Draw boxes around all links of all bodies
for i in range(p.getNumBodies()):  # iterate through all bodies
    # Note: here we use addUserDebugBox() function from `HandyPyBullet`
    p.addUserDebugBox(p.getAABB(i), label=f"uid:{i}")

    for j in range(p.getNumJoints(i)):
        # Note: here we use ...Py() function from `HandyPyBullet`
        joint_info = p.getJointInfoPy(i, j)
        p.addUserDebugBox(p.getAABB(i, j), color=[
                          0, 1, 0], label=f"uid:{i}, jid:{j}, {joint_info['jointName']}")
        break

# Create a constraint to keep the first r2d2 in z axis.
# You can use your mouse to interact with it.
p.createConstraint(parentBodyUniqueId=0, parentLinkIndex=-1, childBodyUniqueId=r2d2Id, childLinkIndex=-1,
                   jointType=p.JOINT_PRISMATIC, jointAxis=[0, 0, 1], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])

# Get the base collision shape of the first r2d2
# Note: here we use another ...Py() function from `HandyPyBullet`
joint_info = p.getJointInfoPy(r2d2Id, 0)
if len(joint_info) > 0:
    # Note: here we use prettyPrint() function from `PrettyFloatPyBullet`
    p.prettyPrint(joint_info)
    print("")

dt = 1./240.
while 1:
    # Try to get mouse information during interaction.
    e = p.getMouseEventsPy()
    if len(e)>0:
        if e[0]['buttonIndex']!=-1:
            p.prettyPrint(e)

    # Step simulation
    p.stepSimulation()
    # Sleep for a while
    p.sleep(dt)

