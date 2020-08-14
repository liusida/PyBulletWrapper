# Author: Sida Liu 2020 (learner.sida.liu@gmail.com)
# This is an even more complicated demo of attaching many falling convex hulls.
# press a to create new convex hulls.

import pybullet as p
# This tricky import brings back the IntelliSense if this repo exists as a subfolder of a workspace.
if __name__ == "__main__":
    from pybullet_wrapper.base import BaseWrapperPyBullet
    from pybullet_wrapper.handy import HandyPyBullet
    from pybullet_wrapper.pretty_float import PrettyFloatPyBullet
else:
    from .pybullet_wrapper.base import BaseWrapperPyBullet
    from .pybullet_wrapper.handy import HandyPyBullet
    from .pybullet_wrapper.pretty_float import PrettyFloatPyBullet

# This demo need these three packages
import numpy as np
import trimesh # for creating convex hulls.
from pyquaternion import Quaternion # for converting local and global coordinates.
# Note: Quaternion is in (w, x, y, z) format, but orientation in pybullet is in (x, y, z, w) format!

p = BaseWrapperPyBullet(p)
p = PrettyFloatPyBullet(p)
p = HandyPyBullet(p)


def compute_offset(cid, cpos):
    """Compute relative coordinates from global coordinates."""
    global p
    pos, orn = p.getBasePositionAndOrientation(bodyUniqueId=cid)
    raw_offset = np.array(cpos) - np.array(pos)
    orn = np.roll(orn, shift=1)
    orn_inv = Quaternion(orn).inverse
    offset = np.array(orn_inv.rotate(raw_offset))
    return offset


def new_bodies(num):
    """Create several convex hulls and let them fall."""
    for i in range(num):
        vertices = np.random.random(size=[15, 3]) * 0.5
        mesh = trimesh.Trimesh(vertices=vertices).convex_hull  # get convex shape
        color = np.random.random(size=[4])
        color[3] = 1  # Alpha
        color[2] = max(color[2]+0.5, 1) # Nice blue-ish color
        indices = (mesh.faces).reshape(-1)
        cid = p.createCollisionShapePy(shapeType=p.GEOM_MESH, vertices=mesh.vertices)
        vid = p.createVisualShapePy(shapeType=p.GEOM_MESH, vertices=mesh.vertices, indices=indices, rgbaColor=color, specularColor=[1, 1, 1])
        newObjId = p.createMultiBodyPy(baseMass=mesh.mass, basePosition=[0, 0, 4*i], baseCollisionShapeIndex=cid, baseVisualShapeIndex=vid, baseInertialFramePosition=mesh.center_mass)
        p.changeDynamicsPy(newObjId, -1, lateralFriction=0.3, spinningFriction=0.1, rollingFriction=0.1)


p.start()
p.setGravity(0, 0, -10)
floorId = p.loadURDF("plane.urdf")
p.changeDynamicsPy(floorId, -1, lateralFriction=0.3)

# create 10 hulls for free
# new_bodies(10)

connected = {}
while 1:
    e = p.getKeyboardEventsPy()
    if ord('a') in e and e[ord('a')]&p.KEY_WAS_RELEASED:
        # create another 10 hulls
        new_bodies(10)

    ret = p.getContactPointsPy()
    if len(ret) > 0:
        # print(len(ret))
        for c in ret:
            if (c['bodyUniqueIdA'], c['bodyUniqueIdB']) in connected:
                # for same pair, don't attachment twice
                continue
            if c['bodyUniqueIdA'] == floorId or c['bodyUniqueIdB'] == floorId:
                # don't attach to the floor
                continue
            offset0 = compute_offset(c['bodyUniqueIdA'], c['positionOnA'])
            offset1 = compute_offset(c['bodyUniqueIdB'], c['positionOnB'])
            constraintId = p.createConstraintPy(parentBodyUniqueId=c['bodyUniqueIdA'], parentLinkIndex=c['linkIndexA'], childBodyUniqueId=c['bodyUniqueIdB'], childLinkIndex=c['linkIndexB'],
                                 jointType=p.JOINT_FIXED, jointAxis=[0, 1, 0], parentFramePosition=offset0*1.2, childFramePosition=offset1*1.2)
            p.changeConstraintPy(userConstraintUniqueId=constraintId, maxForce=1)
            connected[(c['bodyUniqueIdA'], c['bodyUniqueIdB'])] = 1
            connected[(c['bodyUniqueIdB'], c['bodyUniqueIdA'])] = 1

    p.stepSimulation()
    p.sleep(0.004)
