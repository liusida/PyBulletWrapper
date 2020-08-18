#
# A convenient, gym-like wrapper for pybullet
# https://github.com/liusida/PyBulletWrapper
#
# Author of this file:
#   2020 Sida Liu (learner.sida.liu@gmail.com)
# License:
#   MIT
# Description:
#   This wrapper provides some Python style interfaces to pybullet.
#   The original pybullet returns are arrays, but dictionaries are more handy if we don't want to remember the details.
# Support Version:
#   pybullet 2.8.5 (installed via `pip install pybullet`)
#   Python 3.6
# Reference:
#   (1) PyBullet Quickstart Guide
#   (2) pybullet.c

import time
import math
import tempfile
import pybullet_data

from .base import BaseWrapperPyBullet
from .utils import *


class HandyPyBullet(BaseWrapperPyBullet):
    def __init__(self, p):
        self.__physics_client_id = -1
        super().__init__(p)

    # Private:
    def __checkReturn(self, API, ret, length):
        """Check the length of the return from API."""
        if len(ret) != length:
            error(f"ERROR: The structure returned from {API} has {len(ret)} items, which is inconsistent with the documentation.")
            print(f"Please check PyBullet's version. Only 2.8.5 is supported.")
            print("\n")
            print(ret)
            return False
        return True

    def __parseCommonReturn(self, API, retArray, keys):
        """Parse the return (one record) from API into dictionary."""
        if self.__checkReturn(API, retArray, len(keys)):
            retDictionary = {}
            for i, k in enumerate(keys):
                retDictionary[k] = retArray[i]
            return retDictionary
        return retArray

    def __parseListReturns(self, API, retArray, keys):
        """Parse the return (a list of records) from API into a list of dictionaries."""
        if len(retArray) > 0 and self.__checkReturn(API, retArray[0], len(keys)):
            retList = []
            for item in retArray:
                retDictionary = {}
                for i, k in enumerate(keys):
                    retDictionary[k] = item[i]
                retList.append(retDictionary)
            return retList
        return retArray

    def __constructOutParameters(self, in_parameters, valid_parameters):
        """
        To deal with default parameters, we need to construct out_parameters to call pybullet.c
        if the value is None, it will be consider as default.
        valid_parameters in each functions are copied from pybullet.c (from `kwlist`)
        """
        out_parameters = {}
        for key in valid_parameters:
            if key in in_parameters and in_parameters[key] is not None:
                out_parameters[key] = in_parameters[key]
        return out_parameters

    # Public:
    # New handy additional inteface (by combining the functionalities in the original inteface)
    def connectPy(self, withGUI=True, withPanels=False):
        """Start the simulation."""
        if withGUI:
            self.__physics_client_id = self.connect(self.GUI)
            self.showPanelsPy(withPanels)
        else:
            self.__physics_client_id = self.connect(self.DIRECT)
        self.resetPy()
        return self.__physics_client_id

    def resetPy(self, withGUI=True, withData=True, withGravity=True, withFloor=True, defaultTimeStep=.01):
        if self.__physics_client_id < 0:
            self.__physics_client_id = self.connectPy(withGUI=withGUI)
        self.resetSimulation()
        if withData:
            self.setAdditionalSearchPath(pybullet_data.getDataPath())
            if withFloor:
                planeId = self.loadURDFPy("plane.urdf", globalScaling=10)
                self.changeDynamicsPy(bodyUniqueId=planeId, linkIndex=-1, restitution=0.3)
        if withGravity:
            self.setGravity(0, 0, -9.8)
        self.setTimeStep(defaultTimeStep)

    def sleepPy(self, n=0.03):
        """time.sleep(n)"""
        time.sleep(n)

    def showPanelsPy(self, on=True):
        self.configureDebugVisualizer(self.COV_ENABLE_GUI, on)

    def addUserDebugBoxPy(self, twoCorners=[(0, 0, 0), (1, 1, 1)], color=[1, 0, 0], label=""):
        """
        Draw a box on the screen using addUserDebugLine()
        Label the box using addUserDebugText()
        """
        a, b = twoCorners
        lines = []
        a1 = [a[0], a[1], b[2]]
        a2 = [a[0], b[1], a[2]]
        a3 = [b[0], a[1], a[2]]
        b1 = [b[0], b[1], a[2]]
        b2 = [b[0], a[1], b[2]]
        b3 = [a[0], b[1], b[2]]

        lines.append((a, a1))
        lines.append((a, a2))
        lines.append((a, a3))

        lines.append((b, b1))
        lines.append((b, b2))
        lines.append((b, b3))

        lines.append((a1, b2))
        lines.append((a1, b3))
        lines.append((a2, b1))
        lines.append((a2, b3))
        lines.append((a3, b1))
        lines.append((a3, b2))

        lineIds = []
        for line in lines:
            lineId = self.addUserDebugLine(line[0], line[1], lineColorRGB=color)
            lineIds.append(lineId)

        if len(label) > 0:
            self.addUserDebugText(label, textPosition=a1)

        return lineIds

    def getRayFromTo(self, mouseX, mouseY):
        """
        Copied from pybullet examples: addPlanarReflection.py
        """
        width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = self.getDebugVisualizerCamera()
        camPos = [
            camTarget[0] - dist * camForward[0], camTarget[1] - dist * camForward[1],
            camTarget[2] - dist * camForward[2]
        ]
        farPlane = 10000
        rayForward = [(camTarget[0] - camPos[0]), (camTarget[1] - camPos[1]), (camTarget[2] - camPos[2])]
        invLen = farPlane * 1. / (math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] *
                                            rayForward[1] + rayForward[2] * rayForward[2]))
        rayForward = [invLen * rayForward[0], invLen * rayForward[1], invLen * rayForward[2]]
        rayFrom = camPos
        oneOverWidth = float(1) / float(width)
        oneOverHeight = float(1) / float(height)
        dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
        dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
        rayToCenter = [
            rayFrom[0] + rayForward[0], rayFrom[1] + rayForward[1], rayFrom[2] + rayForward[2]
        ]
        rayTo = [
            rayFrom[0] + rayForward[0] - 0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] -
            float(mouseY) * dVer[0], rayFrom[1] + rayForward[1] - 0.5 * horizon[1] + 0.5 * vertical[1] +
            float(mouseX) * dHor[1] - float(mouseY) * dVer[1], rayFrom[2] + rayForward[2] -
            0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]
        ]
        return rayFrom, rayTo

    # Override pybullet functions below:
    # Those ...Py() functions are proxy to the originial interface.
    # These functions can give a hit of the parameters and give dictionary returns.
    # Some original functions, like getKeyboardEvents(), already returns a dictionary, so there's no need to implement handy version.

    def getDebugVisualizerCameraPy(self):
        keys = ["width", "height", "viewMatrix", "projectionMatrix", "cameraUp", "cameraForward", "horizontal", "vertical", "yaw", "pitch", "dist", "target", ]
        retArray = self.getDebugVisualizerCamera()
        return self.__parseCommonReturn("getDebugVisualizerCamera", retArray, keys)

    def getMouseEventsPy(self, physicsClientId=None):
        keys = ["eventType", "mousePosX", "mousePosY", "buttonIndex", "buttonState"]
        valid_parameters = ["physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getMouseEvents(**out_parameters)
        return self.__parseListReturns("getMouseEvents", retArray, keys)

    def getBasePositionAndOrientationPy(self, bodyUniqueId, physicsClientId=None):
        # Note: The orientation of an object can be represented as a rotation of an object from its original unrotated orientation, i.e. (0,0,0,1).
        # orientation.rotate(relative_position) + translation = global_position
        # relative_position = orientation.inverse.rorate(global_position - translation)
        keys = ["position", "orientation"]
        valid_parameters = ["bodyUniqueId", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getBasePositionAndOrientation(**out_parameters)
        return self.__parseCommonReturn("getBasePositionAndOrientation", retArray, keys)

    def getJointInfoPy(self, bodyUniqueId, jointIndex, physicsClientId=None):
        keys = ["jointIndex", "jointName", "jointType", "qIndex", "uIndex", "flags", "jointDamping", "jointFriction", "jointLowerLimit",
                "jointUpperLimit", "jointMaxForce", "jointMaxVelocity", "linkName", "jointAxis", "parentFramePos", "parentFrameOrn", "parentIndex", ]
        valid_parameters = ["bodyUniqueId", "jointIndex", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getJointInfo(**out_parameters)
        return self.__parseCommonReturn("getJointInfo", retArray, keys)

    def getJointStatePy(self, bodyUniqueId, jointIndex, physicsClientId=None):
        keys = ["jointPosition", "jointVelocity", "jointReactionForces", "appliedJointMotorTorque", ]
        valid_parameters = ["bodyUniqueId", "jointIndex", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getJointState(**out_parameters)
        return self.__parseCommonReturn("getJointState", retArray, keys)

    def getJointStatesPy(self, bodyUniqueId, jointIndices, physicsClientId=None):
        keys = ["jointPosition", "jointVelocity", "jointReactionForces", "appliedJointMotorTorque", ]
        valid_parameters = ["bodyUniqueId", "jointIndices", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getJointStates(**out_parameters)
        return self.__parseListReturns("getJointStates", retArray, keys)

    # Note:
    # According to the source code `pybullet.c`, the returned value has 6 items when computeLinkVelocity is 0, but 8 otherwise.
    # This is different from the documentation.
    def getLinkStatePy(self, bodyUniqueId, linkIndex, computeLinkVelocity=None, computeForwardKinematics=None, physicsClientId=None,):
        keys = ["linkWorldPosition", "linkWorldOrientation", "localInertialFramePosition", "localInertialFrameOrientation",
                "worldLinkFramePosition", "worldLinkFrameOrientation", "worldLinkLinearVelocity", "worldLinkAngularVelocity", ]
        keys_without_computeLinkVelocity = keys[:6]
        valid_parameters = ["bodyUniqueId", "linkIndex", "computeLinkVelocity", "computeForwardKinematics", "physicsClientId", ]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getLinkState(**out_parameters)

        if computeLinkVelocity:
            return self.__parseCommonReturn("getLinkState", retArray, keys)
        return self.__parseCommonReturn("getLinkState", retArray, keys_without_computeLinkVelocity)

    def getLinkStatesPy(self, bodyUniqueId, linkIndices, computeLinkVelocity=None, computeForwardKinematics=None, physicsClientId=None,):
        keys = ["linkWorldPosition", "linkWorldOrientation", "localInertialFramePosition", "localInertialFrameOrientation",
                "worldLinkFramePosition", "worldLinkFrameOrientation", "worldLinkLinearVelocity", "worldLinkAngularVelocity", ]
        keys_without_computeLinkVelocity = keys[:6]
        valid_parameters = ["bodyUniqueId", "linkIndices", "computeLinkVelocity", "computeForwardKinematics", "physicsClientId", ]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getLinkStates(**out_parameters)

        if computeLinkVelocity:
            return self.__parseListReturns("getLinkStates", retArray, keys)
        return self.__parseListReturns("getLinkStates", retArray, keys_without_computeLinkVelocity)

    def getConstraintInfoPy(self, constraintUniqueId, physicsClientId=None):
        keys = ["parentBodyUniqueId", "parentJointIndex", "childBodyUniqueId", "childLinkIndex", "constraintType", "jointAxis", "jointPivotInParent", "jointPivotInChild",
                "jointFrameOrientationParent", "jointFrameOrientationChild", "maxAppliedForce", "gearRatio", "gearAuxLink", "relativePositionTarget", "erp", ]
        valid_parameters = ["constraintUniqueId", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getConstraintInfo(**out_parameters)
        return self.__parseCommonReturn("getConstraintInfo", retArray, keys)

    def getDynamicsInfoPy(self, bodyUniqueId, linkIndex, physicsClientId=None):
        keys = ["mass", "lateralFriction", "localInertiaDiagonal", "localInertialPos", "localInertialOrn", "restitution",
                "rollingFriction", "spinningFriction", "contactDamping", "contactStiffness", "bodyType", "collisionMargin", ]
        valid_parameters = ["bodyUniqueId", "linkIndex", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getDynamicsInfo(**out_parameters)
        return self.__parseCommonReturn("getDynamicsInfo", retArray, keys)

    def getCameraImagePy(self, width, height, viewMatrix=None, projectionMatrix=None, lightDirection=None, lightColor=None, lightDistance=None, shadow=None,
                         lightAmbientCoeff=None, lightDiffuseCoeff=None, lightSpecularCoeff=None, renderer=None, flags=None, physicsClientId=None,):
        keys = ["width", "height", "rgbPixels", "depthPixels", "segmentationMaskBuffer"]
        valid_parameters = [
            "width", "height", "viewMatrix", "projectionMatrix", "lightDirection", "lightColor", "lightDistance", "shadow", "lightAmbientCoeff",
            "lightDiffuseCoeff", "lightSpecularCoeff", "renderer", "flags", "projectiveTextureView", "projectiveTextureProj", "physicsClientId", ]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getCameraImage(**out_parameters)
        return self.__parseCommonReturn("getDynamicsInfo", retArray, keys)

    def getVisualShapeDataPy(self, objectUniqueId, flags=None, physicsClientId=None):
        keys = ["objectUniqueId", "linkIndex", "visualGeometryType", "dimensions", "meshAssetFileName",
                "localVisualFramePosition", "localVisualFrameOrientation", "rgbaColor", "textureUniqueId", ]
        keys_without_VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS = ["objectUniqueId", "linkIndex", "visualGeometryType",
                                                             "dimensions", "meshAssetFileName", "localVisualFramePosition", "localVisualFrameOrientation", "rgbaColor", ]
        valid_parameters = ["objectUniqueId", "flags", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getVisualShapeData(**out_parameters)
        if flags & self.VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS:
            return self.__parseListReturns("getVisualShapeData", retArray, keys)
        return self.__parseListReturns("getVisualShapeData", retArray, keys_without_VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS,)

    def getContactPointsPy(self, bodyA=None, bodyB=None, linkIndexA=None, linkIndexB=None, physicsClientId=None,):
        # Note: positionOnA and positionOnB are in global Cartesian coordinates.
        keys = ["contactFlag", "bodyUniqueIdA", "bodyUniqueIdB", "linkIndexA", "linkIndexB", "positionOnA", "positionOnB", "contactNormalOnB",
                "contactDistance", "normalForce", "lateralFriction1", "lateralFrictionDir1", "lateralFriction2", "lateralFrictionDir2", ]
        valid_parameters = ["bodyA", "bodyB", "linkIndexA", "linkIndexB", "physicsClientId", ]
        valid_parameters = ["bodyA", "bodyB", "linkIndexA", "linkIndexB", "physicsClientId", ]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getContactPoints(**out_parameters)
        return self.__parseListReturns("getContactPoints", retArray, keys)

    def getClosestPointsPy(self, bodyA, bodyB, distance, linkIndexA=None, linkIndexB=None, physicsClientId=None,):
        keys = ["contactFlag", "bodyUniqueIdA", "bodyUniqueIdB", "linkIndexA", "linkIndexB", "positionOnA", "positionOnB", "contactNormalOnB",
                "contactDistance", "normalForce", "lateralFriction1", "lateralFrictionDir1", "lateralFriction2", "lateralFrictionDir2", ]
        # Note: There are four -1's because according to pybullet.c, there are four unused parameters:
        # collisionShapePositionAObj, collisionShapePositionBObj, collisionShapeOrientationA, and collisionShapeOrientationBObj.
        valid_parameters = ["bodyA", "bodyB", "distance", "linkIndexA", "linkIndexB", "collisionShapeA", "collisionShapeB",
                            "collisionShapePositionA", "collisionShapePositionB", "collisionShapeOrientationA", "collisionShapeOrientationB", "physicsClientId", ]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getClosestPoints(**out_parameters)
        return self.__parseListReturns("getClosestPoints", retArray, keys)

    def getCollisionShapeDataPy(self, objectUniqueId, linkIndex=None, physicsClientId=None):
        keys = ["objectUniqueId", "linkIndex", "geometryType", "dimensions", "filename", "localFramePos", "localFrameOrn", ]
        valid_parameters = ["objectUniqueId", "linkIndex", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getCollisionShapeData(**out_parameters)
        return self.__parseListReturns("getCollisionShapeData", retArray, keys)

    # Simple proxy functions just for IntelliSense
    def createMultiBodyPy(self, baseMass=None, baseCollisionShapeIndex=None, baseVisualShapeIndex=None, basePosition=None, baseOrientation=None,
                          baseInertialFramePosition=None, baseInertialFrameOrientation=None, linkMasses=None, linkCollisionShapeIndices=None,
                          linkVisualShapeIndices=None, linkPositions=None, linkOrientations=None, linkInertialFramePositions=None,
                          linkInertialFrameOrientations=None, linkParentIndices=None, linkJointTypes=None, linkJointAxis=None, useMaximalCoordinates=None,
                          flags=None, batchPositions=None, physicsClientId=None,):
        valid_parameters = [
            "baseMass", "baseCollisionShapeIndex", "baseVisualShapeIndex", "basePosition", "baseOrientation", "baseInertialFramePosition",
            "baseInertialFrameOrientation", "linkMasses", "linkCollisionShapeIndices", "linkVisualShapeIndices", "linkPositions", "linkOrientations",
            "linkInertialFramePositions", "linkInertialFrameOrientations", "linkParentIndices", "linkJointTypes", "linkJointAxis", "useMaximalCoordinates",
            "flags", "batchPositions", "physicsClientId", ]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.createMultiBody(**out_parameters)

    def changeDynamicsPy(self, bodyUniqueId, linkIndex, mass=None, lateralFriction=None, spinningFriction=None, rollingFriction=None, restitution=None,
                         linearDamping=None, angularDamping=None, contactStiffness=None, contactDamping=None, frictionAnchor=None, localInertiaDiagonal=None,
                         ccdSweptSphereRadius=None, contactProcessingThreshold=None, activationState=None, jointDamping=None, anisotropicFriction=None,
                         maxJointVelocity=None, collisionMargin=None, jointLowerLimit=None, jointUpperLimit=None, jointLimitForce=None, physicsClientId=None,):
        valid_parameters = [
            "bodyUniqueId", "linkIndex", "mass", "lateralFriction", "spinningFriction", "rollingFriction", "restitution", "linearDamping", "angularDamping",
            "contactStiffness", "contactDamping", "frictionAnchor", "localInertiaDiagonal", "ccdSweptSphereRadius", "contactProcessingThreshold",
            "activationState", "jointDamping", "anisotropicFriction", "maxJointVelocity", "collisionMargin", "jointLowerLimit", "jointUpperLimit",
            "jointLimitForce", "physicsClientId", ]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.changeDynamics(**out_parameters)

    def createCollisionShapePy(
            self, shapeType=None, radius=None, halfExtents=None, height=None, fileName=None, meshScale=None, planeNormal=None, flags=None,
            collisionFramePosition=None, collisionFrameOrientation=None, vertices=None, indices=None, heightfieldTextureScaling=None, heightfieldData=None,
            numHeightfieldRows=None, numHeightfieldColumns=None, replaceHeightfieldIndex=None, physicsClientId=None):
        # Note: passing in `indices` will implicitly cause bullet to use flags=p.GEOM_FORCE_CONCAVE_TRIMESH
        valid_parameters = [
            "shapeType", "radius", "halfExtents", "height", "fileName", "meshScale", "planeNormal", "flags", "collisionFramePosition",
            "collisionFrameOrientation", "vertices", "indices", "heightfieldTextureScaling", "heightfieldData", "numHeightfieldRows", "numHeightfieldColumns",
            "replaceHeightfieldIndex", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.createCollisionShape(**out_parameters)

    def createVisualShapePy(
            self, shapeType=None, radius=None, halfExtents=None, length=None, fileName=None, meshScale=None, planeNormal=None, flags=None, rgbaColor=None,
            specularColor=None, visualFramePosition=None, visualFrameOrientation=None, vertices=None, indices=None, normals=None, uvs=None,
            physicsClientId=None):
        valid_parameters = ["shapeType", "radius", "halfExtents", "length", "fileName", "meshScale", "planeNormal", "flags", "rgbaColor",
                            "specularColor", "visualFramePosition", "visualFrameOrientation", "vertices", "indices", "normals", "uvs", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.createVisualShape(**out_parameters)

    def createConstraintPy(self, parentBodyUniqueId, parentLinkIndex, childBodyUniqueId, childLinkIndex, jointType, jointAxis, parentFramePosition,
                           childFramePosition, parentFrameOrientation=None, childFrameOrientation=None, physicsClientId=None):
        valid_parameters = ["parentBodyUniqueId", "parentLinkIndex", "childBodyUniqueId", "childLinkIndex", "jointType", "jointAxis",
                            "parentFramePosition", "childFramePosition", "parentFrameOrientation", "childFrameOrientation", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.createConstraint(**out_parameters)

    def addUserDebugLinePy(
            self, lineFromXYZ, lineToXYZ, lineColorRGB=None, lineWidth=None, lifeTime=None, parentObjectUniqueId=None, parentLinkIndex=None,
            replaceItemUniqueId=None, physicsClientId=None):
        valid_parameters = ["lineFromXYZ", "lineToXYZ", "lineColorRGB", "lineWidth", "lifeTime",
                            "parentObjectUniqueId", "parentLinkIndex", "replaceItemUniqueId", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.addUserDebugLine(**out_parameters)

    def getKeyboardEventsPy(self, physicsClientId=None):
        """return a dictionary {key_value: state_value}"""
        valid_parameters = ["physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.getKeyboardEvents(**out_parameters)

    def changeConstraintPy(
            self, userConstraintUniqueId, jointChildPivot=None, jointChildFrameOrientation=None, maxForce=None, gearRatio=None, gearAuxLink=None,
            relativePositionTarget=None, erp=None, physicsClientId=None):
        valid_parameters = ["userConstraintUniqueId", "jointChildPivot", "jointChildFrameOrientation",
                            "maxForce", "gearRatio", "gearAuxLink", "relativePositionTarget", "erp", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.changeConstraint(**out_parameters)

    def setJointMotorControl2Py(
            self, bodyUniqueId, jointIndex, controlMode, targetPosition=None, targetVelocity=None, force=None, positionGain=None, velocityGain=None,
            maxVelocity=None, physicsClientId=None):
        valid_parameters = ["bodyUniqueId", "jointIndex", "controlMode", "targetPosition",
                            "targetVelocity", "force", "positionGain", "velocityGain", "maxVelocity", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.setJointMotorControl2(**out_parameters)

    def setJointMotorControlArrayPy(
            self, bodyUniqueId, jointIndices, controlMode, targetPositions=None, targetVelocities=None, forces=None, positionGains=None, velocityGains=None,
            physicsClientId=None):
        valid_parameters = ["bodyUniqueId", "jointIndices", "controlMode", "targetPositions",
                            "targetVelocities", "forces", "positionGains", "velocityGains", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.setJointMotorControlArray(**out_parameters)

    def getNumJointsPy(self, bodyUniqueId, physicsClientId=None):
        valid_parameters = ["bodyUniqueId", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.getNumJoints(**out_parameters)

    def loadURDFPy(
            self, fileName, basePosition=None, baseOrientation=None, useMaximalCoordinates=None, useFixedBase=None, flags=None, globalScaling=None,
            physicsClientId=None):
        valid_parameters = ["fileName", "basePosition", "baseOrientation", "useMaximalCoordinates", "useFixedBase", "flags", "globalScaling", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.loadURDF(**out_parameters)

    def loadXacroPy(
            self, fileName, basePosition=None, baseOrientation=None, useMaximalCoordinates=None, useFixedBase=None, flags=None, globalScaling=None,
            physicsClientId=None):
        valid_parameters = ["fileName", "basePosition", "baseOrientation", "useMaximalCoordinates", "useFixedBase", "flags", "globalScaling", "physicsClientId"]
        from .xacro_wrapper import xacroWrapper as x  # import xacro support as needed
        urdf = x.xacro_to_urdf(fileName)  # read .xacro file
        f = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
        print(urdf, file=f)  # write .urdf file into a tempfile
        fileName = f.name  # get the filename for loadURDF
        debug(f"load urdf from xacro, urdf filename: {fileName}")
        f.close()
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.loadURDF(**out_parameters)

    def setCollisionFilterPairPy(self, bodyUniqueIdA, bodyUniqueIdB, linkIndexA, linkIndexB, enableCollision, physicsClientId=None):
        valid_parameters = ["bodyUniqueIdA", "bodyUniqueIdB", "linkIndexA", "linkIndexB", "enableCollision", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.setCollisionFilterPair(**out_parameters)

    def rayTestBatchPy(
            self, rayFromPositions, rayToPositions, numThreads=None, parentObjectUniqueId=None, parentLinkIndex=None, reportHitNumber=None,
            collisionFilterMask=None, fractionEpsilon=None, physicsClientId=None):
        keys = ["objectUniqueId", "linkIndex", "hitFraction", "hitPosition", "hitNormal"]
        valid_parameters = ["rayFromPositions", "rayToPositions", "numThreads", "parentObjectUniqueId",
                            "parentLinkIndex", "reportHitNumber", "collisionFilterMask", "fractionEpsilon", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.rayTestBatch(**out_parameters)
        return self.__parseListReturns("getMouseEvents", retArray, keys)

    def getLinkStatePy(self, bodyUniqueId, linkIndex, computeLinkVelocity=None, computeForwardKinematics=None, physicsClientId=None):
        keys = ["linkWorldPosition", "linkWorldOrientation", "localInertialFramePosition", "localInertialFrameOrientation",
                "worldLinkFramePosition", "worldLinkFrameOrientation", "worldLinkLinearVelocity", "worldLinkAngularVelocity"]
        valid_parameters = ["bodyUniqueId", "linkIndex", "computeLinkVelocity", "computeForwardKinematics", "physicsClientId"]
        if computeLinkVelocity is None:
            keys = keys[:6]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        retArray = self.getLinkState(**out_parameters)
        return self.__parseCommonReturn("getLinkState", retArray, keys)

    def setPhysicsEngineParameterPy(
            self, fixedTimeStep=None, numSolverIterations=None, useSplitImpulse=None, splitImpulsePenetrationThreshold=None, numSubSteps=None,
            collisionFilterMode=None, contactBreakingThreshold=None, maxNumCmdPer1ms=None, enableFileCaching=None, restitutionVelocityThreshold=None, erp=None,
            contactERP=None, frictionERP=None, enableConeFriction=None, deterministicOverlappingPairs=None, allowedCcdPenetration=None, jointFeedbackMode=None,
            solverResidualThreshold=None, contactSlop=None, enableSAT=None, constraintSolverType=None, globalCFM=None, minimumSolverIslandSize=None,
            reportSolverAnalytics=None, warmStartingFactor=None, sparseSdfVoxelSize=None, numNonContactInnerIterations=None, physicsClientId=None):
        valid_parameters = [
            "fixedTimeStep", "numSolverIterations", "useSplitImpulse", "splitImpulsePenetrationThreshold", "numSubSteps", "collisionFilterMode",
            "contactBreakingThreshold", "maxNumCmdPer1ms", "enableFileCaching", "restitutionVelocityThreshold", "erp", "contactERP", "frictionERP",
            "enableConeFriction", "deterministicOverlappingPairs", "allowedCcdPenetration", "jointFeedbackMode", "solverResidualThreshold", "contactSlop",
            "enableSAT", "constraintSolverType", "globalCFM", "minimumSolverIslandSize", "reportSolverAnalytics", "warmStartingFactor", "sparseSdfVoxelSize",
            "numNonContactInnerIterations", "physicsClientId"]
        out_parameters = self.__constructOutParameters(locals(), valid_parameters)
        return self.setPhysicsEngineParameter(**out_parameters)
