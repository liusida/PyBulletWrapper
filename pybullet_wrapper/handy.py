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

from .base import BaseWrapperPyBullet

import time
import pybullet_data

COLOR_RED = "\033[0;31m"
COLOR_OFF = "\033[0m"


class HandyPyBullet(BaseWrapperPyBullet):
    # Handy shortcuts implemented here
    def start(self, withGUI=True, withPanels=False, withData=True):
        if withGUI:
            self.connect(self.GUI)
        else:
            self.connect(self.DIRECT)

        if not withPanels:
            self.configureDebugVisualizer(self.COV_ENABLE_GUI, 0)

        if withData:
            self.setAdditionalSearchPath(pybullet_data.getDataPath())

    def sleep(self, n=0.1):
        time.sleep(n)

    def addUserDebugBox(self, twoCorners=[(0, 0, 0), (1, 1, 1)], color=[1, 0, 0], label=""):
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

        for line in lines:
            self.addUserDebugLine(line[0], line[1], lineColorRGB=color)

        if len(label) > 0:
            self.addUserDebugText(label, textPosition=a1)

    # Override getXXX functions to make the result variable using a dictionary
    def checkReturn(self, API, ret, length):
        if (len(ret) != length):
            print(
                f"{COLOR_RED}ERROR: The structure returned from {API} has {len(ret)} items, which is inconsistent with the documentation.{COLOR_OFF}")
            print(f"Please check PyBullet's version. Only 2.8.5 is supported.")
            print("\n")
            print(ret)
            return False
        return True

    def parseCommonReturn(self, API, retArray, keys):
        if self.checkReturn(API, retArray, len(keys)):
            retDictionary = {}
            for i, k in enumerate(keys):
                retDictionary[k] = retArray[i]
            return retDictionary
        return retArray

    def parseListReturns(self, API, retArray, keys):
        if len(retArray) > 0 and self.checkReturn(API, retArray[0], len(keys)):
            retList = []
            for item in retArray:
                retDictionary = {}
                for i, k in enumerate(keys):
                    retDictionary[k] = item[i]
                retList.append(retDictionary)
            return retList
        return retArray

    def getDebugVisualizerCameraPy(self):
        keys = ['width', 'height', 'viewMatrix', 'projectionMatrix', 'cameraUp',
                'cameraForward', 'horizontal', 'vertical', 'yaw', 'pitch', 'dist', 'target']
        retArray = self.getDebugVisualizerCamera()
        return self.parseCommonReturn("getDebugVisualizerCamera", retArray, keys)

    def getMouseEventsPy(self, physicsClientId=0):
        keys = ['eventType', 'mousePosX',
                'mousePosY', 'buttonIndex', 'buttonState']
        retArray = self.getMouseEvents(physicsClientId)
        return self.parseListReturns("getMouseEvents", retArray, keys)

    # getKeyboardEvents() already returns a dictionary, no need to implement handy version.

    def getBasePositionAndOrientationPy(self, objectUniqueId, physicsClientId=0):
        keys = ['position', 'orientation']
        retArray = self.getBasePositionAndOrientation(
            objectUniqueId, physicsClientId)
        return self.parseCommonReturn("getBasePositionAndOrientation", retArray, keys)

    def getJointInfoPy(self, bodyUniqueId, jointIndex, physicsClientId=0):
        keys = ['jointIndex', 'jointName', 'jointType', 'qIndex', 'uIndex', 'flags', 'jointDamping', 'jointFriction', 'jointLowerLimit',
                'jointUpperLimit', 'jointMaxForce', 'jointMaxVelocity', 'linkName', 'jointAxis', 'parentFramePos', 'parentFrameOrn', 'parentIndex']
        retArray = self.getJointInfo(
            bodyUniqueId, jointIndex, physicsClientId)
        return self.parseCommonReturn("getJointInfo", retArray, keys)

    def getJointStatePy(self, bodyUniqueId, jointIndex, physicsClientId=0):
        keys = ['jointPosition', 'jointVelocity',
                'jointReactionForces', 'appliedJointMotorTorque']
        retArray = self.getJointState(
            bodyUniqueId, jointIndex, physicsClientId)
        return self.parseCommonReturn("getJointState", retArray, keys)

    def getJointStatesPy(self, bodyUniqueId, jointIndices, physicsClientId=0):
        keys = ['jointPosition', 'jointVelocity',
                'jointReactionForces', 'appliedJointMotorTorque']
        retArray = self.getJointStates(
            bodyUniqueId, jointIndices, physicsClientId)
        return self.parseListReturns("getJointStates", retArray, keys)

    # Note:
    # According to the source code `pybullet.c`, the returned value has 6 items when computeLinkVelocity is 0, but 8 otherwise.
    # This is different from the documentation.
    def getLinkStatePy(self, bodyUniqueId, linkIndex, computeLinkVelocity=0, computeForwardKinematics=0, physicsClientId=0):
        keys = ['linkWorldPosition', 'linkWorldOrientation', 'localInertialFramePosition', 'localInertialFrameOrientation',
                'worldLinkFramePosition', 'worldLinkFrameOrientation', 'worldLinkLinearVelocity', 'worldLinkAngularVelocity']
        keys_without_computeLinkVelocity = keys[:6]
        retArray = self.getLinkState(
            bodyUniqueId, linkIndex, computeLinkVelocity, computeForwardKinematics, physicsClientId)
        if computeLinkVelocity:
            return self.parseCommonReturn("getLinkState", retArray, keys)
        return self.parseCommonReturn("getLinkState", retArray, keys_without_computeLinkVelocity)

    def getLinkStatesPy(self, bodyUniqueId, linkIndices, computeLinkVelocity=0, computeForwardKinematics=0, physicsClientId=0):
        keys = ['linkWorldPosition', 'linkWorldOrientation', 'localInertialFramePosition', 'localInertialFrameOrientation',
                'worldLinkFramePosition', 'worldLinkFrameOrientation', 'worldLinkLinearVelocity', 'worldLinkAngularVelocity']
        keys_without_computeLinkVelocity = keys[:6]
        retArray = self.getLinkStates(
            bodyUniqueId, linkIndices, computeLinkVelocity, computeForwardKinematics, physicsClientId)
        if computeLinkVelocity:
            return self.parseListReturns("getLinkStates", retArray, keys)
        return self.parseListReturns("getLinkStates", retArray, keys_without_computeLinkVelocity)

    def getConstraintInfoPy(self, constraintUniqueId, physicsClientId=0):
        keys = ['parentBodyUniqueId', 'parentJointIndex', 'childBodyUniqueId', 'childLinkIndex', 'constraintType', 'jointAxis', 'jointPivotInParent', 'jointPivotInChild',
                'jointFrameOrientationParent', 'jointFrameOrientationChild', 'maxAppliedForce', 'gearRatio', 'gearAuxLink', 'relativePositionTarget', 'erp']
        retArray = self.getConstraintInfo(
            constraintUniqueId, physicsClientId)
        return self.parseCommonReturn('getConstraintInfo', retArray, keys)

    def getDynamicsInfoPy(self, bodyUniqueId, linkIndex, physicsClientId=0):
        keys = ['mass', 'lateralFriction', 'localInertiaDiagonal', 'localInertialPos', 'localInertialOrn', 'restitution',
                'rollingFriction', 'spinningFriction', 'contactDamping', 'contactStiffness', 'bodyType', 'collisionMargin']
        retArray = self.getDynamicsInfo(
            bodyUniqueId, linkIndex, physicsClientId)
        return self.parseCommonReturn('getDynamicsInfo', retArray, keys)

    def getCameraImagePy(self, width, height, viewMatrix=None, projectionMatrix=None, lightDirection=None, lightColor=None, lightDistance=-1, shadow=-1, lightAmbientCoeff=-1, lightDiffuseCoeff=-1, lightSpecularCoeff=-1, renderer=-1, flags=-1, physicsClientId=0):
        keys = ['width', 'height', 'rgbPixels',
                'depthPixels', 'segmentationMaskBuffer']
        retArray = self.getCameraImage(width, height, viewMatrix, projectionMatrix, lightDirection, lightColor, lightDistance,
                                       shadow, lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff, renderer, flags, physicsClientId)
        return self.parseCommonReturn('getDynamicsInfo', retArray, keys)

    def getVisualShapeDataPy(self, objectUniqueId, flags=-1, physicsClientId=0):
        keys = ['objectUniqueId', 'linkIndex', 'visualGeometryType', 'dimensions', 'meshAssetFileName',
                'localVisualFramePosition', 'localVisualFrameOrientation', 'rgbaColor', 'textureUniqueId']
        keys_without_VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS = ['objectUniqueId', 'linkIndex', 'visualGeometryType', 'dimensions', 'meshAssetFileName',
                                                             'localVisualFramePosition', 'localVisualFrameOrientation', 'rgbaColor']
        retArray = self.getVisualShapeData(
            objectUniqueId, flags, physicsClientId)
        if (flags & self.VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS):
            return self.parseListReturns('getVisualShapeData', retArray, keys)
        return self.parseListReturns('getVisualShapeData', retArray, keys_without_VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS)

    def getContactPointsPy(self, bodyA=-1, bodyB=-1, linkIndexA=-2, linkIndexB=-2, physicsClientId=0):
        keys = ['contactFlag', 'bodyUniqueIdA', 'bodyUniqueIdB', 'linkIndexA', 'linkIndexB', 'positionOnA', 'positionOnB', 'contactNormalOnB',
                'contactDistance', 'normalForce', 'lateralFriction1', 'lateralFrictionDir1', 'lateralFriction2', 'lateralFrictionDir2']
        retArray = self.getContactPoints(
            bodyA, bodyB, linkIndexA, linkIndexB, physicsClientId)
        return self.parseListReturns("getContactPoints", retArray, keys)

    def getClosestPointsPy(self, bodyA, bodyB, distance, linkIndexA=-2, linkIndexB=-2, physicsClientId=0):
        keys = ['contactFlag', 'bodyUniqueIdA', 'bodyUniqueIdB', 'linkIndexA', 'linkIndexB', 'positionOnA', 'positionOnB', 'contactNormalOnB',
                'contactDistance', 'normalForce', 'lateralFriction1', 'lateralFrictionDir1', 'lateralFriction2', 'lateralFrictionDir2']
        # Note: There are four -1's because according to pybullet.c, there are four unused parameters:
        # collisionShapePositionAObj, collisionShapePositionBObj, collisionShapeOrientationA, and collisionShapeOrientationBObj.
        retArray = self.getClosestPoints(
            bodyA, bodyB, distance, linkIndexA, linkIndexB, -1, -1, -1, -1, physicsClientId)
        return self.parseListReturns("getClosestPoints", retArray, keys)

    def getCollisionShapeDataPy(self, objectUniqueId, linkIndex=-1, physicsClientId=0):
        keys = ['objectUniqueId', 'linkIndex', 'geometryType',
                'dimensions', 'filename', 'localFramePos', 'localFrameOrn']
        retArray = self.getCollisionShapeData(
            objectUniqueId, linkIndex, physicsClientId)
        return self.parseListReturns("getCollisionShapeData", retArray, keys)
