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

import pybullet


class BaseWrapperPyBullet(object):
    def __generate_source_code(self):
        """Generate the source code for proxying the whole interface. This is only for IntelliSense and Auto Completion."""
        for member in dir(self.p):
            if not member.startswith("_"):
                print(f"self.{member} = p2.{member}")

    def __getattr__(self, name):
        """
        Basic mechanism:
            calling __getattribute__() will not trigger __getattr__(), so we override __getattr__, and calling __getattribute__ accordingly.
        """
        # Check for special member variable `p` and methods implemented in this class
        if name == 'p' or name in type(self).__dict__:
            return self.__getattribute__(self, name)
        # Otherwise, call pybullet's function
        return getattr(self.p, name)

    def __init__(self, p):
        self.p = p

        # Lines below are generated by self.__generate_source_code()
        # This is for IntelliSense and Auto Completion to suggest the original interface.
        # Only need to do this once (by checking the first const variable).
        if "ACTIVATION_STATE_DISABLE_SLEEPING" not in dir():
            import pybullet as p2
            self.ACTIVATION_STATE_DISABLE_SLEEPING = p2.ACTIVATION_STATE_DISABLE_SLEEPING
            self.ACTIVATION_STATE_DISABLE_WAKEUP = p2.ACTIVATION_STATE_DISABLE_WAKEUP
            self.ACTIVATION_STATE_ENABLE_SLEEPING = p2.ACTIVATION_STATE_ENABLE_SLEEPING
            self.ACTIVATION_STATE_ENABLE_WAKEUP = p2.ACTIVATION_STATE_ENABLE_WAKEUP
            self.ACTIVATION_STATE_SLEEP = p2.ACTIVATION_STATE_SLEEP
            self.ACTIVATION_STATE_WAKE_UP = p2.ACTIVATION_STATE_WAKE_UP
            self.AddFileIOAction = p2.AddFileIOAction
            self.B3G_ALT = p2.B3G_ALT
            self.B3G_BACKSPACE = p2.B3G_BACKSPACE
            self.B3G_CONTROL = p2.B3G_CONTROL
            self.B3G_DELETE = p2.B3G_DELETE
            self.B3G_DOWN_ARROW = p2.B3G_DOWN_ARROW
            self.B3G_END = p2.B3G_END
            self.B3G_F1 = p2.B3G_F1
            self.B3G_F10 = p2.B3G_F10
            self.B3G_F11 = p2.B3G_F11
            self.B3G_F12 = p2.B3G_F12
            self.B3G_F13 = p2.B3G_F13
            self.B3G_F14 = p2.B3G_F14
            self.B3G_F15 = p2.B3G_F15
            self.B3G_F2 = p2.B3G_F2
            self.B3G_F3 = p2.B3G_F3
            self.B3G_F4 = p2.B3G_F4
            self.B3G_F5 = p2.B3G_F5
            self.B3G_F6 = p2.B3G_F6
            self.B3G_F7 = p2.B3G_F7
            self.B3G_F8 = p2.B3G_F8
            self.B3G_F9 = p2.B3G_F9
            self.B3G_HOME = p2.B3G_HOME
            self.B3G_INSERT = p2.B3G_INSERT
            self.B3G_LEFT_ARROW = p2.B3G_LEFT_ARROW
            self.B3G_PAGE_DOWN = p2.B3G_PAGE_DOWN
            self.B3G_PAGE_UP = p2.B3G_PAGE_UP
            self.B3G_RETURN = p2.B3G_RETURN
            self.B3G_RIGHT_ARROW = p2.B3G_RIGHT_ARROW
            self.B3G_SHIFT = p2.B3G_SHIFT
            self.B3G_SPACE = p2.B3G_SPACE
            self.B3G_UP_ARROW = p2.B3G_UP_ARROW
            self.CNSFileIO = p2.CNSFileIO
            self.CONSTRAINT_SOLVER_LCP_DANTZIG = p2.CONSTRAINT_SOLVER_LCP_DANTZIG
            self.CONSTRAINT_SOLVER_LCP_PGS = p2.CONSTRAINT_SOLVER_LCP_PGS
            self.CONSTRAINT_SOLVER_LCP_SI = p2.CONSTRAINT_SOLVER_LCP_SI
            self.CONTACT_RECOMPUTE_CLOSEST = p2.CONTACT_RECOMPUTE_CLOSEST
            self.CONTACT_REPORT_EXISTING = p2.CONTACT_REPORT_EXISTING
            self.COV_ENABLE_DEPTH_BUFFER_PREVIEW = p2.COV_ENABLE_DEPTH_BUFFER_PREVIEW
            self.COV_ENABLE_GUI = p2.COV_ENABLE_GUI
            self.COV_ENABLE_KEYBOARD_SHORTCUTS = p2.COV_ENABLE_KEYBOARD_SHORTCUTS
            self.COV_ENABLE_MOUSE_PICKING = p2.COV_ENABLE_MOUSE_PICKING
            self.COV_ENABLE_PLANAR_REFLECTION = p2.COV_ENABLE_PLANAR_REFLECTION
            self.COV_ENABLE_RENDERING = p2.COV_ENABLE_RENDERING
            self.COV_ENABLE_RGB_BUFFER_PREVIEW = p2.COV_ENABLE_RGB_BUFFER_PREVIEW
            self.COV_ENABLE_SEGMENTATION_MARK_PREVIEW = p2.COV_ENABLE_SEGMENTATION_MARK_PREVIEW
            self.COV_ENABLE_SHADOWS = p2.COV_ENABLE_SHADOWS
            self.COV_ENABLE_SINGLE_STEP_RENDERING = p2.COV_ENABLE_SINGLE_STEP_RENDERING
            self.COV_ENABLE_TINY_RENDERER = p2.COV_ENABLE_TINY_RENDERER
            self.COV_ENABLE_VR_PICKING = p2.COV_ENABLE_VR_PICKING
            self.COV_ENABLE_VR_RENDER_CONTROLLERS = p2.COV_ENABLE_VR_RENDER_CONTROLLERS
            self.COV_ENABLE_VR_TELEPORTING = p2.COV_ENABLE_VR_TELEPORTING
            self.COV_ENABLE_WIREFRAME = p2.COV_ENABLE_WIREFRAME
            self.COV_ENABLE_Y_AXIS_UP = p2.COV_ENABLE_Y_AXIS_UP
            self.DIRECT = p2.DIRECT
            self.ER_BULLET_HARDWARE_OPENGL = p2.ER_BULLET_HARDWARE_OPENGL
            self.ER_NO_SEGMENTATION_MASK = p2.ER_NO_SEGMENTATION_MASK
            self.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX = p2.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX
            self.ER_TINY_RENDERER = p2.ER_TINY_RENDERER
            self.ER_USE_PROJECTIVE_TEXTURE = p2.ER_USE_PROJECTIVE_TEXTURE
            self.GEOM_BOX = p2.GEOM_BOX
            self.GEOM_CAPSULE = p2.GEOM_CAPSULE
            self.GEOM_CONCAVE_INTERNAL_EDGE = p2.GEOM_CONCAVE_INTERNAL_EDGE
            self.GEOM_CYLINDER = p2.GEOM_CYLINDER
            self.GEOM_FORCE_CONCAVE_TRIMESH = p2.GEOM_FORCE_CONCAVE_TRIMESH
            self.GEOM_HEIGHTFIELD = p2.GEOM_HEIGHTFIELD
            self.GEOM_MESH = p2.GEOM_MESH
            self.GEOM_PLANE = p2.GEOM_PLANE
            self.GEOM_SPHERE = p2.GEOM_SPHERE
            self.GRAPHICS_CLIENT = p2.GRAPHICS_CLIENT
            self.GRAPHICS_SERVER = p2.GRAPHICS_SERVER
            self.GRAPHICS_SERVER_MAIN_THREAD = p2.GRAPHICS_SERVER_MAIN_THREAD
            self.GRAPHICS_SERVER_TCP = p2.GRAPHICS_SERVER_TCP
            self.GUI = p2.GUI
            self.GUI_MAIN_THREAD = p2.GUI_MAIN_THREAD
            self.GUI_SERVER = p2.GUI_SERVER
            self.IK_DLS = p2.IK_DLS
            self.IK_HAS_JOINT_DAMPING = p2.IK_HAS_JOINT_DAMPING
            self.IK_HAS_NULL_SPACE_VELOCITY = p2.IK_HAS_NULL_SPACE_VELOCITY
            self.IK_HAS_TARGET_ORIENTATION = p2.IK_HAS_TARGET_ORIENTATION
            self.IK_HAS_TARGET_POSITION = p2.IK_HAS_TARGET_POSITION
            self.IK_SDLS = p2.IK_SDLS
            self.JOINT_FEEDBACK_IN_JOINT_FRAME = p2.JOINT_FEEDBACK_IN_JOINT_FRAME
            self.JOINT_FEEDBACK_IN_WORLD_SPACE = p2.JOINT_FEEDBACK_IN_WORLD_SPACE
            self.JOINT_FIXED = p2.JOINT_FIXED
            self.JOINT_GEAR = p2.JOINT_GEAR
            self.JOINT_PLANAR = p2.JOINT_PLANAR
            self.JOINT_POINT2POINT = p2.JOINT_POINT2POINT
            self.JOINT_PRISMATIC = p2.JOINT_PRISMATIC
            self.JOINT_REVOLUTE = p2.JOINT_REVOLUTE
            self.JOINT_SPHERICAL = p2.JOINT_SPHERICAL
            self.KEY_IS_DOWN = p2.KEY_IS_DOWN
            self.KEY_WAS_RELEASED = p2.KEY_WAS_RELEASED
            self.KEY_WAS_TRIGGERED = p2.KEY_WAS_TRIGGERED
            self.LINK_FRAME = p2.LINK_FRAME
            self.MAX_RAY_INTERSECTION_BATCH_SIZE = p2.MAX_RAY_INTERSECTION_BATCH_SIZE
            self.MJCF_COLORS_FROM_FILE = p2.MJCF_COLORS_FROM_FILE
            self.PD_CONTROL = p2.PD_CONTROL
            self.POSITION_CONTROL = p2.POSITION_CONTROL
            self.PosixFileIO = p2.PosixFileIO
            self.RESET_USE_DEFORMABLE_WORLD = p2.RESET_USE_DEFORMABLE_WORLD
            self.RESET_USE_DISCRETE_DYNAMICS_WORLD = p2.RESET_USE_DISCRETE_DYNAMICS_WORLD
            self.RESET_USE_SIMPLE_BROADPHASE = p2.RESET_USE_SIMPLE_BROADPHASE
            self.RemoveFileIOAction = p2.RemoveFileIOAction
            self.SENSOR_FORCE_TORQUE = p2.SENSOR_FORCE_TORQUE
            self.SHARED_MEMORY = p2.SHARED_MEMORY
            self.SHARED_MEMORY_GUI = p2.SHARED_MEMORY_GUI
            self.SHARED_MEMORY_KEY = p2.SHARED_MEMORY_KEY
            self.SHARED_MEMORY_KEY2 = p2.SHARED_MEMORY_KEY2
            self.SHARED_MEMORY_SERVER = p2.SHARED_MEMORY_SERVER
            self.STABLE_PD_CONTROL = p2.STABLE_PD_CONTROL
            self.STATE_LOGGING_ALL_COMMANDS = p2.STATE_LOGGING_ALL_COMMANDS
            self.STATE_LOGGING_CONTACT_POINTS = p2.STATE_LOGGING_CONTACT_POINTS
            self.STATE_LOGGING_CUSTOM_TIMER = p2.STATE_LOGGING_CUSTOM_TIMER
            self.STATE_LOGGING_GENERIC_ROBOT = p2.STATE_LOGGING_GENERIC_ROBOT
            self.STATE_LOGGING_MINITAUR = p2.STATE_LOGGING_MINITAUR
            self.STATE_LOGGING_PROFILE_TIMINGS = p2.STATE_LOGGING_PROFILE_TIMINGS
            self.STATE_LOGGING_VIDEO_MP4 = p2.STATE_LOGGING_VIDEO_MP4
            self.STATE_LOGGING_VR_CONTROLLERS = p2.STATE_LOGGING_VR_CONTROLLERS
            self.STATE_LOG_JOINT_MOTOR_TORQUES = p2.STATE_LOG_JOINT_MOTOR_TORQUES
            self.STATE_LOG_JOINT_TORQUES = p2.STATE_LOG_JOINT_TORQUES
            self.STATE_LOG_JOINT_USER_TORQUES = p2.STATE_LOG_JOINT_USER_TORQUES
            self.STATE_REPLAY_ALL_COMMANDS = p2.STATE_REPLAY_ALL_COMMANDS
            self.TCP = p2.TCP
            self.TORQUE_CONTROL = p2.TORQUE_CONTROL
            self.UDP = p2.UDP
            self.URDF_ENABLE_CACHED_GRAPHICS_SHAPES = p2.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
            self.URDF_ENABLE_SLEEPING = p2.URDF_ENABLE_SLEEPING
            self.URDF_ENABLE_WAKEUP = p2.URDF_ENABLE_WAKEUP
            self.URDF_GLOBAL_VELOCITIES_MB = p2.URDF_GLOBAL_VELOCITIES_MB
            self.URDF_GOOGLEY_UNDEFINED_COLORS = p2.URDF_GOOGLEY_UNDEFINED_COLORS
            self.URDF_IGNORE_COLLISION_SHAPES = p2.URDF_IGNORE_COLLISION_SHAPES
            self.URDF_IGNORE_VISUAL_SHAPES = p2.URDF_IGNORE_VISUAL_SHAPES
            self.URDF_INITIALIZE_SAT_FEATURES = p2.URDF_INITIALIZE_SAT_FEATURES
            self.URDF_MAINTAIN_LINK_ORDER = p2.URDF_MAINTAIN_LINK_ORDER
            self.URDF_MERGE_FIXED_LINKS = p2.URDF_MERGE_FIXED_LINKS
            self.URDF_PRINT_URDF_INFO = p2.URDF_PRINT_URDF_INFO
            self.URDF_USE_IMPLICIT_CYLINDER = p2.URDF_USE_IMPLICIT_CYLINDER
            self.URDF_USE_INERTIA_FROM_FILE = p2.URDF_USE_INERTIA_FROM_FILE
            self.URDF_USE_MATERIAL_COLORS_FROM_MTL = p2.URDF_USE_MATERIAL_COLORS_FROM_MTL
            self.URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL = p2.URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL
            self.URDF_USE_SELF_COLLISION = p2.URDF_USE_SELF_COLLISION
            self.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = p2.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
            self.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT = p2.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
            self.URDF_USE_SELF_COLLISION_INCLUDE_PARENT = p2.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
            self.VELOCITY_CONTROL = p2.VELOCITY_CONTROL
            self.VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS = p2.VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS
            self.VR_BUTTON_IS_DOWN = p2.VR_BUTTON_IS_DOWN
            self.VR_BUTTON_WAS_RELEASED = p2.VR_BUTTON_WAS_RELEASED
            self.VR_BUTTON_WAS_TRIGGERED = p2.VR_BUTTON_WAS_TRIGGERED
            self.VR_CAMERA_TRACK_OBJECT_ORIENTATION = p2.VR_CAMERA_TRACK_OBJECT_ORIENTATION
            self.VR_DEVICE_CONTROLLER = p2.VR_DEVICE_CONTROLLER
            self.VR_DEVICE_GENERIC_TRACKER = p2.VR_DEVICE_GENERIC_TRACKER
            self.VR_DEVICE_HMD = p2.VR_DEVICE_HMD
            self.VR_MAX_BUTTONS = p2.VR_MAX_BUTTONS
            self.VR_MAX_CONTROLLERS = p2.VR_MAX_CONTROLLERS
            self.WORLD_FRAME = p2.WORLD_FRAME
            self.ZipFileIO = p2.ZipFileIO
            self.addUserData = p2.addUserData
            self.addUserDebugLine = p2.addUserDebugLine
            self.addUserDebugParameter = p2.addUserDebugParameter
            self.addUserDebugText = p2.addUserDebugText
            self.applyExternalForce = p2.applyExternalForce
            self.applyExternalTorque = p2.applyExternalTorque
            self.calculateInverseDynamics = p2.calculateInverseDynamics
            self.calculateInverseKinematics = p2.calculateInverseKinematics
            self.calculateInverseKinematics2 = p2.calculateInverseKinematics2
            self.calculateJacobian = p2.calculateJacobian
            self.calculateMassMatrix = p2.calculateMassMatrix
            self.calculateVelocityQuaternion = p2.calculateVelocityQuaternion
            self.changeConstraint = p2.changeConstraint
            self.changeDynamics = p2.changeDynamics
            self.changeTexture = p2.changeTexture
            self.changeVisualShape = p2.changeVisualShape
            self.computeDofCount = p2.computeDofCount
            self.computeProjectionMatrix = p2.computeProjectionMatrix
            self.computeProjectionMatrixFOV = p2.computeProjectionMatrixFOV
            self.computeViewMatrix = p2.computeViewMatrix
            self.computeViewMatrixFromYawPitchRoll = p2.computeViewMatrixFromYawPitchRoll
            self.configureDebugVisualizer = p2.configureDebugVisualizer
            self.connect = p2.connect
            self.createCollisionShape = p2.createCollisionShape
            self.createCollisionShapeArray = p2.createCollisionShapeArray
            self.createConstraint = p2.createConstraint
            self.createMultiBody = p2.createMultiBody
            self.createSoftBodyAnchor = p2.createSoftBodyAnchor
            self.createVisualShape = p2.createVisualShape
            self.createVisualShapeArray = p2.createVisualShapeArray
            self.disconnect = p2.disconnect
            self.enableJointForceTorqueSensor = p2.enableJointForceTorqueSensor
            self.error = p2.error
            self.executePluginCommand = p2.executePluginCommand
            self.getAABB = p2.getAABB
            self.getAPIVersion = p2.getAPIVersion
            self.getAxisAngleFromQuaternion = p2.getAxisAngleFromQuaternion
            self.getAxisDifferenceQuaternion = p2.getAxisDifferenceQuaternion
            self.getBasePositionAndOrientation = p2.getBasePositionAndOrientation
            self.getBaseVelocity = p2.getBaseVelocity
            self.getBodyInfo = p2.getBodyInfo
            self.getBodyUniqueId = p2.getBodyUniqueId
            self.getCameraImage = p2.getCameraImage
            self.getClosestPoints = p2.getClosestPoints
            self.getCollisionShapeData = p2.getCollisionShapeData
            self.getConnectionInfo = p2.getConnectionInfo
            self.getConstraintInfo = p2.getConstraintInfo
            self.getConstraintState = p2.getConstraintState
            self.getConstraintUniqueId = p2.getConstraintUniqueId
            self.getContactPoints = p2.getContactPoints
            self.getDebugVisualizerCamera = p2.getDebugVisualizerCamera
            self.getDifferenceQuaternion = p2.getDifferenceQuaternion
            self.getDynamicsInfo = p2.getDynamicsInfo
            self.getEulerFromQuaternion = p2.getEulerFromQuaternion
            self.getJointInfo = p2.getJointInfo
            self.getJointState = p2.getJointState
            self.getJointStateMultiDof = p2.getJointStateMultiDof
            self.getJointStates = p2.getJointStates
            self.getJointStatesMultiDof = p2.getJointStatesMultiDof
            self.getKeyboardEvents = p2.getKeyboardEvents
            self.getLinkState = p2.getLinkState
            self.getLinkStates = p2.getLinkStates
            self.getMatrixFromQuaternion = p2.getMatrixFromQuaternion
            self.getMeshData = p2.getMeshData
            self.getMouseEvents = p2.getMouseEvents
            self.getNumBodies = p2.getNumBodies
            self.getNumConstraints = p2.getNumConstraints
            self.getNumJoints = p2.getNumJoints
            self.getNumUserData = p2.getNumUserData
            self.getOverlappingObjects = p2.getOverlappingObjects
            self.getPhysicsEngineParameters = p2.getPhysicsEngineParameters
            self.getQuaternionFromAxisAngle = p2.getQuaternionFromAxisAngle
            self.getQuaternionFromEuler = p2.getQuaternionFromEuler
            self.getQuaternionSlerp = p2.getQuaternionSlerp
            self.getUserData = p2.getUserData
            self.getUserDataId = p2.getUserDataId
            self.getUserDataInfo = p2.getUserDataInfo
            self.getVREvents = p2.getVREvents
            self.getVisualShapeData = p2.getVisualShapeData
            self.invertTransform = p2.invertTransform
            self.isConnected = p2.isConnected
            self.isNumpyEnabled = p2.isNumpyEnabled
            self.loadBullet = p2.loadBullet
            self.loadMJCF = p2.loadMJCF
            self.loadPlugin = p2.loadPlugin
            self.loadSDF = p2.loadSDF
            self.loadSoftBody = p2.loadSoftBody
            self.loadTexture = p2.loadTexture
            self.loadURDF = p2.loadURDF
            self.multiplyTransforms = p2.multiplyTransforms
            self.rayTest = p2.rayTest
            self.rayTestBatch = p2.rayTestBatch
            self.readUserDebugParameter = p2.readUserDebugParameter
            self.removeAllUserDebugItems = p2.removeAllUserDebugItems
            self.removeAllUserParameters = p2.removeAllUserParameters
            self.removeBody = p2.removeBody
            self.removeCollisionShape = p2.removeCollisionShape
            self.removeConstraint = p2.removeConstraint
            self.removeState = p2.removeState
            self.removeUserData = p2.removeUserData
            self.removeUserDebugItem = p2.removeUserDebugItem
            self.renderImage = p2.renderImage
            self.resetBasePositionAndOrientation = p2.resetBasePositionAndOrientation
            self.resetBaseVelocity = p2.resetBaseVelocity
            self.resetDebugVisualizerCamera = p2.resetDebugVisualizerCamera
            self.resetJointState = p2.resetJointState
            self.resetJointStateMultiDof = p2.resetJointStateMultiDof
            self.resetJointStatesMultiDof = p2.resetJointStatesMultiDof
            self.resetSimulation = p2.resetSimulation
            self.resetVisualShapeData = p2.resetVisualShapeData
            self.restoreState = p2.restoreState
            self.rotateVector = p2.rotateVector
            self.saveBullet = p2.saveBullet
            self.saveState = p2.saveState
            self.saveWorld = p2.saveWorld
            self.setAdditionalSearchPath = p2.setAdditionalSearchPath
            self.setCollisionFilterGroupMask = p2.setCollisionFilterGroupMask
            self.setCollisionFilterPair = p2.setCollisionFilterPair
            self.setDebugObjectColor = p2.setDebugObjectColor
            self.setDefaultContactERP = p2.setDefaultContactERP
            self.setGravity = p2.setGravity
            self.setInternalSimFlags = p2.setInternalSimFlags
            self.setJointMotorControl = p2.setJointMotorControl
            self.setJointMotorControl2 = p2.setJointMotorControl2
            self.setJointMotorControlArray = p2.setJointMotorControlArray
            self.setJointMotorControlMultiDof = p2.setJointMotorControlMultiDof
            self.setJointMotorControlMultiDofArray = p2.setJointMotorControlMultiDofArray
            self.setPhysicsEngineParameter = p2.setPhysicsEngineParameter
            self.setRealTimeSimulation = p2.setRealTimeSimulation
            self.setTimeOut = p2.setTimeOut
            self.setTimeStep = p2.setTimeStep
            self.setVRCameraState = p2.setVRCameraState
            self.startStateLogging = p2.startStateLogging
            self.stepSimulation = p2.stepSimulation
            self.stopStateLogging = p2.stopStateLogging
            self.submitProfileTiming = p2.submitProfileTiming
            self.syncBodyInfo = p2.syncBodyInfo
            self.syncUserData = p2.syncUserData
            self.unloadPlugin = p2.unloadPlugin
            self.unsupportedChangeScaling = p2.unsupportedChangeScaling
            self.vhacd = p2.vhacd
