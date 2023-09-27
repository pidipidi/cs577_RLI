import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
os.sys.path.insert(0, currentdir)


import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p
from . import kuka
import random
import pybullet_data
from pkg_resources import parse_version

largeValObservation = 100

RENDER_HEIGHT = 720
RENDER_WIDTH = 960


class KukaGymEnv(gym.Env):
  metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}
  """Class for Kuka environment with a block object.

  In each episode one block object is randomly placed for grasping.
  """

  def __init__(self,
               urdfRoot=pybullet_data.getDataPath(),
               actionRepeat=1,
               isEnableSelfCollision=True,
               renders=False,
               isDiscrete=True,
               maxSteps=1000):
    #print("KukaGymEnv __init__")
    self._isDiscrete = isDiscrete
    self._timeStep = 1. / 240.
    self._urdfRoot = urdfRoot
    self._actionRepeat = actionRepeat
    self._isEnableSelfCollision = isEnableSelfCollision
    self._observation = []
    self._envStepCounter = 0
    self._renders = renders
    self._maxSteps = maxSteps
    self.terminated = 0
    self._cam_dist = 1.3
    self._cam_yaw = 180
    self._cam_pitch = -40

    self._p = p
    if self._renders:
      cid = p.connect(p.SHARED_MEMORY)
      if (cid < 0):
        cid = p.connect(p.GUI)
      p.resetDebugVisualizerCamera(1.3, 180, -41, [0.52, -0.2, -0.33])
    else:
      p.connect(p.DIRECT)
    #timinglog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "kukaTimings.json")
    self.seed()
    self.reset()

    # Determine the observation space
    observationDim = len(self.getExtendedObservation())
    observation_high = np.array([largeValObservation] * observationDim)
    self.observation_space = spaces.Box(-observation_high, observation_high)

    # Determine the action space
    if (self._isDiscrete):
      self.action_space = spaces.Discrete(7)
    else:
      action_dim = 3
      self._action_bound = 1
      action_high = np.array([self._action_bound] * action_dim)
      self.action_space = spaces.Box(-action_high, action_high)
      
    self.viewer = None

  def reset(self):
    """ Reset KukaGymEnv """
    self.terminated = 0
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    p.setTimeStep(self._timeStep)
    p.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"), [0, 0, -1])

    p.loadURDF(os.path.join(self._urdfRoot, "table/table.urdf"), 0.5000000, 0.00000, -.820000,
               0.000000, 0.000000, 0.0, 1.0)

    # Randomize the pose of the target block
    xpos = 0.5 + 0.1 * random.random()
    ypos = -0.1 + 0.3 * random.random() #-0.1 + 0.2 * random.random() #
    ang = 3.14 * 0.5 - 3.1415925438 * random.random()
    orn = p.getQuaternionFromEuler([0, 0, ang])
    self.blockUid = p.loadURDF(os.path.join(self._urdfRoot, "block.urdf"), xpos, ypos, -0.15,
                               orn[0], orn[1], orn[2], orn[3])

    p.setGravity(0, 0, -10)
    self._kuka = kuka.Kuka(urdfRootPath=self._urdfRoot, timeStep=self._timeStep)
    self._envStepCounter = 0
    p.stepSimulation()
    p.stepSimulation()
    p.stepSimulation()
    self._observation = self.getExtendedObservation()
    return np.array(self._observation)

  def __del__(self):
    p.disconnect()

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def getExtendedObservation(self):
    """ Return the observation """
    ## self._observation = self._kuka.getObservation()
    gripperState = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaGripperIndex,1)
    gripperPos = gripperState[0]
    gripperOrn = gripperState[1]
    ## gripperLinearVel = gripperState[6] # If you need, you can use
    ## gripperAngularVel = gripperState[7] # If you need, you can use
    blockPos, blockOrn = p.getBasePositionAndOrientation(self.blockUid)

    invGripperPos, invGripperOrn = p.invertTransform(gripperPos, gripperOrn)
    gripperMat = p.getMatrixFromQuaternion(gripperOrn)
    dir0 = [gripperMat[0], gripperMat[3], gripperMat[6]]
    dir1 = [gripperMat[1], gripperMat[4], gripperMat[7]]
    dir2 = [gripperMat[2], gripperMat[5], gripperMat[8]]

    gripperEul = p.getEulerFromQuaternion(gripperOrn)
    blockPosInGripper, blockOrnInGripper = p.multiplyTransforms(invGripperPos, invGripperOrn,
                                                                blockPos, blockOrn)
    projectedBlockPos2D = [blockPosInGripper[0], blockPosInGripper[1]]
    blockEulerInGripper = p.getEulerFromQuaternion(blockOrnInGripper)

    # --------------------------------------------------------------
    # Place your code here! (Define your observation and scale it)
    # --------------------------------------------------------------
    
    #we return the relative x,y position and euler angle of block in gripper space
    blockInGripperPosXYEulZ = [ (np.exp(10.*blockPosInGripper[0])-1.),
                                (np.exp(10.*blockPosInGripper[1])-1.),
                                (blockEulerInGripper[2])]

    blockInGripperPosXYEulZ[2] += np.pi/2.0
    if blockInGripperPosXYEulZ[2] > np.pi/2.:
      blockInGripperPosXYEulZ[2] -= np.pi
    if blockInGripperPosXYEulZ[2] < -np.pi/2.:
      blockInGripperPosXYEulZ[2] += np.pi

    # scaling
    blockInGripperPosXYEulZ[2] /= np.pi/2.
    blockInGripperPosXYEulZ[2] *= 10.
      
    # Define observations
    self._observation = list(blockInGripperPosXYEulZ) 
    # --------------------------------------------------------------
    
    return self._observation

  
  def step(self, action, **kwargs):
    """ 
    A function to apply a selected action to the robot end effector.

    Args:
      action: 1 integer action index or 
      3-vector parameterizing XYZ offset, vertical angle offset
      (radians), and grasp angle (radians).
    Returns:
      observation: Next observation.
      reward: Float of the per-step reward as a result of taking the action.
      done: Bool of whether or not the episode has ended.
      debug: Dictionary of extra information provided by environment.
    """
    
    if (self._isDiscrete):
      # For discrete action
      # --------------------------------------------------------------
      # Place your code here! (Define actions)
      # --------------------------------------------------------------
      dv = 0.01 # velocity per physics step.
      # desired position displacement along x axis 
      dx = [0, -dv, dv, 0, 0, 0, 0][action]
      # desired position displacement along y axis 
      dy = [0, 0, 0, -dv, dv, 0, 0][action]
      # desired rotation displacement along yaw axis 
      da = [0, 0, 0, 0, 0, -0.1, 0.1][action]
      # --------------------------------------------------------------

      # Finger
      f = 0.3
      realAction = [dx, dy, -0.005, da, f]
    else:
      # For continuous action, we rescale the action command
      # for convenience.
      dv = 0.005
      dx = action[0] * dv
      dy = action[1] * dv
      da = action[2] * 0.05
      f = 0.3
      realAction = [dx, dy, -0.005, da, f]
    return self.step2(realAction)

  
  def step2(self, action, **kwargs):
    """ 
    A function to apply a displacement command to the end effector.

    Parameters
    ----------
    action : list
        a list of displacement command for [x, y, z, rz, f]
    """
    
    for i in range(self._actionRepeat):
      self._kuka.applyAction(action)
      
      p.stepSimulation()
      p.stepSimulation()
      if self._termination():
        break
      self._envStepCounter += 1
    if self._renders:
      time.sleep(self._timeStep)
    self._observation = self.getExtendedObservation()

    done = self._termination()

    # --------------------------------------------------------------
    # Place your code here! (Define action cost if you want)
    # --------------------------------------------------------------
    
    if (self._isDiscrete):
      npaction = np.array([action[0], action[1], action[3]])
      if npaction.any()>0:
        actionCost = 0.1
      else:
        actionCost = 0
    else:
      npaction = np.array([action[0], action[1], action[3]])
      actionCost = np.linalg.norm(npaction)* 10. 
      
    reward = self._reward() #- actionCost
    # --------------------------------------------------------------
    
    return np.array(self._observation), reward, done, {}

  def render(self, mode="rgb_array", close=False):
    if mode != "rgb_array":
      return np.array([])

    base_pos, orn = self._p.getBasePositionAndOrientation(self._kuka.kukaUid)
    view_matrix = self._p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=base_pos,
                                                            distance=self._cam_dist,
                                                            yaw=self._cam_yaw,
                                                            pitch=self._cam_pitch,
                                                            roll=0,
                                                            upAxisIndex=2)
    proj_matrix = self._p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
                                                     nearVal=0.1,
                                                     farVal=100.0)
    (_, _, px, _, _) = self._p.getCameraImage(width=RENDER_WIDTH,
                                              height=RENDER_HEIGHT,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=self._p.ER_BULLET_HARDWARE_OPENGL)
    #renderer=self._p.ER_TINY_RENDERER)

    rgb_array = np.array(px, dtype=np.uint8)
    rgb_array = np.reshape(rgb_array, (RENDER_HEIGHT, RENDER_WIDTH, 4))

    rgb_array = rgb_array[:, :, :3]
    return rgb_array

  def _termination(self):
    """ 
    A function to determine the termination status and execute post actions.
    """
    # Get the gripper position
    state = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaGripperIndex)
    actualGripperPos = state[0]

    # Prevent to step over the max step count
    if (self.terminated or self._envStepCounter > self._maxSteps):
      self._observation = self.getExtendedObservation()
      return True

    # Threshold hold to determine the tray contact
    maxDist = 0.01 
    closestPoints = p.getClosestPoints(self._kuka.trayUid, self._kuka.kukaUid, maxDist)

    # If the gripper is very close to the tray, attempt to grasp the object 
    if (len(closestPoints)>0 ): 
      self.terminated = 1

      ## print("terminating, closing gripper, attempting grasp with height {}".format(actualGripperPos[2]))
      # Start grasp and terminate
      fingerAngle = 0.3
      for i in range(100):
        graspAction = [0, 0, 0.0001, 0, fingerAngle]
        self._kuka.applyAction(graspAction)
        p.stepSimulation()
        fingerAngle = fingerAngle - (0.3 / 100.)
        if (fingerAngle < 0):
          fingerAngle = 0

      for i in range(1000):
        graspAction = [0, 0, 0.001, 0, fingerAngle]
        self._kuka.applyAction(graspAction)
        p.stepSimulation()
        blockPos, blockOrn = p.getBasePositionAndOrientation(self.blockUid)
        if (blockPos[2] > 0.2):
          break
        state = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaGripperIndex)
        actualGripperPos = state[0]
        if (actualGripperPos[2] >= 0.3):
          break

      self._observation = self.getExtendedObservation()
      return True
    return False

  def _reward(self):
    """ A function to return a reward value. """
    
    # You can use any of following information to compose your reward
    blockPos, blockOrn = p.getBasePositionAndOrientation(self.blockUid)
    closestPoints = p.getClosestPoints(self.blockUid, self._kuka.kukaUid, 1000, -1,
                                       self._kuka.kukaGripperIndex)
    reward = 0
    numPt  = len(closestPoints)
    
    #------------------------------------------------------------
    # Place your code here (Define your reward)
    # -----------------------------------------------------------
    # OPTION: add distance-based penalty
    ## if (numPt > 0):
    ##   reward = -(closestPoints[0][8])**2  #* 10

    # OPTION: add collision/workspace penalty
    ## if obs[0]>0.6 or obs[0]<0.5: reward -= 0.1
    ## if obs[1]>0.22 or obs[1]<-0.22: reward -= 0.1
    
    # Add a sparse reward (i.e., lifting the grasped block)    
    if (blockPos[2] > 0):
      reward += 1000 #100 #10000
      #print("successfully grasped a block!!! reward={}".format(reward))
    #------------------------------------------------------------
    
    return reward

  if parse_version(gym.__version__) < parse_version('0.9.6'):
    _render = render
    _reset = reset
    _seed = seed
    _step = step

  def get_block_pos(self):
    blockPos, blockOrn = p.getBasePositionAndOrientation(self.blockUid)
    return blockPos
