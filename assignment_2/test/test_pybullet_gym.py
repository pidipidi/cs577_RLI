import time,gym
gym.logger.set_level(40)
import numpy as np
from pybulletgym.pybullet_envs.bullet.kukaGymEnv import KukaGymEnv

# Load the KUKA Robotic Grasping Environment
env = KukaGymEnv(renders=True, isDiscrete=True)
_ = env.render(mode='human')
env.isRender=True
s = env.reset()
cnt,rsum = 0,0

# Run random movements
while 1:
    cnt += 1 # increase counter
    
    a = env.action_space.sample()
    obs, r, done, _ = env.step(action=a)
    ## print (a, obs)
            
    rsum += r
    still_open = env.render(mode='human')
    if done:
        env.reset()
        
    time.sleep(1./240.)
    #time.sleep(1./10.)
p.disconnect()
env.close()
print ("Done. ravg:[%.3f]"%(rsum/cnt))

