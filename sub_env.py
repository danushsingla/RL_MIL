import gym
from gym import spaces
import numpy as np
import os

# Shape of the image. L, W, # of channels
SHAPE = [50,80,3]

#to be finished
class SubEnv(gym.Env):
    def __init__(self):
	#For camera rgb obs space
        self.observation_space = spaces.Dict({
	spaces.Box(low=0, high=255, shape=(SHAPE[0], SHAPE[1], SHAPE[2]), dtype=uint8) 

	#linear orientation
	spaces.Box(low=-50, high=50, shape=(3,)
	#angular orientation
	sapce.Box(low=-360, high=360, shape=(3,)

	#must still do linear+angular velcoty 
    def _get_obs(self):
        # Get image from RL_subscriber through pipe
        with open("image_pipe", 'rb') as pipe:
            img_data = pipe.read(SHAPE[0]*SHAPE[1]*SHAPE[2])
            if img_data:
                image = np.frombuffer(img_data, dtype=np.uint8).reshape((SHAPE[0], SHAPE[1], SHAPE[2]))
                return image


# Register the environment in the gym
register(
    id="SubjugatorAgent-v0",
    entry_point=SubEnv
)
