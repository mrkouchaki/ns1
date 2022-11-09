#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import gym
import tensorflow as tf
#import tensorflow.contrib.slim as slim
import tf_slim as slim
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from tensorflow import keras
import argparse
import ns3gym
from ns3gym import ns3env

#env = gym.make('ns3-v0')
env = ns3env.Ns3Env()
ob_space = env.observation_space
ac_space = env.action_space
print("Observation space: ", ob_space,  ob_space.dtype)
print("Action space: ", ac_space, ac_space.n)
