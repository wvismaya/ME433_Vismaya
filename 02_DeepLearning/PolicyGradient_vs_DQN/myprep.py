import sys
import gym
from pathlib import Path
import pylab
import math
import matplotlib.pyplot as plt
import numpy as np
import random
from collections import deque
from keras.layers import Dense
from keras.models import Sequential
from keras.optimizers import Adam
from keras.optimizers import SGD
from keras.models import load_model
#keras.optimizers.SGD(lr=0.01, momentum=0.0, decay=0.0, nesterov=False)

#monte-carlo update rule
discount_factor = 0.75
learning_rate = 10**-3
hidden1, hidden2, hidden3 = 24, 24, 24

epsilon = 1.
##[0,1)
epsilon_decay = 0.999
epsilon_min = 0.01
batch_size = 64
train_start = 1000

dqn_memory = deque(maxlen=2000)

# define Python user-defined exceptions to log out of EPISODE loop
class Error(Exception):
   pass

class SuccessReached(Error):
   pass

# Ensuring the same model remains for both strategies
# Build Model
def build_model(state_sizex):
    model = Sequential()
    model.add(Dense(hidden1, input_dim=state_sizex, activation='relu', kernel_initializer='glorot_uniform'))
    model.add(Dense(hidden2, activation='relu', kernel_initializer='glorot_uniform'))
    model.add(Dense(hidden3, activation='relu', kernel_initializer='glorot_uniform'))
    model.add(Dense(action_size, activation='softmax', kernel_initializer='glorot_uniform'))
    model.summary()
    model.compile(loss="categorical_crossentropy", optimizer=Adam(lr=learning_rate))
    return model

# update policy network every episode
def train_model_policy(modelx, statesx, actionsx, rewardsx):
    episode_length = len(statesx)
    discounted_rewards = np.zeros_like(rewardsx)
    running_add = 0
    for t in reversed(range(0, len(rewardsx))):
        running_add = running_add*discount_factor + rewardsx[t]
        discounted_rewards[t] = running_add
    discounted_rewards = discounted_rewards*1.0
    ## Calculate expected value
    discounted_rewards -= np.mean(discounted_rewards)
    discounted_rewards /= np.std(discounted_rewards)
    update_inputs = np.zeros((episode_length, state_size))
    advantages = np.zeros((episode_length, action_size))
    
    for i in range(episode_length):
        update_inputs[i] = statesx[i]
        advantages[i][actionsx[i]] = discounted_rewards[i]
        modelx.fit(update_inputs, advantages, epochs=1, verbose=0)

# update for DQN
def train_model_dqn(modelx, tmodelx, memx):
    global batch_size
    if len(memx) < train_start:
        return
    batch_size = min(batch_size, len(memx))
    mini_batch = random.sample(memx, batch_size)
    
    update_input = np.zeros((batch_size, state_size))
    update_target = np.zeros((batch_size,state_size))
    action, reward, done = [], [], []
    
    for i in range(batch_size):
        update_input[i] = mini_batch[i][0]
        action.append(mini_batch[i][1])
        reward.append(mini_batch[i][2])
        update_target[i] = mini_batch[i][3]
        done.append(mini_batch[i][4])
        
    target = modelx.predict(update_input)
    target_val = tmodelx.predict(update_target)
    
    for i in range(batch_size):
        if done[i]:
            target[i][action[i]] = reward[i]
        else:
            target[i][action[i]] = reward[i] + discount_factor * (np.amax(target_val[i]))
    
    modelx.fit(update_input, target, batch_size=batch_size, epochs=1, verbose=0)


######################################
#### REFERENCES
######################################
## Woongwon, Youngmoo, Hyeokreal, Uiryeong, Keon, "Minimal and Clean Reinforcement Learning Examples", (2017 commit), Github Repository , https://github.com/rlcode/reinforcement-learning
## Giuseppe Bonaccorso, "cartpole.py" (2016), Github Code , https://gist.github.com/giuseppebonaccorso/7040b10a13520c4b0340b8a89dc8262f
## Henry Jia, "OpenAI gym cartpole" (2016), Github Repository, https://github.com/HenryJia/cartpole
## Abhishek Mishra, "pg_rnn" (2017), Github Repository, https://github.com/abhishm/pg_rnn 
######################################