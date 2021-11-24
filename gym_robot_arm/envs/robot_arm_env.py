import gym
import pygame
import random
import math as m
import numpy as np

from gym import error, spaces
from scipy.spatial.distance import euclidean


class RobotArmEnvV0(gym.Env):
  def __init__(self):
    self.config = {'viewer': None, 'window_size': [400, 400], 'screen_color': (255, 255, 255),
                   'target_color': (255, 0, 0), 'min_angle': 0, 'max_angle': 90, 'link_size': 75,
                   'arm_ori': (200, 300), 'rate': 1, 'clock_tick': 60}
    self.viewer = self.config['viewer']

    self.target_pos = self.get_random_pos()
    self.joints_angle = self.get_random_angles()

    self.action = {0: 'HOLD',
                   1: 'INC_J1',
                   2: 'DEC_J1',
                   3: 'INC_J2',
                   4: 'DEC_J2',
                   5: 'INC_J1_J2',
                   6: 'DEC_J1_J2',
                   7: 'INC_J1_DEC_J2',
                   8: 'DEC_J1_INC_J2'}
    self.action_space = spaces.Discrete(len(self.action))
  
  def render(self, mode='human'):
    if self.viewer == None:
      pygame.init()
      pygame.display.set_caption('RobotArm-Env')
      self.screen = pygame.display.set_mode(self.config['window_size'])
      self.clock = pygame.time.Clock()
      self.viewer = 'HUMAN'
    self.screen.fill(self.config['screen_color'])
    self.draw_target()
    self.draw_arm(self.joints_angle)
    self.clock.tick(self.config['clock_tick'])
    pygame.display.flip()
  
  def get_random_angles(self):
    joint1_angle = random.uniform(self.config['min_angle'], self.config['max_angle'])
    joint2_angle = random.uniform(self.config['min_angle'], self.config['max_angle'])
    return [joint1_angle, joint2_angle]
  
  def forward_kinematics(self, joints_angle):
    x_joint2 = self.config['arm_ori'][0] + self.config['link_size'] * m.cos(m.radians(joints_angle[0]))
    y_joint2 = self.config['arm_ori'][1] - self.config['link_size'] * m.sin(m.radians(joints_angle[0]))
    x_eff = x_joint2 + self.config['link_size'] * m.cos(m.radians(joints_angle[0] + joints_angle[1]))
    y_eff = y_joint2 - self.config['link_size'] * m.sin(m.radians(joints_angle[0] + joints_angle[1]))
    return x_joint2, y_joint2, x_eff, y_eff
  
  def get_random_pos(self):
    joints_angle = self.get_random_angles()
    _, _, x, y = self.forward_kinematics(joints_angle)
    return x, y
  
  def draw_target(self):
    pygame.draw.circle(self.screen, self.config['target_color'], self.target_pos, 10)
  
  def draw_arm(self, joints_angle):
    x_joint2, y_joint2, x_eff, y_eff = self.forward_kinematics(joints_angle)
    pygame.draw.circle(self.screen, (0, 0, 0), self.config['arm_ori'], 10)
    pygame.draw.line(self.screen, (0, 0, 0), self.config['arm_ori'], (x_joint2, y_joint2), width=5)
    pygame.draw.circle(self.screen, (0, 0, 0), (x_joint2, y_joint2), 10)
    pygame.draw.line(self.screen, (0, 0, 0), (x_joint2, y_joint2), (x_eff, y_eff), width=5)
    pygame.draw.circle(self.screen, (0, 0, 255), (x_eff, y_eff), 10)
  
  def step(self, action):
    _, _, x_eff, y_eff = self.forward_kinematics(self.joints_angle)
    prev_dist = euclidean(self.target_pos, (x_eff, y_eff))

    if self.action[action] == "INC_J1":
      self.joints_angle[0] += self.config['rate']
    elif self.action[action] == "DEC_J1":
      self.joints_angle[0] -= self.config['rate']
    elif self.action[action] == "INC_J2":
      self.joints_angle[1] += self.config['rate'] 
    elif self.action[action] == "DEC_J2":
      self.joints_angle[1] -= self.config['rate']
    elif self.action[action] == "INC_J1_J2":
      self.joints_angle[0] += self.config['rate']
      self.joints_angle[1] += self.config['rate'] 
    elif self.action[action] == "DEC_J1_J2":
      self.joints_angle[0] -= self.config['rate']
      self.joints_angle[1] -= self.config['rate']
    elif self.action[action] == "INC_J1_DEC_J2":
      self.joints_angle[0] += self.config['rate']
      self.joints_angle[1] -= self.config['rate']
    elif self.action[action] == "DEC_J1_INC_J2":
      self.joints_angle[0] -= self.config['rate']
      self.joints_angle[1] += self.config['rate']
    
    self.joints_angle = np.clip(self.joints_angle, self.config['min_angle'], self.config['max_angle'])

    _, _, x_eff, y_eff = self.forward_kinematics(self.joints_angle)
    current_dist = euclidean(self.target_pos, (x_eff, y_eff))
    
    reward = -1
    done = False

    if current_dist < 10:
      reward = 100
      done = True
    elif current_dist < prev_dist:
      reward = 1

    info = {}
    observation = pygame.surfarray.array3d(self.screen).swapaxes(0, 1)  # WHC -> HWC
    return observation, reward, done, info
  
  def reset(self):
    self.target_pos = self.get_random_pos()
    self.current_score = 0
    observation = pygame.surfarray.array3d(self.screen).swapaxes(0, 1)  # WHC -> HWC
    return observation
  
  def close(self):
    if self.viewer != None:
      pygame.quit()
  
  def get_screen(self):
    return pygame.surfarray.array3d(self.screen).swapaxes(0, 1)