import gym
import pygame
import random
import math as m
import numpy as np

from gym import spaces
from pygame import gfxdraw
from scipy.spatial.distance import euclidean


def forward_kinematics(joints_angle, arm_ori, link_size, to_int=False):
  '''
  Parameters:
    * joints_angle : list or tuple of size 2
    * arm_ori : list or tuple of size 2 -> (x=int, y=int)
    * link_size : int
  '''
  x_joint2 = arm_ori[0] + link_size * m.cos(m.radians(joints_angle[0]))
  y_joint2 = arm_ori[1] - link_size * m.sin(m.radians(joints_angle[0]))
  x_eff = x_joint2 + link_size * m.cos(m.radians(joints_angle[0] + joints_angle[1]))
  y_eff = y_joint2 - link_size * m.sin(m.radians(joints_angle[0] + joints_angle[1]))

  if to_int:
    return int(x_joint2), int(y_joint2), int(x_eff), int(y_eff)
  else:
    return x_joint2, y_joint2, x_eff, y_eff


class RobotArmEnvV0(gym.Env):
  def __init__(self):
    self.config = {'viewer': None, 'window_size': [400, 400], 'screen_color': (255, 255, 255),
                   'target_color': (255, 0, 0), 'min_angle': 0, 'max_angle_joint1': 90, 'link_size': 75,
                   'arm_ori': (200, 300), 'rate': 5, 'clock_tick': 60, 'max_angle_joint2': 180}
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

    pygame.init()
  
  def render(self, mode='human', draw_target=True, to_draw=None):
    if self.viewer == None:
      self.viewer = mode

      if mode == 'human':
        pygame.display.set_caption('RobotArm-Env')
        self.screen = pygame.display.set_mode(self.config['window_size'])
        self.clock = pygame.time.Clock()
      else:
        self.screen = pygame.Surface(self.config['window_size'])

    self.screen.fill(self.config['screen_color'])

    if draw_target:
      self.draw_target()
    
    if to_draw is not None:
      self.screen.blit(to_draw, (0, 0))

    self.draw_arm(self.joints_angle)

    if mode == 'human':
      self.clock.tick(self.config['clock_tick'])
      pygame.display.flip()
  
  def get_random_angles(self):
    joint1_angle = random.uniform(self.config['min_angle'], self.config['max_angle_joint1'])
    joint2_angle = random.uniform(self.config['min_angle'], self.config['max_angle_joint2'])
    return [joint1_angle, joint2_angle]
  
  def forward_kinematics(self, joints_angle):
    return forward_kinematics(joints_angle, self.config['arm_ori'], self.config['link_size'])
  
  def inverse_kinematics(self, x, y, keep_joint_angles=True):
    # Put x and y in correct coordinate system
    x = x - self.config['arm_ori'][0]
    y = self.config['arm_ori'][1] - y

    link_size = self.config['link_size']
    joint2_angle = m.acos(np.clip((x**2 + y**2 - 2 * link_size**2) / (2 * link_size**2), -1, 1))
    joint1_angle = m.atan2(y, x+1e-9) - m.atan2(link_size * m.sin(joint2_angle), link_size + link_size * m.cos(joint2_angle))

    joint1_angle = np.clip(m.degrees(joint1_angle), self.config['min_angle'], self.config['max_angle_joint1'])
    joint2_angle = np.clip(m.degrees(joint2_angle), self.config['min_angle'], self.config['max_angle_joint2'])

    if keep_joint_angles:
      self.joints_angle = [joint1_angle, joint2_angle]

    return joint1_angle, joint2_angle
  
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
    self.x_eff, self.y_eff = x_eff, y_eff
  
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
    
    joint1_angle = np.clip(self.joints_angle[0], self.config['min_angle'], self.config['max_angle_joint1'])
    joint2_angle = np.clip(self.joints_angle[1], self.config['min_angle'], self.config['max_angle_joint2'])
    self.joints_angle = [joint1_angle, joint2_angle]

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


class RobotArmEnvV1(gym.Env):
  def __init__(self):
    self.config = {'window_size': [400, 400], 'screen_color': (255, 255, 255), 'rate': 5, 'clock_tick': 120,
                   'target_color': (255, 0, 0), 'min_angle': 0, 'max_angle_joint1': 90, 'link_size': 75,
                   'arm_ori': (200, 300), 'max_angle_joint2': 180, 'only_background': False}

    self.screen = None
    self.clock = None

    self.action = {0: 'HOLD',
                   1: 'INC_J1',
                   2: 'DEC_J1',
                   3: 'INC_J2',
                   4: 'DEC_J2'}
    self.action_space = spaces.Discrete(len(self.action))
  
  def _get_random_joint_angles(self):
    joint1_angle = random.uniform(self.config['min_angle'], self.config['max_angle_joint1'])
    joint2_angle = random.uniform(self.config['min_angle'], self.config['max_angle_joint2'])
    return [joint1_angle, joint2_angle]
  
  def _get_random_target_position(self, to_int=True):
    '''By getting a random arm position we ensure that the target is reachable'''
    _, _, x_eff, y_eff = forward_kinematics(self.joints_angle, self.config['arm_ori'], self.config['link_size'])
    is_too_close = True
    while is_too_close:
      joints_angle = self._get_random_joint_angles()
      _, _, x, y = forward_kinematics(joints_angle, self.config['arm_ori'], self.config['link_size'])
      is_too_close = euclidean((x_eff, y_eff), (x, y)) <= 20
    return (int(x), int(y)) if to_int else (x, y)
  
  def _rotate_link(self, xy_start, xy_end, joint_angle, lw=5):  # lw = link_width
    return [(xo + r * m.sin(joint_angle), yo + r * m.cos(joint_angle))\
              for (xo, yo), r in zip([xy_start, xy_start, xy_end, xy_end], [-lw, lw, lw, -lw])]
  
  def _draw_arm(self):
    x_joint2, y_joint2, x_eff, y_eff = forward_kinematics(self.joints_angle, self.config['arm_ori'],
                                                          self.config['link_size'], to_int=True)
    x_ori, y_ori = self.config['arm_ori']
    # Draw shoulder
    gfxdraw.aacircle(self.surf, x_ori, y_ori, 10, (0, 0, 0))
    gfxdraw.filled_circle(self.surf, x_ori, y_ori, 10, (0, 0, 0))
    # Draw arm
    new_coords = self._rotate_link(self.config['arm_ori'], (x_joint2, y_joint2), m.radians(self.joints_angle[0]))
    gfxdraw.aapolygon(self.surf, new_coords, (0, 0, 0))
    gfxdraw.filled_polygon(self.surf, new_coords, (0, 0, 0))
    # Draw elbow
    gfxdraw.aacircle(self.surf, x_joint2, y_joint2, 10, (0, 0, 0))
    gfxdraw.filled_circle(self.surf, x_joint2, y_joint2, 10, (0, 0, 0))
    # Draw forearm
    new_coords = self._rotate_link((x_joint2, y_joint2), (x_eff, y_eff), m.radians(sum(self.joints_angle)))
    gfxdraw.aapolygon(self.surf, new_coords, (0, 0, 0))
    gfxdraw.filled_polygon(self.surf, new_coords, (0, 0, 0))
    # Draw hand
    gfxdraw.aacircle(self.surf, x_eff, y_eff, 10, (0, 0, 255))
    gfxdraw.filled_circle(self.surf, x_eff, y_eff, 10, (0, 0, 255))
  
  def _draw_target(self):
    gfxdraw.aacircle(self.surf, self.target_pos[0], self.target_pos[1], 10, self.config['target_color'])
    gfxdraw.filled_circle(self.surf, self.target_pos[0], self.target_pos[1], 10, self.config['target_color'])
  
  def get_screen(self):
    return pygame.surfarray.array3d(self.screen).swapaxes(0, 1)

  def reset(self, only_target=False):
    if not only_target:
      self.joints_angle = self._get_random_joint_angles()
    self.target_pos = self._get_random_target_position()

  def step(self, action):
    _, _, x_eff, y_eff = forward_kinematics(self.joints_angle, self.config['arm_ori'], self.config['link_size'])
    prev_dist = euclidean(self.target_pos, (x_eff, y_eff))

    if self.action[action] == "INC_J1":
      self.joints_angle[0] += self.config['rate']
    elif self.action[action] == "DEC_J1":
      self.joints_angle[0] -= self.config['rate']
    elif self.action[action] == "INC_J2":
      self.joints_angle[1] += self.config['rate'] 
    elif self.action[action] == "DEC_J2":
      self.joints_angle[1] -= self.config['rate']
    
    joint1_angle = np.clip(self.joints_angle[0], self.config['min_angle'], self.config['max_angle_joint1'])
    joint2_angle = np.clip(self.joints_angle[1], self.config['min_angle'], self.config['max_angle_joint2'])
    self.joints_angle = [joint1_angle, joint2_angle]

    _, _, x_eff, y_eff = forward_kinematics(self.joints_angle, self.config['arm_ori'], self.config['link_size'])
    current_dist = euclidean(self.target_pos, (x_eff, y_eff))

    reward = 0
    done = False

    if current_dist < 10:
      reward = 10
      done = True
    elif current_dist < prev_dist:
      reward = 1
    
    # observation, reward, done, info
    return self.joints_angle, reward, done, None

  def render(self, mode='human'):
    if self.screen is None:
      if self.config['only_background']:
        self.screen = pygame.Surface(self.config['window_size'])
      else:
        pygame.init()
        pygame.display.init()
        self.screen = pygame.display.set_mode(self.config['window_size'])
    if self.clock is None:
      self.clock = pygame.time.Clock()

    self.surf = pygame.Surface(self.config['window_size'])
    self.surf.fill(self.config['screen_color'])

    self._draw_arm()
    self._draw_target()

    self.screen.blit(self.surf, (0, 0))
    if mode == 'human':
      pygame.event.pump()
      self.clock.tick(self.config["clock_tick"])
      pygame.display.flip()

  def close(self):
    if self.screen is not None:
      pygame.display.quit()
      pygame.quit()