import math
import pygame
import random
import numpy as np
import gymnasium as gym

from gymnasium import spaces
from pygame import gfxdraw
from scipy.spatial.distance import euclidean
from typing import Tuple, Optional


def forward_kinematics(joints_angle: Tuple[float, float], arm_ori: Tuple[int, int], link_size: int, to_int: bool = False):
    x_joint2 = arm_ori[0] + link_size * math.cos(math.radians(joints_angle[0]))
    y_joint2 = arm_ori[1] - link_size * math.sin(math.radians(joints_angle[0]))
    x_eff = x_joint2 + link_size * math.cos(math.radians(joints_angle[0] + joints_angle[1]))
    y_eff = y_joint2 - link_size * math.sin(math.radians(joints_angle[0] + joints_angle[1]))
    return (int(x_joint2), int(y_joint2), int(x_eff), int(y_eff)) if to_int else (x_joint2, y_joint2, x_eff, y_eff)


class RobotArmEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}
    
    def __init__(self, render_mode: Optional[str] = None):
        self.config = {
            'window_size': (400, 400), 'screen_color': (255, 255, 255), 'rate': 5,
            'clock_tick': 120, 'target_color': (255, 0, 0), 'min_angle': 0,
            'max_angle_joint1': 90, 'link_size': 75, 'arm_ori': (200, 300),
            'max_angle_joint2': 180
        }

        self.action_mapping = {0: 'HOLD', 1: 'INC_J1', 2: 'DEC_J1', 3: 'INC_J2', 4: 'DEC_J2'}
        self.action_space = spaces.Discrete(len(self.action_mapping))
        self.observation_space = spaces.Box(low=0, high=180, shape=(2,), dtype=np.float32)
        
        self.render_mode = render_mode
        self.screen = None
        self.clock = None
        self.joints_angle = self._get_random_joint_angles()
        self.target_pos = self._get_random_target_position()
    
    def get_screen(self):
        return pygame.surfarray.array3d(self.screen).swapaxes(0, 1)  # 400x400x3
    
    def _get_random_joint_angles(self) -> Tuple[float, float]:
        return [random.uniform(self.config['min_angle'], self.config['max_angle_joint1']),
                random.uniform(self.config['min_angle'], self.config['max_angle_joint2'])]
    
    def _get_random_target_position(self) -> Tuple[int, int]:
        while True:
            joints_angle = self._get_random_joint_angles()
            _, _, x, y = forward_kinematics(joints_angle, self.config['arm_ori'], self.config['link_size'])
            if euclidean(forward_kinematics(self.joints_angle, self.config['arm_ori'], self.config['link_size'])[2:], (x, y)) > 20:
                return int(x), int(y)

    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        self.joints_angle = self._get_random_joint_angles()
        self.target_pos = self._get_random_target_position()
        return np.array(self.joints_angle, dtype=np.float32), {}
    
    def step(self, action: int):
        if action == 1:
            self.joints_angle[0] += self.config['rate']
        elif action == 2:
            self.joints_angle[0] -= self.config['rate']
        elif action == 3:
            self.joints_angle[1] += self.config['rate']
        elif action == 4:
            self.joints_angle[1] -= self.config['rate']

        self.joints_angle = [
            np.clip(self.joints_angle[0], self.config['min_angle'], self.config['max_angle_joint1']),
            np.clip(self.joints_angle[1], self.config['min_angle'], self.config['max_angle_joint2'])
        ]
        
        _, _, x_eff, y_eff = forward_kinematics(self.joints_angle, self.config['arm_ori'], self.config['link_size'])
        current_dist = euclidean(self.target_pos, (x_eff, y_eff))
        done = current_dist < 10
        reward = 10 if done else -current_dist / 100  # Encouraging getting closer
        
        return np.array(self.joints_angle, dtype=np.float32), reward, done, False, {}

    def render(self):
        if self.screen is None:
            pygame.init()
            pygame.display.init()
            self.screen = pygame.display.set_mode(self.config['window_size'])
            self.clock = pygame.time.Clock()
            
        if self.render_mode == "human":
            self._draw()
            pygame.display.flip()
            self.clock.tick(self.config["clock_tick"])
        elif self.render_mode == "rgb_array":
            self._draw()
            return pygame.surfarray.array3d(self.screen).swapaxes(0, 1)
    
    def _rotate_link(self, xy_start, xy_end, joint_angle, lw=5):  # lw = link_width
        return [(xo + r * math.sin(joint_angle), yo + r * math.cos(joint_angle))\
                for (xo, yo), r in zip([xy_start, xy_start, xy_end, xy_end], [-lw, lw, lw, -lw])]
        
    def _draw(self):
        self.screen.fill(self.config['screen_color'])
        self._draw_target()
        self._draw_arm()
    
    def _draw_arm(self):
        x_joint2, y_joint2, x_eff, y_eff = forward_kinematics(self.joints_angle, self.config['arm_ori'], self.config['link_size'], to_int=True)
        x_ori, y_ori = self.config['arm_ori']

        gfxdraw.filled_circle(self.screen, x_ori, y_ori, 10, (0, 0, 0))  # Draw shoulder
        gfxdraw.filled_circle(self.screen, x_joint2, y_joint2, 10, (0, 0, 0))  # Draw elbow
        gfxdraw.filled_circle(self.screen, x_eff, y_eff, 10, (0, 0, 255))  # Draw hand

        new_coords = self._rotate_link(self.config['arm_ori'], (x_joint2, y_joint2), math.radians(self.joints_angle[0]))
        gfxdraw.filled_polygon(self.screen, new_coords, (0, 0, 0))  # Draw arm

        new_coords = self._rotate_link((x_joint2, y_joint2), (x_eff, y_eff), math.radians(sum(self.joints_angle)))
        gfxdraw.filled_polygon(self.screen, new_coords, (0, 0, 0))  # Draw forearm
    
    def _draw_target(self):
        gfxdraw.filled_circle(self.screen, self.target_pos[0], self.target_pos[1], 10, self.config['target_color'])
    
    def close(self):
        if self.screen:
            pygame.display.quit()
            pygame.quit()
            self.screen = None