# Taken from https://pytorch.org/tutorials/intermediate/reinforcement_q_learning.html and modified
import os
import gym
import math
import torch
import random
import torchvision
import numpy as np
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

from PIL import Image
from tqdm import tqdm
from itertools import count
from collections import namedtuple, deque


class DataTransformer(object):
  def __init__(self):
    self.to_tensor = torchvision.transforms.ToTensor()
  
  def __call__(self, image):  # [400, 400, 3]
    image = self.to_tensor(image)  # [3, 400, 400]
    image = image[:, 125:325, 30:370]  # CROP -> [3, 200, 340]
    image = F.interpolate(image.unsqueeze(0), size=(100, 170))
    return image  # add batch_dim -> BCHW


class DQN(nn.Module):
    def __init__(self, h, w, outputs):
        super(DQN, self).__init__()
        self.conv1 = nn.Conv2d(3, 16, kernel_size=5, stride=2)
        self.bn1 = nn.BatchNorm2d(16)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, stride=2)
        self.bn2 = nn.BatchNorm2d(32)
        self.conv3 = nn.Conv2d(32, 32, kernel_size=5, stride=2)
        self.bn3 = nn.BatchNorm2d(32)

        # Number of Linear input connections depends on output of conv2d layers
        # and therefore the input image size, so compute it.
        def conv2d_size_out(size, kernel_size = 5, stride = 2):
            return (size - (kernel_size - 1) - 1) // stride  + 1
        convw = conv2d_size_out(conv2d_size_out(conv2d_size_out(w)))
        convh = conv2d_size_out(conv2d_size_out(conv2d_size_out(h)))
        linear_input_size = convw * convh * 32
        self.head = nn.Linear(linear_input_size, outputs)

    # Called with either one element to determine next action, or a batch
    # during optimization. Returns tensor([[left0exp,right0exp]...]).
    def forward(self, x):
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        return self.head(x.view(x.size(0), -1))


class ReplayMemory(object):
    def __init__(self, capacity):
        self.memory = deque([], maxlen=capacity)

    def push(self, *args):
        """Save a transition"""
        self.memory.append(Transition(*args))

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)


class Trainer(object):
    def __init__(self, config):
        base_config = {'batch_size': 128, 'gamma': 0.999, 'eps_start': 0.9, 'eps_end': 0.05, 'eps_decay': 200,
                       'target_update': 10, 'memory_size': 10000, 'n_episodes': 50, 'max_iter_ep': 500,
                       'model_path': 'arm_dqn_model.pt', 'n_eval_ep': 10}
        self.config = {**base_config, **config}

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self.instanciate_params()
        self.instanciate_model()

        self.steps_done = 0
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=1e-5)
        self.criterion = nn.SmoothL1Loss()
    
    def instanciate_params(self):
        self.env = gym.make('gym_robot_arm:robot-arm-v0')
        self.env.render()
        self.dt = DataTransformer()

        init_screen = self.dt(self.env.get_screen())
        _, _, self.screen_height, self.screen_width = init_screen.shape

        self.n_actions = self.env.action_space.n

        self.memory = ReplayMemory(self.config['memory_size'])
    
    def instanciate_model(self):
        self.policy_net = DQN(self.screen_height, self.screen_width, self.n_actions).to(self.device)
        self.target_net = DQN(self.screen_height, self.screen_width, self.n_actions).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()
    
    def select_action(self, state, training=True):
        if training:
            sample = random.random()
            eps_threshold = self.config['eps_end'] + (self.config['eps_start'] - self.config['eps_end']) *\
                            math.exp(-1. * self.steps_done / self.config['eps_decay'])
            self.steps_done += 1

            if sample > eps_threshold:
                with torch.no_grad():
                    # t.max(1) will return largest column value of each row.
                    # second column on max result is index of where max element was
                    # found, so we pick action with the larger expected reward.
                    return self.policy_net(state.to(self.device)).max(1)[1].view(1, 1)
            else:
                return torch.tensor([[random.randrange(self.n_actions)]], device=self.device, dtype=torch.long)
        else:
            with torch.no_grad():
                return self.policy_net(state.to(self.device)).max(1)[1].view(1, 1)
    
    def optimize_model(self):
        if len(self.memory) < self.config['batch_size']:
            return
        transitions = self.memory.sample(self.config['batch_size'])
        # Transpose the batch (see https://stackoverflow.com/a/19343/3343043 for
        # detailed explanation). This converts batch-array of Transitions
        # to Transition of batch-arrays.
        batch = Transition(*zip(*transitions))

        # Compute a mask of non-final states and concatenate the batch elements
        # (a final state would've been the one after which simulation ended)
        non_final_mask = torch.tensor(tuple(map(lambda s: s is not None, batch.next_state)), device=self.device, dtype=torch.bool)
        non_final_next_states = torch.cat([s for s in batch.next_state if s is not None])
        state_batch = torch.cat(batch.state)
        action_batch = torch.cat(batch.action)
        reward_batch = torch.cat(batch.reward)

        # Compute Q(s_t, a) - the model computes Q(s_t), then we select the
        # columns of actions taken. These are the actions which would've been taken
        # for each batch state according to policy_net
        state_action_values = self.policy_net(state_batch.to(self.device)).gather(1, action_batch)

        # Compute V(s_{t+1}) for all next states.
        # Expected values of actions for non_final_next_states are computed based
        # on the "older" target_net; selecting their best reward with max(1)[0].
        # This is merged based on the mask, such that we'll have either the expected
        # state value or 0 in case the state was final.
        next_state_values = torch.zeros(self.config['batch_size'], device=self.device)
        next_state_values[non_final_mask] = self.target_net(non_final_next_states.to(self.device)).max(1)[0].detach()
        # Compute the expected Q values
        expected_state_action_values = (next_state_values * self.config['gamma']) + reward_batch

        # Compute Huber loss
        loss = self.criterion(state_action_values, expected_state_action_values.unsqueeze(1))

        # Optimize the model
        self.optimizer.zero_grad()
        loss.backward()
        for param in self.policy_net.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()
    
    def train(self):
        for i_episode in tqdm(range(self.config['n_episodes'])):
            # Initialize the environment and state
            self.env.reset()
            self.env.render()
            # last_screen = dt(env.get_screen())
            # current_screen = dt(env.get_screen())
            # state = current_screen - last_screen
            state = self.dt(self.env.get_screen())
            # for t in count():
            for t in range(self.config['max_iter_ep']):
                # Select and perform an action
                action = self.select_action(state)
                _, reward, done, _ = self.env.step(action.item())
                self.env.render()
                reward = torch.tensor([reward], device=self.device)

                # Observe new state
                # last_screen = current_screen
                # current_screen = dt(env.get_screen())
                if not done:
                    # next_state = current_screen - last_screen
                    next_state = self.dt(self.env.get_screen())
                else:
                    next_state = None
                
                # img = Image.fromarray((next_state.squeeze(0).permute(1, 2, 0)*255).numpy().astype(np.uint8), mode='RGB')
                # img.show()
                # input('wait')

                # Store the transition in memory
                self.memory.push(state, action, next_state, reward)

                # Move to the next state
                state = next_state

                # Perform one step of the optimization (on the policy network)
                self.optimize_model()
                if done:
                    break
            # Update the target network, copying all weights and biases in DQN
            if i_episode % self.config['target_update'] == 0:
                self.target_net.load_state_dict(self.policy_net.state_dict())

        self.env.close()

        self.save_model()
    
    @torch.no_grad()
    def evaluation(self):
        self.policy_net.eval()

        for i_episode in range(self.config['n_eval_ep']):
            self.env.reset()
            self.env.render()
            state = self.dt(self.env.get_screen())

            for t in range(self.config['max_iter_ep']):
                with torch.no_grad():
                    action = self.select_action(state, training=False)
                _, reward, done, _ = self.env.step(action.item())
                self.env.render()

                state = self.dt(self.env.get_screen())

                if done:
                    break

        self.policy_net.train()
    
    def load_model(self, path=None):
        path = self.config['model_path'] if path is None else path
        self.policy_net.load_state_dict(torch.load(path))
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()
    
    def save_model(self, path=None):
        path = self.config['model_path'] if path is None else path
        torch.save(self.policy_net.state_dict(), path)


if __name__ == "__main__":
    Transition = namedtuple('Transition', ('state', 'action', 'next_state', 'reward'))
    trainer = Trainer({})

    rep = input('Load trained model? (y or n): ')
    if rep == 'y':
        trainer.load_model()

    rep = input('Train network? (y or n): ')
    if rep == 'y':
        trainer.train()

    rep = input('Eval network? (y or n): ')
    if rep == 'y':
        trainer.evaluation()