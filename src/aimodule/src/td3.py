import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Implementation of Twin Delayed Deep Deterministic Policy Gradients (TD3)
# Paper: https://arxiv.org/abs/1802.09477


class Actor(nn.Module):

    def __init__(self, action_dim, max_action, max_turn_angle):
        super(Actor, self).__init__()

        flat_size = 32 * 9 * 14

        self.lr = nn.LeakyReLU()
        self.tanh = nn.Tanh()
        self.sigm = nn.Sigmoid()

        self.conv1 = nn.Conv2d(3, 32, 8, stride=2)
        self.conv2 = nn.Conv2d(32, 32, 4, stride=2)
        self.conv3 = nn.Conv2d(32, 32, 4, stride=2)
        self.conv4 = nn.Conv2d(32, 32, 4, stride=1)

        self.bn1 = nn.BatchNorm2d(32)
        self.bn2 = nn.BatchNorm2d(32)
        self.bn3 = nn.BatchNorm2d(32)
        self.bn4 = nn.BatchNorm2d(32)

        self.dropout = nn.Dropout(.5)

        self.lin1 = nn.Linear(flat_size, 512)
        self.lin2 = nn.Linear(512, action_dim)

        self.max_action = max_action
        self.max_turn_angle = max_turn_angle

    def forward(self, x):
        x = self.bn1(self.lr(self.conv1(x)))
        x = self.bn2(self.lr(self.conv2(x)))
        x = self.bn3(self.lr(self.conv3(x)))
        x = self.bn4(self.lr(self.conv4(x)))
        x = x.view(x.size(0), -1)  # flatten
        x = self.dropout(x)
        x = self.lr(self.lin1(x))

        x = self.lin2(x)
        x[:, 0] = self.max_action * self.sigm(x[:, 0])  # because we don't want the duckie to go backwards
        x[:, 1] = self.max_turn_angle * self.tanh(x[:, 1])

        return x


class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()

        flat_size = 32 * 9 * 14

        # Q1 architecture
        self.lr = nn.LeakyReLU()
        self.dropout = nn.Dropout(.5)

        self.conv1 = nn.Conv2d(3, 32, 8, stride=2)
        self.conv2 = nn.Conv2d(32, 32, 4, stride=2)
        self.conv3 = nn.Conv2d(32, 32, 4, stride=2)
        self.conv4 = nn.Conv2d(32, 32, 4, stride=1)

        self.bn1 = nn.BatchNorm2d(32)
        self.bn2 = nn.BatchNorm2d(32)
        self.bn3 = nn.BatchNorm2d(32)
        self.bn4 = nn.BatchNorm2d(32)

        self.lin1 = nn.Linear(flat_size, 256)
        self.lin2 = nn.Linear(256 + action_dim, 128)
        self.lin3 = nn.Linear(128, 1)

        # Q2 architecture
        self.conv5 = nn.Conv2d(3, 32, 8, stride=2)
        self.conv6 = nn.Conv2d(32, 32, 4, stride=2)
        self.conv7 = nn.Conv2d(32, 32, 4, stride=2)
        self.conv8 = nn.Conv2d(32, 32, 4, stride=1)

        self.bn5 = nn.BatchNorm2d(32)
        self.bn6 = nn.BatchNorm2d(32)
        self.bn7 = nn.BatchNorm2d(32)
        self.bn8 = nn.BatchNorm2d(32)

        self.lin4 = nn.Linear(flat_size, 256)
        self.lin5 = nn.Linear(256 + action_dim, 128)
        self.lin6 = nn.Linear(128, 1)

    def forward(self, x, u):
        x1 = self.bn1(self.lr(self.conv1(x)))
        x1 = self.bn2(self.lr(self.conv2(x1)))
        x1 = self.bn3(self.lr(self.conv3(x1)))
        x1 = self.bn4(self.lr(self.conv4(x1)))
        x1 = x1.view(x1.size(0), -1)
        x1 = self.lr(self.lin1(x1))
        x1 = self.lr(self.lin2(torch.cat([x1, u], 1)))
        x1 = self.lin3(x1)

        x2 = self.bn5(self.lr(self.conv5(x)))
        x2 = self.bn6(self.lr(self.conv6(x2)))
        x2 = self.bn7(self.lr(self.conv7(x2)))
        x2 = self.bn8(self.lr(self.conv8(x2)))
        x2 = x2.view(x2.size(0), -1)
        x2 = self.lr(self.lin4(x2))
        x2 = self.lr(self.lin5(torch.cat([x2, u], 1)))
        x2 = self.lin6(x2)

        return x1, x2

    def Q1(self, x, u):
        x1 = self.bn1(self.lr(self.conv1(x)))
        x1 = self.bn2(self.lr(self.conv2(x1)))
        x1 = self.bn3(self.lr(self.conv3(x1)))
        x1 = self.bn4(self.lr(self.conv4(x1)))
        x1 = x1.view(x1.size(0), -1)
        x1 = self.lr(self.lin1(x1))
        x1 = self.lr(self.lin2(torch.cat([x1, u], 1)))
        x1 = self.lin3(x1)

        return x1


class TD3(object):
    def __init__(self, state_dim, action_dim, max_action, max_turn_angle):
        self.actor = Actor(action_dim, max_action, max_turn_angle).to(device)
        self.actor_target = Actor(action_dim, max_action, max_turn_angle).to(device)
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=1e-5)

        self.critic = Critic(state_dim, action_dim).to(device)
        self.critic_target = Critic(state_dim, action_dim).to(device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=1e-3)

        self.max_action = max_action
        self.max_turn_angle = max_turn_angle

    def select_action(self, state):
        state = torch.FloatTensor(np.expand_dims(state, axis=0)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()

    def save(self, filename, directory):
        torch.save(self.actor.state_dict(), '%s/%s_actor.pth' % (directory, filename))
        torch.save(self.critic.state_dict(), '%s/%s_critic.pth' % (directory, filename))

    def load(self, filename, directory):
        self.actor.load_state_dict(torch.load('%s/%s_actor.pth' % (directory, filename)))
        self.critic.load_state_dict(torch.load('%s/%s_critic.pth' % (directory, filename)))

    def load_actor(self, filename, directory):
        self.actor.load_state_dict(torch.load('%s/%s_actor.pth' % (directory, filename)))
