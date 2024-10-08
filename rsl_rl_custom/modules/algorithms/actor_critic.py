#  Copyright 2021 ETH Zurich, NVIDIA CORPORATION
#  SPDX-License-Identifier: BSD-3-Clause


from __future__ import annotations

import torch
import torch.nn as nn
from torch.distributions import Normal
import torch.optim as optim

class ActorCritic(nn.Module):
    is_recurrent = False

    def __init__(
        self,
        num_actor_obs,
        num_critic_obs,
        num_actions,
        actor_hidden_dims=[256, 256, 256],
        critic_hidden_dims=[256, 256, 256],
        activation="elu",
        init_noise_std=1.0,
        **kwargs,
    ):
        if kwargs:
            print(
                "ActorCritic.__init__ got unexpected arguments, which will be ignored: "
                + str([key for key in kwargs.keys()])
            )
        super().__init__()
        activation = get_activation(activation)

        mlp_input_dim_a = num_actor_obs
        mlp_input_dim_c = num_critic_obs
        # Policy
        actor_layers = []
        actor_layers.append(nn.Linear(mlp_input_dim_a, actor_hidden_dims[0]))
        actor_layers.append(activation)
        for layer_index in range(len(actor_hidden_dims)):
            if layer_index == len(actor_hidden_dims) - 1:
                actor_layers.append(nn.Linear(actor_hidden_dims[layer_index], num_actions))
            else:
                actor_layers.append(nn.Linear(actor_hidden_dims[layer_index], actor_hidden_dims[layer_index + 1]))
                actor_layers.append(activation)
        self.actor = nn.Sequential(*actor_layers)

        # Value function
        critic_layers = []
        critic_layers.append(nn.Linear(mlp_input_dim_c, critic_hidden_dims[0]))
        critic_layers.append(activation)
        for layer_index in range(len(critic_hidden_dims)):
            if layer_index == len(critic_hidden_dims) - 1:
                critic_layers.append(nn.Linear(critic_hidden_dims[layer_index], 1))
            else:
                critic_layers.append(nn.Linear(critic_hidden_dims[layer_index], critic_hidden_dims[layer_index + 1]))
                critic_layers.append(activation)
        self.critic = nn.Sequential(*critic_layers)

        # # Safety Value Function
        # self.safety_critics = []
        # for idx in range(n_critics):
        #     safety_critic_layers = []
        #     safety_critic_layers.append(nn.Linear(num_actor_obs + num_actions, safety_critic_hidden_dims[0]))
        #     safety_critic_layers.append(get_activation("relu"))
        #     for layer_index in range(len(safety_critic_hidden_dims)):
        #         if layer_index == len(safety_critic_hidden_dims) - 1:
        #             safety_critic_layers.append(nn.Linear(safety_critic_hidden_dims[layer_index], 1))
        #         else:
        #             safety_critic_layers.append(nn.Linear(safety_critic_hidden_dims[layer_index], safety_critic_hidden_dims[layer_index + 1]))
        #             safety_critic_layers.append(get_activation("relu"))
        #     safety_critic_layers.append(nn.Sigmoid())
        #     safety_critic = nn.Sequential(*safety_critic_layers)
            
        #     self.safety_critics.append(safety_critic)
        # self.safety_critic_optimizer = optim.Adam(self.safety_critic.parameters(), lr=0.003)
        

            
        # self.safety_critic = SafetyCritic(num_actor_obs + num_actions, safety_critic_hidden_dims[0], 1)
        # self.safety_critic_optimizer = optim.Adam(self.safety_critic.parameters(), lr=0.001)


        print(f"Critic MLP: {self.critic}")

        # Action noise
        self.std = nn.Parameter(init_noise_std * torch.ones(num_actions))
        self.distribution = None
        # disable args validation for speedup
        Normal.set_default_validate_args = False

        # seems that we get better performance without init
        # self.init_memory_weights(self.memory_a, 0.001, 0.)
        # self.init_memory_weights(self.memory_c, 0.001, 0.)
        
        self.output_values = []

    @staticmethod
    # not used at the moment
    def init_weights(sequential, scales):
        [
            torch.nn.init.orthogonal_(module.weight, gain=scales[idx])
            for idx, module in enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))
        ]

    def reset(self, dones=None):
        pass

    def forward(self):
        raise NotImplementedError

    @property
    def action_mean(self):
        return self.distribution.mean

    @property
    def action_std(self):
        return self.distribution.stddev

    @property
    def entropy(self):
        return self.distribution.entropy().sum(dim=-1)

    def update_distribution(self, observations):
        mean = self.actor(observations)        # actor 신경망 -> 확률분포얻음 (observed state -> mean of action distribution) 
        self.distribution = Normal(mean, mean * 0.0 + self.std)

    def act(self, observations, **kwargs):
        self.update_distribution(observations) # observation한 state에 따라 확률 분포 업데이트
        return self.distribution.sample()

    def get_actions_log_prob(self, actions):
        return self.distribution.log_prob(actions).sum(dim=-1)

    def act_inference(self, observations):
        actions_mean = self.actor(observations) # 
        return actions_mean

    def evaluate(self, critic_observations, **kwargs):
        value = self.critic(critic_observations)
        return value

    def get_action_samples(self, observations):
        mean = self.actor(observations)
        self.distribution = Normal(mean, mean * 0.0 + self.std)
        return self.distribution.sample()
    
    # 후크(Hook) 함수 정의
    def custom_hook(self, module, input, output):
        self.output_values.append(output)
        
    def regist_hook(self, idx, model: nn.Sequential):
        return model[idx].register_forward_hook(self.custom_hook)
        
    def get_latent_vector(self):
        latent_vector = self.output_values.copy()
        self.output_values = []
        return latent_vector[0]
    
# Policy Network
class Actor(nn.Module):
    is_recurrent = False
    def __init__(
        self,
        num_actor_obs,
        num_critic_obs,
        num_actions,
        actor_hidden_dims=[256, 256, 256],
        activation="elu",
        init_noise_std=1.0,
        **kwargs,
    ):  
        super().__init__()
        activation = get_activation(activation)
        num_of_state = num_actor_obs
        # Policy
        actor_layers = []
        actor_layers.append(nn.Linear(num_of_state, actor_hidden_dims[0]))
        actor_layers.append(activation)
        for layer_index in range(len(actor_hidden_dims)):
            if layer_index == len(actor_hidden_dims) - 1:
                actor_layers.append(nn.Linear(actor_hidden_dims[layer_index], num_actions))
            else:
                actor_layers.append(nn.Linear(actor_hidden_dims[layer_index], actor_hidden_dims[layer_index + 1]))
                actor_layers.append(activation)
        self.actor = nn.Sequential(*actor_layers)
        
        print(f"Actor MLP: {self.actor}")
        
        # Action noise
        self.std = nn.Parameter(init_noise_std * torch.ones(num_actions))
        self.distribution : Normal = None # type: ignore
        # disable args validation for speedup
        Normal.set_default_validate_args = False # type: ignore

    @staticmethod
    # not used at the moment
    def init_weights(sequential, scales):
        [
            torch.nn.init.orthogonal_(module.weight, gain=scales[idx])
            for idx, module in enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))
        ]

    def reset(self, dones=None):
        pass

    def forward(self):
        raise NotImplementedError

    @property
    def action_mean(self):
        return self.distribution.mean 

    @property
    def action_std(self):
        return self.distribution.stddev

    @property
    def entropy(self):
        return self.distribution.entropy().sum(dim=-1)        
    
    def update_distribution(self, observations):
        mean = self.actor(observations)        # actor 신경망 -> 확률분포얻음 (observed state -> mean of action distribution) 
        self.distribution = Normal(mean, mean * 0.0 + self.std)

    def act(self, observations, **kwargs) -> torch.Tensor:
        self.update_distribution(observations) # observation한 state에 따라 확률 분포 업데이트
        return self.distribution.sample()

    def get_actions_log_prob(self, actions):
        return self.distribution.log_prob(actions).sum(dim=-1)

    def act_inference(self, observations):
        actions_mean = self.actor(observations) # 
        return actions_mean
    
    def get_action_samples(self, observations):
        mean = self.actor(observations)
        self.distribution = Normal(mean, mean * 0.0 + self.std)
        return self.distribution.sample()
        
# Value Network
class Critic(nn.Module):
    is_recurrent = False
    def __init__(
        self,
        num_actor_obs,
        num_critic_obs,
        num_actions,
        critic_hidden_dims=[256, 256, 256],
        activation="elu",
        init_noise_std=1.0,
        **kwargs,
    ):  
        super().__init__()
        activation = get_activation(activation)
        num_of_state = num_critic_obs
        # Policy
        critic_layers = []
        critic_layers.append(nn.Linear(num_of_state, critic_hidden_dims[0]))
        critic_layers.append(activation)
        for layer_index in range(len(critic_hidden_dims)):
            if layer_index == len(critic_hidden_dims) - 1:
                critic_layers.append(nn.Linear(critic_hidden_dims[layer_index], 1))
            else:
                critic_layers.append(nn.Linear(critic_hidden_dims[layer_index], critic_hidden_dims[layer_index + 1]))
                critic_layers.append(activation)
        self.critic = nn.Sequential(*critic_layers)
        
        print(f"Critic MLP: {self.critic}")
            
    @staticmethod
    # not used at the moment
    def init_weights(sequential, scales):
        [
            torch.nn.init.orthogonal_(module.weight, gain=scales[idx])
            for idx, module in enumerate(mod for mod in sequential if isinstance(mod, nn.Linear))
        ]

    def reset(self, dones=None):
        pass

    def forward(self):
        raise NotImplementedError

    def evaluate(self, critic_observations, **kwargs):
        value = self.critic(critic_observations)
        return value
        
class SafetyCritic(nn.Module):
    def __init__(
        self, 
        num_obs, 
        num_actions,
        
        activation="relu",
        n_critics: int = 2,
        safety_critic_hidden_dims: list = [256, 256, 256],
    ):
        super(SafetyCritic, self).__init__()
        
        activation = get_activation(activation)
        qvalue_input_dim = num_obs + num_actions
        
        self.safety_networks = []
        for i in range(n_critics):
            safety_layers = []
            safety_layers.append(nn.Linear(qvalue_input_dim, safety_critic_hidden_dims[0]))
            safety_layers.append(activation)
            for layer_index in range(len(safety_critic_hidden_dims)):
                if layer_index == len(safety_critic_hidden_dims) - 1:
                    safety_layers.append(nn.Linear(safety_critic_hidden_dims[layer_index], 1))
                else:
                    safety_layers.append(nn.Linear(safety_critic_hidden_dims[layer_index], safety_critic_hidden_dims[layer_index + 1]))
                    safety_layers.append(activation)
            safety_layers.append(nn.Sigmoid())
            
            safety_network = nn.Sequential(*safety_layers)
            self.safety_networks.append(safety_network)
            
        self.qf0 = self.safety_networks[0]
        self.qf1 = self.safety_networks[1]
        # self.qf0 = nn.Sequential(
        #     nn.Linear(input_size, hidden_size),
        #     nn.ReLU(),
        #     nn.Linear(hidden_size, hidden_size),
        #     nn.ReLU(),
        #     nn.Linear(hidden_size, output_size),
        #     nn.Sigmoid()
        # )
        # self.qf1 = nn.Sequential(
        #     nn.Linear(input_size, hidden_size),
        #     nn.ReLU(),
        #     nn.Linear(hidden_size, hidden_size),
        #     nn.ReLU(),
        #     nn.Linear(hidden_size, output_size),
        #     nn.Sigmoid()
        # )
        # self.q_networks = [self.qf0, self.qf1]
        print(f"Safety Critic MLP: {self.safety_networks}")
    
    def forward(self, qvalue_input):
        return tuple(safety_network(qvalue_input) for safety_network in self.safety_networks)
        # qf0_output = self.qf0(qvalue_input)
        # qf1_output = self.qf1(qvalue_input)
        # return qf0_output, qf1_output

    def compute(self, model, obs, actions, minimum=False):
        qvalue_input = torch.cat([obs, actions], dim=-1)
        with torch.no_grad():
            out = model(qvalue_input)
        out = torch.cat(out, dim=1)
        out, _ = torch.min(out, dim=1, keepdim=True) if minimum else torch.max(out, dim=1, keepdim=True)
        return out        
    
    def forward_safety_critic(self, observations, actions) -> tuple[torch.Tensor, torch.Tensor]:
        qvalue_input = torch.cat([observations, actions], dim=-1)
        return self.forward(qvalue_input)
        # return tuple(safe_net(qvalue_input) for safe_net in self.safety_critic_layers)
        
    def compute_safety_critic(self, observations, actions, minimum=False) -> torch.Tensor:
        qvalue_input = torch.cat([observations, actions], dim=-1)
        with torch.no_grad():
            out = self.forward(qvalue_input)
        out = torch.cat(out, dim=1)
        out, _ = torch.min(out, dim=1, keepdim=True) if minimum else torch.max(out, dim=1, keepdim=True)
        return out

def get_activation(act_name):
    if act_name == "elu":
        return nn.ELU()
    elif act_name == "selu":
        return nn.SELU()
    elif act_name == "relu":
        return nn.ReLU()
    elif act_name == "crelu":
        return nn.LeakyReLU()
    elif act_name == "tanh":
        return nn.Tanh()
    elif act_name == "sigmoid":
        return nn.Sigmoid()
    else:
        print("invalid activation function!")
        return None
