Copyright (c) 2018 Nikhil Barhate
import torch
import torch.nn as nn
from torch.distributions import MultivariateNormal
import levelling
import gym
import numpy as np
import pybulletgym
import pybulletgym.envs
import time
import csv
import pybullet as  p
import sys
import os
os.environ['KMP_DUPLICATE_LIB_OK']='True'

filename = "results.csv"
fields = ['reward', 'batch']
rows = []
f = open("results.csv", "w")


device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


class Memory:
    def __init__(self):
        self.actions = []
        self.states = []
        self.logprobs = []
        self.rewards = []
        self.is_terminals = []
    
    def clear_memory(self):
        del self.actions[:]
        del self.states[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.is_terminals[:]

class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, action_std):
        super(ActorCritic, self).__init__()
        # action mean range -1 to 1
        self.actor =  nn.Sequential(
                nn.Linear(11, 64),
                nn.Tanh(),
                nn.Linear(64, 32),
                nn.Tanh(),
                nn.Linear(32, action_dim),
                nn.Tanh()
                )
        # critic
        self.critic = nn.Sequential(
                nn.Linear(11, 64),
                nn.Tanh(),
                nn.Linear(64, 32),
                nn.Tanh(),
                nn.Linear(32, 1)
                )
        self.action_var = torch.full((action_dim,), action_std*action_std).to(device)
        
    def forward(self):
        raise NotImplementedError
    
    def act(self, state, memory):
        action_mean = self.actor(state)
        cov_mat = torch.diag(self.action_var).to(device)
        
        dist = MultivariateNormal(action_mean, cov_mat)
        action = dist.sample()
        action_logprob = dist.log_prob(action)
        
        memory.states.append(state)
        memory.actions.append(action)
        memory.logprobs.append(action_logprob)
        
        return action.detach()
    
    def evaluate(self, state, action):   
        action_mean = self.actor(state)
        
        action_var = self.action_var.expand_as(action_mean)
        cov_mat = torch.diag_embed(action_var).to(device)
        
        dist = MultivariateNormal(action_mean, cov_mat)
        
        action_logprobs = dist.log_prob(action)
        dist_entropy = dist.entropy()
        state_value = self.critic(state)
        
        return action_logprobs, torch.squeeze(state_value), dist_entropy

class PPO:
    def __init__(self, state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip):
        self.lr = lr
        self.betas = betas
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.K_epochs = K_epochs
        
        self.policy = ActorCritic(state_dim, action_dim, action_std).to(device)
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=lr, betas=betas)
        
        self.policy_old = ActorCritic(state_dim, action_dim, action_std).to(device)
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        self.MseLoss = nn.MSELoss()
    
    def select_action(self, state, memory):
        state = torch.FloatTensor(state.reshape(1, -1)).to(device)
        return self.policy_old.act(state, memory).cpu().data.numpy().flatten()
    
    def update(self, memory):
        # Monte Carlo estimate of rewards:
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(memory.rewards), reversed(memory.is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
        
        # Normalizing the rewards:
        rewards = torch.tensor(rewards, dtype=torch.float32).to(device)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-5)
        
        # convert list to tensor
        old_states = torch.squeeze(torch.stack(memory.states).to(device), 1).detach()
        old_actions = torch.squeeze(torch.stack(memory.actions).to(device), 1).detach()
        old_logprobs = torch.squeeze(torch.stack(memory.logprobs), 1).to(device).detach()
        e = 0
        # Optimize policy for K epochs:
        for _ in range(self.K_epochs):
            e+=1
            # Evaluating old actions and values :
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            print("epoch",e)
            # Finding the ratio (pi_theta / pi_theta__old):
            ratios = torch.exp(logprobs - old_logprobs.detach())

            # Finding Surrogate Loss:
            advantages = rewards - state_values.detach()   
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2) + 0.5*self.MseLoss(state_values, rewards) - 0.01*dist_entropy
            
            # take gradient step
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()
            
            
        # Copy new weights into old policy:
        self.policy_old.load_state_dict(self.policy.state_dict())
        
def main():
    ############## Hyperparameters ##############
    env_name = "LevellingPyBullet-v0"
    render = True
    solved_reward = 200000000000000        # stop training if avg_reward > solved_reward
    log_interval = 1    # print avg reward in the interval
    max_episodes = 100000000        # max training episodes
    max_timesteps = 1000      # max timesteps in one episode
    
    update_timestep = 32000 # update policy every n timesteps
    action_std = 0.5            # constant std for action distribution (Multivariate Normal)
    K_epochs = 100           # update policy for K epochs
    eps_clip = 0.2             # clip parameter for PPO
    gamma = 0.99           # discount factor
    
    lr = 0.0003               # parameters for Adam optimizer
    betas = (0.9, 0.999)
    
    random_seed = 11       #############################################
    
    # creating environment
    env = gym.make(env_name)
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    print("action dimesion:",action_dim)
    averagerew = []
    breaker = -1000
    breaklist = []
    
    if random_seed:
        print("Random Seed: {}".format(random_seed))
        torch.manual_seed(random_seed)
        env.seed(random_seed)
        np.random.seed(random_seed)
    
    memory = Memory()
    ppo = PPO(state_dim, action_dim, action_std, lr, betas, gamma, K_epochs, eps_clip)
    print(lr,betas)
    
    # logging variables
    running_reward = 0
    avg_length = 0
    time_step = 0
    global c
    c = 0
    eplist=[]
    csvlist = []
    batch_number=1
    # training loop
    for i_episode in range(1, max_episodes+1):
        env.render(mode='human')
        state = env.reset()
        c+=1
        
        for t in range(max_timesteps):
            time_step +=1
            action = ppo.select_action(state, memory)
            action = action
            
            #print(action)
            env.step(action, c)
            state, reward, done, c, _ = env.step(action, c)
            if c%200==0:
                time.sleep(0.02)
            memory.rewards.append(reward)
            memory.is_terminals.append(done)
            if time_step % update_timestep == 0 and breaker<1500000:
                batch_number+=1
                ppo.update(memory)
                memory.clear_memory()
                time_step = 0
            running_reward += (reward)
            
            if c%200==0:
                print(reward)


            if done:
                break
            
        avg_length += t
        
        # stop training if avg_reward > solved_reward
        if breaker > (log_interval*solved_reward):
            print("########## Solved! ##########")
            torch.save(ppo.policy.state_dict(), './PPO_continuous_solved_{}.pth'.format(env_name))
            break
        
        # save every 500 episodes
        if i_episode % 500 == 0:
            torch.save(ppo.policy.state_dict(), './PPO_continuous_{}.pth'.format(env_name))
            
        # logging


        avg_length = float(avg_length/log_interval)
        averagerew.append(running_reward)
        batchlist = []
        log_freq=32
        if c%log_freq==0:
            print("")
            print("Batch number: ",batch_number)
            print("Last", log_freq, "episodes average reward: ",np.mean(averagerew))
            print("Minium reward last", log_freq ,"episodes: ",min(averagerew))
            print("Maxium reward last", log_freq, "episodes: ",max(averagerew))
            print('Episode {} \t Avg length: {} \t Latest reward: {}'.format(i_episode, avg_length, running_reward))
            csvlist.append(np.mean(averagerew))
            breaklist.append(np.mean(averagerew))
            batchlist.append(batch_number)
            eplist.append(c)
            with open("results.csv", "a") as f:
                      writer = csv.writer(f, delimiter=';')
                      writer.writerows(zip(batchlist,eplist,csvlist))
            csvlist.clear()
            batchlist.clear()
            print("max so far: ",max(breaklist))
            breaker = float(max(breaklist))
            eplist.clear()
            averagerew.clear()
        running_reward = 0
        avg_length = 0
        #if c%50==0:
            #print("Episode: ",c)

            
            
if __name__ == '__main__':
    main()

#p.disconnect()


    