import gym
import ros_foxy_gazebo_gym

env = gym.make('CartPoleEnv')
obs, info = env.reset()
done = False

total_episodes = 1000
episode = 0
ep_reward = 0

while True:
    action = env.action_space.sample()
    observation, reward, done, turc, info = env.step(action[0])
    ep_reward+=reward
    if env.iterator >= env.max_episode_length:
        print(f"episode: {episode}, episode reward: {ep_reward}")
        episode+=1
        ep_reward = 0
        obs, info = env.reset()
    elif done:
        print(f"episode: {episode}, episode reward: {ep_reward}")
        episode+=1
        ep_reward = 0
        obs, info = env.reset()
        
    if episode == total_episodes:
        env.close()
        break
