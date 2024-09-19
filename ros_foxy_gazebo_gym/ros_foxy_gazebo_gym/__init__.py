from gym.envs.registration import register

register(
        id='CartPoleEnv',
        entry_point='ros_foxy_gazebo_gym.envs.cartpole_env:CartPoleEnv',
        )
