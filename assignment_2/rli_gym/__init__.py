from gym.envs.registration import register

register(
    id='reaching-v0',
    entry_point='rli_gym.envs:ReachingEnv2D',
)

register(
    id='reaching-v1',
    entry_point='rli_gym.envs:ReachingEnv2D_v1'
)

