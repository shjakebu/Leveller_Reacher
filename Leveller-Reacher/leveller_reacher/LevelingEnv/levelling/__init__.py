from gym.envs.registration import register
register(
    id='LevellingPyBullet-v0',
    entry_point='levelling.envs:LevellingPyBulletEnv'
)
