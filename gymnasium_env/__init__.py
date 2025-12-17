from gymnasium.envs.registration import register

register(
    id="RobotArmEnv-v0",
    entry_point="gymnasium_env.envs:RobotArmEnv",
)