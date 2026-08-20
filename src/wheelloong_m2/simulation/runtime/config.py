"""The only frequency configuration for the offline teleoperation simulation."""

TARGET_HZ = 120
IK_HZ = 250
SIMULATION_HZ = 1000

TARGET_PERIOD_S = 1.0 / TARGET_HZ
IK_PERIOD_S = 1.0 / IK_HZ
SIMULATION_PERIOD_S = 1.0 / SIMULATION_HZ
