# sav_gripper
This project provides a pressure regulation of the soft gripper that employs a feed-forward proportional control strategy. Notably, the feed-forward proportional control only sets the PWM signal for the air pump. At the same time, the PWM command of the flight controller directly determines the digital signals for activating solenoid valves.

For details, please see this preprint: H. C. Cheung, C.-W. Chang, B. Jiang, C.-Y. Wen, and H. K. Chu, "A modular pneumatic soft gripper design for aerial grasping and landing," http://arxiv.org/abs/2311.00390 (preprint, submitted to IEEE RoboSoft 2024)
