# ROS Turtlebot3 Reinforcement learning with OpenAI Gymnasium API

This project was build using Pyton 3, Open AI Gymnasium environment, on Linux Ubuntu 20.04 LTS and with use of Stable Baselines package.<br>
Robot was trained using PyTorch framework with reinforced learning techniqe and Tensorboard for showing the data.


## Description
This projects main goal was to train Turtlebot mobile robot using neural network. Main robot's task was to reach the destination point in specjaly prepared arena without coliding with any obstacle. 
Turtlebot movement based on 2 data input: lidar - laser sensor mounted on top of the robot providing information about the surroundings, odometry - data whick commes from robots wheels that measures distance traveled by each of wheels providing information about robot's localization in the environment. Based on theese data, neural network was to determine the appropriate path for the robot. Model was trained using model free algorithms and particular of them was tested during traingin robot. Robots main source of information about quality of activities was reward function which, which had to be construced well enough for the model to be abble to discover which actions were desirable and which were not.

Reseach environment was ROS and Gazebo. <br>
ROS stands for Robot Operating System and its open source environment in Linux operating system created for robotic solutions. It commes with plenty of useful tools and packages which contains drivers and models for training. It also provide with all sensory such as lidar and odometry.<br>
Gazebo is a simulator that offers advanced physics for robots and environments. It was created to ensure conditions as close to reality as possible.<br>

Becouse Open AI with Gymnasium Environment was created with prototyping in mind, the goal of the project was not fully achieved for tooday. Plenty of alghoritms and reward functions were applied, but robot was never fully abble to learn how to move in environment with obstacles. To be abble to train robot in the way that he fully controlls his actions, another set of tools and new knowledge is required. 

## Test results

Selected test results are presented below. This graph shows reward-time ratio where x axis is training iterations numbers (aka. time - 1 iteration took about 0.02 - 0.1s) and y axis is episode (all iterations till falure or reaching goal) reward sum. 

1. No obstacle, diffrent alhorithms and reward functions.

   - Training algorithm - DQN (Deep Q-Network)
   - Only odometry data with no lidar.
   - Training time - 9h
   - Reward function - rewarding model for looking directly at the target and moving toward it.
   - Observations - Very long training time, model was finding "optimal" action set by spinning around.
   ![test_res_1](https://github.com/user-attachments/assets/788adaa8-5414-4b9a-a60c-dd1770b4da5f)

2. No obstacle, diffrent alhorithms and reward functions.
   - Training algorithm - DQN
   - Only odometry data with no lidar.
   - Training time - 1.5h
   - Reward function - rewarding model only for looking directly at the target.
   - Observations - after a while, robot started driving around its own axis.
   ![test_res_2](https://github.com/user-attachments/assets/4ca2e72d-7785-418f-80b2-ed0281118761)

3. No obstacle, PPO (Proximal policy optimization).
   - Training algorithm - Pink - PPO, Blue - DQN
   - Only odometry data with no lidar.
   - Training time - Blue - 3h, Pink - 4h
   - Reward function - rewarding model only for looking directly at the target.
   - Observations - model trained with PPO algorith was succesful and reached goal of arriving to the target in every iteration.
     ![test_res_3](https://github.com/user-attachments/assets/c81df426-d452-4ca3-b68b-896bd03441e5)

    
4. No obstacle, PPO and additional element in observation matrix.
   - Training algorith - PPO
   - Yelow - only odometry, Pink - odometry and lidar data.
   - Training time - Yelow - 4h, Pink - 7.5h
   - Reward function - rewarding model only for looking directly at the target
   - Observations - even better results after much longer training time with bigger observation matrix.<br>
   Model ready for testing in environment with obstacles.
     ![test_res_4](https://github.com/user-attachments/assets/4d61d9f2-83f4-4499-860f-4d7f0143437a)

5. Single obstacle on the way to the target, training model aquired in previous test.
   - Training algorith - PPO
   - Odometry and lidar data
   - Training time - 4.5h (longer time due to bigger observation matrix)
   - Reward function - rewarding model for getting closer to the target and subtracting points for colision with obstacle and getting too close to the obstacle.
   - Observations - for training time, model has shown no progress and even regressed. Visualy, no metodology in robot's movement could be observed.
   ![test_res_5](https://github.com/user-attachments/assets/c3a3ba6e-4d7c-4fd8-9261-17219bd2deef)

There were many other tests for environment with obstacles performed by none of them showed significant improvement. For further development, an algorithm designed specificly for this task is needed. 
