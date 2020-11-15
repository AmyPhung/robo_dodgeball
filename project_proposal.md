Final Project Proposal
Project Contributors: Nathan, Amy, Ever
Project Concept
What is the main idea of the project?
The main idea of our project is using machine learning to teach a robot how to play dodgeball. The robot will need a camera and the ability to move sideways with respect to its camera. The robot will need to learn a control policy based on its camera feed and commands supplied by a human user. This data will be collected while balls are rolled towards the robot in simulation. 

Our first goals will revolve around initial setup and generation of training data. Initially, we’ll likely use Gazebo’s ground truth to tell us the x,y positions of the balls and train using that data instead of camera data. As we develop our project further, we’ll add more complexity, such as using the camera feed inste ad of ground truth ball positions. Depending on how far we take our project, we may dip into reinforcement learning to have the robot optimize its control policy.
Motivation
Our team is interested in getting more experience with and a deeper understanding of machine learning, particularly as it pertains to figuring out a controls policy for a robot. Although we’re specifically focusing on control policy in this project (i.e. figuring out how to play dodgeball),  being able to “teach” a robot behaviors has numerous real-world applications since it allows for more flexibility and robustness  ranging from “teaching” a robot what it means to navigate through an environment while avoiding obstacles (applicable in self driving cars, search-and-rescue robots, etc.), to “teaching” a robot dexterity to manipulate fragile objects (applicable in scientific sampling robots, 
 
Provide some motivation for your project. Why does your team want to pursue this? What possible applications does the project have? Given our discussions in class, are there possible implications for society? (Note: We’re not expecting a lengthy analysis, but some ideas and consideration.)
Goals
At the conclusion of this project, here are some goals that we’re thinking of achieving. We’re first going to attempt to get the MVP down, then after that, explore different goals as they match our interests (i.e. we won’t do all of them, but we have different directions we might go).
MVP: Have a neato that uses machine learning to detect objects rolling towards it and avoids it given these constraints
Driving only along 1 axis (no turning)
1-2 Objects, straight trajectory
Balls are all the same size
Training data includes ground truth locations of balls relative to the neato, and cmd_vel commands (from human driving)
Output of model will be cmd_vel commands
Model can be run real-time 
DOD (definition of done): A neato drives back and forth, avoiding balls rolling towards it
Level 1: Have a neato use lidar to detect and avoid objects
Driving only on 1 axis
1-2 objects
Lidar data (potentially downsampled) -> neural net -> controller
Level 1: Have a neato use computer vision to detect and avoid objects
Driving only on 1 axis
1-2 objects
Camera input -> binary mask -> neural net -> controller
Level 2: Create a (linear) motion model for everything
Use a kalman filter to continuously keep track of 
Level 2: Have a neato use past data to take into account predictions of motion of the balls
Same as using computer vision or lidar to detect object, but now the neato takes into account past frames
Level 2:
Have a drone learn to strafe away from balls that are thrown up at it
Level 2: Base RL case - avoid 2 balls thrown at you
Figure out RL with gazebo
Build on MVP - it “learns” to avoid balls better (negative cost for hitting obstacles)
Still using hard-coded data
Include a weight towards not moving
Level 3: Next-level RL case - avoid 2 balls thrown at you ~with no training data~
See if RL can figure out control policy from scratch
Level 4: Compare End-to-End learning techniques vs intermediate representations
Level 4: Real roomba and real drone compete in dodgeball

What topics will you explore and what will you generate? What frameworks / algorithms are you planning to explore (do your best to answer this even if things are still fuzzy)? What is your MVP? What are your stretch goals?

Timeline
Week 1: Train a robot to avoid obstacles in “1D case”
(Amy) Get the neato simulator up and running with a camera facing towards the right (send Nathan & ever a link to a fork)
(Nathan) DOD: Write a (parametric) script that generates balls and rolls them in a particular direction at a particular speed
(Amy) Write a script that generates “training data” for supervised learning
Determine how to log training data
Training data includes: positions & velocities of all balls (relative to the neato) and the neato (ground-truth), commands we send to the neato
Hypothesis: Things far away from the neato don’t matter, things near the neato will influence the commands. Pass in the “3 closest objects” - if there’s no object, just include a very large value
(Ever) Set up a data pipeline for machine learning code that takes in training data and outputs a model that can be used with live ROS data (ROS bag to neural network)
(Ever) Set up a website (github site)

Week 2:
Colormask for balls, group into objects, and get centroid and size, replace 
Have the neato drive straight to avoid the balls
feed this into the net
Use camera feed - estimate position of balls with camera info - train a neural net to detect ball positions (train off of gazebo ground truth)
Constrain to only 1 or 2 balls

Week 3+:
tbd
Risks
What do you view as the biggest risks to you being successful on this project?
Gazebo may not lend itself well to ML tasks ie(reinforcement learning doesn’t seem to have tons of support)
Problem can easily become overscoped ie, dodging multiple balls and teaching a more complex robot (drone or omnidirectional)
Our first iterations of supervised model learning may not be effective and we will have to pivot
There are likely to be several ML models being developed, this creates a large demand for training data that could be very time consuming
Questions to ask
RL in gazebo
Omnidirectional bump sensor
Scoping
Pytorch vs Tensorflow
