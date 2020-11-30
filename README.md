This repo holds code for the final course project in Computational Introduction to Robotics, FA 2020. The code is authored by Amy Phung, Everardo Gonzalez, and Nathan Faber.

Project Website: https://everardog.github.io/ml_comprobofinal/

# Dependencies:
+ inputs (for joystick controller) `pip install inputs`
+ Tensorflow `pip install tensorflow`

# Usage
+ `roslaunch ml_comprobo record_training_data`
    + Args:
       + `dodgeball_prefix` default: "ball"
       + `robot_name` default: "mobile_base"
       + `num_dodgeballs` default: "2"
       + `use_joystick` default: "true"
    + When using the joystick:
       + press a to start recording and start ball spawner
       + press b to stop and save dataset
       + use the left joystick to drive robot forwards/backwards
    + When using the keyboard:
       + press s to start recording and start ball spawner
       + press spacebar to stop and save dataset
       + press i or . to drive forwards/backwards

# TODO:
+ make velocity relative to robot coords
