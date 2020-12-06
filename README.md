This repo holds code for the final course project in Computational Introduction to Robotics, FA 2020. The code is authored by Amy Phung, Everardo Gonzalez, and Nathan Faber.

Project Website: https://everardog.github.io/ml_comprobofinal/

# Dependencies:
+ inputs (for joystick controller) `pip3 install inputs`
+ LSTM model:
    + Tensorflow `pip3 install tensorflow`
    + If you get an error message about cuda when importing, run this: `sudo apt install nvidia-cuda-toolkit`
+ Standard model:
    + PyTorch `pip3 install torch`
    + Sklearn `pip3 install sklearn`

# Usage
+ To record dataset: `roslaunch ml_comprobo record_training_data`
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
+ To train dataset: `python3 train_LTSM.py`
   + Double-check the file to ensure paths are set correctly
+ To test dataset in gazebo: `roslaunch ml_comprobo run_model.launch`
   + Make sure the model arg is set correctly in the launch file

# TODO:
+ make velocity relative to robot coords
+ split training and data recorder
+ Add param to auto-start ball spawn
