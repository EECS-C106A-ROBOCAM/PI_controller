# PI_controller

## Instruction

### Arduino
1. Edit `ServoControl.ino` found in `Arduino/ServoControl/` to include additional servos
2. Upload `ServoControlino` to the Arduino

### ROS
1. Launch `arudino_command_as_launch.launch`
2. Using `rostopic pub`, publish desired servo angles via the terminal to the topic `/PI_controller/goal`
