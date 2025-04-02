# README

## Obstacle Avoidance Logic

### Overview
This README explains how the obstacle avoidance functionality was tested and integrated with the final project file.

### File Descriptions
- **avoiding_test.py:** Original testing code that includes obstacle avoidance logic with autonomous forward movement when the environment is clear. This allows the robot to reposition itself autonomously without user commands. This part can be tested by adding into ` test.4.0.py `. The performance of this file was tested in gazebo which I also attach the mp4 file with this folder.
- **avoiding_logic_with_FSM:** Final project obstacle avoidance module where the autonomous forward movement feature was removed. Instead, the robot’s movement is controlled by external commands (such as voice commands made by Xiangyu and motion control made by Botong). It should be used by adding into ` test.5.0.py `

### Integration Process
1. **Incorporating Autonomous Forward Movement:**  
   - The autonomous forward movement feature from the original testing code (` avoiding_test.py `) was reintroduced as a state within the FSM in `test5.0.py`.  
   - This state activates when the environment is determined to be safe, allowing the robot to move forward autonomously.  

2. **Maintaining Compatibility:**  
   - The FSM logic continues to support other control modules such as voice commands by Xiangyu.  
   - Obstacle avoidance remains a priority, with forward movement only occurring when no obstacles are detected by the sonar and cliff sensors.  

### Usage
- Integrate the updated obstacle avoidance logic into `test5.0.py` by replacing the relevant parts.  
- Ensure compatibility with other modules, such as voice control, by properly importing and initializing dependencies.  
