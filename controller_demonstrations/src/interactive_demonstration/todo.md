TODO for Cyril's project:

 - move the SayState into its own python file (import it to use it)
 - create a smach.StateMachine out of the ChangeSpeed states (the entire branch).
   - The ChangeSpeedMachine will be a single state in the UserInteraction machine
   - Move the ChangeSpeedMachine into its own python file.
 - Give the dialogue system the ability to list the commands that it knows (in the GetCommand state)
 - Have the GetCommand constructor take in a list of available commands.
 - publish the robot desired speed to a topic: /robot_control/desired_speed
 - publish the command to execute to a topic: /robot_control/desired_command
