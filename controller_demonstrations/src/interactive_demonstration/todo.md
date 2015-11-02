TODO for Cyril's project:

 - Have the GetCommand constructor take in a list of available commands. (DONE)
 now: GetCommand() -> knows left/right/faster/...
 desired: GetCommand(['left', 'right', 'up', 'down', 'dance']) -> knows those commands
 - clear self.msg when entering/leaving ReadyState (DONE)
 - Give the dialogue system the ability to list the commands that it knows (in the GetCommand state) (DONE)
 


 - acknowledge substring containing 'command' in ReadyState



  - move the SayState into its own python file (import it to use it)
 - create a smach.StateMachine out of the ChangeSpeed states (the entire branch).
   - The ChangeSpeedMachine will be a single state in the UserInteraction machine
   - Move the ChangeSpeedMachine into its own python file.
 - publish the robot desired speed to a topic: /robot_control/desired_speed
 - publish the command to execute to a topic: /robot_control/desired_command
