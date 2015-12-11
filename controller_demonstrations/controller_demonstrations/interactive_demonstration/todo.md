TODO for Cyril's project:

 - Have the GetCommand constructor take in a list of available commands. (DONE)
 now: GetCommand() -> knows left/right/faster/...
 desired: GetCommand(['left', 'right', 'up', 'down', 'dance']) -> knows those commands
 - clear self.msg when entering/leaving ReadyState (DONE)
 - Give the dialogue system the ability to list the commands that it knows (in the GetCommand state) (DONE)
 - acknowledge substring containing 'command' in ReadyState (DONE)
 - publish the robot desired speed to a topic: /robot_control/desired_speed (DONE)
 - publish the command to execute to a topic: /robot_control/desired_command (DONE)
 - move the SayState into its own python file (import it to use it) (DONE)
 - create a smach.StateMachine out of the ChangeSpeed states (the entire branch). (DONE)
   - The ChangeSpeedMachine will be a single state in the UserInteraction machine (DONE)
   - Move the ChangeSpeedMachine into its own python file. (DONE)


- viewer:=0 argument in launch file (remove duplicate file) (DONE)
- remove collecting datas branch (DONE)
- adds modification to the state (start and stop) : demonstration (it has to stop until the user say start) (DONE)
make a generic confirmation state: ask yes/no, receive yes/correct/okay vs no/wrong/false (DONE)

- Separate GetConfirmation State into two or three :
1) Listen for command (1 and 2 are doable separately if I succeed using user-data)
2) Ask confirmation
3) Listen for GetConfirmation		--> (DONE)
Read : http://wiki.ros.org/smach/Tutorials/User%20Data (Done)


create new package inside root of nl-dynamics	(DONE)
	user_voice_recording
	- dependencies: pocketsphinx
	- contents; language model (dict ...), launch file (microphone, recognizer)
lots of dictionary updates: yes no -dance front back start stop right wrong true false quit stop done start (DONE)





add 'start over' ability from any state (In development with the reset of every states)




