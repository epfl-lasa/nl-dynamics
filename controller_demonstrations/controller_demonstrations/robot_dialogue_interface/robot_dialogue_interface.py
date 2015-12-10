import rospy


class RobotDialogueInterface(object):

    def __init__(self):

        # Things the robot knows how to do. map of string -> stuff.
        self._known_commands = {}

    def known_commands(self):
        """
        :return: List of known commands (strings).
        """
        return self._known_commands.keys()

    def record_command(self, command):
        """
        Record a new command using _robot_record_command and then store it if
        the returned information is valid.

        :param command: String command.
        :return: True if command was added
        """
        ret = self._robot_record_command(command)
        if ret:
            rospy.loginfo('Adding a command for [{}]'.format(command))
            self._known_commands[command] = ret
            return True
        rospy.loginfo('Recording command {} failed'.format(command))
        return False

    def execute_command(self, command):
        """
        Execute a command if it is known.
        :param command: String version of the command.
        :return: False if command is unknown, otherwise the result of
        _robot_do_command.
        """

        stuff_to_do = self._known_commands.get(command, None)

        if stuff_to_do is None:
            return False
        return self._robot_do_command(stuff_to_do)

    def change_speed(self, speed):
        """
        Change the speed of the robot.
        :param speed: New speed.
        :return:
        """
        return self._robot_set_speed(speed)

    def _robot_do_command(self, *args, **kwargs):
        """
        Implement this to make the robot do stuff.
        :param args:
        :param kwargs:
        :return: True if command was executed, false otherwise.
        """
        raise NotImplementedError('Implement this in each robot interface.')

    def _robot_record_command(self, *args, **kwargs):
        """
        Implement this to make the robot record stuff. This should return the
        recorded stuff to store (i.e. what the robot will execute later time).
        :param args:
        :param kwargs:
        :return:
        """
        raise NotImplementedError('Implement this in each robot interface.')

    def _robot_set_speed(self, *args, **kwargs):
        """
        Implement this to make the robot set the speed.
        :param args:
        :param kwargs:
        :return:
        """
        raise NotImplementedError('Implement this in each robot interface.')


def run():
    print 'Robot dialogue interface'
    interface = RobotDialogueInterface()

if __name__ == '__main__':
    run()
