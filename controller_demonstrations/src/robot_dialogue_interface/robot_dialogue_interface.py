
class RobotDialogueInterface(object):

    def __init__(self):

        # Things the robot knows how to do. map of string -> stuff.
        self._known_commands = {}

    def known_commands(self):
        return self._known_commands.keys()

    def record_command(self, command):
        ret = self._robot_record_command(command)
        if ret:
            self._known_commands[command] = ret
        pass

    def execute_command(self, command):

        stuff_to_do = self._known_commands.get(command, None)

        if stuff_to_do is None:
            return False
        return self._robot_do_command(stuff_to_do)

    def _robot_do_command(self, *args, **kwargs):
        raise NotImplementedError('Implement this in each robot interface.')

    def _robot_record_command(self, *args, **kwargs):
        raise NotImplementedError('Implement this in each robot interface.')

    def _robot_set_speed(self, *args, **kwargs):
        raise NotImplementedError('Implement this in each robot interface.')


def run():
    print 'hello world'
    interface = RobotDialogueInterface()

if __name__ == '__main__':
    run()
