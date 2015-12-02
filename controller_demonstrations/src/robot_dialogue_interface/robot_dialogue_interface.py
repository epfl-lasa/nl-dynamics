
class RobotDialogueInterface(object):

    def __init__(self):

        self._known_commands = {}


        pass

    def known_commands(self):
        return self._known_commands.keys()

    def record_command(self, command):
        pass

    def execute_command(self, command):

        to_do = self._known_commands.get(command, None)

        if to_do is None:
            return False
        return self._robot_do_command(to_do)

    def _robot_do_command(self, **kwargs):
        raise NotImplementedError('Implement this in each robot interface.')

    def _robot_record_command(self, *klist, **kwargs):
        raise NotImplementedError('Implement this in each robot interface.')

    def _robot_set_speed(self, **kwargs):
        raise NotImplementedError('Implement this in each robot interface.')


def run():
    print 'hello world'
    interface = RobotDialogueInterface()

if __name__ == '__main__':
    run()
