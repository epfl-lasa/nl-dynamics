#! /usr/bin/env python
import sys
import argparse
import readline

import rospy

from std_msgs.msg import String

from correction_publisher import PublishCorrections

"""
Send multiple commands at once, using stdin as the input
"""

topic_command_parsed = "nl_command_parsed"
topic_microphone_active = "nl_command_received"
topic_kuka_pause = "KUKA/PauseCommand"

def is_finished(command, allow_single_letter):
    # Returns True if the user wants to stop (input = 'quit' or 'done'). single
    # letter stop is allowed if the input is exactly one letter.

    finished = False

    if command == 'quit': finished = True
    if command == 'stop': finished = True

    if allow_single_letter:
        if command == 'q': finished = True

    return finished

class SimpleCompleter(object):
    # Tab-completion for a set of possible choices.
    def __init__(self, choices):
        self.choices = sorted(choices)
        return

    def complete(self, text, state):
        response = None
        if state == 0:
            # This is the first time for this text, so build a match list.
            if text:
                self.matches = [s
                                for s in self.choices
                                if s and s.startswith(text)]
            else:
                self.matches = self.choices[:]

        # Return the state'th item from the match list,
        # if we have that many.
        try:
            response = self.matches[state]
        except IndexError:
            response = None
        return response

class DemonstrationPlayback(object):
    def __init__(self, arguments):
        parser = argparse.ArgumentParser(
            description=('Load a directory of demonstration data (stored as bag '
                         'files).'))
        parser.add_argument('--demo_dir', metavar='directory', required=True)
        args = parser.parse_args(arguments)

        # The time for the last microphone activity.
        self._last_time_microphone_active = rospy.Time()

        self._demo_publisher = PublishCorrections(args.demo_dir)

        words = self._demo_publisher.words()
        words.extend(['stop', 'quit'])

        # Use the tab key for completion, using the words in the demo publisher
        # (along with stop/quit).
        readline.set_completer(SimpleCompleter(words).complete)
        readline.parse_and_bind('tab: complete')

        # Subscribe to voice commands channel.
        rospy.Subscriber(topic_command_parsed, String,
                         self.nl_command_received_callback)

        # Store the publisher for kuka pause messages.
        self._kuka_pause_publisher = rospy.Publisher(
            topic_kuka_pause, String, queue_size=100)

        pass

    def get_command(self, allow_single_letter):
        # Returns the NL command, or None if we are finished.
        command = raw_input('Enter command:')
        if is_finished(command, allow_single_letter):
            command = None

        return command

    def nl_command_received_callback(self, msg):
        command_str = msg.data
        rospy.loginfo('Received NL command: {}'.format(command_str))
        self._demo_publisher.process_command(command_str,
                                             use_current_state_as_anchor=True)

    def nl_microphone_active_callback(self, data):
        # NOTE this is not currently in use.
        rospy.loginfo('Microphone active -- pausing robot')
        msg = String("PAUSE")
        self._kuka_pause_publisher.publish(msg)

        # Store the time of reception.
        self._last_time_microphone_active = rospy.Time.now()

    def run_send_multiple_commands(self):
        try:
            finished = False
            while not finished:
                # Get the command from the keyboard (option 1). Note the audio
                # recognizer is still running using callbacks.
                nl_command = self.get_command(allow_single_letter=True)
                if not nl_command:
                    finished = True
                    rospy.loginfo('Finished.')
                    break

                rospy.loginfo('Command: {}'.format(nl_command))
                self._demo_publisher.process_command(nl_command, use_current_state_as_anchor=True)

        except rospy.ROSInterruptException as e:
            print e
            pass

        rospy.loginfo('Goodbye.')



if __name__ == '__main__':
    args = sys.argv[1:]  # argv[0] is the program name.

    playback = DemonstrationPlayback(arguments=args)
    playback.run_send_multiple_commands()
