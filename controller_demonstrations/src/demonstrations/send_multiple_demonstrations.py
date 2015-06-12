#! /usr/bin/env python
import sys
import argparse
import readline

import rospy

from correction_publisher import PublishCorrections

"""
Send multiple commands at once, using stdin as the input
"""


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


def get_command(allow_single_letter):
    # Returns the NL command, or None if we are finished.
    command = raw_input('Enter command:')
    if is_finished(command, allow_single_letter):
        command = None

    return command


def run_send_multiple_commands(arguments):
    parser = argparse.ArgumentParser(
        description=('Load a directory of demonstration data (stored as bag '
                     'files).'))
    parser.add_argument('--demo_dir', metavar='directory', required=True)
    args = parser.parse_args(arguments)

    demo_publisher = PublishCorrections(args.demo_dir)

    words = demo_publisher.words()
    words.extend(['stop', 'quit'])

    # Use the tab key for completion, using the words in the demo publisher
    # (along with stop/quit).
    readline.set_completer(SimpleCompleter(words).complete)
    readline.parse_and_bind('tab: complete')

    try:
        finished = False
        while not finished:
            nl_command = get_command(allow_single_letter=True)  # TODO get from opts
            if not nl_command:
                finished = True
                rospy.loginfo('Finished.')
                break

            rospy.loginfo('Command: {}'.format(nl_command))
            demo_publisher.process_command(nl_command, use_current_state_as_anchor=True)

    except rospy.ROSInterruptException as e:
        print e
        pass

    rospy.loginfo('Goodbye.')



if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    run_send_multiple_commands(arguments)

