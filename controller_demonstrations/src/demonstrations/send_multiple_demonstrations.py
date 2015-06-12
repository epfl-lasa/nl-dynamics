#! /usr/bin/env python
import sys
import argparse
import time

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
    if command == 'done': finished = True

    if allow_single_letter:
        if command == 'q': finished = True

    return finished


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

