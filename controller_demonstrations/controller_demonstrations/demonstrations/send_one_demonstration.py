#! /usr/bin/env python
import sys
import argparse
import time

import rospy

from correction_publisher import PublishCorrections


def run_send_one_command(arguments):
    parser = argparse.ArgumentParser(
        description=('Load a directory of demonstration data (stored as bag '
                     'files).'))
    parser.add_argument('--demo_dir', metavar='directory', required=True)
    parser.add_argument('command', nargs='+')
    args = parser.parse_args(arguments)

    demo_publisher = PublishCorrections(args.demo_dir)

    nl_command = ' '.join(args.command)

    # Sleep long enough to get the Kuka State and have all messages register
    # with the core.
    time.sleep(1.0)

    try:
        demo_publisher.process_command(nl_command, use_current_state_as_anchor=True)
        time.sleep(0.5)  # Sleep to make sure the message goes out.
    except rospy.ROSInterruptException as e:
        print e
        pass

    # Do not spin: only one command goes out. rospy.spin()


if __name__ == '__main__':
    arguments = sys.argv[1:]  # argv[0] is the program name.
    rospy.init_node('send_demonstrations', anonymous=False)
    run_send_one_command(arguments)
