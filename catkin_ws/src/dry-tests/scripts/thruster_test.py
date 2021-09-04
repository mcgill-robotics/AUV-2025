#!/usr/bin/env python

"""
Thruster Dry Test

This cycles through each thruster turning it on and asking the user to
confirm functionality.
"""

import rospy
from rospy import Publisher

from ...controls.msg import ThrusterCommands
from console_format import format


class ThrusterTest:

    def __init__(self):
        self.cmd = ThrusterCommands()
        self.topic = rospy.get_param('/drytest/thrusters/topic')

        # Magnitude of the pulse sent to the thrusters
        self.pulse = 100

        # Message Frequency
        self.freq = 10

        # Dictionary of thrusters indexed by the message definitions
        self.thrusters = {
            self.cmd.SURGE_PORT: 'Port Surge',
            self.cmd.SURGE_STARBOARD: 'Surge Starboard',
            self.cmd.SWAY_BOW: 'Bow Sway',
            self.cmd.SWAY_STERN: 'Stern Sway',
            self.cmd.HEAVE_BOW_PORT: 'Port Bow Heave',
            self.cmd.HEAVE_BOW_STARBOARD: 'Starboard Bow Heave',
            self.cmd.HEAVE_STERN_PORT: 'Port Stern Heave',
            self.cmd.HEAVE_STERN_STARBOARD: 'Starboard Stern Heave'
        }

        # Stores the results of the test
        self.results = {
            'passes': [],
            'fails': []
        }

        self.pub = Publisher(self.topic, ThrusterCommands, queue_size=5)

    def drytest_thrusters(self):

        rate = rospy.Rate(self.freq)

        for i in range(0, len(self.thrusters)):
            answer = 'n'
            is_working = False

            print('\n\t' + format.UNDERLINE + format.OKBLUE +
                  self.thrusters[i] + format.ENDC)

            # Tests the same thruster until answer is changed to y
            while(answer.lower() == 'n'):
                raw_input('\tPress enter to send a pulse ')

                # Publish a command at the specified frequency
                self.set_command(i)
                for j in range(0, self.freq):
                    self.pub.publish(self.cmd)
                    rate.sleep()

                answer = raw_input('\tIs it running? [y/N] ')

                # Adds thruster to func results if yes
                if answer.lower() == 'y':
                    is_working = True

                # Ask to try again if anything else
                else:
                    answer = raw_input('\tSkip? [Y/n] ')

            if (is_working):
                self.results['passes'].append(self.thrusters[i])
                print (format.OKGREEN + format.BOLD +
                       '\tThe thruster is working' + format.ENDC)
            else:
                self.results['fails'].append(self.thrusters[i])
                print (format.FAIL + format.BOLD + '\tThe thruster is not '
                       'working' + format.ENDC)

    def set_command(self, thruster):
        self.cmd.thruster_commands = [0, 0, 0, 0, 0, 0, 0, 0]
        self.cmd.thruster_commands[thruster] = self.pulse

    def run_test(self):
        is_error = False

        print (format.OKBLUE + format.BOLD + '\n\n'
               '  #######################\n'
               '  ## TESTING THRUSTERS ##\n'
               '  #######################\n' + format.ENDC)

        self.drytest_thrusters()

        # SUMMARY -------------------------------------------------------------
        print('\n\n  ' + format.UNDERLINE + format.OKBLUE + 'Summary' +
              format.ENDC)

        # Prints All func Thrusters
        print (format.OKGREEN + format.BOLD + '  Functional Thrusters are: ')
        for thruster in self.results['passes']:
                print ("  - " + thruster)

        # Prints All Non-func Thrusters
        print (format.FAIL + format.BOLD + '  Non-functional Thrusters are: ')
        for thruster in self.results['fails']:
                print ("  - " + thruster)
                is_error = True
        print (format.ENDC)

        print (format.OKBLUE + '  Finished testing thrusters\n' +
               format.ENDC)

        return is_error
