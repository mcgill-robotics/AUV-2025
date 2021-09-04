#!/usr/bin/env python

import time
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import Imu, PointCloud

from console_format import format
from wait_for_message import *


class SensorTest:

    def __init__(self):
        # Load sensors from the config file
        self.sensors = rospy.get_param('/drytest/sensors')

        # Delay between sensor checks
        self.delay = 1

        # Map messages types to strings
        # TODO There might be a better way of doing this, but
        # marginally better than using eval()...
        self.messages = {
            'Float32': Float32,
            'Float64': Float64,
            'Imu': Imu,
            'PointCloud': PointCloud
        }

        # Stores the results of the test
        self.results = {
            'passes': [],
            'fails': []
        }

    def check_sensor(self, sensor):
        answer = 'y'
        skip = 'x'

        print('\n\t' + format.UNDERLINE + format.OKBLUE + sensor['name'] +
              format.ENDC)

        skip = raw_input('\tMake sure the sensor is is launched.\n'
                         '\tEnter most keys to continue [s to skip] ')

        if (skip.lower() == 's'):
            print(format.WARNING + '\tSkipped...' + format.ENDC)
            self.results['passes'].append(sensor['name'])
            return True

        while (answer.lower() == 'y'):
            is_responsive = True
            print('\tWaiting for sensor feedback...')

            # Test the output of each topic listed in the config
            for topic in sensor['topics']:
                name = topic['name']
                msg_type = self.messages[topic['type']]

                try:
                    wait_for_message(name, msg_type, 3)

                except Exception:
                    print (format.WARNING + '\t' + topic['name'] +
                           ' is unresponsive' + format.ENDC)
                    is_responsive = False
                    pass

            if is_responsive:
                answer = 'n'
            else:
                answer = raw_input('\tOne or more of the messages was not '
                                   'being published.\n\tTry again? [y/N] ')

        if is_responsive:
            self.results['passes'].append(sensor['name'])
            print (format.OKGREEN + format.BOLD + '\tThe sensor is working!' +
                   format.ENDC)
        else:
            self.results['fails'].append(sensor['name'])
            print (format.FAIL + format.BOLD + '\tThe sensor is not working' +
                   format.ENDC)

    def run_test(self):
        is_error = False

        print (format.OKBLUE + format.BOLD + '\n\n'
               '  #####################\n'
               '  ## TESTING SENSORS ##\n'
               '  #####################\n' + format.ENDC)

        # TEST SENSORS --------------------------------------------------------
        for sensor in self.sensors:
            self.check_sensor(sensor)
            time.sleep(self.delay)

        # SUMMARY -------------------------------------------------------------
        print('\n\n  ' + format.UNDERLINE + format.OKBLUE + 'Summary' +
              format.ENDC)

        # Prints All Functional Sensors
        print (format.OKGREEN + format.BOLD + '  Functional Sensors are: ')
        for sensor in self.results['passes']:
            print('  - ' + sensor)

        # Prints All Functional Sensors
        print (format.FAIL + format.BOLD + '  Non-functional Sensors are: ')
        for sensor in self.results['fails']:
            print('  - ' + sensor)
            is_error = True
        print (format.ENDC)

        print (format.OKBLUE + '  Finished testing sensors\n' + format.ENDC)

        return is_error
