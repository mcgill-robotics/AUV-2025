#!/usr/bin/env python

import rospy
import os

from camera_test import CameraTest
from thruster_test import ThrusterTest
from sensor_test import SensorTest
from console_format import format

if __name__ == "__main__":
    rospy.init_node("drytest")

    is_error = False
    run_test = 'n'
    results = {'passes': [], 'fails': []}

    os.system('clear')

    # INITIALIZE --------------------------------------------------------------
    print(format.GREY + format.BOLD + format.DIM +
          '\n>>>>>>>>>>>> Starting Drytest' + format.ENDC)

    print(format.RED + format.BOLD +
          '\n  #########################\n'
          '  ##                     ##\n'
          '  ## ' + format.WHITE + '  McGill Robotics  ' + format.RED + ' ##\n'
          '  ## ' + format.WHITE + '    AUV Drytest    ' + format.RED + ' ##\n'
          '  ##                     ##\n'
          '  #########################')

    # CHECK SENSORS -----------------------------------------------------------
    run_test = raw_input(format.WARNING + '\n Test sensors? [y/N]: ' +
                         format.ENDC)

    if (run_test.lower() == 'y'):
        sensor_test = SensorTest()

        is_sensor_error = sensor_test.run_test()
        is_error = is_sensor_error or is_error

        results['passes'].append(sensor_test.results['passes'])
        results['fails'].append(sensor_test.results['fails'])

    else:
        print (format.OKBLUE + ' Skipped sensor tests' + format.ENDC)

    # CHECK THRUSTERS ---------------------------------------------------------
    run_test = raw_input(format.WARNING + '\n Test Thrusters? [y/N]: ' +
                         format.ENDC)

    if (run_test.lower() == 'y'):
        thruster_test = ThrusterTest()

        is_thruster_error = thruster_test.run_test()
        is_error = is_thruster_error or is_error

        results['passes'].append(thruster_test.results['passes'])
        results['fails'].append(thruster_test.results['fails'])

    else:
        print (format.OKBLUE + ' Skipped thruster tests' + format.ENDC)

    # CHECK CAMERAS -----------------------------------------------------------
    run_test = raw_input(format.WARNING + '\n Test Cameras? [y/N]: ' +
                         format.ENDC)

    if (run_test.lower() == 'y'):
        camera_test = CameraTest()

        is_camera_error = camera_test.run_test()
        is_error = is_camera_error or is_error

        results['passes'].append(camera_test.results['passes'])
        results['fails'].append(camera_test.results['fails'])

    else:
        print (format.OKBLUE + ' Skipped camera tests' + format.ENDC)

    # CONCLUSION --------------------------------------------------------------
    if (is_error):
        print(format.BOLD + format.FAIL + '\n\n'
              '  ###############\n'
              '  #             #\n'
              '  #   Failing   #\n'
              '  #             #\n'
              '  ###############\n' + format.ENDC)

        print ('\n' + format.FAIL + ' Errors:')
        for test in results['fails']:
            for fail in test:
                print (' ' + fail)
        print (format.ENDC)

    else:
        print(format.BOLD + format.OKGREEN + '\n\n'
              '  ###############\n'
              '  #             #\n'
              '  #   Passing   #\n'
              '  #             #\n'
              '  ###############\n' + format.ENDC)

    print(format.GREY + format.BOLD + format.DIM +
          '\n<<<<<<<<<<<< Finished Drytest\n' + format.ENDC)
