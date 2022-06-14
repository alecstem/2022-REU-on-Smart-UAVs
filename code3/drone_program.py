from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import argparse
import os
from threading import Thread

"""
HOW TO UESE: go to the directory c:\\users\\lcbba\\pycharmprojects\\pythonproject\\venv\\scripts
then enter in dronekit-sitl copter --home=(enter lats here), (enter longs here), (enter height here), (enter yaw here)
then go to seccond cmd and go to the directory c:\\users\\lcbba\\pycharmprojects\\pythonproject\\venv\\scripts
then enter mavproxy
then open up mission planner and connect using 5760
you are now good to run the program in python
"""

def cmd_line(combined, start_lat, start_long): os.system(
        'c:\\users\\lcbba\\pycharmprojects\\pythonproject\\venv\\scripts\\dronekit-sitl copter --home=' + str(start_lat) + ', ' + str(start_long) + ', ' + '0, 180')

def main(combined, start_lat, start_long):
        parser = argparse.ArgumentParser()
        print('main has been started ')
        time.sleep(30)

        parser.add_argument('--connect', default='tcp:127.0.0.1:5762')
        args = parser.parse_args()

        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % args.connect)
        vehicle = connect(args.connect, baud=115200, wait_ready=True)

        # 921600 is the baudrate that you have set in the mission plannar or qgc

        # Function to arm and then takeoff to a user specified altitude
        def arm_and_takeoff(aTargetAltitude):
            print("Basic pre-arm checks")
            # Don't let the user try to arm until autopilot is ready
            while not vehicle.is_armable:
                print(" Waiting for vehicle to initialise...")
                time.sleep(1)

            print("Arming motors")
            # Copter should arm in GUIDED mode
            vehicle.mode = 'GUIDED'
            VehicleMode('GUIDED')
            print(vehicle.mode)
            time.sleep(5)
            vehicle.armed = True

            while not vehicle.armed:
                print(" Waiting for arming...")
                time.sleep(1)

            print("Taking off!")
            vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

            # Check that vehicle has reached takeoff altitude
            while True:
                print(" Altitude: ", vehicle.location.global_relative_frame.alt)
                # Break and return from function just below target altitude.
                if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                    print("Reached target altitude")
                    break
                time.sleep(1)

        # Initialize the takeoff sequence to 15m
        arm_and_takeoff(20)

        print("Take off complete")

        vehicle.airspeed = 15  # m/s

        #combined = []

        for i in range(len(combined)):
            a_location = LocationGlobalRelative(combined[i][0], combined[i][1], combined[i][2])
            vehicle.simple_goto(a_location)
            # Hover for 10 seconds
            time.sleep(.008)

        # if get_location_metres(original_location, dNorth, dEast) == a_location:
        print("Now let's land")
        vehicle.mode = VehicleMode('RTL')
        time.sleep(10)
        print(vehicle.mode)

        # Close vehicle object
        vehicle.close()

def run(combined, start_lat, start_long):

    Thread(target=cmd_line, args=(combined, start_lat, start_long,)).start()
    Thread(target=main, args=(combined, start_lat, start_long,)).start()
