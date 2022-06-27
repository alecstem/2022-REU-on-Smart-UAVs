'''
This is the code that will connect the drone to mission planner, mavproxy, and the python script to fly the path generated.
'''
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import time
import argparse
import os
from threading import Thread

"""
HOW TO UESE: go to the directory c:\\users\\lcbba\\pycharmprojects\\pythonproject\\venv\\scripts
then enter in dronekit-sitl copter --home=(enter lats here), (enter longs here), (enter height here), (enter yaw here)
then go to second cmd and go to the directory c:\\users\\lcbba\\pycharmprojects\\pythonproject\\venv\\scripts
then enter mavproxy
then open up mission planner and connect using 5760
you are now good to run the program in python
"""

def cmd_line(combined, start_lat, start_long, start_height): os.system(
        'c:\\users\\lcbba\\pycharmprojects\\pythonproject\\venv\\scripts\\dronekit-sitl copter --home=' + str(start_lat) + ', ' + str(start_long) + ', ' + '0, 180')

def main(combined, start_lat, start_long, start_height):
        time.sleep(30)
        parser = argparse.ArgumentParser()
        print('main has been started ')
        '''
        i = 0
        while i in range(24):
            print(i)
            i += 1
            time.sleep(1)
            '''
        #input('hit enter: ')
        #time.sleep(5)

        parser.add_argument('--connect', default='tcp:127.0.0.1:5762')
        args = parser.parse_args()

        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % args.connect)
        vehicle = connect(args.connect, baud=115200, wait_ready=False)
        time.sleep(10)

        # 921600 is the baudrate that you have set in the mission plannar or qgc

        # Function to arm and then takeoff to a user specified altitude
        def arm_and_takeoff(aTargetAltitude):
            print("Basic pre-arm checks")
            # Don't let the user try to arm until autopilot is ready
            while not vehicle.is_armable:
                print("Waiting for vehicle to initialise...")
                time.sleep(1)

            print("Arming motors")
            # Copter should arm in GUIDED mode
            vehicle.mode = VehicleMode('GUIDED')
            # VehicleMode('GUIDED')
            #print(vehicle.mode)
            #time.sleep(2)
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
        vehicle.airspeed = 15  # m/s
        arm_and_takeoff(start_height)

        print("Take off complete")

        vehicle.airspeed = 15  # m/s

        #combined = []

        for i in range(len(combined)):
            def get_distance_metres(aLocation1, aLocation2):
                """
                Returns the ground distance in metres between two LocationGlobal objects.

                This method is an approximation, and will not be accurate over large distances and close to the
                earth's poles. It comes from the ArduPilot test code:
                https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
                """
                dlat = aLocation2.lat - aLocation1.lat
                dlong = aLocation2.lon - aLocation1.lon
                return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
            def distance_to_current_waypoint(combined):
                """
                Gets distance in metres to the current waypoint.
                It returns None for the first waypoint (Home location).
                """
                nextwaypoint = LocationGlobalRelative(combined[i][0], combined[i][1], combined[i][2])
                if nextwaypoint == 0:
                    return None
                targetWaypointLocation = LocationGlobalRelative(combined[i][0], combined[i][1], combined[i][2])
                distancetopoint = get_distance_metres(vehicle.location.global_relative_frame, targetWaypointLocation)
                print(int(distancetopoint))
                return int(distancetopoint)
            current_location = vehicle.location.global_relative_frame
            print(current_location)
            a_location = LocationGlobalRelative(combined[i][0], combined[i][1], combined[i][2])
            print('location a : ' + str(a_location))
            # print(distance_to_current_waypoint(combined))
            if int(distance_to_current_waypoint(combined)) > 0 & int(distance_to_current_waypoint(combined)) < 8:
                vehicle.simple_goto(a_location)
                # time.sleep(.1)
            elif int(distance_to_current_waypoint(combined)) > 15:
                time.sleep(2)
            elif int(distance_to_current_waypoint(combined)) > 20:
                time.sleep(1)
            else:
                time.sleep(.1)
            # Hover for 10 seconds                     # this is where we are slowing down

        # if get_location_metres(original_location, dNorth, dEast) == a_location:
        print("Now let's land")
        vehicle.mode = VehicleMode('LAND')
        time.sleep(10)
        print(vehicle.mode)

        # Close vehicle object
        vehicle.close()

def run(combined, start_lat, start_long, start_height):
    Thread(target=main, args=(combined, start_lat, start_long, start_height,)).start()
    Thread(target=cmd_line, args=(combined, start_lat, start_long, start_height,)).start()

