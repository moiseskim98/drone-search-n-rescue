#!/usr/bin/env python3
'''
General ardupilot vehicle controller for Webots 2023a

AP_FLAKE8_CLEAN
'''


import time
import argparse
from ardupilot_library import WebotsArduVehicle


def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--motors", "-m",
                        type=str,
                        default="front left propeller, front right propeller, rear left propeller, rear right propeller",
                        help="Comma spaced list of motor names in ardupilot numerical order (ex --motors \"m1,m2,m3, m4\")")
    parser.add_argument("--reversed-motors", "-r",
                        type=str,
                        default=None,
                        help="Comma spaced list of motors to reverse (starting from 1, in ardupilot order)")
    parser.add_argument("--bidirectional-motors",
                        type=bool,
                        default=False,
                        help="If the motors are bidirectional (as is the case for Rovers usually)")
    parser.add_argument("--uses-propellers",
                        type=bool,
                        default=True,
                        help="Whether the vehicle uses propellers. This is important as we need to linearize thrust if so")
    parser.add_argument("--motor-cap",
                        type=float,
                        default=float('inf'),
                        help="Motor velocity cap. This is useful for the crazyflie which default has way too much power")

    parser.add_argument("--accel",
                        type=str,
                        default=None,
                        help="Webots accelerometer name")
    parser.add_argument("--imu",
                        type=str,
                        default="inertial unit",
                        help="Webots IMU name")
    parser.add_argument("--gyro",
                        type=str,
                        default="gyro",
                        help="Webots gyro name")
    parser.add_argument("--gps",
                        type=str,
                        default="gps",
                        help="Webots GPS name")

    parser.add_argument("--camera",
                        type=str,
                        default="camera",
                        help="Webots Camera name (optional)")
    parser.add_argument("--camera-fps",
                        type=int,
                        default=10,
                        help="Camera FPS. Note lower FPS is faster")
    parser.add_argument("--camera-port",
                        type=int,
                        default=None,
                        help="Port to stream grayscale camera images to. "
                             "If no port is supplied the camera will not be streamed.")

    parser.add_argument("--rangefinder",
                        type=str,
                        default=None,
                        help="Webots RangeFinder name (optional)")
    parser.add_argument("--rangefinder-fps",
                        type=int,
                        default=10,
                        help="rangefinder FPS. Note lower FPS is faster")
    parser.add_argument("--rangefinder-port",
                        type=int,
                        default=None,
                        help="Port to stream grayscale rangefinder images to. "
                             "If no port is supplied the rangefinder will not be streamed.")

    parser.add_argument("--instance", "-i",
                        type=int,
                        default=0,
                        help="Drone instance to match the SITL. This allows multiple vehicles")
    parser.add_argument("--sitl-address",
                        type=str,
                        default="172.22.128.1",
                        help="IP address of the SITL (useful with WSL2 eg \"172.24.220.98\")")

    return parser.parse_args()


if __name__ == "__main__":
    args = get_args()

    # parse string arguments into lists
    motors = [x.strip() for x in args.motors.split(',')]
    if args.reversed_motors:
        reversed_motors = [int(x) for x in args.reversed_motors.split(",")]
    else:
        reversed_motors = []

    vehicle = WebotsArduVehicle(motor_names=motors,
                                reversed_motors=reversed_motors,
                                accel_name=args.accel,
                                imu_name=args.imu,
                                gyro_name=args.gyro,
                                gps_name=args.gps,
                                camera_name=args.camera,
                                camera_fps=args.camera_fps,
                                camera_stream_port=args.camera_port,
                                rangefinder_name=args.rangefinder,
                                rangefinder_fps=args.rangefinder_fps,
                                rangefinder_stream_port=args.rangefinder_port,
                                instance=args.instance,
                                motor_velocity_cap=args.motor_cap,
                                bidirectional_motors=args.bidirectional_motors,
                                uses_propellers=args.uses_propellers,
                                sitl_address=args.sitl_address)

    # User code (ex: connect via drone kit and take off)
    # ...

    t1 = vehicle.getTime()

    roll_disturbance = 0
    pitch_disturbance = 0
    yaw_disturbance = 0
    camera_pitch_disturbance = 0
    latch_position = 0
    previous_message = ""

    # Specify the patrol coordinates
    waypoints = [[-30, 20], [-60, 20], [-60, 10], [-30, 5]]
    # target altitude of the robot in meters
    target_altitude = 1

    while True:

        if vehicle.getTime() > (t1+0.2):
            t1 = vehicle.getTime()
            if not vehicle.webots_connected():
                break
        # Deal with the pressed keyboard key.
        keyboard = vehicle.getKeyboard()
        k = keyboard.getKey()
        message = ''
        if k == ord('D'):
            message = 'deploy'
        elif k == ord('A'):
            message = 'autopilot'
        elif k == ord('S'):
            message = 'stop'
        elif k == ord('F'):
            message = 'found'
        elif k == ord('I'):
            message = 'help'
            print("hola")
        elif k == keyboard.UP:
            pitch_disturbance = -2.0
        elif k == keyboard.DOWN:
            pitch_disturbance = 2.0
        elif k == keyboard.LEFT:
            yaw_disturbance = 1.3
        elif k == keyboard.RIGHT:
            yaw_disturbance = -1.3
        elif k == (keyboard.SHIFT + keyboard.UP):
            target_altitude = target_altitude + 0.05
        elif k == (keyboard.SHIFT + keyboard.DOWN):
            target_altitude = target_altitude - 0.05
        elif k == (keyboard.SHIFT + keyboard.LEFT):
            roll_disturbance = 1.0
        elif k == (keyboard.SHIFT + keyboard.RIGHT):
            roll_disturbance = -1.0
        elif k == ord('R'):
            latch_position = 0.5
        elif k == ord('C'):
            latch_position = 0.0
        else:
            pitch_disturbance = 0
            yaw_disturbance = 0
            roll_disturbance = 0

        # Read sensors
        target = vehicle.object_identifier(187)

        if target is not None:
            [pitch_disturbance,yaw_disturbance,camera_pitch_disturbance] = vehicle.move_to_target(
                object,pitch_disturbance,yaw_disturbance,camera_pitch_disturbance)
            
            vehicle.propeller_calculation(target_altitude, 0, pitch_disturbance, yaw_disturbance)
            vehicle.camera_position(camera_pitch_disturbance)

        #time.sleep(1)
