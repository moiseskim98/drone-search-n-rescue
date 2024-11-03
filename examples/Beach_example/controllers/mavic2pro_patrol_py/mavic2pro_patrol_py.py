# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Example of Python controller for Mavic patrolling around the house.
   Open the robot window to see the camera view.
   This demonstrates how to go to specific world coordinates using its GPS, imu and gyroscope.
   The drone reaches a given altitude and patrols from waypoint to waypoint."""

from controller import Robot
import math
import sys
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")

class Enumerate(object):
    def __init__(self, names):
        for number, name in enumerate(names.split()):
            setattr(self, name, number)

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic (Robot):
    
    # Constants, empirically found.
    K_VERTICAL_THRUST = 72  # with this thrust, the drone lifts.
    # Vertical offset where the robot actually targets to stabilize itself.
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0        # P constant of the vertical PID.
    K_ROLL_P = 40           # P constant of the roll PID.
    K_PITCH_P = 30          # P constant of the pitch PID.
    
    
    MAX_YAW_DISTURBANCE = 1.4
    MAX_PITCH_DISTURBANCE = -1
    # Precision between the target position and the robot position in meters
    target_precision = 0.5

    Mode = Enumerate('DEPLOY AUTOPILOT FOUND RELEASE STOP')
    #StimeStep = 32
    maxSpeed = 10.0
    mode = Mode.DEPLOY
    
    EPSILON = 0.1
    EPSILON_DROP = 15
    
    target_id = 178

    v0 = 2.5
    alfa = 0.2

    # yaw PID
    kpOx = 0.50;
    kiOx = 0.0000; 
    kdOx = 0.0001;

    # pitch PID
    kpOy = 4.00;
    kiOy = 0.010; 
    kdOy = 0.0001;
    
    # Controller
    EOx = 0
    EOy = 0
    eO_1x = 0
    eO_1y = 0

    def __init__(self):
        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        # Get and enable devices.
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.camera.recognitionEnable(self.time_step)
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)
        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        self.camera_roll_motor = self.getDevice("camera roll")
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(0)
        self.latch = self.getDevice('latch')
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        # maybe erase it
        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0
        
        # Controller
        self.EOx = 0
        self.EOy = 0
        self.eO_1x = 0
        self.eO_1y = 0
        
        self.keyboard.enable(self.time_step)
        self.keyboard = self.getKeyboard()
        
        print("Start the drone\n")

    def flight_planning_algorithm(self, waypoints, verbose_movement=False, verbose_target=False):

        roll, pitch, yaw = self.imu.getRollPitchYaw()
        x_pos, y_pos, altitude = self.gps.getValues()
        self.current_pose = [x_pos, y_pos, altitude, roll, pitch, yaw]


        if self.target_position[0:2] == [0, 0]:  # Initialization
            self.target_position[0:2] = waypoints[0]
            if verbose_target:
                print("First target: ", self.target_position[0:2])

        # if the robot is at the position with a precision of target_precision
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):

            self.target_index += 1
            if self.target_index > len(waypoints) - 1:
                self.target_index = 0
            self.target_position[0:2] = waypoints[self.target_index]
            if verbose_target:
                print("Target reached! New target: ",
                      self.target_position[0:2])

        # This will be in ]-pi;pi]
        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1], self.target_position[0] - self.current_pose[0])
        # This is now in ]-2pi;2pi[
        angle_left = self.target_position[2] - self.current_pose[5]
        # Normalize turn angle to ]-pi;pi]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if (angle_left > np.pi):
            angle_left -= 2 * np.pi

        # Turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        # non proportional and decreasing function
        pitch_disturbance = clamp(
            np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        if verbose_movement:
            distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) + (
                (self.target_position[1] - self.current_pose[1]) ** 2))
            print("remaning angle: {:.4f}, remaning distance: {:.4f}".format(
                angle_left, distance_left))
        return yaw_disturbance, pitch_disturbance

    def move_to_target(self, objects,pitch,yaw,camera):
        """
        Move the robot to the given coordinates
        Parameters:
            waypoints (list): list of X,Y coordinates
            verbose_movement (bool): whether to print remaning angle and distance or not
            verbose_target (bool): whether to print targets or not
        Returns:
            yaw_disturbance (float): yaw disturbance (negative value to go on the right)
            pitch_disturbance (float): pitch disturbance (negative value to go forward)
        """
        pitch_disturbance = pitch
        yaw_disturbance = yaw
        camera_yaw_disturbance = camera
        for object in objects:
            if object.getId() == self.target_id:
    
                try:
                    # Orientation error
                    eOx = (object.getPositionOnImage()[0]-400.0/2.0)/(400.0)*0.785
                    eOy = (object.getPositionOnImage()[1]-240.0/2.0)/(240.0)*0.785
        
                    # Exponential controller with PID
                    # y focal length
                    fy = 240/(2*math.atan(0.785/2)) #fov = 0.785, height = 240 pixels
                
                    obj_y = 1.2*fy/object.getPositionOnImage()[1];
                    e = [0, obj_y/math.tan(eOy)]
                
                    # Position error
                    eP = math.sqrt(pow(e[0],2)+pow(e[1],2))
                    
                    
                    # Angular speed controller
                    eO_Dx = eOx - self.eO_1x
                    eO_Dy = eOy - self.eO_1y
                    
                    self.EOx = self.EOx + eOx
                    self.EOy = self.EOy + eOy
                    
                    # Linear speed controller
                    kP = self.v0*(1-math.exp(-self.alfa*pow(eP,2)))/eP
                    
                    pitch_disturbance = -1.0*kP*eP
                    
                    yaw_disturbance = clamp(-1.0*(self.kpOx*eOx + self.kiOx*self.EOx*self.time_step/1000.0 + self.kdOx*eO_Dx*1000.0/self.time_step),-1.3,1.3)
                    camera_yaw_disturbance = clamp(1.0*(self.kpOy*eOy + self.kiOy*self.EOy*self.time_step/1000.0 + self.kdOy*eO_Dy*1000.0/self.time_step),-1.5,1.5)
                    
                    self.eO_1x = eOx
                    self.eO_1y = eOy
                    
                    #print("yaw: " + str(yaw_disturbance) + " [" +str(eOx),str(eOy),str(eP)+"]")   
                    #print("pitch : " + str(pitch_disturbance) + " yaw: " + str(yaw_disturbance) + " camera: " + str(camera_yaw_disturbance))
                    '''
                    if yaw_disturbance > 1.3:
                        yaw_disturbance = 2
                    elif yaw_disturbance < -1.3:
                        yaw_disturbance = -2
                    
                    if camera_yaw_disturbance > 1.6:
                        camera_yaw_disturbance = 1.6
                    elif camera_yaw_disturbance < -0.5:
                        camera_yaw_disturbance = -0.5
                    '''

                    if eP<self.EPSILON_DROP:
                        #print("Hola")
                        self.mode = self.Mode.RELEASE
                except:
                    e = [0,0]
        return pitch_disturbance, yaw_disturbance, camera_yaw_disturbance

    def run(self):
        t1 = self.getTime()

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0
        camera_pitch_disturbance = 0
        latch_position = 0
        previous_message = ""

        # Specify the patrol coordinates
        waypoints = [[26, 46], [70, 46], [70, 60], [26, 60]]
        # target altitude of the robot in meters
        self.target_altitude = 1
        while self.step(self.time_step) != -1:
        
            # Deal with the pressed keyboard key.
            k = self.keyboard.getKey()
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
                #self.displayHelp()
                message = 'help'
            elif k == self.keyboard.UP:
                message = 'front'
            elif k == self.keyboard.DOWN:
                message = 'back'
            elif k == self.keyboard.LEFT:
                message = 'turn left'
            elif k == self.keyboard.RIGHT:
                message = 'turn right'
            elif k == (self.keyboard.SHIFT + self.keyboard.UP):
                message = 'up'
            elif k == (self.keyboard.SHIFT + self.keyboard.DOWN):
                message = 'down'
            elif k == (self.keyboard.SHIFT + self.keyboard.LEFT):
                message = 'left'
            elif k == (self.keyboard.SHIFT + self.keyboard.RIGHT):
                message = 'right'
            elif k == ord('R'):
                message = 'release'
            elif k == ord('C'):
                message = 'close'
            
            # Send a new message through the emitter device.
            if message != '':# and message != previous_message:
                previous_message = message
                if message == 'front':
                    pitch_disturbance = -2.0
                elif message == 'back':
                    pitch_disturbance = 2.0
                elif message == 'turn right':
                    yaw_disturbance = -1.3
                elif message == 'turn left':
                    yaw_disturbance = 1.3
                elif message == 'up':
                    self.target_altitude = self.target_altitude + 0.05
                    previous_message = ''
                elif message == 'down':
                    self.target_altitude = self.target_altitude - 0.05
                    previous_message = ''
                elif message == 'left':
                    roll_disturbance = 1.0
                elif message == 'right':
                    roll_disturbance = -1.0
                elif message == 'release':
                    latch_position = 0.5
                elif message== 'close':
                    latch_position = 0.0

            # Read sensors
            objects = self.camera.getRecognitionObjects()
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            #print("x: " + str(x_pos) + " y: " + str(y_pos))
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            #self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])
            
            #for object in objects:
            #    print(str(object.getId()))

            if(altitude > self.target_altitude - 1) & (self.mode == self.Mode.DEPLOY):
                # as soon as it reach the target altitude, compute the disturbances to go to the given waypoints.
                self.mode = self.Mode.FOUND
                
            
            if self.mode == self.Mode.DEPLOY:
                if abs(altitude-self.target_altitude)<1:
                    self.mode = self.Mode.FOUND
            elif self.mode == self.Mode.FOUND:
                yaw_disturbance, pitch_disturbance = self.flight_planning_algorithm(waypoints)
                pitch_disturbance, yaw_disturbance, camera_pitch_disturbance = self.move_to_target(
                    objects,pitch_disturbance, yaw_disturbance, camera_pitch_disturbance)
            elif self.mode == self.Mode.RELEASE:
                self.latch.setPosition(0.5)
                _, _, camera_pitch_disturbance = self.move_to_target(
                    objects,pitch_disturbance, yaw_disturbance, camera_pitch_disturbance)


            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)
            
            # camera position
            self.camera_roll_motor.setPosition(clamp(-0.115 * roll_acceleration,-0.5,.5))
            self.camera_pitch_motor.setPosition(clamp(-0.1 * pitch_acceleration + 0.3 + camera_pitch_disturbance,-0.5,1.63))
        
            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(clamp(front_left_motor_input,-100,100))
            self.front_right_motor.setVelocity(clamp(-front_right_motor_input,-100,100))
            self.rear_left_motor.setVelocity(clamp(-rear_left_motor_input,-100,100))
            self.rear_right_motor.setVelocity(clamp(rear_right_motor_input,-100,100))


# To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
# with a linear and angular damping both of 0.5
robot = Mavic()
robot.run()
