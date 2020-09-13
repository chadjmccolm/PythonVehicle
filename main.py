# todo:
#  - seperation of regen and braking
#  - incline

import time
import os
from speedCommands import *

timestep = 0.01
# Speed commands are in kmph

f = open('workfile.csv', 'w')

# Interpolate the speed commands down to every 0.01 seconds. 
steps = int(speedCommands[len(speedCommands)-1][0] / timestep + 1)

calculatedCommands = []
index = 0
firstPoint = speedCommands[index]
secondPoint = speedCommands[index+1]

for step in range(steps):

    # Get the time of this command
    steptime = step * timestep
    
    # Check if that's between the first two points, if not, move the points along until you find the relevant points
    while(steptime < firstPoint[0] or steptime > secondPoint[0]):
        index = index + 1
        firstPoint = speedCommands[index]
        secondPoint = speedCommands[index+1]

    # Interpolate between the points
    speed = (secondPoint[1]-firstPoint[1])/(secondPoint[0]-firstPoint[0])*(steptime - firstPoint[0]) + firstPoint[1]
    pair = [steptime, speed]
    calculatedCommands.append(pair)

print("Path Generated with " + str(len(calculatedCommands)) + " steps.\n")

class VehicleBody:

    # Properties of every vehicle
    mass = 226
    wheelbase = 1.40
    cog_z = 0.4
    cog_x = 0.70

    C_rr = 0.02

    frontal_area = 0.95
    C_d = 0.46

    # Inputs to calculations
    force = 0.0

    # Forward calculated values
    acceleration = 0.0
    velocity = 0.0
    displacement = 0.0
    weight_front_wheel = 0.0
    weight_rear_wheel = 0.0

    def calculate(self):

        # Compensate for rolling resistance
        if self.velocity > 0:
            self.force = self.force - self.C_rr * self.mass * 9.81

        # Compensate for drag
        if self.velocity: 
            self.force = self.force - 0.5 * self.C_d * 1.225 * self.velocity * self.velocity * self.frontal_area

        # Calculate Position
        self.acceleration = self.force / self.mass
        self.velocity = self.velocity + self.acceleration * timestep
        self.displacement = self.displacement + self.velocity * timestep

        # Calculate Weight Distribution
        self.weight_front_wheel = (self.mass * 9.81 * self.cog_x - self.force * self.cog_z) / self.wheelbase
        self.weight_rear_wheel = self.mass * 9.81 - self.weight_front_wheel

class Wheel: 

    # Properties of the wheel
    mu = 1.2
    diameter = 0.63

    # Inputs to calculations
    normal_force = 0
    ground_speed = 0

    applied_torque = 0

    # Forward calculated values
    maximum_torque = 0
    rotational_speed = 0

    applied_force = 0

    def calculate(self):

        # Calculate the maximum torque
        self.maximum_torque = self.mu * self.diameter / 2 * self.normal_force

        # Calculate rotational speed
        self.rotational_speed = self.ground_speed * 2 / self.diameter

        # Back calculate motivational force
        self.applied_force = self.applied_torque / (self.diameter / 2)

class Axle:

    # Properties of the axle
    ratio = 4

    # Inputs to calculations
    maximum_torque_output = 0
    rotational_speed_output = 0
    torque_input = 0

    # Forward calculated values
    maximum_torque_input = 0
    rotational_speed_input = 0

    # Backwards calculated values
    torque_output = 0

    def calculate(self):

        # Make calculations
        self.maximum_torque_input = self.maximum_torque_output / self.ratio
        self.rotational_speed_input = self.rotational_speed_output * self.ratio
        self.torque_output = self.torque_input * self.ratio

class Driver: 

    # Properties of the driver
    K_p = 0.03
    K_i = 0.001
    K_d = 0

    error = 0
    integral_error = 0
    timestep = 0

    # Inputs to calculations
    currentSpeed = 0
    desiredSpeed = 0

    # Forward calculated values
    effort = 0

    def calculate(self):

        self.error = self.desiredSpeed - self.currentSpeed
        self.integral_error = self.integral_error + self.error

        self.effort = self.error * self.K_p + self.integral_error * self.K_i
        self.effort = max(min(self.effort, 100), -100)

class Motor: 

    # Properties of the motor
    
    # Inputs to calculations
    max_torque = 0
    rotational_speed = 0
    effort = 0
    timestep = 0

    # Forward calculated values
    output_torque = 0
    output_power = 0
    energy_used = 0

    def calculate(self):

        # Control torque
        self.output_torque = self.max_torque * self.effort

        self.output_power = self.output_torque * self.rotational_speed
        
        if self.output_power > 0:
            self.energy_used = self.energy_used + self.output_power * self.timestep

vehicle = VehicleBody()
rear_wheel = Wheel()
final_drive = Axle()
driver = Driver()
driver.timestep = timestep
motor = Motor()
motor.timestep = timestep

print("Objects Created.\n")

f.write("Time, Target Speed, Actual Speed, Power Required, Energy Required, Distance Travelled\n")

print("Starting Calculation.\n")
start_time = time.time()

for command in calculatedCommands:
    
    vehicle.calculate()

    rear_wheel.normal_force = vehicle.weight_rear_wheel
    rear_wheel.ground_speed = vehicle.velocity
    rear_wheel.calculate()

    final_drive.maximum_torque_output = rear_wheel.maximum_torque
    final_drive.rotational_speed_output = rear_wheel.rotational_speed
    final_drive.calculate()

    motor.max_torque = final_drive.maximum_torque_input
    motor.rotational_speed = final_drive.rotational_speed_input
    motor.calculate()

    final_drive.torque_input = motor.output_torque
    final_drive.calculate()

    rear_wheel.applied_torque = final_drive.torque_output
    rear_wheel.calculate()

    vehicle.force = rear_wheel.applied_force

    driver.currentSpeed = vehicle.velocity / 27.7*100
    driver.desiredSpeed = command[1]
    driver.calculate()

    motor.effort = driver.effort

    #print(str(command[0]) + ", " + str(vehicle.velocity / 27.7*100))
    #print("Speed difference: " + str(vehicle.velocity / 27.7*100 - command[1]) + "\tEffort: " + str(motor.effort))
    #print(str(command[1]) + ", " + str(vehicle.velocity / 27.7*100))
    f.write(str(command[0]) + "," + str(command[1]) + "," + str(vehicle.velocity / 27.7*100) + "," + str(motor.output_power) + "," + str(motor.energy_used) + "," + str(vehicle.displacement) + "\n")
    #print(motor.output_power)

elapsed_time = time.time() - start_time
print("Calculation Complete in " + str(elapsed_time) + " seconds.")
f.close()
os.startfile("workfile.csv")