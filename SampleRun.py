import numpy as np
import time
from ControlsGuiModel import *
import matplotlib.pyplot as plt

motor = SystemModel()
open_loop = OpenLoopControl(motor)
closed_loop = ClosedLoopControl(motor)
# motor properties can be changed by modifying the value or can be read for a display.
motor.K_t = .136 # Nm Torque constant
motor.B_v = 0.003 #0.05 #0.0003 # Nm/(deg/s) Coefficient of viscous friction
motor.B_s = 0.02 #0.1 #0.002 # Nm Coefficient of static friction
motor.J_m = 0.00051 # kg*m^2 Rotor Moment of Inertia


M_a = .25	# kg
L_a = .01 # m
# updates the arms moment of inertia based on the length and mass provided.  The value itself can be read using motor.M_a and motor.L_a
motor.update_arm_mass(M_a) # updates the arm moment of inertia based on M_a and L_a
motor.update_arm_length(L_a) # updates the arm moment of inertia based on M_a and L_a

motor.use_grav = 1  # 0 for no gravity 1 for with gravity

# =========================================
# example OpenLoopControl

print('Starting open loop example')
open_loop.time_to_run = 5 # seconds
open_loop.current_cmd = .3 # Amps
open_loop.start_run = True # flag used to set timer will be set to false on first call to open_loop.step_control.  

angle = [] 
model_time = []
start_time = time.monotonic()

# this is just an example you can leave it running until it restarts plotting the most recent returned value.
while time.monotonic() - start_time < 10 :
	model_time.append(time.monotonic() - start_time)
	angle.append(open_loop.step_control())
	time.sleep(.04) # this simulates waiting to update the graphs so we aren't drawing too fast which can slow the system.

angle_deg = [x*180/np.pi for x in angle]
plt.figure(1)
plt.plot(model_time,angle_deg)
plt.title('Open Loop')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
print('Finished open loop example')

# =========================================
# example ClosedLoopControl	

print('Starting closed loop example')
motor.reset()
closed_loop.kp = .5
closed_loop.ki = .05
closed_loop.kd = 0.01#.5

closed_loop.angle_ref = np.deg2rad(45)

start_time = time.monotonic()
angle = [] 
model_time = []
# this is just an example you can leave it running until it restarts plotting the most recent returned value.
while time.monotonic() - start_time < 3 :
	model_time.append(time.monotonic() - start_time)
	angle.append(closed_loop.step_control())
	time.sleep(.02) # this simulates waiting to update the graphs so we aren't drawing too fast which can slow the system. Looks like the other overhead is about .02 s as well so this should update around 25 Hz.

angle_deg = np.rad2deg(angle) # [x*180/np.pi for x in angle]
plt.figure(2)
plt.plot(model_time,angle_deg)
plt.title('Closed Loop')
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')

print('Finished closed loop example')

plt.show()