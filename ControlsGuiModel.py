# Model
import numpy as np
import time
from scipy.integrate import odeint
import os, sys
from datetime import datetime

G = 9.81 # Gravitational constant  
if getattr(sys, 'frozen', False):
    # frozen
    SAVE_DIR = os.path.join(os.path.dirname(sys.executable), 'logs')
else:
    SAVE_DIR = os.path.join(os.path.dirname(__file__), 'logs') # we want to store the files in the same place this file lives in a directory called logs
# print(SAVE_DIR)
if not os.path.exists(SAVE_DIR): # check if the directory exists
    os.makedirs(SAVE_DIR) # if it doesn't exist create it.
    
class SystemModel:
    
    def __init__(self):
        self.theta_rad = [0]*3 #[np.pi/2]*3 #[0, 0, 0] # Motor angle
        self.d_theta = [0, 0] # Motor velocity
        self.dd_theta = [0] # Motor acceleration

        self.time = 0 # current time of the model
        self.timestep = .001 # default time step size.
        self.time_prev = 0 # record time of last call
        self.time_diff = 0 # record the difference between call times,  this is just used for logging.
        # motor parameters
        self.K_t = 0.0 # Torque constant
        self.B_v = 0.0 # Coefficient of viscous friction
        self.B_s = 0.0 # Coefficient of static friction
        self.J_m = 0.0 # Rotor Moment of Inertia
        
        # arm parameters
        self.M_a = 0.0 # Arm Mass
        self.L_a = 0.0 # Length of the Arm
        self.J_a = 0.0 # Arm Moment of inertia
        self.update_arm_inertia() # updates the arm moment of inertia based on M_a and L_a
    
        self.use_grav = 0
        self.should_log = False
        self.current_applied = 0 
        
    def reset(self):
        self.theta_rad = [0, 0, 0] # Motor angle
        self.d_theta = [0, 0] # Motor velocity
        self.dd_theta = [0] # Motor acceleration
        self.time_prev = 0
    
    def update_arm_mass(self, M_a):
        self.M_a = M_a # Arm Mass
        self.update_arm_inertia()
        
    def update_arm_length(self, L_a):
        self.L_a = L_a # Length of the Arm
        self.update_arm_inertia()
        
    def update_arm_inertia(self):
        self.J_a = 1/3 * self.M_a * np.square(self.L_a) # Arm Moment of inertia
        
    def update_K_t(self, K_t):
        self.K_t = K_t
        return self.K_t
    
    def update_B_v(self, B_v):
        self.B_v = B_v
        return self.B_v
    
    def update_B_s(self, B_s):
        self.B_s = B_s
        return self.B_s
    
    def update_J_m(self, J_m):
        self.J_m = J_m
        return self.J_m
    
    def update_timestep(self, timestep):
        self.timestep = timestep
        return self.timestep
    
    def update_logging(self, logOption):
        self.should_log = logOption
        print(self.should_log)
        return self.should_log
        
    def model(self, thetas, t, i):
        theta = thetas[0]
        theta_dot = thetas[1]
        theta_dot_dot = (self.K_t * i - self.B_v * theta_dot \
                        - self.M_a * self.L_a * G * self.use_grav * np.sin(theta) \
                        - np.sign(theta_dot) * self.B_s)\
                        /(self.J_m + self.J_a)
        return (theta_dot, theta_dot_dot)
        
    
    def step_model (self, i):
        # i is the current command
        self.time = self.time + self.timestep # increment the time
        #self.time = time.monotonic();
        self.current_applied = i 
        
        if self.time_prev == 0 : # if we haven't run it before just pretend the last time was one timestep back.
            self.time_prev = self.time - self.timestep        
        
        ##++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # take one step using the current values
        # dd_theta_current = (self.K_t * i - self.B_v * self.d_theta[0] \
                            # - self.M_a * self.L_a * G * self.use_grav * np.sin(self.theta[0]) \
                            # - np.sign(self.d_theta[0]) * self.B_s)\
                            # /(self.J_m + self.J_a) 
        # d_theta_current = self.dd_theta[0] * self.timestep + self.d_theta[0]
        # theta_current = .5 * self.dd_theta[0] * np.square(self.timestep) + self.d_theta[0] * self.timestep + self.theta[0]
        ##++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        ##============================================================
        # Using odeint
        sol = odeint(self.model, [self.theta_rad[0], self.d_theta[0]], [0, self.time - self.time_prev],  args = (i,))
        
        theta_current = sol[-1,0]
        d_theta_current = sol[-1,1]
        #dd_theta_current = 
        
        self.time_prev = self.time
        ##============================================================
        
        #theta_current = np.arccos((self.K_t * i - (self.J_m + self.J_a) * self.dd_theta[0] - self.B_v * self.d_theta[0] - np.sign(self.d_theta[0]) * self.B_s) / (self.M_a * self.L_a * G))
        self.theta_rad.insert(0,theta_current) # add the updated value to the front of the list
        self.theta_rad.pop(-1) # remove the last value so the list stays the same size
        # update the velocity and acceleration
        
        #d_theta_current = (self.theta[1] - self.theta[0]) / self.timestep
        self.d_theta.insert(0,d_theta_current) # add the updated value to the front of the list
        self.d_theta.pop(-1) # remove the last value so the list stays the same size
        
        #dd_theta_current = (self.d_theta[1] - self.d_theta[0]) / self.timestep
        # self.dd_theta.insert(0,dd_theta_current) # add the updated value to the front of the list
        # self.dd_theta.pop(-1) # remove the last value so the list stays the same size
        
        return self.theta_rad
    
class OpenLoopControl:
    def __init__(self, model):
        self.time_to_run = 0
        self.start_time = 0
        self.time_elapsed = 0
        self.current_cmd = 0
        self.model = model
        self.time_prev = 0
        
        self.start_run = False
        self.log_name = 'temp.csv'
        self.model_labels = ['time', 'time_diff', 'theta_rad', 'd_theta', 'dd_theta', 'current_applied', \
                            'K_t', 'B_s', 'B_v', 'J_m', 'M_a', 'L_a','J_a', 'use_grav']
        self.controller_labels = ['time_prev', 'time_to_run', 'start_time', 'time_elapsed', 'current_cmd']
        self.data_file = '' # want something to store the file object, this is a quick way of doing it where it gets changed later, which isn't great but is quick.
        
        
    def update_current_cmd(self, cmd):
        self.current_cmd = cmd
        return self.current_cmd
    
    def update_time_to_run(self, time_to_run):
        self.time_to_run = time_to_run
        return self.time_to_run
        
    def step_control(self, time_to_run = None, current_cmd = None):
        # print('step_control: ', self.model.time)
        if self.start_run : # if we are starting a new run record the start time and reset the flag
            # update current and time command from gui
            #self.time_to_run = time_to_run
            #self.current_cmd = current_cmd
            #self.start_time = time.monotonic()
            self.start_run = False
            #self.create_log()
        # step_start_time = time.monotonic() # record the time we are starting the step
        # if self.time_to_run > step_start_time - self.start_time : # if we haven't gone past the time_to_run wait a timestep then step the model to help slow down the display.  This will be a bit slower than the actual time but should be good enough for this.  The order should help a bit.
        # print(self.time_to_run, self.model.timestep, self.model.time)
        if self.time_prev == 0 : # if we haven't run it before just pretend the last time was one timestep back.
            self.time_prev = self.model.time - self.model.timestep    
        self.model.time_diff = self.model.time - self.time_prev
        # # Logging 
        if self.model.should_log:
            self.log_data()
        self.time_prev = self.model.time #record previous time
        if self.time_to_run > self.model.time :
            self.model.step_model(self.current_cmd) # step the model
            
            
        else : # if we have run the current command for the full time, send 0 current till a new run is started.
            self.model.step_model(0) # step the model
        #self.log_data()    
        return self.model.theta_rad[0]
    
    def create_log(self):
        start_time = datetime.now()
        time_str = start_time.strftime("%Y_%m_%d_%Hh%Mm%Ss")
        control_type = '_open_loop'
        
        # you need to initilize these before calling these
        file_base = time_str + control_type
        file_extension = '.csv'
        
        self.log_name = os.path.join(SAVE_DIR, file_base + file_extension)
        
        self.data_file = open(self.log_name, 'a')
        
        labels_csv = ""
        for label in self.model_labels :
            labels_csv += label + ", "
        for label in self.controller_labels:
            labels_csv += label + ", "
        labels_csv = labels_csv[:-2] # remove the final coma and space
        self.data_file.write(labels_csv)
        self.data_file.write("\n ")
        #print('wrote ' + str(self.log_name) + ': \n' + labels_csv)
        
    def log_data(self):
        data_csv = '';
        for label in self.model_labels :
            #print(label + ' = ' + str(getattr(self.model, label )))
            attribute_data = getattr(self.model, label)
            if type(attribute_data) is list: 
                attribute_data = attribute_data[0]
            data_csv += str(attribute_data) + ', '
        for label in self.controller_labels:
            attribute_data = getattr(self, label)
            if type(attribute_data) is list: 
                attribute_data = attribute_data[0]
            data_csv += str(attribute_data) + ', '
        data_csv = data_csv[:-2] # remove the final coma and space
        self.data_file.write(data_csv)
        self.data_file.write("\n ")
        #print('wrote ' + str(self.log_name) + ': \n' + data_csv)
    
    def close_log(self):
        self.data_file.close()

class ClosedLoopControl:
    def __init__(self, model):
        # PID gains
        self.kp = 0
        self.ki = 0
        self.kd = 0
        
        self.time = 0 # current time of the model
        self.time_prev = 0 # record time of last call
        self.e_der = 0
        self.e = 0 #this version of the variable isn't used in calculations, just logging
        
        self.error_sum = 0 # integral term
        
        self.angle_ref = 0 # reference (desired) angle
        
        self.current_cmd = 0
        
        self.model = model
        self.e_prev = self.angle_ref - self.model.theta_rad[0] # stores the previous error for derivative term
        
        self.log_name = 'temp.csv'
        self.model_labels = ['time', 'time_diff', 'theta_rad', 'd_theta', 'dd_theta', 'current_applied',\
                            'K_t', 'B_s', 'B_v', 'J_m', 'M_a', 'L_a','J_a', 'use_grav']
        self.controller_labels = ['time_prev', 'kp', 'ki', 'kd', 'error_sum', 'e', 'e_prev', 'angle_ref', 'current_cmd']
        self.data_file = '' # want something to store the file object, this is a quick way of doing it where it gets changed later, which isn't great but is quick.
        
        
    def update_angle_ref(self, angle):
        self.angle_ref = np.deg2rad(angle)
        return self.angle_ref
    
    def update_kp(self, kp):
        self.kp = kp
        return self.kp
    
    def update_ki(self, ki):
        self.ki = ki
        return self.ki
    
    def update_kd(self, kd):
        self.kd = kd
        return self.kd
        
    def step_control(self):
        
        # self.time = time.monotonic();
        self.time = self.model.time
        # print('step_control: ', self.model.time)
        
        if self.time_prev == 0 : # if we haven't run it before just pretend the last time was one timestep back.
            self.time_prev = self.time - self.model.timestep
        # print(self.e_prev)
        if self.time == 0:
            self.e_prev = self.angle_ref - self.model.theta_rad[0]
            #self.create_log()
        # this method of tracking time isn't great because it doesn't align with the model calls but should be functional.  
        self.model.time_diff = self.time - self.time_prev
        # print(self.model.time, self.model.time_diff)
        
        # step_start_time = time.monotonic() # record the time we are starting the step
        # time_cur = time.monotonic() # record the time we are starting
        e = self.angle_ref - self.model.theta_rad[0]# error angle
        self.e = e #this is just for writing to the csv, it does not effect the equation
        self.error_sum = self.error_sum + e * self.model.time_diff # we will multiply by the timestep later and assume they are all equal
        self.e_der = (e - self.e_prev) / self.model.time_diff
        self.current_cmd = (self.kp * e +\
                            self.ki * self.error_sum +\
                            self.kd * self.e_der)
        # print('time_prev', self.time_prev, 'time: ', self.time, 'angle_ref: ', self.angle_ref, 'current_cmd: ', self.current_cmd, 'e: ', e, 'e_prev: ', self.e_prev)
        # # Logging 
        if self.model.should_log:
            self.log_data()
        self.e_prev = e # store the current error for the next run
        self.time_prev = self.time #record previous time
        self.model.step_model(self.current_cmd) # step the model
        #self.log_data()
        
        return self.model.theta_rad[0] 
 
        
    def reset_int(self):
        self.error_sum = 0
        
    def create_log(self):
        start_time = datetime.now()
        time_str = start_time.strftime("%Y_%m_%d_%Hh%Mm%Ss")
        control_type = '_closed_loop'
        
        # you need to initilize these before calling these
        file_base = time_str + control_type
        file_extension = '.csv'
        
        self.log_name = os.path.join(SAVE_DIR, file_base + file_extension)
        
        self.data_file = open(self.log_name, 'a')
        
        labels_csv = ""
        for label in self.model_labels :
            labels_csv += label + ", "
        for label in self.controller_labels:
            labels_csv += label + ", "
        labels_csv = labels_csv[:-2] # remove the final coma and space
        self.data_file.write(labels_csv)
        self.data_file.write("\n ")
        #print('wrote ' + str(self.log_name) + ': \n' + labels_csv)
        
    def log_data(self):
        data_csv = '';
        for label in self.model_labels :
            # print(label + ' = ' + str(getattr(self.model, label )))
            attribute_data = getattr(self.model, label)
            if type(attribute_data) is list: 
                attribute_data = attribute_data[0]
            data_csv += str(attribute_data) + ', '
        for label in self.controller_labels:
            attribute_data = getattr(self, label)
            if type(attribute_data) is list: 
                attribute_data = attribute_data[0]
            data_csv += str(attribute_data) + ', '
        data_csv = data_csv[:-2] # remove the final coma and space
        self.data_file.write(data_csv)
        self.data_file.write("\n ")
        #print('wrote ' + str(self.log_name) + ': \n' + data_csv)
        
    def close_log(self):
        self.data_file.close()