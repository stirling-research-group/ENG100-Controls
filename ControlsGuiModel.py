# Model
import numpy as np
import time
from scipy.integrate import odeint

G = 9.81 # Gravitational constant    
    
class SystemModel:
    
    def __init__(self):
        self.theta = [0]*3 #[np.pi/2]*3 #[0, 0, 0] # Motor angle
        self.d_theta = [0, 0] # Motor velocity
        self.dd_theta = [0] # Motor acceleration

        self.time = 0 # current time of the model
        self.timestep = .001 # default time step size.
        self.time_prev = 0 # record time of last call
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
        
    def reset(self):
        self.theta = [0, 0, 0] # Motor angle
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
        #self.time = self.time + self.timestep # increment the time
        self.time = time.monotonic();
        
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
        sol = odeint(self.model, [self.theta[0], self.d_theta[0]], [0, self.time - self.time_prev],  args = (i,))
        
        theta_current = sol[-1,0]
        d_theta_current = sol[-1,1]
        #dd_theta_current = 
        
        self.time_prev = self.time
        ##============================================================
        
        #theta_current = np.arccos((self.K_t * i - (self.J_m + self.J_a) * self.dd_theta[0] - self.B_v * self.d_theta[0] - np.sign(self.d_theta[0]) * self.B_s) / (self.M_a * self.L_a * G))
        self.theta.insert(0,theta_current) # add the updated value to the front of the list
        self.theta.pop(-1) # remove the last value so the list stays the same size
        # update the velocity and acceleration
        
        #d_theta_current = (self.theta[1] - self.theta[0]) / self.timestep
        self.d_theta.insert(0,d_theta_current) # add the updated value to the front of the list
        self.d_theta.pop(-1) # remove the last value so the list stays the same size
        
        #dd_theta_current = (self.d_theta[1] - self.d_theta[0]) / self.timestep
        # self.dd_theta.insert(0,dd_theta_current) # add the updated value to the front of the list
        # self.dd_theta.pop(-1) # remove the last value so the list stays the same size
        
        return self.theta
    
class OpenLoopControl:
    def __init__(self, model):
        self.time_to_run = 0
        self.start_time = 0
        self.time_elapsed = 0
        self.current_cmd = 0
        self.model = model
        self.start_run = False
        
    def update_current_cmd(self, cmd):
        self.current_cmd = cmd
        return self.current_cmd
    
    def update_time_to_run(self, time_to_run):
        self.time_to_run = time_to_run
        return self.time_to_run
        
    def step_control(self, time_to_run = None, current_cmd = None):
        if self.start_run : # if we are starting a new run record the start time and reset the flag
            # update current and time command from gui
            #self.time_to_run = time_to_run
            #self.current_cmd = current_cmd
            self.start_time = time.monotonic()
            self.start_run = False
        step_start_time = time.monotonic() # record the time we are starting the step
        if self.time_to_run > step_start_time - self.start_time : # if we haven't gone past the time_to_run wait a timestep then step the model to help slow down the display.  This will be a bit slower than the actual time but should be good enough for this.  The order should help a bit.
            self.model.step_model(self.current_cmd) # step the model
            
            
        else : # if we have run the current command for the full time, send 0 current till a new run is started.
            self.model.step_model(0) # step the model
            
        return self.model.theta[0]
        
    def step_control_internal_timer(self, time_to_run = None, current_cmd = None):
        if self.start_run : # if we are starting a new run record the start time and reset the flag
            # update current and time command from gui
            #self.time_to_run = time_to_run
            #self.current_cmd = current_cmd
            self.start_time = time.monotonic()
            self.start_run = False
        step_start_time = time.monotonic() # record the time we are starting the step
        if self.time_to_run > step_start_time - self.start_time : # if we haven't gone past the time_to_run wait a timestep then step the model to help slow down the display.  This will be a bit slower than the actual time but should be good enough for this.  The order should help a bit.
            time_cur = time.monotonic() # record the time we are starting
            self.model.step_model(self.current_cmd) # step the model
            while time_cur - step_start_time < self.model.timestep :    # wait till we get past the timestep.
                time_cur = time.monotonic()
            
        else : # if we have run the current command for the full time, send 0 current till a new run is started.
            time_cur = time.monotonic() # record the time we are starting
            self.model.step_model(0) # step the model
            while time_cur - step_start_time < self.model.timestep :    # wait till we get past the timestep.
                time_cur = time.monotonic()
        return self.model.theta[0]

class ClosedLoopControl:
    def __init__(self, model):
        # PID gains
        self.kp = 0
        self.ki = 0
        self.kd = 0
        
        self.time = 0 # current time of the model
        self.time_prev = 0 # record time of last call
        
        self.error_sum = 0 # integral term
        self.e_prev = 0 # stores the previous error for derivative term
        
        self.angle_ref = 0 # reference (desired) angle
        
        self.current_cmd = 0
        
        self.model = model
        
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
        
        self.time = time.monotonic();
        
        if self.time_prev == 0 : # if we haven't run it before just pretend the last time was one timestep back.
            self.time_prev = self.time - self.model.timestep    
        # this method of tracking time isn't great because it doesn't align with the model calls but should be functional.  
        time_between_calls = self.time - self.time_prev
        
        # step_start_time = time.monotonic() # record the time we are starting the step
        # time_cur = time.monotonic() # record the time we are starting
        e = self.model.theta[0] - self.angle_ref # error angle
        # print(self.time_prev, self.model.timestep, self.model.theta[0])
        self.error_sum = self.error_sum + e # we will multiply by the timestep later and assume they are all equal
        self.current_cmd = -1 * (self.kp * e +\
                            self.ki * self.error_sum * time_between_calls +\
                            self.kd * (e - self.e_prev) / time_between_calls) 
        self.e_prev = e # store the current error for the next run
        self.model.step_model(self.current_cmd) # step the model
        
        return self.model.theta[0]    
        
    def step_control_internal_timer(self):
        step_start_time = time.monotonic() # record the time we are starting the step
        time_cur = time.monotonic() # record the time we are starting
        e = self.model.theta[0] - self.angle_ref # error angle
        self.error_sum = self.error_sum + e # we will multiply by the timestep later and assume they are all equal
        self.current_cmd = self.kp * e +\
                            self.ki * self.error_sum * self.model.timestep +\
                            self.kd * (e - self.e_prev) / self.model.timestep
        self.e_prev = e # store the current error for the next run
        self.model.step_model(self.current_cmd) # step the model
        while time_cur - step_start_time < self.model.timestep :    # wait till we get past the timestep this makes it close to realtime
            time_cur = time.monotonic()
        return self.model.theta[0]    
        
        
    def reset_int(self):
        self.error_sum = 0