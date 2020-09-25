from ControlsGuiModel import SystemModel, OpenLoopControl, ClosedLoopControl
import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import time
import tkinter
from tkinter import ttk

inputs = ['Mass (kg):', 'Length (m):', 'Desired Angle:',
          'Ki:', 'Bs:', 'Bv:', 'Jm:',
          'Current CMD (A):', 'Time (s):',
          'P:', 'I:', 'D:']
defaultVals = {'Input': inputs,
               'Default': [0.25, 0.010, 45,
                           0.136, 0.003, 0.020, 0.00051,
                           0.30, 5.00,
                           0.50, 0.05, 0.01],
               'Max': [999.00, 999.000, 720,
                       999.00, 999.00, 999.00, 999.00,
                       120.00, 50.00,
                       999.00, 999.00, 999.00],
               'Min': [0.00, 0.00, -720,
                       0, 0, 0, 0,
                       0.00, -50.00,
                       0.00, 0.00, 0.00],
               'Format': ['%.2f', '%.3f', '%.0f',
                          '%.3f', '%.4f', '%.3f', '%.5f',
                          '%.2f', '%.2f',
                          '%.2f', '%.3f', '%.3f'],
               'Steps': [0.05, 0.001, 5,
                         0.001, 0.0001, 0.001, 0.00001,
                         0.01, 0.10,
                         0.01, 0.001, 0.001],
               'Spinbox' : [0,0,0,0,0,0,0,0,0,0,0,0]}

defaultDf = pd.DataFrame(defaultVals)
helpDf = pd.read_csv('Documentation/inputDescriptions.txt', sep=" : ",
                     header=None, names=['Label', 'Text'], engine='python')

class App(tkinter.Frame):
    def __init__(self, master=None):
       super().__init__(master)
       self.gravityVar = tkinter.IntVar()
       self.loopVar = tkinter.StringVar()
       self.option_add('*Font', 'Arial 10')
       self.option_add('*Button.Font', 'Arial 10 bold')
       self.option_add('*Button.Foreground', '#FFF')
       self.option_add('*Spinbox.*Button.background', '#000000')
       # create master grid
       self.master = master
       self.master.grid_columnconfigure(0, weight=1)
       self.master.grid_columnconfigure(1, weight=1)
       # self.master.grid_columnconfigure(2, weight=1)
       for i in range(30):
           self.master.grid_rowconfigure(i, weight=1)
       self.grid(row=0, column=0, sticky='nsew')
       self.grid_columnconfigure(0, weight=1)
       self.grid_rowconfigure(0, weight=1)
       self.grid_columnconfigure(1, weight=1)
       self.grid_rowconfigure(1, weight=1)
       # title window
       self.master.title('ENG100 Controls')
       # create widgets and frames
       self.InitWidgets()
       # init model
       self.openRun = False
       self.closedRun = False
       self.motor, self.open_loop, self.closed_loop = self.InitModel()
       self.SetMotorValues()
       self.openLoopAnimation, self.closedLoopAnimation = self.InitAnimations()
       # time.sleep(0.1)
       # Window.MinimizeWindow(self.master)
       
    def InitAnimations(self):
        """
        Calls functions to initialize the open/closed loop animations
        """
        self.OpenLoopInit()
        self.openLoopAnimation = matplotlib.animation.FuncAnimation(self.scrollPlotFigure, self.OpenLoopAnimate, interval=20, repeat=True)
        self.openLoopAnimation.event_source.stop()
        self.ClosedLoopInit()
        self.closedLoopAnimation = matplotlib.animation.FuncAnimation(self.scrollPlotFigure, self.ClosedLoopAnimate, interval=20)
        self.closedLoopAnimation.event_source.stop()
        return self.openLoopAnimation, self.closedLoopAnimation
       
    def InitModel(self):
        """
        Initializes the model and returns the motor, open_loop, and closed_loop
        for use later
        """
        motor = SystemModel()
        open_loop = OpenLoopControl(motor)
        closed_loop = ClosedLoopControl(motor)
        return motor, open_loop, closed_loop
       
    def InitWidgets(self):
        """
        Sets up all the initial widgets in the GUI
        """
        self.motorArmFigure, self.motorArmCanvas = self.CreatePlot(0,0,15)
        self.motorArm = self.PlotMotorArm()
        self.scrollPlotFigure, self.scrollPlotCanvas = self.CreatePlot(15,0,15)
        self.MotorArmTabs()
        self.gravityButton = self.GravityButtons()
        self.controlTabs = self.ControlOptions()
        self.runButton = self.StartStopButton()
        resetFrame = self.ResetButtonFrame()
        # self.restartSimButton = self.RestartSimulationButton(resetFrame)
        self.resetDefaultsButton = self.ResetDefaultsButton(resetFrame)
        
    def SetArmProperties(self):
        """
        Sets the arm properties based on the user inputs
        """
        massInput, lengthInput, angleInput = self.ReturnMLAControls()
        M_a = massInput	# kg
        L_a = lengthInput # m
        # updates the arms moment of inertia based on the length and mass provided.  The value itself can be read using motor.M_a and motor.L_a
        self.motor.update_arm_mass(M_a) # updates the arm moment of inertia based on M_a and L_a
        self.motor.update_arm_length(L_a) # updates the arm moment of inertia based on M_a and L_a
        self.closed_loop.angle_ref = np.deg2rad(angleInput)
    
    def SetGravityProperties(self):
        gravitySetting = self.gravityVar.get()
        self.motor.use_grav = gravitySetting  # 0 for no gravity 1 for with gravity
        return self.motor.use_grav
        
    def SetMotorProperties(self):
        """
        Sets the motor properties based on the user inputs
        """
        # motor properties can be changed by modifying the value or can be read for a display.
        kiInput, bsInput, bvInput, jmInput = self.ReturnMotorProperties()
        self.motor.K_t = kiInput # Nm Torque constant
        self.motor.B_v = bsInput #0.05 #0.0003 # Nm/(deg/s) Coefficient of viscous friction
        self.motor.B_s = bvInput #0.1 #0.002 # Nm Coefficient of static friction
        self.motor.J_m = jmInput # kg*m^2 Rotor Moment of Inertia
    
    def SetMotorValues(self):
        """
        Resets motor and retrieves user's inputs for motor, arm, and open
        loop properties
        """
        self.motor.reset()
        self.SetMotorProperties()
        self.SetArmProperties()
        self.SetGravityProperties()
        self.SetOpenLoopProperties()
        
    def SetOpenLoopProperties(self):
        """
        Sets the current cmd and time to run for the open loop plotting
        """
        cmdAInput, timeInput = self.ReturnCommandControls()
        self.open_loop.time_to_run = timeInput # seconds
        self.open_loop.current_cmd = cmdAInput # Amps
    
    def CreatePlot(self, row, column, rowSpanCount):
        """
        Creates a matplotlib plot at the given row + column
        """
        frame = self.CreateFrame(row, column, 'nsew')
        frame.grid_configure(rowspan=rowSpanCount, columnspan=1)
        frame.grid_columnconfigure(column, weight=1)
        frame.grid_rowconfigure(row, weight=1)
        # figure = plt.Figure()
        # ax = figure.add_subplot(111)
        figure, ax = plt.subplots(1,1)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (deg)')
        canvas = FigureCanvasTkAgg(figure, master=frame)
        canvas.draw()
        canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')
        canvas.get_tk_widget().grid_rowconfigure(0, weight=1)
        canvas.get_tk_widget().grid_columnconfigure(0, weight=1)
        return figure, canvas
    
    def MotorArmTabs(self):
        frame = tkinter.LabelFrame(master=self, text='Motor Arm:', relief='ridge')
        frame.grid(row=0, column=1, columnspan=1, sticky='nsew')
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_rowconfigure(0, weight=1)
        frame.grid_columnconfigure(1, weight=1)
        frame.grid_rowconfigure(1, weight=1)
        # self.CreateToolTip(frame, 'Motor Arm')
        motorArmTabs = ttk.Notebook(frame, padding=0)
        armTab = self.CreateTab(motorArmTabs)
        motorTab = self.CreateTab(motorArmTabs)
        motorArmTabs.add(armTab, text='Arm Properties')
        motorArmTabs.add(motorTab, text='Motor Properties')
        motorArmTabs.grid()
        motorArmTabs.grid_rowconfigure(0, weight=1)
        motorArmTabs.grid_rowconfigure(1, weight=1)
        motorArmTabs.grid_rowconfigure(2, weight=1)
        motorArmTabs.grid_rowconfigure(3, weight=1)
        motorArmTabs.grid_columnconfigure(0, weight=1)
        motorArmTabs.grid_columnconfigure(1, weight=1)
        self.MLAControls(armTab)
        self.MotorProperties(motorTab)
        
    def MLAControls(self, armTab):
        """
        Mass, length, and desired angle spinboxes
        """
        labels = ['Mass (kg):', 'Length (m):', 'Desired Angle:']
        armTab.grid_columnconfigure(0, weight=1)
        armTab.grid_columnconfigure(1, weight=1)
        for i, label in enumerate(labels):
            armTab.grid_rowconfigure(i, weight=1)
            spinbox = self.AddSpinboxGrid(i, label, armTab)
            spinbox.config(command=self.SetArmProperties)
            
    def MotorProperties(self, motorTab):
        """
        Creates the motor properties spinboxes
        """
        propertyLabels = ['Ki:', 'Bs:', 'Bv:', 'Jm:']
        motorTab.grid_columnconfigure(0, weight=1)
        motorTab.grid_columnconfigure(1, weight=1)
        for i, label in enumerate(propertyLabels):
            motorTab.grid_rowconfigure(i, weight=1)
            spinbox = self.AddSpinboxGrid(i, label, motorTab)
            spinbox.config(command=self.SetMotorProperties)
            
    def GravityButtons(self):
        frame = tkinter.LabelFrame(master=self, text='Gravity:', relief='ridge')
        frame.grid(row=5, column=1, columnspan=1, rowspan=1, sticky='nsew')
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_columnconfigure(1, weight=1)
        frame.grid_rowconfigure(0, weight=1)
        frame.grid_rowconfigure(1, weight=1)
        onButton = tkinter.Radiobutton(frame, text='On', value=1,
                                       variable=self.gravityVar,
                                       command=self.PlotGravitySymbol)
        offButton = tkinter.Radiobutton(frame, text='Off', value=0,
                                        variable=self.gravityVar,
                                        command=self.PlotGravitySymbol)
        onButton.grid(row=0, column=0, sticky='nsew')
        offButton.grid(row=0, column=1, sticky='nsew')
        onButton.invoke()
        return onButton
        
    def ControlOptions(self):
        frame = tkinter.LabelFrame(master=self, text='Control:', relief='ridge')
        frame.grid(row=15, column=1, columnspan=2, rowspan=5, sticky='nsew')
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_rowconfigure(0, weight=1)
        # self.CreateToolTip(frame, 'Control')
        controlTabs = ttk.Notebook(frame, padding=(20, 10, 20, 10))
        openTab = self.CreateTab(controlTabs)
        closedTab = self.CreateTab(controlTabs)
        controlTabs.add(openTab, text='Open')
        controlTabs.add(closedTab, text='Closed')
        controlTabs.grid()
        self.CommandControls(openTab)
        self.PIDControls(closedTab)
        return controlTabs
    
    def CommandControls(self, cmdFrame):
        """
        Sets up command control spinboxes
        """
        labels = ['Current CMD (A):', 'Time (s):']
        cmdFrame.grid_columnconfigure(0, weight=1)
        cmdFrame.grid_columnconfigure(1, weight=1)
        for i, label in enumerate(labels):
            cmdFrame.grid_rowconfigure(i, weight=1)
            spinbox = self.AddSpinboxGrid(i, label, cmdFrame)
            spinbox.config(command=self.SetOpenLoopProperties)
        return cmdFrame
    
    def PIDControls(self, pidFrame):
        """
        Sets up PID spinboxes
        """
        labels = ['P:', 'I:', 'D:']
        for i, label in enumerate(labels):
            self.AddSpinboxGrid(i, label, pidFrame)
            buttonLabel = 'Reset ' + label.replace(':', '')
            if i != 1:
                reset = tkinter.Button(pidFrame, text=buttonLabel, bg='#0288D1',
                                command=lambda x=label: self.ResetSpinbox(x))
            else:
                reset = tkinter.Button(pidFrame, text=buttonLabel, bg='#0288D1',
                                command=self.ResetI)
                self.CreateToolTip(reset, buttonLabel)
            reset.grid(row=i, column=2, sticky='nsew')
        return pidFrame
    
    def ResetButtonFrame(self):
        """
        Contains restartSimButton and resetDefaultsButton
        """
        resetFrame = self.CreateFrame(26, 1, 'nsew')
        resetFrame.grid_columnconfigure(0, weight=1)
        resetFrame.grid_rowconfigure(0, weight=1)
        resetFrame.grid_columnconfigure(1, weight=1)
        return resetFrame

    def RestartSimulationButton(self, resetFrame):
        """
        Restarts the simulation using the user's inputs
        """
        restartSimButton = tkinter.Button(resetFrame, text='Restart Simulation',
                                          bg='#0288D1',
                                          command=self.RestartSimulation)
        restartSimButton.grid(row=0, column=0, padx=5, sticky='nsew')
        return restartSimButton
    
    def ResetDefaultsButton(self, resetFrame):
        """
        Restores all default input values
        """
        resetDefaultsButton = tkinter.Button(resetFrame,
                                             text='Reset to Default Values',
                                             bg='#0288D1',
                                             command=self.ResetDefaults)
        resetDefaultsButton.grid(row=0, column=1, sticky='nsew')
        return resetDefaultsButton
    
    def StartStopButton(self):
        """
        Creates the Start/Stop button for running the program
        """
        frame = self.CreateFrame(25, 1, 'nse')
        frame.grid_configure(columnspan=2)
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_rowconfigure(0, weight=1)
        runButton = tkinter.Button(frame, text='Start', bg='#009688', command=self.ActivateButton)
        runButton.grid(row=0, column=0, sticky='nsew')
        return runButton
    
    def AddSpinboxGrid(self, i, label, frame):
        """
        Adds the label and spinbox grid to the given frame at the given row
        """
        minVal = defaultDf.loc[defaultDf['Input'] == label, 'Min'].iloc[0]
        maxVal = defaultDf.loc[defaultDf['Input'] == label, 'Max'].iloc[0]
        defaultVal = defaultDf.loc[defaultDf['Input'] == label, 'Default'].iloc[0]
        stepVal = defaultDf.loc[defaultDf['Input'] == label, 'Steps'].iloc[0]
        formatVal = defaultDf.loc[defaultDf['Input'] == label, 'Format'].iloc[0]
        spinbox = self.CreateSpinboxGrid(i, 0, frame, label, minVal, maxVal)
        spinbox.insert(0, defaultVal)
        spinbox.config(increment=stepVal, format=formatVal, takefocus=False)
        defaultDf.loc[defaultDf['Input'] == label, 'Spinbox'] = spinbox
        return spinbox
    
    def CreateSpinboxGrid(self, rowNum, colNum, frame, label, fromNum, toNum):
        """
        Creates a label and spinbox grid
        """
        spinboxLabel = tkinter.Label(frame, text=label, justify='left',
                                     anchor='nw', wraplength=100)
        spinbox = tkinter.Spinbox(frame, from_=fromNum, to=toNum)
        spinboxLabel.grid(row=rowNum, column=colNum, padx=5, pady=5, sticky='nsw')
        spinbox.grid(row=rowNum, column=colNum+1, padx=5, pady=5, sticky='nsew')
        spinbox.grid_rowconfigure(rowNum, weight=1)
        spinbox.grid_columnconfigure(colNum, weight=1)
        self.CreateToolTip(spinboxLabel, label)
        return spinbox
        
    def CreateFrame(self, rowNum, colNum, stickyValue):
        """
        Creates a frame for holding widgets
        """
        frame = tkinter.Frame(master=self)
        frame.grid(row=rowNum, column=colNum, padx=(30, 10), pady=10,
                   sticky=stickyValue)
        return frame
    
    def CreateTab(self, frame):
        """
        Creates a tab for the Notebook widget
        """
        tab = tkinter.Frame(master=frame)
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_columnconfigure(1, weight=1)
        tab.grid_columnconfigure(2, weight=1)
        tab.grid_rowconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)
        tab.grid_rowconfigure(2, weight=1)
        return tab
    
    def CreateToolTip(self, widget, label):
        """
        Adds a tooltip hover event to the given widget based on the label
        """
        toolTip = ToolTip(widget)
        formattedLabel = (label.replace(':', '')).split(' (')[0]
        text = helpDf.loc[helpDf['Label'] == formattedLabel, 'Text'].iloc[0]
        def enter(event):
            toolTip.showtip(text)
        def leave(event):
            toolTip.hidetip()
        widget.bind('<Enter>', enter)
        widget.bind('<Leave>', leave)
    
    def ReturnControlType(self):
        """
        Returns the type of control currently selected, denoted by 0 (open)
        or 1 (closed)
        """
        currentTab = self.controlTabs.index('current')
        return currentTab
    
    def ReturnMLAControls(self):
        """
        Get values from mass, length, and desired angle spinboxes
        """
        mass = defaultDf.loc[defaultDf['Input'] == 'Mass (kg):', 'Spinbox'].iloc[0]
        length = defaultDf.loc[defaultDf['Input'] == 'Length (m):', 'Spinbox'].iloc[0]
        angle = defaultDf.loc[defaultDf['Input'] == 'Desired Angle:', 'Spinbox'].iloc[0]
        massInput = float(mass.get())
        lengthInput = float(length.get())
        angleInput = float(angle.get())
        return massInput, lengthInput, angleInput
            
    def ReturnCommandControls(self):
        """
        Get values from command control spinboxes
        """
        cmdA = defaultDf.loc[defaultDf['Input'] == 'Current CMD (A):', 'Spinbox'].iloc[0]
        time = defaultDf.loc[defaultDf['Input'] == 'Time (s):', 'Spinbox'].iloc[0]
        cmdAInput = float(cmdA.get())
        timeInput = float(time.get())    
        return cmdAInput, timeInput
    
    def ReturnPID(self):
        """
        Get values from PID spinboxes
        """
        p = defaultDf.loc[defaultDf['Input'] == 'P:', 'Spinbox'].iloc[0]
        i = defaultDf.loc[defaultDf['Input'] == 'I:', 'Spinbox'].iloc[0]
        d = defaultDf.loc[defaultDf['Input'] == 'D:', 'Spinbox'].iloc[0]
        pInput = float(p.get())
        iInput = float(i.get())
        dInput = float(d.get())
        return pInput, iInput, dInput
            
    def ReturnMotorProperties(self):
        ki = defaultDf.loc[defaultDf['Input'] == 'Ki:', 'Spinbox'].iloc[0]
        bs = defaultDf.loc[defaultDf['Input'] == 'Bs:', 'Spinbox'].iloc[0]
        bv = defaultDf.loc[defaultDf['Input'] == 'Bv:', 'Spinbox'].iloc[0]
        jm = defaultDf.loc[defaultDf['Input'] == 'Jm:', 'Spinbox'].iloc[0]
        kiInput = float(ki.get())
        bsInput = float(bs.get())
        bvInput = float(bv.get())
        jmInput = float(jm.get())
        return kiInput, bsInput, bvInput, jmInput
    
    def ResetI(self):
        defaultI = 0.050
        self.closed_loop.reset_int()
        iSpinbox = defaultDf.loc[defaultDf['Input'] == 'I:', 'Spinbox'].iloc[0]
        iSpinbox.delete(0, 'end')
        iSpinbox.insert(0, defaultI)
        
    def ResetSpinbox(self, inputType):
        print(inputType)
        spinbox = defaultDf.loc[defaultDf['Input'] == inputType, 'Spinbox'].iloc[0]
        defaultVal = defaultDf.loc[defaultDf['Spinbox'] == spinbox, 'Default'].iloc[0]
        spinbox.delete(0, 'end')
        spinbox.insert(0, defaultVal)
    
    def ResetDefaults(self):
        """
        Resets the inputs to their default values
        """
        # self.gravityVar.set(1)
        self.gravityButton.invoke()
        for i, label in enumerate(inputs):
            spinbox = defaultDf.loc[defaultDf['Input'] == label, 'Spinbox'].iloc[0]
            defaultVal = defaultDf.loc[defaultDf['Input'] == label, 'Default'].iloc[0]
            if label == 'Desired Angle:':
                defaultVal = int(defaultVal)
            spinbox.delete(0, 'end')
            spinbox.insert(0, defaultVal)
    
    def RestartSimulation(self):
        # self.runButton.invoke()
        self.motor, self.open_loop, self.closed_loop = self.InitModel()
        self.SetMotorValues()
        self.openLoopAnimation, self.closedLoopAnimation = self.InitAnimations()
        self.RotateMotorArm(0)
        # if self.runButton['text'] == 'Start':
        #     self.runButton.invoke()
        return (self.motor, self.open_loop, self.closed_loop,
                self.openLoopAnimation, self.closedLoopAnimation)
    
    def ActivateButton(self):
        if self.runButton['text'] == 'Start':
            self.runButton.configure(text='Stop', bg='#d32f2f', command=self.ActivateButton)
            self.StartAnimation()
        elif self.runButton['text'] == 'Stop':
            self.runButton.configure(text='Start', bg='#009688', command=self.ActivateButton)
            self.StopAnimation()
    
    def StartAnimation(self):
        loopType = self.ReturnControlType()
        # self.SetMotorValues()
        self.RestartSimulation()
        if loopType == 0:
            self.openRun = True
            self.closedRun = False
            self.OpenLoopInit()
            self.openLoopAnimation.event_source.start()
            self.closedLoopAnimation.event_source.stop()
        elif loopType == 1:
            self.openRun = False
            self.closedRun = True
            self.ClosedLoopInit()
            self.closedLoopAnimation.event_source.start()
            self.openLoopAnimation.event_source.stop()
        return self.openRun, self.closedRun
        
    def StopAnimation(self):
        loopType = self.ReturnControlType()
        
        if loopType == 0:
            self.openLoopAnimation.event_source.stop()
            self.openRun = False
            return self.openRun
        elif loopType == 1:
            self.closedLoopAnimation.event_source.stop()
            self.closedRun = False
            return self.closedRun
            
    def PlotGravitySymbol(self):
        """
        Add/remove gravity symbol to plot when gravity is switched on/off
        """
        try:
            gravitySetting = self.SetGravityProperties()
        except:
            gravitySetting = self.gravityVar.get()
        figure = self.motorArmFigure
        ax = figure.get_axes()[0]
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        if gravitySetting == 1:
            self.gravitySymbol = ax.annotate('g', xy=(10,7), xytext=(10, 10),
                                             size=18, ha='center', va='top',
                                        arrowprops=dict(facecolor='black', shrink=0.1))
        elif gravitySetting == 0:
            try:
                self.gravitySymbol.remove()
            except:
                pass
        figure.canvas.draw()
        return self.gravitySymbol
    
    def PlotAngleText(self, angle, figure, ax):
        text = 'Angle: ' + str(round(angle, 0))
        try:
            self.angleText.remove()
        except:
            pass
        self.angleText = ax.annotate(text, xy=(0,10), xytext=(0, 10), size=10,
                                     ha='center', va='top')
        return self.angleText
    
    def PlotMotorArm(self):
        """
        Creates the initial motor arm image
        """
        figure = self.motorArmFigure
        ax = figure.get_axes()[0]
        self.motorArmCenter = plt.Circle((5, 5), radius=1)
        self.motorArmRect = plt.Rectangle((4.5, 2), 1, 3)
        patches = [self.motorArmCenter, self.motorArmRect]
        self.motorArm = PatchCollection(patches, fc='gray')
        ax.add_collection(self.motorArm)
        ax.spines['left'].set_position('center')
        ax.spines['right'].set_color('none')
        ax.spines['bottom'].set_position('center')
        ax.spines['top'].set_color('none')
        ax.set_xticks([0, 10])
        ax.set_xticklabels([270, 90])
        ax.set_yticks([0, 10])
        ax.set_yticklabels([0, 180])
        ax.set_xlabel('')
        ax.set_ylabel('')
        for i, spine in ax.spines.items():
            spine.set_zorder(-1)
        ax.set_axisbelow(True)
        ax.axis('scaled')
        # figure.tight_layout()
        figure.canvas.draw()
        return self.motorArm
        
    def RotateMotorArm(self, angleDeg):
        """
        Rotates the motor arm image based on angleDeg input
        """
        figure = self.motorArmFigure
        ax = figure.get_axes()[0]
        angleRotate = matplotlib.transforms.Affine2D().rotate_deg_around(5,5, angleDeg) + ax.transData
        self.motorArm.set_transform(angleRotate)
        self.angleText = self.PlotAngleText(angleDeg, figure, ax)
        figure.canvas.draw()
        return self.motorArm, self.angleText 

    def OpenLoopInit(self):
        """
        Initializes the open loop animation
        """
        # cmdAInput, timeInput = self.ReturnCommandControls()
        # self.open_loop.time_to_run = timeInput # seconds
        # self.open_loop.current_cmd = cmdAInput # Amps
        self.SetOpenLoopProperties()
        self.open_loop.start_run = True # flag used to set timer will be set to false on first call to open_loop.step_control.  
        
        figure = self.scrollPlotFigure
        figure.clear()
        ax = figure.add_subplot(111)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (deg)')
        ax.set_xlim(-10, 0)
        figure.canvas.draw()
        self.angle = [] 
        self.model_time = []
        self.start_time = time.monotonic()
        self.line = []
        return self.start_time, self.angle, self.model_time, self.line
    
    def OpenLoopAnimate(self, i):
        """
        Animates the open loop plot as long as the 'Stop' button has not been
        pushed.
        """
        # while self.runCommand == True:
        while self.openRun == True:
            figure = self.scrollPlotFigure
            ax = figure.get_axes()[0]
            ax.set_title('Open Loop')
            step_control = self.open_loop.step_control(i)
            self.model_time.append(time.monotonic() - self.start_time)
            self.angle.append(step_control)
            angle_deg = np.rad2deg(self.angle)
            if self.line == []:
               self.line, = ax.plot(self.model_time, angle_deg, color='blue')
            else:
                self.line.set_xdata(self.model_time)
                self.line.set_ydata(angle_deg)
            ax.set_xlim((self.model_time[-1]-10), self.model_time[-1])
            ax.set_ylim(angle_deg.min(), angle_deg.max()+10)
            figure.canvas.draw()
            self.RotateMotorArm(angle_deg[-1])
            return self.line,

    def ClosedLoopInit(self):
        """
        Initializes the closed loop animation
        """
        massInput, lengthInput, angleInput = self.ReturnMLAControls()
        M_a = massInput	# kg
        L_a = lengthInput # m
        # updates the arms moment of inertia based on the length and mass provided.  The value itself can be read using motor.M_a and motor.L_a
        self.motor.update_arm_mass(M_a) # updates the arm moment of inertia based on M_a and L_a
        self.motor.update_arm_length(L_a) # updates the arm moment of inertia based on M_a and L_a
        
        pInput, iInput, dInput = self.ReturnPID()
        self.closed_loop.kp = pInput
        self.closed_loop.ki = iInput
        self.closed_loop.kd = dInput
        
        self.closed_loop.angle_ref = np.deg2rad(angleInput)
        
        gravitySetting = self.gravityVar.get()
        self.motor.use_grav = gravitySetting  # 0 for no gravity 1 for with gravity
        figure = self.scrollPlotFigure
        figure.clear()
        ax = figure.add_subplot(111)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (deg)')
        ax.set_xlim(-10, 0)
        figure.canvas.draw()
        self.start_time = time.monotonic()
        self.angle = [] 
        self.model_time = []
        self.line = []
        return self.start_time, self.angle, self.model_time, self.line
        
    def ClosedLoopAnimate(self, i):
        """
        Animates the closed loop plot as long as the 'Stop' button has not been
        pushed.
        """
        # while self.runCommand == True:
        while self.closedRun == True:
            step_control = self.closed_loop.step_control()
            figure = self.scrollPlotFigure
            ax = figure.get_axes()[0]
            ax.set_title('Closed Loop')
            
            self.model_time.append(time.monotonic() - self.start_time)
            self.angle.append(step_control)
            angle_deg = np.rad2deg(self.angle)
            if self.line == []:
               self.line, = ax.plot(self.model_time, angle_deg, color='blue')
            else:
                self.line.set_xdata(self.model_time)
                self.line.set_ydata(angle_deg)
            ax.set_xlim((self.model_time[-1]-10), self.model_time[-1])
            ax.set_ylim(angle_deg.min(), angle_deg.max()+10)
            figure.canvas.draw()
            self.RotateMotorArm(angle_deg[-1])
            return self.line,
        
class ToolTip(object):

    def __init__(self, widget):
        self.widget = widget
        self.tipwindow = None
        self.id = None
        self.x = self.y = 0

    def showtip(self, text):
        """
        Displays tooltip on hover
        """
        self.text = text
        if self.tipwindow or not self.text:
            return
        x, y, cx, cy = self.widget.bbox('insert')
        x = x + self.widget.winfo_rootx() + 20
        y = y + cy + self.widget.winfo_rooty() + 20
        self.tipwindow = tw = tkinter.Toplevel(self.widget)
        tw.wm_overrideredirect(1)
        tw.wm_geometry('+%d+%d' % (x, y))
        label = tkinter.Label(tw, text=self.text, justify='left',
                      background='#ffffe0', relief='solid', borderwidth=1,
                      font=('tahoma', '10', 'normal'), wraplength=200)
        label.pack(ipadx=1)

    def hidetip(self):
        """
        Hides tooltip once mouse leaves hover object
        """
        tw = self.tipwindow
        self.tipwindow = None
        if tw:
            tw.destroy()
    
class Window:
    def AddMenuBar(self):
        """
        Creates menu at the top of the window
        """
        menuBar = tkinter.Menu(self)
        resizeMenu = tkinter.Menu(menuBar, tearoff=0)
        helpMenu = tkinter.Menu(menuBar, tearoff=0)
        resizeMenu.add_command(label='Fullscreen Window', command=lambda: Window.MaximizeWindow(self))
        resizeMenu.add_command(label='Small Window', command=lambda: Window.MinimizeWindow(self))
        helpMenu.add_command(label='Definitions', command=lambda: Window.HelpWindow(self))
        menuBar.add_cascade(label='Window Resize Options', menu=resizeMenu)
        menuBar.add_cascade(label='Help', menu=helpMenu)
        self.config(menu=menuBar)
        
    def HelpWindow(self):
        helpWindow = tkinter.Toplevel()
        helpWindow.iconphoto(False, icon)
        helpWindow.title('Definitions')
        textBox = tkinter.Text(helpWindow)
        Window.ReturnHelpContent(self, textBox)
        textBox.configure(state='disabled')
        scroll = tkinter.Scrollbar(helpWindow, command=textBox.yview)
        textBox.grid(row=0, column=0, sticky='nsew')
        scroll.grid(row=0, column=1, sticky='nsew')
        textBox['yscrollcommand'] = scroll.set
        
    def ReturnHelpContent(self, textBox):
        with open('Documentation/inputDescription.txt', 'r') as f:
            textBox.insert('1.0', f.read())
        
    # def CenterWindow(self):
    #     w = self.winfo_screenwidth()
    #     h = self.winfo_screenheight()
    #     self.geometry('%dx%d' % (w/2, h/2))
    #     size = tuple(int(pos) for pos in self.geometry().split('+')[0].split('x'))
    #     x = w/2 - size[0]/2
    #     y = h/2 - size[1]/2
    #     self.geometry('%dx%d+%d+%d' % (w/2, h/2, x, y))
        
    def MaximizeWindow(self):
        """
        Resizes the window to full screen width and height
        """
        w = self.winfo_screenwidth()
        h = self.winfo_screenheight()
        print(w, h)
        self.geometry('%dx%d+%d+%d' % (w, h, 0, 0))
        print(self.geometry())
        
    def MinimizeWindow(self):
        """
        Resizes the window to half the screen width and height
        """
        w = self.winfo_screenwidth()
        h = self.winfo_screenheight()
        print(w/2, h/2)
        # Window.CenterWindow(self)
        self.geometry('%dx%d+%d+%d' % (w*.5, h*.5, 0, 0))
        print(self.geometry())
        
def _delete_window():
    try:
        root.destroy()
    except:
        pass

if __name__ == '__main__':
    root = tkinter.Tk()
    root.resizable(True, True)
    # root.minsize(960, 540)
    root.protocol("WM_DELETE_WINDOW", _delete_window)
    icon = tkinter.PhotoImage(file='favicon.png')
    root.iconphoto(False, icon)
    app = App(master=root)
    Window.AddMenuBar(root)
    # time.sleep(0.1)
    # Window.MinimizeWindow(root)
    app.mainloop()
    
