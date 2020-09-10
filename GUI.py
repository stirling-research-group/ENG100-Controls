import tkinter
from tkinter import ttk
import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
import threading
import time
from ControlsGuiModel import SystemModel, OpenLoopControl, ClosedLoopControl
runCommand = False

class App(tkinter.Frame):
    def __init__(self, master=None):
       super().__init__(master)
       self.gravityVar = tkinter.IntVar()
       self.loopVar = tkinter.StringVar()
       self.option_add('*Font', 'Arial 12')
       self.option_add('*Button.Font', 'Arial 12 bold')
       self.option_add('*Button.Foreground', '#FFF')
       # create master grid
       self.master = master
       self.master.grid_columnconfigure(0, weight=1)
       self.master.grid_columnconfigure(1, weight=1)
       self.master.grid_columnconfigure(2, weight=1)
       for i in range(30):
           self.master.grid_rowconfigure(i, weight=1)
       self.grid(row=0, column=0, sticky='nsew')
       self.grid_columnconfigure(0, weight=1)
       self.grid_rowconfigure(0, weight=1)
       self.grid_columnconfigure(1, weight=1)
       self.grid_rowconfigure(1, weight=1)
       # title window
       self.master.title('ENG100')
       # init spinbox dictionary
       self.spinboxes = {}
       self.runCommand = False
        # create widgets and frames
       self.InitWidgets()
       
    def InitWidgets(self):
        """
        Sets up all the initial widgets in the GUI
        """
        self.motorArmFigure, self.motorArmCanvas = self.CreatePlot(0,0,15)
        self.motorArm = self.PlotMotorArm()
        self.scrollPlotFigure, self.scrollPlotCanvas = self.CreatePlot(15,0,15)
        # self.MLAControls()
        # self.MotorProperties()
        self.MotorArmTabs()
        self.GravityButtons()
        self.controlTabs = self.ControlOptions()
        self.runButton = self.StartStopButton()
        # self.RotateMotorArm()
    
    def CreatePlot(self, row, column, rowSpanCount):
        """
        Creates a matplotlib plot at the given row + column
        """
        frame = self.CreateFrame(row, column, 'nsew')
        frame.grid_configure(rowspan=rowSpanCount, columnspan=1)
        frame.grid_columnconfigure(column, weight=1)
        frame.grid_rowconfigure(row, weight=1)
        figure = plt.Figure()
        ax = figure.add_subplot(111)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (deg)')
        canvas = FigureCanvasTkAgg(figure, master=frame)
        canvas.draw()
        canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')
        return figure, canvas
    
    def MotorArmTabs(self):
        frame = tkinter.LabelFrame(master=self, text='Motor Arm:', relief='ridge')
        frame.grid(row=0, column=1, columnspan=2, rowspan=4, sticky='nsew')
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_rowconfigure(0, weight=1)
        motorArmTabs = ttk.Notebook(frame, padding=0)
        armTab = self.CreateTab(motorArmTabs)
        motorTab = self.CreateTab(motorArmTabs)
        motorArmTabs.add(armTab, text='Arm Properties')
        motorArmTabs.add(motorTab, text='Motor Properties')
        motorArmTabs.grid()
        self.MLAControls(armTab)
        self.MotorProperties(motorTab)
        
    def MLAControls(self, armTab):
        """
        Mass, length, and desired angle spinboxes
        """
        # frame = self.CreateFrame(0, 1, 'nsew')
        # frame.grid_configure(columnspan=1, rowspan=3)
        # frame.grid_columnconfigure(0, weight=1)
        # frame.grid_columnconfigure(1, weight=1)
        labels = ['Mass (kg):', 'Length (m):', 'Desired Angle:']
        minVals = [0.00, 0.00, -720]
        maxVals = [999.00, 999.00, 720]
        defaultVals = [0.25, 0.01, 45]
        stepVals = [0.05, 0.001, 5]
        formatVals = ['%.2f', '%.2f', '%.0f']
        for i, (label, minVal, maxVal) in enumerate(zip(labels, minVals, maxVals)):
            self.spinboxes[label] = self.CreateSpinboxGrid(i, 0, armTab, label,
                                                     minVal, maxVal)
            self.spinboxes[label].insert(0, defaultVals[i])
            self.spinboxes[label].config(increment=stepVals[i],
                                         format=formatVals[i])
            armTab.grid_rowconfigure(i, weight=1)
            
    def MotorProperties(self, motorTab):
        """
        Creates the motor properties spinboxes
        """
        # frame = self.CreateFrame(31, 0, 'nsew')
        # frame.grid_configure(columnspan=4)
        propertyLabels = ['Ki:', 'Bs:', 'Bv:', 'Jm:']
        minVals = [0, 0, 0, 0]
        maxVals = [999.00, 999.00, 999.00, 999.00,]
        defaultVals = [.136, 0.003, 0.020, 0.00051]
        stepVals = [0.001, 0.0001, 0.001, 0.00001]
        formatVals = ['%.3f', '%.4f', '%.3f', '%.5f']
        # spinColumn = 1
        motorTab.grid_columnconfigure(0, weight=1)
        motorTab.grid_columnconfigure(1, weight=1)
        for i, label in enumerate(propertyLabels):
            motorTab.grid_rowconfigure(i, weight=1)
            self.spinboxes[label] = self.CreateSpinboxGrid(i, 0, motorTab,
                                                           label, minVals[i],
                                                           maxVals[i])
            self.spinboxes[label].insert(0, defaultVals[i])
            self.spinboxes[label].config(increment=stepVals[i],
                                         format=formatVals[i],
                                         takefocus=False)
            # spinColumn += 2
            
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
        
    def ControlOptions(self):
        frame = tkinter.LabelFrame(master=self, text='Control:', relief='ridge')
        frame.grid(row=15, column=1, columnspan=2, rowspan=5, sticky='nsew')
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_rowconfigure(0, weight=1)
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
        minVals = [0.00, -50.00]
        maxVals = [120.00, 50.00]
        defaultVals = [0.30, 5.00]
        stepVals = [0.01, 0.10]
        formatVals = ['%.2f', '%.2f']
        cmdFrame.grid_columnconfigure(0, weight=1)
        cmdFrame.grid_columnconfigure(1, weight=1)
        for i, label in enumerate(labels):
            cmdFrame.grid_rowconfigure(i, weight=1)
            self.spinboxes[label] = self.CreateSpinboxGrid(i, 0, cmdFrame, label,
                                                     minVals[i], maxVals[i])
            self.spinboxes[label].insert(0, defaultVals[i])
            self.spinboxes[label].config(increment=stepVals[i],
                                         format=formatVals[i],
                                         takefocus=False)
        return cmdFrame
    
    def PIDControls(self, pidFrame):
        """
        Sets up PID spinboxes
        """
        labels = ['P:', 'I:', 'D:']
        minVals = [0.00, 0.00, 0.00]
        maxVals = [999.00, 999.00, 999.00]
        defaultVals = [0.50, 0.05, 0.01]
        stepVals = [0.01, 0.001, 0.001]
        formatVals = ['%.2f', '%.3f', '%.3f']
        for i, label in enumerate(labels):
            self.spinboxes[label] = self.CreateSpinboxGrid(i, 0, pidFrame, label,
                                                           minVals[i], maxVals[i])
            self.spinboxes[label].insert(0, defaultVals[i])
            self.spinboxes[label].config(increment=stepVals[i],
                                         format=formatVals[i],
                                         takefocus=False)
        resetButton = tkinter.Button(pidFrame, text='Reset I',
                                     bg='#0288D1', command=self.ResetI)
        resetButton.grid(row=1, column=2, sticky='nsew')
        return pidFrame

    def RestartSimulationButton(self):
        """
        Restarts the simulation using the user's inputs
        """
        pass
    
    def RestoreDefaultsButton(self):
        """
        Restores all default inputs
        """
        pass
    
    def StartStopButton(self):
        """
        Creates the Start/Stop button for running the program
        """
        frame = self.CreateFrame(25, 1, 'nse')
        frame.grid_configure(columnspan=2)
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_rowconfigure(0, weight=1)
        runButton = tkinter.Button(frame, text='Start', bg='#009688', command=self.ActivateButton)
        runButton.grid(row=0, column=0, columnspan=2, pady=10, sticky='nsew')
        return runButton
    
    def CreateSpinboxGrid(self, rowNum, colNum, frame, label, fromNum, toNum):
        """
        Creates a label and spinbox grid
        """
        spinboxLabel = tkinter.Label(frame, text=label, justify='left', anchor='nw')
        spinbox = tkinter.Spinbox(frame, from_=fromNum, to=toNum)
        spinboxLabel.grid(row=rowNum, column=colNum, padx=5, pady=5, sticky='nsw')
        spinbox.grid(row=rowNum, column=colNum+1, padx=5, pady=5, sticky='nsew')
        spinbox.grid_rowconfigure(rowNum, weight=1)
        spinbox.grid_columnconfigure(colNum, weight=1)
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
        massInput = float(self.spinboxes.get('Mass (kg):').get())
        lengthInput = float(self.spinboxes.get('Length (m):').get())
        angleInput = float(self.spinboxes.get('Desired Angle:').get())
        return massInput, lengthInput, angleInput
            
    def ReturnCommandControls(self):
        """
        Get values from command control spinboxes
        """
        cmdA = float(self.spinboxes.get('Current CMD (A):').get())
        timeInput = float(self.spinboxes.get('Time (s):').get())    
        return cmdA, timeInput
    
    def ReturnPID(self):
        """
        Get values from PID spinboxes
        """
        spinboxes = self.spinboxes
        pInput = float(spinboxes.get('P:').get())
        iInput = float(spinboxes.get('I:').get())
        dInput = float(spinboxes.get('D:').get())
        return pInput, iInput, dInput
    
    def ResetI(self):
        defaultI = 0.050
        ClosedLoopControl.reset_int(self)
        iSpinbox = self.spinboxes.get('I:')
        iSpinbox.delete(0, 'end')
        iSpinbox.insert(0, defaultI)
        # for label in labels:
        #     spinbox = self.spinboxes.get(label)
        #     spinbox.delete(0, 'end')
        #     spinbox.insert(0, 0)
            
    def ReturnMotorProperties(self):
        spinboxes = self.spinboxes
        kiInput = float(spinboxes.get('Ki:').get())
        bsInput = float(spinboxes.get('Bs:').get())
        bvInput = float(spinboxes.get('Bv:').get())
        jmInput = float(spinboxes.get('Jm:').get())
        return kiInput, bsInput, bvInput, jmInput
    
    def ActivateButton(self):
        # global runCommand
        if self.runButton['text'] == 'Start':
            print(self.runButton['text'])
            self.runButton.configure(text='Stop', bg='#d32f2f', command=self.ActivateButton)
            global runCommand
            runCommand = True
            print(runCommand)
            self.plotAnimation = self.StartModel()
        elif self.runButton['text'] == 'Stop':
            print(self.runButton['text'])
            self.runButton.configure(text='Start', bg='#009688', command=self.ActivateButton)
            runCommand = False
            print(runCommand)
            # self.plotAnimation.event_source.stop()
        return runCommand
            
    def StartModel(self):
        motor = SystemModel()
        open_loop = OpenLoopControl(motor)
        closed_loop = ClosedLoopControl(motor)
        # motor properties can be changed by modifying the value or can be read for a display.
        kiInput, bsInput, bvInput, jmInput = self.ReturnMotorProperties()
        motor.K_t = kiInput # Nm Torque constant
        motor.B_v = bsInput #0.05 #0.0003 # Nm/(deg/s) Coefficient of viscous friction
        motor.B_s = bvInput #0.1 #0.002 # Nm Coefficient of static friction
        motor.J_m = jmInput # kg*m^2 Rotor Moment of Inertia
        
        massInput, lengthInput, angleInput = self.ReturnMLAControls()
        M_a = massInput	# kg
        L_a = lengthInput # m
        # updates the arms moment of inertia based on the length and mass provided.  The value itself can be read using motor.M_a and motor.L_a
        motor.update_arm_mass(M_a) # updates the arm moment of inertia based on M_a and L_a
        motor.update_arm_length(L_a) # updates the arm moment of inertia based on M_a and L_a
        
        gravitySetting = self.gravityVar.get()
        motor.use_grav = gravitySetting  # 0 for no gravity 1 for with gravity
        
        loopType = self.ReturnControlType()
        motor.reset()
        
        if loopType == 0:
            self.PlotOpenLoop(motor, open_loop)
        elif loopType == 1:
            self.plotAnimation = self.PlotClosedLoop(motor, closed_loop, angleInput)
        return self.plotAnimation
            
    def PlotGravitySymbol(self):
        """
        Add/remove gravity symbol to plot when gravity is switched on/off
        """
        gravitySetting = self.gravityVar.get()
        figure = self.motorArmFigure
        # figure.clear()
        ax = figure.add_subplot(111)
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.axis('off')
        if gravitySetting == 1:
            self.gravitySymbol = ax.annotate('g', xy=(10,7), xytext=(10, 10),
                                             size=22, ha='center', va='top',
                                        arrowprops=dict(facecolor='black', shrink=0.1))
        elif gravitySetting == 0:
            self.gravitySymbol.remove()
        figure.canvas.draw()
        return self.gravitySymbol
    
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
        ax.axis('scaled')
        figure.canvas.draw()
        return self.motorArm
        
    def PlotOpenLoop(self, motor, open_loop):
        cmdAInput, timeInput = self.ReturnCommandControls()
        open_loop.time_to_run = timeInput # seconds
        open_loop.current_cmd = cmdAInput # Amps
        open_loop.start_run = True # flag used to set timer will be set to false on first call to open_loop.step_control.  
        
        figure = self.scrollPlotFigure
        figure.clear()
        ax = figure.add_subplot(111)
        ax.set_title('Open Loop')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (deg)')
        figure.canvas.draw()
        
        angle = [] 
        model_time = []
        start_time = time.monotonic()
        line, = ax.plot([],[])
        
        # this is just an example you can leave it running until it restarts plotting the most recent returned value.
        while time.monotonic() - start_time < 10 :
            model_time.append(time.monotonic() - start_time)
            angle.append(open_loop.step_control())
            time.sleep(.04) # this simulates waiting to update the graphs so we aren't drawing too fast which can slow the system.
            angle_deg = [x*180/np.pi for x in angle]
            line.remove()
            line, = ax.plot(model_time, angle_deg, color='blue')
            figure.canvas.draw()
            self.RotateMotorArm(angle_deg[-1])

        # angle_deg = [x*180/np.pi for x in angle]
        # figure = self.scrollPlotFigure
        # figure.clear()
        # ax = figure.add_subplot(111)
        # ax.plot(model_time,angle_deg)
        # ax.set_title('Open Loop')
        # ax.set_xlabel('Time (s)')
        # ax.set_ylabel('Angle (deg)')
        # self.scrollPlotCanvas.draw()
        print('Finished open loop example')
        
    def PlotClosedLoop(self, motor, closed_loop, angleInput):
        motor.reset()
        pInput, iInput, dInput = self.ReturnPID()
        # closed_loop.kp = .5
        # closed_loop.ki = .05
        # closed_loop.kd = 0.01#.5
        closed_loop.kp = pInput
        closed_loop.ki = iInput
        closed_loop.kd = dInput
        
        # closed_loop.angle_ref = np.deg2rad(45)
        closed_loop.angle_ref = np.deg2rad(angleInput)
        
        figure = self.scrollPlotFigure
        figure.clear()
        ax = figure.add_subplot(111)
        # ax.plot(model_time,angle_deg)
        ax.set_title('Closed Loop')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (deg)')
        ax.set_xlim(-10, 0)
        figure.canvas.draw()
        
        start_time = time.monotonic()
        angle = [] 
        model_time = []
        line, = ax.plot([],[])
        # this is just an example you can leave it running until it restarts plotting the most recent returned value.
        # while time.monotonic() - start_time < 3 :
        # time.sleep(.02)
        self.plotAnimation = matplotlib.animation.FuncAnimation(figure, self.ClosedLoopUpdate, fargs=(figure, ax, angle, line, model_time, start_time, closed_loop), interval=20)
        # while self.runCommand == True:
        #     # print(self.runCommand)
        #     # if self.runCommand == False:
        #     #     break
        #     # else:
        #     model_time.append(time.monotonic() - start_time)
        #     angle.append(closed_loop.step_control())
        #     time.sleep(.02)
        #     angle_deg = np.rad2deg(angle)
        #     line.remove()
        #     line, = ax.plot(model_time, angle_deg, color='blue')
        #     ax.set_xlim((model_time[-1]-10), model_time[-1])
        #     # plotAnimation = matplotlib.animation.FuncAnimation(figure, self.ClosedLoopUpdate, fargs=(figure, ax, angle, model_time), interval=200)
        #     figure.canvas.draw()
        #     self.RotateMotorArm(angle_deg[-1])
        #     # time.sleep(.02) # this simulates waiting to update the graphs so we aren't drawing too fast which can slow the system. Looks like the other overhead is about .02 s as well so this should update around 25 Hz.
        
        # angle_deg = np.rad2deg(angle) # [x*180/np.pi for x in angle]
        # figure = self.scrollPlotFigure
        # figure.clear()
        # ax = figure.add_subplot(111)
        # ax.plot(model_time,angle_deg)
        # ax.set_title('Closed Loop')
        # ax.set_xlabel('Time (s)')
        # ax.set_ylabel('Angle (deg)')
        # figure.canvas.draw()
        # print('Finished closed loop example')
        return self.plotAnimation
        
    def RotateMotorArm(self, angleDeg):
        if runCommand:
            # angleDeg = np.rad2deg(angle)
            figure = self.motorArmFigure
            ax = figure.get_axes()[0]
            angleRotate = matplotlib.transforms.Affine2D().rotate_deg_around(5,5, angleDeg) + ax.transData
            self.motorArm.set_transform(angleRotate)
            figure.canvas.draw()
            return self.motorArm      
        
    def ClosedLoopUpdate(self, i, figure, ax, angle, line, model_time, start_time, closed_loop):
        if runCommand:
            model_time.append(time.monotonic() - start_time)
            angle.append(closed_loop.step_control())
            angle_deg = np.rad2deg(angle)
            try:
                line.remove()
            except:
                pass
            line, = ax.plot(model_time, angle_deg, color='blue')
            angle_deg = np.rad2deg(angle)
            ax.set_xlim((model_time[-1]-10), model_time[-1])
            # ax.plot(model_time, angle_deg)
            # line.set_xdata(model_time)
            # line.set_ydata(angle_deg)
            # plt.show()
            figure.canvas.draw()
            time.sleep(.02)
            self.RotateMotorArm(angle_deg[-1])
            return line,
    
class Window:
    def CenterWindow(self):
        w = self.winfo_screenwidth()
        h = self.winfo_screenheight()
        self.geometry("%dx%d" % (w/2, h/2))
        size = tuple(int(pos) for pos in self.geometry().split('+')[0].split('x'))
        x = w/2 - size[0]/2
        y = h/2 - size[1]/2
        self.geometry("%dx%d+%d+%d" % (size + (x, y)))
        
            
if __name__ == '__main__':
    root = tkinter.Tk()
    # root.minsize(600,400)
    # root.geometry('600x400')
    app = App(master=root)
    Window.CenterWindow(root)
    app.mainloop()
    
