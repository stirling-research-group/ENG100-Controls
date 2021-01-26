from ControlsGuiModel import SystemModel, OpenLoopControl, ClosedLoopControl
from PyQt5 import QtCore, QtWidgets, uic
import matplotlib
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)
from matplotlib.collections import PatchCollection
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import HelpWindow
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import time

# helpDf = pd.read_csv('Documentation/inputDescriptions.txt', sep=" : ",
#                      header=None, names=['Label', 'Text'], engine='python')

# Load UI file (Created with QT Designer)
windowUI = uic.loadUiType('MainWindow.ui')[0]
# Window class
class EngWindow(QtWidgets.QMainWindow, windowUI):
    def __init__(self, parent=None):

        # Setup main window
        QtWidgets.QMainWindow.__init__(self, parent)
        # Load UI from QT Developer
        self.setupUi(self)
        stylesheetFile = 'Styling.qss'
        with open(stylesheetFile, 'r') as styleSheet:
            self.setStyleSheet(styleSheet.read())
        self.gravitySetting = MotorArm.ToggleGravity(self)
        self.ResetDefaults.clicked.connect(self.ResetDefaultValues)
        self.ResetPlots.clicked.connect(self.ResetPlotsClicked)
        self.InitWindow()
        self.InitSpinboxes()
        self.defaultVals = {'massInput': 0.25, 'lengthInput': 0.010,
                            'angleInput': 45, 'kiInput': 0.136,
                            'bsInput': 0.003, 'bvInput': 0.020,
                            'jmInput': 0.00051, 'cmdInput': 0.30,
                            'timeInput': 5.00, 'pInput': 0.50,
                            'iInput': 0.05, 'dInput': 0.01,
                            'timestepInput': 0.001, 'startAngleInput': 0}
        
    def InitWindow(self):
        """
        Initializes the main window
        """
        # init UI components
        self.setWindowTitle('Open and Closed Loop Control Demo')
        self.actionView_Help_Topics.triggered.connect(lambda: MenuWindows.OpenWindow(self))
        self.actionView_Details.triggered.connect(lambda: MenuWindows.OpenAboutDialog(self))
        self.InitPlots()
        self.GravityOn.toggled.connect(lambda: MotorArm.UpdateGravity(self))
        # init model
        self.openRun = False
        self.closedRun = False
        self.motor, self.open_loop, self.closed_loop = self.InitModel()
        self.SetMotorValues()
        self.openLoopAnimation, self.closedLoopAnimation = Animations.InitAnimations(self)
        self.openLoopAnimation.event_source.stop()
        self.closedLoopAnimation.event_source.stop()
        self.StartStopSimulation.clicked.connect(self.ActivateStartStopButton)
        
    def InitPlots(self):
        """
        Create plots and add to window
        """
        self.motorArmPlot = MatplotlibWidget(self)
        self.ArmPlotHolder.addWidget(self.motorArmPlot)
        self.motorArm = MotorArm.PlotMotorArm(self, self.motorArmPlot)
        self.gravitySymbol = MotorArm.PlotGravitySymbol(self)
        self.scrollPlot = MatplotlibWidget(self)
        self.LoopPlotHolder.addWidget(self.scrollPlot)
        self.toolbar = LoopPlot.CreatePlotToolbar(self, self.scrollPlot.canvas,
                                                  self.LoopPlotHolder)
        
    def InitModel(self):
        """
        Initializes the model and returns the motor, open_loop, and closed_loop
        for use later
        """
        motor = SystemModel()
        open_loop = OpenLoopControl(motor)
        closed_loop = ClosedLoopControl(motor)
        return motor, open_loop, closed_loop
    
    def InitSpinboxes(self):
        # arm properties
        self.massInput.valueChanged.connect(self.motor.update_arm_mass)
        self.lengthInput.valueChanged.connect(self.motor.update_arm_length)
        self.angleInput.valueChanged.connect(lambda angle: ClosedLoopControl.update_angle_ref(self.closed_loop, angle))
        # open loop properties
        self.cmdInput.valueChanged.connect(lambda cmd: OpenLoopControl.update_current_cmd(self.open_loop, cmd))
        self.timeInput.valueChanged.connect(lambda time_to_run: OpenLoopControl.update_time_to_run(self.open_loop, time_to_run))
        # closed loop properties
        self.pInput.valueChanged.connect(lambda kp: ClosedLoopControl.update_kp(self.closed_loop, kp))
        self.iInput.valueChanged.connect(lambda ki: ClosedLoopControl.update_ki(self.closed_loop, ki))
        self.dInput.valueChanged.connect(lambda kd: ClosedLoopControl.update_kd(self.closed_loop, kd))
        # motor properties
        self.kiInput.valueChanged.connect(lambda ki: SystemModel.update_K_t(self.motor, ki))
        self.bsInput.valueChanged.connect(lambda bs: SystemModel.update_B_s(self.motor, bs))
        self.bvInput.valueChanged.connect(lambda bv: SystemModel.update_B_v(self.motor, bv))
        self.jmInput.valueChanged.connect(lambda jm: SystemModel.update_J_m(self.motor, jm))
        # simulation properties
        self.timestepInput.valueChanged.connect(lambda time: SystemModel.update_timestep(self.motor, time))
        self.startAngleInput.valueChanged.connect(lambda: Animations.RotateMotorArm(self, 0))
        
    
    def SetArmProperties(self, event=None):
        """
        Sets the arm properties based on the user inputs
        """
        M_a = self.massInput.value()	# kg
        L_a = self.lengthInput.value() # m
        angleInput = self.angleInput.value()
        # updates the arms moment of inertia based on the length and mass provided.  The value itself can be read using motor.M_a and motor.L_a
        self.motor.update_arm_mass(M_a) # updates the arm moment of inertia based on M_a and L_a
        self.motor.update_arm_length(L_a) # updates the arm moment of inertia based on M_a and L_a
        self.closed_loop.angle_ref = np.deg2rad(angleInput)
    
    def SetGravityProperties(self):
        self.motor.use_grav = self.gravitySetting  # 0 for no gravity 1 for with gravity
        return self.motor.use_grav
        
    def SetMotorProperties(self):
        """
        Sets the motor properties based on the user inputs
        """
        # motor properties can be changed by modifying the value or can be read for a display.
        self.motor.K_t = self.kiInput.value() # Nm Torque constant
        self.motor.B_v = self.bsInput.value() #0.05 #0.0003 # Nm/(deg/s) Coefficient of viscous friction
        self.motor.B_s = self.bvInput.value() #0.1 #0.002 # Nm Coefficient of static friction
        self.motor.J_m = self.jmInput.value() # kg*m^2 Rotor Moment of Inertia
        
    def SetPIDProperties(self):
        """
        Sets the PID properties based on the user inputs
        """
        self.closed_loop.kp = self.pInput.value()
        self.closed_loop.ki = self.iInput.value()
        self.closed_loop.kd = self.dInput.value()
        
    def SetOpenLoopProperties(self):
        """
        Sets the current cmd and time to run for the open loop plotting
        """
        self.open_loop.time_to_run = self.timeInput.value() # seconds
        self.open_loop.current_cmd = self.cmdInput.value() # Amps
        
    def SetSimulationProperties(self):
        self.motor.timestep = self.timestepInput.value()
    
    def SetMotorValues(self):
        """
        Resets motor and retrieves user's inputs for motor, arm, simulation, open, and
        closed loop properties
        """
        self.motor.reset()
        self.SetMotorProperties()
        self.SetArmProperties()
        self.SetGravityProperties()
        self.SetOpenLoopProperties()
        self.SetPIDProperties()
        self.SetSimulationProperties()
        
    def ActivateStartStopButton(self):
        button = self.StartStopSimulation
        if button.isChecked():
            button.setText('Stop')
            self.SetMotorValues()
            # button.setStyleSheet('background-color: #d32f2f')
            Animations.StartAnimation(self)
        else:
            button.setText('Start')
            # button.setStyleSheet('background-color: #009688')
            Animations.StopAnimation(self)
            
    def ResetDefaultValues(self):
        for key, value in self.defaultVals.items():
            spinbox = self.RightPanel.findChild(QtWidgets.QDoubleSpinBox, key)
            spinbox.setValue(value)
            
    def ResetPlotsClicked(self):
        """
        Resets the arm angle to zero, resets the motor and motor values, and
        clears the loop plot
        """
        self.motor, self.open_loop, self.closed_loop = self.InitModel()
        self.SetMotorValues()
        Animations.RotateMotorArm(self, self.startAngleInput.value())
        try:
            self.line.remove()
            self.scrollPlot.canvas.draw()
        except:
            pass
        self.ResetPlots.setEnabled(False)
        return (self.motor, self.open_loop, self.closed_loop,
                self.openLoopAnimation, self.closedLoopAnimation)
            
    def OpenLoopAnimate(self, i):
        """
        Animates the open loop plot as long as the 'Stop' button has not been
        pushed.
        """
        while self.openRun == True:
            figure = self.scrollPlot.figure
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
            Animations.RotateMotorArm(self, angle_deg[-1])
            return self.line,
        self.openLoopAnimation.event_source.stop()
        
    def ClosedLoopAnimate(self, i):
        """
        Animates the closed loop plot as long as the 'Stop' button has not been
        pushed.
        """
        while self.closedRun == True:
            step_control = self.closed_loop.step_control()
            figure = self.scrollPlot.figure
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
            ax.set_ylim(0, angle_deg.max()+10)
            figure.canvas.draw()
            Animations.RotateMotorArm(self, angle_deg[-1])
            return self.line,
        self.closedLoopAnimation.event_source.stop()
    
class MotorArm:
    def ToggleGravity(self):
        if self.GravityOn.isChecked():
            self.gravitySetting = 1
        elif self.GravityOff.isChecked():
            self.gravitySetting = 0
        return self.gravitySetting
    
    def UpdateGravity(self):
        self.gravitySetting = MotorArm.ToggleGravity(self)
        MotorArm.PlotGravitySymbol(self)
    
    def PlotMotorArm(self, plot):
        """
        Creates the initial motor arm image
        """
        figure = plot.figure
        ax = plot.axis
        self.motorArmCenter = plt.Circle((5, 5), radius=1)
        self.motorArmRect = plt.Rectangle((4.5, 2), 1, 3)
        patches = [self.motorArmCenter, self.motorArmRect]
        self.motorArm = PatchCollection(patches, fc='gray')
        ax.add_collection(self.motorArm)
        # RotateMotorArm(self, self.startAngleInput.value())
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
        figure.tight_layout()
        self.motorArmPlot.canvas.draw()
        return self.motorArm
        
    def PlotGravitySymbol(self):
        """
        Add/remove gravity symbol to plot when gravity is switched on/off
        """
        ax = self.motorArmPlot.axis
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        if self.gravitySetting == 1:
            self.gravitySymbol = ax.annotate('g', xy=(10,8.5), xytext=(10, 10),
                                             size=18, ha='center', va='top',
                                             arrowprops=dict(facecolor='black',
                                                             shrink=0.1))
        elif self.gravitySetting == 0:
            try:
                self.gravitySymbol.remove()
            except:
                pass
        self.motorArmPlot.canvas.draw()
        return self.gravitySymbol
    
    def PlotAngleText(self, angle, figure, ax):
        """
        Adds the current angle to the motor arm plot. Converts angle to
        scientific notation if the output is 5 digits or longer
        """
        angleStr = str(int(angle))
        if len(angleStr) >= 5:
            angleStr = '{:.2e}'.format(int(angle))
        text = 'Angle: ' + angleStr
        try:
            self.angleText.remove()
        except:
            pass
        self.angleText = ax.annotate(text, xy=(0,10), xytext=(0, 10), size=10,
                                     ha='center', va='top')
        return self.angleText

class LoopPlot:
    def CreatePlotToolbar(self, canvas, frame):
        """
        Creates a toolbar for the plot and adds to a frame below plot
        """
        toolbar = NavigationToolbar(canvas, self)
        frame.addWidget(toolbar)
        toolbar.setIconSize(QtCore.QSize(24,24))
        return toolbar
    
class MatplotlibWidget(QtWidgets.QWidget):
    """
    Create plot widget
    """
    def __init__(self, parent=None):
        super(MatplotlibWidget, self).__init__(parent)
          
        # matplotlib.style.use('classic')
        font = {'size': 12}
        matplotlib.rc('font', **font)
        self.figure = Figure(facecolor='white')
        self.canvas = FigureCanvas(self.figure)
        self.axis = self.figure.add_subplot(111)
        self.axis.set_xlabel('Time (s)')
        self.axis.set_ylabel('Angle (deg)')
        self.layoutVertical = QtWidgets.QVBoxLayout(self)
        self.layoutVertical.addWidget(self.canvas)
        self.setSizePolicy(QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding))
        self.canvas.draw()
        
class Animations:
    def InitAnimations(self):
        """
        Calls functions to initialize the open/closed loop animations
        """
        Animations.OpenLoopInit(self)
        self.openLoopAnimation = matplotlib.animation.FuncAnimation(self.scrollPlot, self.OpenLoopAnimate, interval=20)
        Animations.ClosedLoopInit(self)
        self.closedLoopAnimation = matplotlib.animation.FuncAnimation(self.scrollPlot, self.ClosedLoopAnimate, interval=20)
        return self.openLoopAnimation, self.closedLoopAnimation
    
    def StartAnimation(self):
        self.ResetPlots.setEnabled(False)
        self.ResetPlotsClicked()
        loopType = self.LoopControlTab.currentIndex()
        if loopType == 0:
            self.openRun = True
            self.closedRun = False
            Animations.OpenLoopInit(self)
            self.openLoopAnimation.event_source.start()
            self.closedLoopAnimation.event_source.stop()
        elif loopType == 1:
            self.openRun = False
            self.closedRun = True
            Animations.ClosedLoopInit(self)
            self.closedLoopAnimation.event_source.start()
            self.openLoopAnimation.event_source.stop()
        return self.openRun, self.closedRun
        
    def StopAnimation(self):
        self.openRun = False
        self.closedRun = False
        self.openLoopAnimation.event_source.stop()
        self.closedLoopAnimation.event_source.stop()
        self.ResetPlots.setEnabled(True)
        return self.openRun, self.closedRun
    
    def RotateMotorArm(self, angleDeg):
        """
        Rotates the motor arm image based on angleDeg input
        """
        figure = self.motorArmPlot.figure
        ax = figure.get_axes()[0]
        startAngle = self.startAngleInput.value()
        updatedAngle = angleDeg + startAngle
        # angleRotate = matplotlib.transforms.Affine2D().rotate_deg_around(5,5, angleDeg) + ax.transData
        angleRotate = matplotlib.transforms.Affine2D().rotate_deg_around(5,5, updatedAngle) + ax.transData
        self.motorArm.set_transform(angleRotate)
        self.angleText = MotorArm.PlotAngleText(self, angleDeg, figure, ax)
        self.motorArmPlot.canvas.draw()
        return self.motorArm, self.angleText 

    def OpenLoopInit(self):
        """
        Initializes the open loop animation
        """
        self.SetOpenLoopProperties()
        self.open_loop.start_run = True # flag used to set timer will be set to false on first call to open_loop.step_control.  
        
        figure = self.scrollPlot.figure
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

    def ClosedLoopInit(self):
        """
        Initializes the closed loop animation
        """
        self.SetArmProperties()
        self.SetPIDProperties()
        
        self.motor.use_grav = self.gravitySetting  # 0 for no gravity 1 for with gravity
        figure = self.scrollPlot.figure
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

class MenuWindows:
    def OpenHelpWindow(self):
        """
        Opens Help Window
        """
        self._subWindow = HelpWindow.HelpWindowObject()
        HelpWindow.HelpWindowObject.LoadHTML(self._subWindow, 'help_files/Overview.html')
        self._subWindow.show()
        self._subWindow.setWindowState(self._subWindow.windowState() & ~QtCore.Qt.WindowMinimized | QtCore.Qt.WindowActive)
        self._subWindow.activateWindow()
        
    def OpenAboutDialog(self):
        aboutDialog = AboutDialog()
        aboutDialog.exec_()
        
class AboutDialog(QtWidgets.QDialog):

    def __init__(self):
        """
        Creates About Window
        """
        super(AboutDialog, self).__init__()
        # Load UI file (Created with QT Designer)
        uic.loadUiType('AboutWindow.ui', self)
        self.setWindowTitle('About')

    
        
app = QtWidgets.QApplication(sys.argv)
mainWindow = EngWindow()
mainWindow.show()
app.exec_()
