from PyQt5 import QtWidgets, QtCore, uic, QtGui
# from PyQt5.QtWebEngineWidgets import QWebEngineView
#from PyQt5.QtWebEngineWidgets import QEngineWebView as QWebView
import os
import xlrd
# from functools import partial
import re, webbrowser

# Load UI file (Created with QT Designer)
help_window = uic.loadUiType('HelpWindow.ui')[0]

# #Get excel file with help bar structure
# scriptpath = os.path.dirname(__file__)
# subtabsFilename = os.path.join(scriptpath, '..\help_files\HelpWindowSubtabs.xlsx')
# wbk = xlrd.open_workbook(subtabsFilename)
# sheet = wbk.sheet_by_index(0)
# links = wbk.sheet_by_index(1)

# Help Window class
class HelpWindowObject(QtWidgets.QMainWindow, help_window):
    def __init__(self, parent=None):
        
        # Setup main window
        QtWidgets.QMainWindow.__init__(self, parent)
        
        # Load UI from QT Developer
        self.setupUi(self)
        self.setWindowTitle('Help')
        
        # Menu Bar actions
        # self.actionClose.triggered.connect(self.close)
        # self.actionMaximize.triggered.connect(self.showMaximized)
        # self.actionSmall_Window.triggered.connect(self.SmallWindow)
        
        # # Set up QTreeView for help subjects
        # self.model = QtGui.QStandardItemModel()
        # self.helpDict = {}
        # self.updateDict(self.helpDict, self.model)
        # self.HelpSubjects.setModel(self.model)
        # self.HelpSubjects.header().hide()
        # self.HelpSubjects.expandAll()

        # self.HelpSubjects.clicked.connect(self.treeView_clicked)
        self.HelpSubjects.itemClicked.connect(self.subject_clicked)
        self.HelpText.urlChanged.connect(self.getItem)
        self.BackButton.clicked.connect(self.HelpText.back)
        self.ForwardButton.clicked.connect(self.HelpText.forward)
   
    def LoadHTML(self, filename):
        """
            Load specific .html file using filename. If filename does not exists,
            search through the HelpWindowSubtabs.xlsx to find the new_filename
            that should be used.
        """
        # remove . from filename before attaching to filepath
        file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../help_files/" + filename))
        file_loc = QtCore.QUrl.fromLocalFile(file_path)
        self.HelpText.load(file_loc)
        self.listOfUrls = [filename]
        
    def subject_clicked(self, item):
        clickedString = item.text()
        filename = clickedString.replace(' ','') + '.html'
        file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "help_files/" + filename))
        file_loc = QtCore.QUrl.fromLocalFile(file_path)
        self.HelpText.load(file_loc)

    def getItem(self, index):
        self.HelpSubjects.selectionModel().clearSelection()
        htmlName = self.HelpText.url().toString(QtCore.QUrl.RemoveScheme)
        file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "help_files/")).replace("\\", "/")
        htmlName = htmlName.replace(file_path, '').replace('/', '').replace('.html', '')
        self.findItem(index, htmlName)
                        
    def findItem(self, index, htmlName):
        """
        Find the htmlName in the HelpSubjects list and set its state to selected
        """
        pass