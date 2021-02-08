from PyQt5 import QtWidgets, QtCore, uic
import os

# Load UI file (Created with QT Designer)
help_window = uic.loadUiType('HelpWindow.ui')[0]

# Help Window class
class HelpWindowObject(QtWidgets.QMainWindow, help_window):
    def __init__(self, parent=None):
        
        # Setup main window
        QtWidgets.QMainWindow.__init__(self, parent)
        
        # Load UI from QT Developer
        self.setupUi(self)
        self.setWindowTitle('Help')
        stylesheetFile = 'Styling.qss'
        with open(stylesheetFile, 'r') as styleSheet:
            self.setStyleSheet(styleSheet.read())

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
        file_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "help_files/" + filename))
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