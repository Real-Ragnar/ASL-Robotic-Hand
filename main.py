#!/usr/bin/env python
import rospy 
from std_msgs.msg import String
import sys									# System calls
import os
sys.path.append("./ui/")
sys.path.append("./src/")
sys.path.append("./lib/")							# Append the ui folder to path
from PyQt4 import QtGui								# Import PyQt module
import MainWindow								# MainWindow Widget from ui folder
import model									# Model of commander
import numpy as np

pub = rospy.Publisher('static_gests', String, queue_size=10)

rospy.init_node('main', anonymous=True)


class Application(QtGui.QMainWindow, MainWindow.Ui_MainWindow):
    def __init__(self, parent=None):
        super(Application, self).__init__(parent)
        self.setupUi(self)

	self.ConsoleOutput.setReadOnly(True)
	self.NewCommand.clicked.connect(self.AddCommand)			# Button Actions
 	self.DeleteCommand.clicked.connect(self.RemoveCommand)
  	self.TeachCommand.clicked.connect(self.Teach_Database)
   	self.UpdateModel.clicked.connect(self.TeachModel)
	self.Execute.clicked.connect(self.ExecuteCommand)

        self.actionExit.triggered.connect(self.close)				# Menu Actions
        self.actionOpen.triggered.connect(self.Open_Database)
	self.actionNew_Database.triggered.connect(self.New_Database)

        self.control = model.CommandModel()					# Model, database and Leap controller
	return

    def toConsole(self, string):
	self.ConsoleOutput.appendPlainText(string)
 	return

    def AddCommand(self):
	command_item = QtGui.QTableWidgetItem()
 	number_item = QtGui.QTableWidgetItem()
 	command_name, ok = QtGui.QInputDialog.getText(self, 'New Command',
                                                 'Enter your command')
	if ok:
     		self.tableWidget.insertRow(0)
     		command_item.setText(command_name)
       		number_item.setText("0")
         	self.tableWidget.setItem(0, 0, command_item)
          	self.tableWidget.setItem(0, 1, number_item)
           	newgroup = self.control.file.create_group(str(command_name))
	return

    def LoadCommand(self, command_name, number_of_datasets):
        command_item = QtGui.QTableWidgetItem()
        number_item = QtGui.QTableWidgetItem()
        self.tableWidget.insertRow(0)
        command_item.setText(command_name)
        number_item.setText(str(number_of_datasets))
        self.tableWidget.setItem(0, 0, command_item)
        self.tableWidget.setItem(0, 1, number_item)
	return

    def RemoveCommand(self):
        selected_row = self.tableWidget.currentRow()
        selected_item = self.tableWidget.currentItem()
        selected_command = selected_item.text()
	reply = QtGui.QMessageBox.question(self, 'Message',
            "Are you sure you want to delete"+selected_item.text()+"?", QtGui.QMessageBox.Yes |
            QtGui.QMessageBox.No, QtGui.QMessageBox.No)
        if reply == QtGui.QMessageBox.Yes:
            del self.control.file[str(selected_command)]
            self.tableWidget.removeRow(selected_row)
            print "Event accepted"
        else:
            print "Event non accepted"
	return

    def New_Database(self):
	rows = self.tableWidget.rowCount()
	for i in range(rows):
     		self.tableWidget.removeRow(0)
        filename = QtGui.QFileDialog.getSaveFileName(self, 'New Database', './', 'HDF5 Databases (*.h5)')
    	self.control.load_file(str(filename))
	self.toConsole("Message: New Database Constructed")
	self.DatabaseLoaded.setChecked(True)
        return

    def Open_Database(self):
        rows = self.tableWidget.rowCount()
        for i in range(rows):
        	self.tableWidget.removeRow(0)
    	filename = QtGui.QFileDialog.getOpenFileName(self, 'Open file', './', 'HDF5 Databases (*.h5)')
    	self.control.load_file(str(filename))
	for group in self.control.file:
    		number_of_datapoints = len(self.control.file[group].items())
    		self.LoadCommand(group, number_of_datapoints)
	self.toConsole("Message: Database opened")
	self.DatabaseLoaded.setChecked(True)
        return

    def Teach_Database(self):
        selected_row = self.tableWidget.currentRow()
	selected_command = self.tableWidget.item(selected_row, 0)
 	selected_number  = self.tableWidget.item(selected_row, 1)
  	try:
       		command = str(selected_command.text())
        except AttributeError:
            	self.toConsole("Error: No command selected. Please select a command from the table")
             	return
   	number  = int(str(selected_number.text()))
  	selected_number.setText(str(number + 1))
 	command = str(selected_command.text())
  	while True:
       		new_data = self.control.take_a_snapshot()
         	if len(new_data) != 0:
			self.control.teach_command(command, str(number+1), new_data)
   			break
   		else:
         		continue
 	return

    def TeachModel(self):
        data = np.array([np.zeros(132)])
        classes = np.array([['null']])
        for group in self.control.file:
        	for dataset in self.control.file[group]:
             		data = np.append(data, np.array([self.control.file[group][dataset][:]]), axis=0)
               		classes = np.append(classes, np.array([[group]]), axis=0)
        data = data[1:, :]
        classes = classes[1:, :]
        self.control.classifier.fit(data, classes.T[0,:])
        self.toConsole("Model Updated")
        self.ActiveModel.setChecked(True)
        return

    def ExecuteCommand(self):
        if self.control.is_database_open == True and self.ActiveModel.isChecked() == True:
        	while True:
        		try:
        			new_data = self.control.take_a_snapshot()
        			new_data = np.array([new_data])
        			command = str(self.control.classifier.predict(new_data)[0])
        			os.system(command)
          			break
    			except ValueError:
          			continue
        	self.toConsole("Executed: " + command)
		pub.publish(command)
         	return
        elif self.control.is_database_open == False:
        	self.toConsole("Error: No database opened")
        elif self.ActiveModel.isChecked() == False:
        	self.toConsole("Error: No active model for executions")
        else:
            	x = None
        return

def main():
    app = QtGui.QApplication(sys.argv)
    form = Application()
    form.show()
    sys.exit(app.exec_())

if __name__ == '__main__':              # if we're running file directly and not importing it
    main()         
