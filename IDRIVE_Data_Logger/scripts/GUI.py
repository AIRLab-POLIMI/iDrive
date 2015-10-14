#!/usr/bin/env python
import rospy
import std_msgs.msg
import sys
from PyQt4 import QtGui, QtCore # importiamo i moduli necessari

STARTING = "starting"
STARTED = "started"
SHUTTING = "shutting down"
STOPPED = "stopped"

class MainWindow(QtGui.QMainWindow):
      def __init__(self):
              QtGui.QMainWindow.__init__(self)
              self.setWindowTitle('I.DRIVE Data Logger')
  	      cWidget = QtGui.QWidget(self)

              grid = QtGui.QGridLayout(cWidget) 
	      grid.setHorizontalSpacing(10)
	      grid.setVerticalSpacing(0)

              velodyne_label = QtGui.QLabel("Velodyne", cWidget)
	      gps_imu_velodyne_label = QtGui.QLabel("GPS/IMU Velodyne", cWidget)
	      prosilica_label = QtGui.QLabel("Prosilica", cWidget) 
	      IMU_label = QtGui.QLabel("IMU Xsense", cWidget)
	      axis_label = QtGui.QLabel("Axis", cWidget)
	      empatica_label = QtGui.QLabel("Empatica", cWidget)
	      procomp_label = QtGui.QLabel("Procomp", cWidget)
	      
 	      velodyne_state = QtGui.QLabel(STARTING, cWidget)
	      gps_imu_velodyne_state = QtGui.QLabel(STARTING, cWidget)
	      prosilica_state = QtGui.QLabel(STARTING, cWidget) 
	      IMU_state = QtGui.QLabel(STARTING, cWidget)
	      axis_state = QtGui.QLabel(STARTING, cWidget)
	      empatica_state = QtGui.QLabel(STARTING, cWidget)
	      procomp_state = QtGui.QLabel(STARTING, cWidget)

	      velodyne_state.setAlignment(QtCore.Qt.AlignCenter) 
	      gps_imu_velodyne_state.setAlignment(QtCore.Qt.AlignCenter) 
	      prosilica_state.setAlignment(QtCore.Qt.AlignCenter) 
	      IMU_state.setAlignment(QtCore.Qt.AlignCenter) 
	      axis_state.setAlignment(QtCore.Qt.AlignCenter) 
	      empatica_state.setAlignment(QtCore.Qt.AlignCenter) 
	      procomp_state.setAlignment(QtCore.Qt.AlignCenter) 

	      velodyne_label.setAlignment(QtCore.Qt.AlignCenter) 
	      gps_imu_velodyne_label.setAlignment(QtCore.Qt.AlignCenter) 
	      prosilica_label.setAlignment(QtCore.Qt.AlignCenter) 
	      IMU_label.setAlignment(QtCore.Qt.AlignCenter) 
	      axis_label.setAlignment(QtCore.Qt.AlignCenter) 
	      empatica_label.setAlignment(QtCore.Qt.AlignCenter) 
	      procomp_label.setAlignment(QtCore.Qt.AlignCenter) 
	      
	      grid.setColumnMinimumWidth(0,100)
	      grid.setColumnMinimumWidth(1,100)
	      grid.setColumnMinimumWidth(2,100)
	      grid.setColumnMinimumWidth(3,100)
	      grid.setColumnMinimumWidth(4,100)
              grid.setColumnMinimumWidth(5,100)
	      grid.setColumnMinimumWidth(6,100)

              grid.addWidget(velodyne_label, 0, 0)
              grid.addWidget(gps_imu_velodyne_label, 0, 1)  
	      grid.addWidget(prosilica_label, 0, 2)
              grid.addWidget(IMU_label, 0, 3)
	      grid.addWidget(axis_label, 0, 4)
              grid.addWidget(empatica_label, 0, 5)
	      grid.addWidget(procomp_label, 0, 6)  
	      grid.addWidget(velodyne_state, 1, 0)
              grid.addWidget(gps_imu_velodyne_state, 1, 1)  
	      grid.addWidget(prosilica_state, 1, 2)
              grid.addWidget(IMU_state, 1, 3)
	      grid.addWidget(axis_state, 1, 4)
              grid.addWidget(empatica_state, 1, 5)
	      grid.addWidget(procomp_state, 1, 6)    

              cWidget.setLayout(grid)
              self.setCentralWidget(cWidget)


def  Draw_GUI():
	app = QtGui.QApplication(sys.argv)
	main = MainWindow()
	main.show()
	sys.exit(app.exec_())
    

if __name__ == '__main__':
	try:
	    Draw_GUI()
	except rospy.ROSInterruptException: pass
    

