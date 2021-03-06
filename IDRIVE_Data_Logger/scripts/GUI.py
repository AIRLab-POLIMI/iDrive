#!/usr/bin/env python
import rospy
import std_msgs.msg
import sys
import heartbeat
from PyQt4 import QtGui, QtCore # importiamo i moduli necessari
from rosgraph_msgs.msg import TopicStatistics
from heartbeat.msg import State

Node_state ={1:"starting", 2:"started", 0:"stopped"};


class MainWindow(QtGui.QMainWindow):
      def __init__(self):
              QtGui.QMainWindow.__init__(self)
              self.setWindowTitle('I.DRIVE Data Logger')
  	      cWidget = QtGui.QWidget(self)

              vbox = QtGui.QVBoxLayout(cWidget);
	 
	      loggerBox = QtGui.QGroupBox(self)
	      loggerBox.setTitle("LOGGERS")
 	      loggerBox.setStyleSheet("""QGroupBox {
					    	border: 1px solid gray;
					    	border-radius: 9px;
					    	margin-top: 0.5em;
					      }
				    QGroupBox::title {
					    		subcontrol-origin: margin;
					  		left: 10px;
					    		padding: 0 3px 0 3px;
						     }""");
	      loggerGrid = QtGui.QGridLayout(loggerBox) 
	      loggerGrid.setHorizontalSpacing(10)
	      loggerGrid.setVerticalSpacing(10)
	
	      loggerBox.setLayout(loggerGrid)
	      
	      vbox.addWidget(loggerBox);

	      empatica_label = QtGui.QLabel("Empatica", cWidget)
	      procomp_label = QtGui.QLabel("Procomp", cWidget)
	      prosilica1_label = QtGui.QLabel("Prosilica 1", cWidget) 
	      prosilica2_label = QtGui.QLabel("Prosilica 2", cWidget) 
      	      xsens_label = QtGui.QLabel("Xsens IMU", cWidget)

	      empatica_topic = QtGui.QLabel("/empatica_sensor", cWidget)
	      procomp_topic = QtGui.QLabel("/procomp_sensor", cWidget)
	      prosilica1_image_topic = QtGui.QLabel("/prosilica1/image_raw", cWidget)
	      prosilica1_info_topic = QtGui.QLabel("/prosilica1/camera_info", cWidget) 	
	      prosilica2_image_topic = QtGui.QLabel("/prosilica2/image_raw", cWidget)
	      prosilica2_info_topic = QtGui.QLabel("/prosilica2/camera_info", cWidget) 
	      xsens_imu_topic = QtGui.QLabel("/xsens/imu", cWidget)
	      xsens_mag_topic = QtGui.QLabel("/xsens/mag", cWidget) 

  	      sensor_name = QtGui.QLabel("Sensor", cWidget)
	      sensor_state = QtGui.QLabel("Sensor state", cWidget)
	      topic_name = QtGui.QLabel("Topic name", cWidget)
	      topic_state = QtGui.QLabel("Topic logger state", cWidget)
	      topic_window_start = QtGui.QLabel("window start", cWidget)
	      topic_window_stop = QtGui.QLabel("window stop", cWidget)
	      topic_delivered_msgs = QtGui.QLabel("Delivered msg", cWidget)
	      topic_dropped_msgs = QtGui.QLabel("Dropped msg", cWidget)
	      topic_traffic_msgs = QtGui.QLabel("Traffic (bytes)", cWidget)	

	      empatica_label.setAlignment(QtCore.Qt.AlignCenter) 
	      procomp_label.setAlignment(QtCore.Qt.AlignCenter) 
	      prosilica1_label.setAlignment(QtCore.Qt.AlignCenter) 
	      prosilica2_label.setAlignment(QtCore.Qt.AlignCenter) 
 	      xsens_label.setAlignment(QtCore.Qt.AlignCenter) 

	      empatica_topic.setAlignment(QtCore.Qt.AlignCenter) 
	      procomp_topic.setAlignment(QtCore.Qt.AlignCenter) 
	      prosilica1_image_topic.setAlignment(QtCore.Qt.AlignCenter) 
	      prosilica1_info_topic.setAlignment(QtCore.Qt.AlignCenter) 
	      prosilica2_image_topic.setAlignment(QtCore.Qt.AlignCenter) 
	      prosilica2_info_topic.setAlignment(QtCore.Qt.AlignCenter) 
	      xsens_imu_topic.setAlignment(QtCore.Qt.AlignCenter)
	      xsens_mag_topic.setAlignment(QtCore.Qt.AlignCenter)
	      
	      sensor_name.setAlignment(QtCore.Qt.AlignCenter)
	      sensor_state.setAlignment(QtCore.Qt.AlignCenter)
	      topic_name.setAlignment(QtCore.Qt.AlignCenter) 
	      topic_state.setAlignment(QtCore.Qt.AlignCenter)
	      topic_window_start.setAlignment(QtCore.Qt.AlignCenter)
	      topic_window_stop.setAlignment(QtCore.Qt.AlignCenter)
	      topic_delivered_msgs.setAlignment(QtCore.Qt.AlignCenter) 
	      topic_dropped_msgs.setAlignment(QtCore.Qt.AlignCenter) 
	      topic_traffic_msgs.setAlignment(QtCore.Qt.AlignCenter)	      

	      self.velodyne_state_val = 0
	      self.gps_imu_velodyne_state_val = 0
	      self.axis_state_val = 0
	      self.empatica_state_val = 0
	      self.procomp_state_val = 0
	      self.prosilica1_state_val = 0 
	      self.prosilica2_state_val = 0 
	      self.xsens_state_val = 0 

	      self.empatica_log_state_val = 0
	      self.procomp_log_state_val = 0
	      self.prosilica1_image_log_state_val = 0
	      self.prosilica1_info_log_state_val = 0
	      self.prosilica2_image_log_state_val = 0
	      self.prosilica2_info_log_state_val = 0
	      self.xsens_imu_log_state_val = 0
	      self.xsens_mag_log_state_val = 0

	      self.empatica_dlv_val = 0;
	      self.empatica_drp_val = 0;
	      self.empatica_bytes_val = 0;
	      self.empatica_start_val = 0;
	      self.empatica_stop_val = 0;
	      self.procomp_dlv_val = 0;
	      self.procomp_drp_val = 0;
	      self.procomp_bytes_val = 0;
	      self.procomp_start_val = 0;
	      self.procomp_stop_val = 0;
	      self.prosilica1_image_dlv_val = 0;
	      self.prosilica1_image_drp_val = 0;
	      self.prosilica1_image_bytes_val = 0;
	      self.prosilica1_image_start_val = 0;
	      self.prosilica1_image_stop_val = 0;
	      self.prosilica1_info_dlv_val = 0;
	      self.prosilica1_info_drp_val = 0;
	      self.prosilica1_info_bytes_val = 0;
	      self.prosilica1_info_start_val = 0;
	      self.prosilica1_info_stop_val = 0;
	      self.prosilica2_image_dlv_val = 0;
	      self.prosilica2_image_drp_val = 0;
	      self.prosilica2_image_bytes_val = 0;
	      self.prosilica2_image_start_val = 0;
	      self.prosilica2_image_stop_val = 0;
	      self.prosilica2_info_dlv_val = 0;
	      self.prosilica2_info_drp_val = 0;
	      self.prosilica2_info_bytes_val = 0;
	      self.prosilica2_info_start_val = 0;
	      self.prosilica2_info_stop_val = 0;
	      self.xsens_imu_dlv_val = 0;
	      self.xsens_imu_drp_val = 0;
	      self.xsens_imu_bytes_val = 0;
	      self.xsens_imu_start_val = 0;
	      self.xsens_imu_stop_val = 0;
	      self.xsens_mag_dlv_val = 0;
	      self.xsens_mag_drp_val = 0;
	      self.xsens_mag_bytes_val = 0;
	      self.xsens_mag_start_val = 0;
	      self.xsens_mag_stop_val = 0;


 	      self.velodyne_state = QtGui.QLabel(Node_state[0], cWidget)
	      self.gps_imu_velodyne_state = QtGui.QLabel(Node_state[0], cWidget)
	      self.axis_state = QtGui.QLabel(Node_state[0], cWidget)
	      self.empatica_state = QtGui.QLabel(Node_state[self.empatica_state_val], cWidget)
	      self.procomp_state = QtGui.QLabel(Node_state[self.procomp_state_val], cWidget)
	      self.prosilica1_state = QtGui.QLabel(Node_state[self.prosilica1_state_val], cWidget) 
	      self.prosilica2_state = QtGui.QLabel(Node_state[self.prosilica2_state_val], cWidget) 
	      self.xsens_state = QtGui.QLabel(Node_state[self.xsens_state_val], cWidget) 

	      self.empatica_log_state = QtGui.QLabel(Node_state[self.empatica_log_state_val], cWidget)
	      self.procomp_log_state = QtGui.QLabel(Node_state[self.procomp_log_state_val], cWidget)
	      self.prosilica1_image_log_state = QtGui.QLabel(Node_state[self.prosilica1_image_log_state_val], cWidget)
	      self.prosilica1_info_log_state = QtGui.QLabel(Node_state[self.prosilica1_info_log_state_val], cWidget)
	      self.prosilica2_image_log_state = QtGui.QLabel(Node_state[self.prosilica2_image_log_state_val], cWidget)
	      self.prosilica2_info_log_state = QtGui.QLabel(Node_state[self.prosilica2_info_log_state_val], cWidget)
	      self.xsens_imu_log_state = QtGui.QLabel(Node_state[self.xsens_imu_log_state_val], cWidget)
	      self.xsens_mag_log_state = QtGui.QLabel(Node_state[self.xsens_mag_log_state_val], cWidget)

	      self.empatica_dlv = QtGui.QLabel(str(self.empatica_dlv_val), cWidget)
	      self.empatica_drp = QtGui.QLabel(str(self.empatica_drp_val), cWidget)
	      self.empatica_bytes = QtGui.QLabel(str(self.empatica_bytes_val), cWidget)
	      self.empatica_start = QtGui.QLabel(str(self.empatica_start_val), cWidget)
	      self.empatica_stop = QtGui.QLabel(str(self.empatica_stop_val), cWidget)
	      self.procomp_dlv = QtGui.QLabel(str(self.procomp_dlv_val), cWidget)
	      self.procomp_drp = QtGui.QLabel(str(self.procomp_drp_val), cWidget)
	      self.procomp_bytes = QtGui.QLabel(str(self.procomp_bytes_val), cWidget)
	      self.procomp_start = QtGui.QLabel(str(self.procomp_start_val), cWidget)
	      self.procomp_stop = QtGui.QLabel(str(self.procomp_stop_val), cWidget)
              self.prosilica1_image_dlv = QtGui.QLabel(str(self.prosilica1_image_dlv_val), cWidget)
	      self.prosilica1_image_drp = QtGui.QLabel(str(self.prosilica1_image_drp_val), cWidget)
	      self.prosilica1_image_bytes = QtGui.QLabel(str(self.prosilica1_image_bytes_val), cWidget)
	      self.prosilica1_image_start = QtGui.QLabel(str(self.prosilica1_image_start_val), cWidget)
	      self.prosilica1_image_stop = QtGui.QLabel(str(self.prosilica1_image_stop_val), cWidget)
	      self.prosilica1_info_dlv = QtGui.QLabel(str(self.prosilica1_info_dlv_val), cWidget)
	      self.prosilica1_info_drp = QtGui.QLabel(str(self.prosilica1_info_drp_val), cWidget)
	      self.prosilica1_info_bytes = QtGui.QLabel(str(self.prosilica1_info_bytes_val), cWidget)
	      self.prosilica1_info_start = QtGui.QLabel(str(self.prosilica1_info_start_val), cWidget)
	      self.prosilica1_info_stop = QtGui.QLabel(str(self.prosilica1_info_stop_val), cWidget)
              self.prosilica2_image_dlv = QtGui.QLabel(str(self.prosilica2_image_dlv_val), cWidget)
	      self.prosilica2_image_drp = QtGui.QLabel(str(self.prosilica2_image_drp_val), cWidget)
	      self.prosilica2_image_bytes = QtGui.QLabel(str(self.prosilica2_image_bytes_val), cWidget)
	      self.prosilica2_image_start = QtGui.QLabel(str(self.prosilica2_image_start_val), cWidget)
	      self.prosilica2_image_stop = QtGui.QLabel(str(self.prosilica2_image_stop_val), cWidget)
	      self.prosilica2_info_dlv = QtGui.QLabel(str(self.prosilica2_info_dlv_val), cWidget)
	      self.prosilica2_info_drp = QtGui.QLabel(str(self.prosilica2_info_drp_val), cWidget)
	      self.prosilica2_info_bytes = QtGui.QLabel(str(self.prosilica2_info_bytes_val), cWidget)
	      self.prosilica2_info_start = QtGui.QLabel(str(self.prosilica2_info_start_val), cWidget)
	      self.prosilica2_info_stop = QtGui.QLabel(str(self.prosilica2_info_stop_val), cWidget)
  	      self.xsens_imu_dlv = QtGui.QLabel(str(self.xsens_imu_dlv_val), cWidget)
	      self.xsens_imu_drp = QtGui.QLabel(str(self.xsens_imu_drp_val), cWidget)
	      self.xsens_imu_bytes = QtGui.QLabel(str(self.xsens_imu_bytes_val), cWidget)
	      self.xsens_imu_start = QtGui.QLabel(str(self.xsens_imu_start_val), cWidget)
	      self.xsens_imu_stop = QtGui.QLabel(str(self.xsens_imu_stop_val), cWidget)
  	      self.xsens_mag_dlv = QtGui.QLabel(str(self.xsens_mag_dlv_val), cWidget)
	      self.xsens_mag_drp = QtGui.QLabel(str(self.xsens_mag_drp_val), cWidget)
	      self.xsens_mag_bytes = QtGui.QLabel(str(self.xsens_mag_bytes_val), cWidget)
	      self.xsens_mag_start = QtGui.QLabel(str(self.xsens_mag_start_val), cWidget)
	      self.xsens_mag_stop = QtGui.QLabel(str(self.xsens_mag_stop_val), cWidget)

	      self.velodyne_state.setAlignment(QtCore.Qt.AlignCenter) 
	      self.gps_imu_velodyne_state.setAlignment(QtCore.Qt.AlignCenter) 
	      self.axis_state.setAlignment(QtCore.Qt.AlignCenter) 
	      self.empatica_state.setAlignment(QtCore.Qt.AlignCenter) 
	      self.procomp_state.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica1_state.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica2_state.setAlignment(QtCore.Qt.AlignCenter)
 	      self.xsens_state.setAlignment(QtCore.Qt.AlignCenter)


	      self.empatica_log_state.setAlignment(QtCore.Qt.AlignCenter) 
	      self.procomp_log_state.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica1_image_log_state.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica1_info_log_state.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica2_image_log_state.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica2_info_log_state.setAlignment(QtCore.Qt.AlignCenter) 
  	      self.xsens_imu_log_state.setAlignment(QtCore.Qt.AlignCenter) 
	      self.xsens_mag_log_state.setAlignment(QtCore.Qt.AlignCenter) 

 	      self.empatica_dlv.setAlignment(QtCore.Qt.AlignCenter) 
	      self.empatica_drp.setAlignment(QtCore.Qt.AlignCenter) 
	      self.empatica_bytes.setAlignment(QtCore.Qt.AlignCenter) 
	      self.empatica_start.setAlignment(QtCore.Qt.AlignCenter) 
	      self.empatica_stop.setAlignment(QtCore.Qt.AlignCenter) 
	      self.procomp_dlv.setAlignment(QtCore.Qt.AlignCenter) 
	      self.procomp_drp.setAlignment(QtCore.Qt.AlignCenter) 
	      self.procomp_bytes.setAlignment(QtCore.Qt.AlignCenter) 
	      self.procomp_start.setAlignment(QtCore.Qt.AlignCenter) 
	      self.procomp_stop.setAlignment(QtCore.Qt.AlignCenter) 
              self.prosilica1_image_dlv.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica1_image_drp.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica1_image_bytes.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica1_image_start.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica1_image_stop.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica1_info_dlv.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica1_info_drp.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica1_info_bytes.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica1_info_start.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica1_info_stop.setAlignment(QtCore.Qt.AlignCenter) 
              self.prosilica2_image_dlv.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica2_image_drp.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica2_image_bytes.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica2_image_start.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica2_image_stop.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica2_info_dlv.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica2_info_drp.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica2_info_bytes.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica2_info_start.setAlignment(QtCore.Qt.AlignCenter) 
	      self.prosilica2_info_stop.setAlignment(QtCore.Qt.AlignCenter) 
 	      self.xsens_imu_dlv.setAlignment(QtCore.Qt.AlignCenter) 
	      self.xsens_imu_drp.setAlignment(QtCore.Qt.AlignCenter) 
	      self.xsens_imu_bytes.setAlignment(QtCore.Qt.AlignCenter) 
	      self.xsens_imu_start.setAlignment(QtCore.Qt.AlignCenter) 
	      self.xsens_imu_stop.setAlignment(QtCore.Qt.AlignCenter) 
  	      self.xsens_mag_dlv.setAlignment(QtCore.Qt.AlignCenter) 
	      self.xsens_mag_drp.setAlignment(QtCore.Qt.AlignCenter) 
	      self.xsens_mag_bytes.setAlignment(QtCore.Qt.AlignCenter) 
	      self.xsens_mag_start.setAlignment(QtCore.Qt.AlignCenter) 
	      self.xsens_mag_stop.setAlignment(QtCore.Qt.AlignCenter) 
	       
		
	      loggerGrid.addWidget(sensor_name, 0, 0)
	      loggerGrid.addWidget(sensor_state, 1, 0)
	      loggerGrid.addWidget(topic_name, 2, 0)
	      loggerGrid.addWidget(topic_state, 3, 0)
	      loggerGrid.addWidget(topic_window_start, 4, 0)
	      loggerGrid.addWidget(topic_window_stop, 5, 0)
	      loggerGrid.addWidget(topic_delivered_msgs, 6, 0) 
	      loggerGrid.addWidget(topic_dropped_msgs, 7, 0) 
	      loggerGrid.addWidget(topic_traffic_msgs, 8, 0)	
	      loggerGrid.addWidget(empatica_label, 0, 1) 
	      loggerGrid.addWidget(self.empatica_state, 1, 1) 
	      loggerGrid.addWidget(empatica_topic, 2, 1)
	      loggerGrid.addWidget(self.empatica_log_state, 3, 1)
	      loggerGrid.addWidget(self.empatica_start, 4, 1)
	      loggerGrid.addWidget( self.empatica_stop, 5, 1)
	      loggerGrid.addWidget(self.empatica_dlv, 6, 1)
	      loggerGrid.addWidget( self.empatica_drp, 7, 1)
	      loggerGrid.addWidget( self.empatica_bytes, 8, 1)	
	      loggerGrid.addWidget(procomp_label, 0, 2) 
	      loggerGrid.addWidget(self.procomp_state, 1, 2)
	      loggerGrid.addWidget(procomp_topic, 2, 2)  
	      loggerGrid.addWidget(self.procomp_log_state, 3, 2)
	      loggerGrid.addWidget(self.procomp_start, 4, 2)
	      loggerGrid.addWidget( self.procomp_stop, 5, 2)
	      loggerGrid.addWidget(self.procomp_dlv, 6, 2)
	      loggerGrid.addWidget( self.procomp_drp, 7, 2)
	      loggerGrid.addWidget( self.procomp_bytes, 8, 2)
	      loggerGrid.addWidget(prosilica1_label, 0, 3, 1, 2) 
	      loggerGrid.addWidget(self.prosilica1_state, 1, 3, 1, 2)
	      loggerGrid.addWidget(prosilica1_image_topic, 2, 3)
	      loggerGrid.addWidget(self.prosilica1_image_log_state, 3, 3)
	      loggerGrid.addWidget(self.prosilica1_image_start, 4, 3)
	      loggerGrid.addWidget( self.prosilica1_image_stop, 5, 3)
	      loggerGrid.addWidget(self.prosilica1_image_dlv, 6, 3)
	      loggerGrid.addWidget( self.prosilica1_image_drp, 7, 3)
	      loggerGrid.addWidget( self.prosilica1_image_bytes, 8, 3)	
	      loggerGrid.addWidget(prosilica1_info_topic, 2, 4)
	      loggerGrid.addWidget(self.prosilica1_info_log_state, 3, 4)
	      loggerGrid.addWidget(self.prosilica1_info_start, 4, 4)
	      loggerGrid.addWidget( self.prosilica1_info_stop, 5, 4)
	      loggerGrid.addWidget(self.prosilica1_info_dlv, 6, 4)
	      loggerGrid.addWidget( self.prosilica1_info_drp, 7, 4)
	      loggerGrid.addWidget( self.prosilica1_info_bytes, 8, 4)	
	      loggerGrid.addWidget(prosilica2_label, 0, 5, 1, 2) 
	      loggerGrid.addWidget(self.prosilica2_state, 1, 5, 1, 2)
	      loggerGrid.addWidget(prosilica2_image_topic, 2, 5)
	      loggerGrid.addWidget(self.prosilica2_image_log_state, 3, 5)
	      loggerGrid.addWidget(self.prosilica2_image_start, 4, 5)
	      loggerGrid.addWidget( self.prosilica2_image_stop, 5, 5)
	      loggerGrid.addWidget(self.prosilica2_image_dlv, 6, 5)
	      loggerGrid.addWidget( self.prosilica2_image_drp, 7, 5)
	      loggerGrid.addWidget( self.prosilica2_image_bytes, 8, 5)	
	      loggerGrid.addWidget(prosilica2_info_topic, 2, 6)
	      loggerGrid.addWidget(self.prosilica2_info_log_state, 3, 6)
	      loggerGrid.addWidget(self.prosilica2_info_start, 4, 6)
	      loggerGrid.addWidget( self.prosilica2_info_stop, 5, 6)
	      loggerGrid.addWidget(self.prosilica2_info_dlv, 6, 6)
	      loggerGrid.addWidget( self.prosilica2_info_drp, 7, 6)
	      loggerGrid.addWidget( self.prosilica2_info_bytes, 8, 6)	
	      loggerGrid.addWidget(xsens_label, 0, 7, 1, 2) 
	      loggerGrid.addWidget(self.xsens_state, 1, 7, 1, 2)
	      loggerGrid.addWidget(xsens_imu_topic, 2, 7)
	      loggerGrid.addWidget(self.xsens_imu_log_state, 3, 7)
	      loggerGrid.addWidget(self.xsens_imu_start, 4, 7)
	      loggerGrid.addWidget( self.xsens_imu_stop, 5, 7)
	      loggerGrid.addWidget(self.xsens_imu_dlv, 6, 7)
	      loggerGrid.addWidget( self.xsens_imu_drp, 7, 7)
	      loggerGrid.addWidget( self.xsens_imu_bytes, 8, 7)	
	      loggerGrid.addWidget(xsens_mag_topic, 2, 8)
	      loggerGrid.addWidget(self.xsens_mag_log_state, 3, 8)
	      loggerGrid.addWidget(self.xsens_mag_start, 4, 8)
	      loggerGrid.addWidget( self.xsens_mag_stop, 5, 8)
	      loggerGrid.addWidget(self.xsens_mag_dlv, 6, 8)
	      loggerGrid.addWidget( self.xsens_mag_drp, 7, 8)
	      loggerGrid.addWidget( self.xsens_mag_bytes, 8, 8)			

              cWidget.setLayout(vbox)
              self.setCentralWidget(cWidget)

      def update_GUI(self):
	      self.empatica_state.setText(Node_state[self.empatica_state_val]) 
	      self.procomp_state.setText(Node_state[self.procomp_state_val])
	      self.prosilica1_state.setText(Node_state[self.prosilica1_state_val]) 
	      self.prosilica2_state.setText(Node_state[self.prosilica2_state_val])
	      self.xsens_state.setText(Node_state[self.xsens_state_val])

	      self.procomp_log_state.setText(Node_state[self.procomp_log_state_val])
	      self.empatica_log_state.setText(Node_state[self.empatica_log_state_val])
	      self.prosilica1_image_log_state.setText(Node_state[self.prosilica1_image_log_state_val])
	      self.prosilica1_info_log_state.setText(Node_state[self.prosilica1_info_log_state_val])
	      self.prosilica2_image_log_state.setText(Node_state[self.prosilica2_image_log_state_val])
	      self.prosilica2_info_log_state.setText(Node_state[self.prosilica2_info_log_state_val])
	      self.xsens_imu_log_state.setText(Node_state[self.xsens_imu_log_state_val])
	      self.xsens_mag_log_state.setText(Node_state[self.xsens_mag_log_state_val])

	      self.empatica_dlv.setText(str(self.empatica_dlv_val))
   	      self.empatica_drp.setText(str(self.empatica_drp_val))
	      self.empatica_bytes.setText(str(self.empatica_bytes_val))
 	      self.empatica_start.setText(str(self.empatica_start_val))
	      self.empatica_stop.setText(str(self.empatica_stop_val))
 	      self.procomp_dlv.setText(str(self.procomp_dlv_val))
   	      self.procomp_drp.setText(str(self.procomp_drp_val))
	      self.procomp_bytes.setText(str(self.procomp_bytes_val))
 	      self.procomp_start.setText(str(self.procomp_start_val))
	      self.procomp_stop.setText(str(self.procomp_stop_val))
	      self.prosilica1_image_dlv.setText(str(self.prosilica1_image_dlv_val))
	      self.prosilica1_image_drp.setText(str(self.prosilica1_image_drp_val))
	      self.prosilica1_image_bytes.setText(str(self.prosilica1_image_bytes_val))
 	      self.prosilica1_image_start.setText(str(self.prosilica1_image_start_val))
	      self.prosilica1_image_stop.setText(str(self.prosilica1_image_stop_val))
	      self.prosilica1_info_dlv.setText(str(self.prosilica1_info_dlv_val))
	      self.prosilica1_info_drp.setText(str(self.prosilica1_info_drp_val))
	      self.prosilica1_info_bytes.setText(str(self.prosilica1_info_bytes_val))
 	      self.prosilica1_info_start.setText(str(self.prosilica1_info_start_val))
	      self.prosilica1_info_stop.setText(str(self.prosilica1_info_stop_val))
              self.prosilica2_image_dlv.setText(str(self.prosilica2_image_dlv_val))
	      self.prosilica2_image_drp.setText(str(self.prosilica2_image_drp_val))
	      self.prosilica2_image_bytes.setText(str(self.prosilica2_image_bytes_val))
 	      self.prosilica2_image_start.setText(str(self.prosilica2_image_start_val))
	      self.prosilica2_image_stop.setText(str(self.prosilica2_image_stop_val))
	      self.prosilica2_info_dlv.setText(str(self.prosilica2_info_dlv_val))
	      self.prosilica2_info_drp.setText(str(self.prosilica2_info_drp_val))
	      self.prosilica2_info_bytes.setText(str(self.prosilica2_info_bytes_val))
 	      self.prosilica2_info_start.setText(str(self.prosilica2_info_start_val))
	      self.prosilica2_info_stop.setText(str(self.prosilica2_info_stop_val))
              self.xsens_imu_dlv.setText(str(self.xsens_imu_dlv_val))
	      self.xsens_imu_drp.setText(str(self.xsens_imu_drp_val))
	      self.xsens_imu_bytes.setText(str(self.xsens_imu_bytes_val))
	      self.xsens_imu_start.setText(str(self.xsens_imu_start_val))
	      self.xsens_imu_stop.setText(str(self.xsens_imu_stop_val))
  	      self.xsens_mag_dlv.setText(str(self.xsens_mag_dlv_val))
	      self.xsens_mag_drp.setText(str(self.xsens_mag_drp_val))
	      self.xsens_mag_bytes.setText(str(self.xsens_mag_bytes_val))
	      self.xsens_mag_start.setText(str(self.xsens_mag_start_val))
	      self.xsens_mag_stop.setText(str(self.xsens_mag_stop_val))
	      

def state_update(data):	
	global main
	print("I heard ",  data.node_name,  data.value)
	if (data.node_name == '/empatica_client'):
		main.empatica_state_val = data.value
	elif (data.node_name == '/procomp_client'):
		main.procomp_state_val = data.value
	elif (data.node_name == '/prosilica1_driver'):
		main.prosilica1_state_val = data.value
	elif (data.node_name == '/prosilica2_driver'):
		main.prosilica2_state_val = data.value
	elif (data.node_name == '/xsens'):
		main.xsens_state_val = data.value
	elif (data.node_name == 'empatica_sensor_logger'):
		main.empatica_log_state_val = data.value
	elif (data.node_name == 'procomp_sensor_logger'):
		main.procomp_log_state_val = data.value
	elif (data.node_name == '/mongodb_log_worker_0_prosilica1_image_raw'):
		main.prosilica1_image_log_state_val = data.value
	elif (data.node_name == 'prosilica1_camera_info_logger'):
		main.prosilica1_info_log_state_val = data.value
	elif (data.node_name == '/mongodb_log_worker_0_prosilica2_image_raw'):
		main.prosilica2_image_log_state_val = data.value
	elif (data.node_name == 'prosilica2_camera_info_logger'):
		main.prosilica2_info_log_state_val = data.value
	elif (data.node_name == 'xsens_imu_logger'):
		main.xsens_imu_log_state_val = data.value
	elif (data.node_name == 'xsens_mag_logger'):
		main.xsens_mag_log_state_val = data.value
	main.update_GUI()	

def topic_statistics(data):
	global main
	if (data.topic == '/empatica_sensor'):
		main.empatica_dlv_val = data.delivered_msgs
		main.empatica_drp_val = data.dropped_msgs
		main.empatica_bytes_val = data.traffic
		main.empatica_start_val = data.window_start.to_sec()
		main.empatica_stop_val = data.window_stop.to_sec()
	elif (data.topic == '/procomp_sensor'):
		main.procomp_dlv_val = data.delivered_msgs
		main.procomp_drp_val = data.dropped_msgs
		main.procomp_bytes_val = data.traffic
		main.procomp_start_val = data.window_start.to_sec()
		main.procomp_stop_val = data.window_stop.to_sec()
	elif (data.topic == '/prosilica1/image_raw'):
		main.prosilica1_image_dlv_val = data.delivered_msgs
		main.prosilica1_image_drp_val = data.dropped_msgs
		main.prosilica1_image_bytes_val = data.traffic
		main.prosilica1_image_start_val = data.window_start.to_sec()
		main.prosilica1_image_stop_val = data.window_stop.to_sec()
	elif (data.topic == '/prosilica1/camera_info'):
		main.prosilica1_info_dlv_val = data.delivered_msgs
		main.prosilica1_info_drp_val = data.dropped_msgs
		main.prosilica1_info_bytes_val = data.traffic
		main.prosilica1_info_start_val = data.window_start.to_sec()
		main.prosilica1_info_stop_val = data.window_stop.to_sec()
	elif (data.topic == '/prosilica2/image_raw'):
		main.prosilica2_image_dlv_val = data.delivered_msgs
		main.prosilica2_image_drp_val = data.dropped_msgs
		main.prosilica2_image_bytes_val = data.traffic
		main.prosilica2_image_start_val = data.window_start.to_sec()
		main.prosilica2_image_stop_val = data.window_stop.to_sec()
	elif (data.topic == '/prosilica2/camera_info'):
		main.prosilica2_info_dlv_val = data.delivered_msgs
		main.prosilica2_info_drp_val = data.dropped_msgs
		main.prosilica2_info_bytes_val = data.traffic
		main.prosilica2_info_start_val = data.window_start.to_sec()
		main.prosilica2_info_stop_val = data.window_stop.to_sec()
	elif (data.topic == '/xsens/imu'):
		main.xsens_imu_dlv_val = data.delivered_msgs
		main.xsens_imu_drp_val = data.dropped_msgs
		main.xsens_imu_bytes_val = data.traffic
		main.xsens_imu_start_val = data.window_start.to_sec()
		main.xsens_imu_stop_val = data.window_stop.to_sec()
	elif (data.topic == '/xsens/mag'):
		main.xsens_mag_dlv_val = data.delivered_msgs
		main.xsens_mag_drp_val = data.dropped_msgs
		main.xsens_mag_bytes_val = data.traffic
		main.xsens_mag_start_val = data.window_start.to_sec()
		main.xsens_mag_stop_val = data.window_stop.to_sec()


	
	main.update_GUI()	

def Draw_GUI():
	global main
	app = QtGui.QApplication(sys.argv)
	main = MainWindow()
	main.show()
	rospy.init_node('GUI', anonymous=True)
 	rospy.Subscriber("state", State, state_update)
	rospy.Subscriber("statistics", TopicStatistics, topic_statistics)
	sys.exit(app.exec_())
	rospy.spin()
    

if __name__ == '__main__':
	try:
	    Draw_GUI()
	except rospy.ROSInterruptException: pass
    

