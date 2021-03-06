#!/usr/bin/env python
# license removed for brevity
import rospy

from heartbeat.HeartbeatClientPython import HeartbeatClientPy

def velodyne_Dat():
	rospy.init_node('pippo')
	hb = HeartbeatClientPy(rospy.get_name())
	hb.start();
	hb.set_node_state(1)
	hb.stop()

if __name__ == '__main__':
    try:
        velodyne_Dat()
    except rospy.ROSInterruptException: pass
    

