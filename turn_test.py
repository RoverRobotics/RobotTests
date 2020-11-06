#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
import time
import os, signal
import subprocess

tpub = rospy.Publisher('/managed/key', Twist, queue_size=10)
rospy.init_node('turn_test')


print 'CAUTION, the robot will move during this test \
. Make sure robot has at least 1 meter of clearance in all directions. \
press enter to continue'

x = raw_input("press enter.....")

# build the bag file
rosbag_proc = subprocess.Popen(["rosbag", "record", "managed/key", "rr_openrover_driver/odom_encoder"])

run_times = (3, 0.5, 3, 0.5)
angularz_set = [
    ((-i/8.0, 0, i/8.0, 0), run_times) for i in range(2, 9, 1)
]

twistmsg = Twist()
for angularz, timez in angularz_set:
    for z, t in zip(angularz, timez):
        print 'commanding %f velocity for %f seconds' % (z, t)
        twistmsg.angular.z = z
    
        time_start = time.time()
        while True:
            tpub.publish(twistmsg)
            if abs(time.time() - time_start) > t:
                break 

twistmsg.angular.z = 0
tpub.publish(twistmsg)
print 'complete'

os.killpg(os.getpgid(rosbag_proc.pid), signal.SIGINT)