#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
import time
import os, signal
import subprocess
import math

tpub = rospy.Publisher('/managed/key', Twist, queue_size=10)
rospy.init_node('turn_test')


print 'Ensure the robot is in CLOSED LOOP mode, and the correct drive type is selected (see launch file)\
CAUTION, the robot will move during this test \
. Make sure robot has at least 1 meter of clearance in all directions. \
press enter to continue'

x = raw_input("press enter.....")

# build the bag file
# rosbag_proc = subprocess.Popen(["rosbag", "record", "managed/key", "rr_openrover_driver/odom_encoder"])

run_times = (3, 0.5, 3, 0.5)

# motion should be pi/8, then pi/6, then pi/4, then pi/2
angularz_set = [
    ((-math.pi / denom, 0, math.pi / denom, 0), run_times) for denom in reversed(range(2, 10, 2))
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

#os.killpg(os.getpgid(rosbag_proc.pid), signal.SIGINT)