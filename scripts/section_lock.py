#!/usr/bin/env python
import rospy
from custom_msgs.msg import *
from custom_msgs.srv import *
from std_msgs.msg import String
import rospkg
import os
import time
import subprocess
from subprocess import Popen
import sys

pub = None

def callback(data):
	global pub
	rospack = rospkg.RosPack()
	pub.publish()
	waitTime = 10

	lockJarPath = rospack.get_path('section_lock') + '/scripts/lock_zk_node.jar'

	# We ask for the section lock
	env = dict(os.environ)
	env['JAVA_OPTS'] = 'foo'
	proc = Popen(['java', '-jar', lockJarPath, 'lock'], env=env,
	             stdout=subprocess.PIPE, stdin=subprocess.PIPE)


	# Signal the truck to stop
	pub.publish("stop")
	while True:
	    line = proc.stdout.readline()
	    print('lock_accepted' in line)
	    if 'lock_accepted' in line:
	        break


	# We've been granted the lock!
	pub.publish("continue")
	proc.communicate(input='\n')

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('section_lock', anonymous=True)

    global pub
    pub = rospy.Publisher('section_lock', String, queue_size=10)

    rospy.Subscriber("section_identifier", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()