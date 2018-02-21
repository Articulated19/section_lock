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


class SectionLock:

    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.init_node('section_lock', anonymous=True)

        self.pub = rospy.Publisher('section_lock', String, queue_size=0)

        rospy.Subscriber("section_identifier", String, self.callback)

        self.handled_crossing = False

    def callback(self, data):

        if not self.handled_crossing:
            self.handled_crossing = True
            self.pub.publish("stop")

            rospack = rospkg.RosPack()
            lock_jar_path = rospack.get_path('section_lock') + '/scripts/lock_zk_node.jar'

            # We ask for the section lock
            env = dict(os.environ)
            env['JAVA_OPTS'] = 'foo'
            proc = Popen(['java', '-jar', lock_jar_path, 'localhost:2181'], env=env,
                         stdout=subprocess.PIPE, stdin=subprocess.PIPE)

            # Signal the truck to stop
            #self.pub.publish("stop")
            while True:
                line = proc.stdout.readline()
                print('lock_accepted' in line)
                if 'lock_accepted' in line:
                    break

            # We've been granted the lock!
            # Tell the truck to continue driving
            self.pub.publish("continue")
            time.sleep(5)    
            self.pub.publish("release")
            self.handled_crossing = False
            # Tell the java zookeeper tool to release the lock
            proc.communicate(input='\n')


if __name__ == '__main__':
    s = SectionLock()
    rospy.spin()
