#!/usr/bin/env python
import rospy
import rospkg
import os
import subprocess
from subprocess import Popen
from std_msgs.msg import *
from custom_msgs.msg import *


class SectionLock:
    clients = {}

    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.init_node('section_lock', anonymous=True)

        self.pub = rospy.Publisher('section_lock', String, queue_size=0)

        rospy.Subscriber("section_identifier", V2I, self.callback)

        self.handled_crossing = False
        self.old_data = None
        self.proc = None

    def callback(self, data):
        # If you try to acquire a diffent lock than the one you are holding you will release the old
        # lock
        if (self.proc is not None) and (data.intersection != self.old_data):
            # Tell the java zookeeper tool to release the lock
            self.proc.communicate(input='\n')
            # Tell section_identifier that you left the section
            self.pub.publish("release")
            self.handled_crossing = False

        if not self.handled_crossing and (data.intersection != self.old_data):
            self.handled_crossing = True
            self.old_data = data.intersection
            self.pub.publish("stop")

            rospack = rospkg.RosPack()
            lock_jar_path = rospack.get_path('section_lock') + '/scripts/lock_zk_node.jar'

            sections_needed = self.calculate_sections_needed(data)
            print sections_needed

            # We ask for the section lock
            env = dict(os.environ)
            env['JAVA_OPTS'] = 'foo'
            self.proc = Popen(['java', '-jar', lock_jar_path, 'localhost:2181', data.intersection,
                               '4'], env=env, stdout=subprocess.PIPE, stdin=subprocess.PIPE)

            # Signal the truck to stop
            # self.pub.publish("stop")

            number_of_messages = 0
            while True:
                line = self.proc.stdout.readline()

                if 'zk_connection_successful' in line:
                    self.pub.publish("zk_connection_successful")

                if 'zk_connection_failed' in line:
                    self.pub.publish("zk_connection_failed")

                if 'lock_accepted' in line:
                    # We've been granted the lock!
                    # Tell the truck to continue driving
                    self.pub.publish("continue")
                    break

    @staticmethod
    def calculate_sections_needed(data):
        action = getattr(data.action, data.intersection)
        print "Action: " + action
        print "Initial direction : " + data.initial_direction

        if data.initial_direction == "right" and action == "turn_left":
            return '4'  # Book all 4 sections
        else:
            return 0


if __name__ == '__main__':
    s = SectionLock()
    rospy.spin()
