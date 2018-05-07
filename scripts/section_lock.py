#!/usr/bin/env python
import rospy
import rospkg
import os
import subprocess
import psutil
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
            try:
                # The process may not be alive so we add this line in a try-catch
                # to avoid exceptions. Not necessary. Just to avoid red prints in command line
                self.proc.communicate(input='\n')
            except (ValueError, OSError):
                pass
            finally:
                # Tell section_identifier that you left the section
                self.pub.publish("release")
                self.handled_crossing = False

        if not self.handled_crossing and (data.intersection != self.old_data):
            self.handled_crossing = True
            self.old_data = data.intersection
            self.pub.publish("stop")

            print data

            rospack = rospkg.RosPack()
            lock_jar_path = rospack.get_path('section_lock') + '/scripts/lock_zk_node.jar'
            print "data.intersection: " + data.intersection

            if data.intersection == "Left_Curve" or data.intersection == "Right_Curve":
                sections_needed = '1'
            else:
                sections_needed = calculate_sections_needed(data)

            # We ask for the section lock
            env = dict(os.environ)
            env['JAVA_OPTS'] = 'foo'
            # 192.168.1.117:2181,192.168.1.107:2181,192.168.1.118:2181,
            print "Sections needed: " + str(sections_needed)
            self.proc = Popen(['java', '-jar', lock_jar_path, 'localhost:2181',
                               data.intersection, data.initial_direction, str(sections_needed)],
                              env=env, stdout=subprocess.PIPE, stdin=subprocess.PIPE)

            # Signal the truck to stop
            # self.pub.publish("stop")

            while True:
                line = self.proc.stdout.readline()

                if 'zk_connection_successful' in line:
                    self.pub.publish("zk_connection_successful")

                if 'zk_connection_failed' in line:
                    self.pub.publish("zk_connection_failed")

                if 'lock_accepted' in line:
                    # We've been granted the lock!
                    # Tell the truck to continue driving
                    # print "I said you can drive"
                    self.pub.publish("continue")
                    break


def calculate_sections_needed(data):
    action = getattr(data.action, data.intersection)
    print "Action: " + action
    print "Initial direction : " + data.initial_direction

    if data.initial_direction == "right" and action == "turn_left":
        return '4'  # Book all 4 sections
    elif data.initial_direction == "right" and action == "turn_right":
        return '4'  # Book all 4 sections
    elif data.initial_direction == "right" and action == "forward":
        return '2'

    if data.initial_direction == "left" and action == "turn_left":
        return '4'  # Book all 4 sections
    elif data.initial_direction == "left" and action == "turn_right":
        return '4'
    elif data.initial_direction == "left" and action == "forward":
        return '2'

    if data.initial_direction == "up" and action == "turn_left":
        return '4'  # Book all 4 sections
    elif data.initial_direction == "up" and action == "turn_right":
        return '4'

    if data.initial_direction == "down" and action == "turn_left":
        return '4'  # Book all 4 sections
    elif data.initial_direction == "down" and action == "turn_right":
        return '4'
    else:
        return '0'


if __name__ == '__main__':
    s = SectionLock()
    rospy.spin()
