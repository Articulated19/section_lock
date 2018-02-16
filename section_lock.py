#!/usr/bin/python
import os
import time
import subprocess
from subprocess import Popen
import sys

waitTime = float(sys.argv[1])

env = dict(os.environ)
env['JAVA_OPTS'] = 'foo'
proc = Popen(['java', '-jar', 'lock_zk_node.jar', 'lock'], env=env,
             stdout=subprocess.PIPE, stdin=subprocess.PIPE)
while True:
    line = proc.stdout.readline()
    print('lock_accepted' in line)
    if 'lock_accepted' in line:
        break


print('out of the loop')
time.sleep(waitTime)
proc.communicate(input='\n')