#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def getchar():
	# Returns a single character from standard input
	import os
	ch = ''
	if os.name == 'nt': # how it works on windows
		import msvcrt
		ch = msvcrt.getch()
	else:
		import tty, termios, sys
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
	if ord(ch) == 3: quit() # handle ctrl+C
	return ch

if __name__ == "__main__":
    rospy.init_node("teleop_publisher")
    teleop_pub = rospy.Publisher("teleop", String, queue_size=10)

    while not rospy.is_shutdown():
        key = key = getchar()
        teleop_pub.publish(key)