#!/usr/bin/env python3

import rospy
from mav_msgs.msg import Actuators
import os

class Parameters:
    def __init__(self, phase, skew, amp, ampback, freq):
        self.Phase = phase
        self.Skew = skew
        self.Amp = amp
        self.Ampback = ampback
        self.Freq = freq
        
class TeleopControllerNode:
    def __init__(self):
        rospy.init_node('teleop_controller_node', anonymous=True)

        self.hz = 1000.0
        self.parameters = Parameters(0, 0, 0, 0, 0)
        
        self.commands = rospy.Publisher('commands', Actuators, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.hz), self.control_loop)        
        
    def getchar(self):          
        # Returns a single character from standard input
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
        if ord(ch) == 3: 
            quit() # handle ctrl+C        
        return ch

    def control_loop(self, event):
        msg = Actuators()
        key = self.getchar()
        if key == 'w':
            self.parameters = Parameters(0, 90, 0, 0, 12)
            rospy.loginfo("Walking forward")

        elif key == 's':
            self.parameters = Parameters(0, 90, 0, 0, 2)
            rospy.loginfo("Walking backward")

        elif key == 'a':
            self.parameters = Parameters(0, -45, 0, 0, 7)
            rospy.loginfo("Turning left")

        elif key == 'd':
            self.parameters = Parameters(0, 45, 0, 0, 7)
            rospy.loginfo("Turning right")

        elif key == 'e':
            self.parameters = Parameters(45, 0, 40, 20, 7)
            rospy.loginfo("Overcoming obstacles")

        elif key == 'q':
            self.parameters = Parameters(45, 0, 40, 20, 1)
            rospy.loginfo("Slowly overcoming obstacles")

        elif key == 'c':
            self.parameters = Parameters(0, 0, 0, 0, 0)
            rospy.loginfo("Stop")
        
        msg.angular_velocities = [0] * 5

        msg.angular_velocities[0] = self.parameters.Phase  # Phase between front and back legs (in degree)
        msg.angular_velocities[1] = self.parameters.Skew  # Phase between front left + back right legs and front right and left back legs
        msg.angular_velocities[2] = self.parameters.Amp  # Amplitude change of all legs
        msg.angular_velocities[3] = self.parameters.Ampback  # Amplitude change of back legs (added to angular_velocities[2])
        msg.angular_velocities[4] = self.parameters.Freq  # Frequency of legs

        self.commands.publish(msg)

if __name__ == '__main__':
    try:
        TeleopControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    


