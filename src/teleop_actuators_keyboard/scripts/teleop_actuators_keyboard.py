#!/usr/bin/env python3

import rospy
from mav_msgs.msg import Actuators
from enum import Enum, auto
import os

class Parameters:
    def __init__(self, phase, skew, amp, ampback, freq):
        self.Phase = phase
        self.Skew = skew
        self.Amp = amp
        self.Ampback = ampback
        self.Freq = freq

class Motion(Enum):
    FOREWARD = auto()
    BACKWARD = auto()
    LEFT = auto()
    RIGHT = auto()
    JUMP = auto()
    SLOW_JUMP = auto()
    STOP = auto()

def getParameters(motion):
    if motion == Motion.FOREWARD:
        return Parameters(0, 90, 0, 0, 12);    
    elif motion == Motion.BACKWARD:
        return Parameters(0, 90, 0, 0, 3);    
    elif motion == Motion.LEFT:
        return Parameters(0, -45, 0, 0, 7); 
    elif motion == Motion.RIGHT:
        return Parameters(0, 45, 0, 0, 7); 
    elif motion == Motion.JUMP:
        return Parameters(45, 0, 40, 20, 7); 
    elif motion == Motion.SLOW_JUMP:
        return Parameters(45, 0, 40, 20, 1); 
    elif motion == Motion.STOP:
        return Parameters(0,0,0,0,0); 


class TeleopControllerNode:
    INSTRUCTION = """
Reading from the keyboard and publishing to Actuators!
---------------------------
Moving with aswd:
s/w - move backward/forward
a/d - rotate left/right

Special keys:
e - Obstacles passing mode
q - Slow obstacles passing mode
x - Stop

CTRL-C to quit
    """
    def __init__(self):
        rospy.init_node('teleop_controller_node', anonymous=True)

        self.hz = 1000.0
        self.parameters = Parameters(0, 0, 0, 0, 0)
        
        self.commands = rospy.Publisher('commands', Actuators, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.hz), self.control_loop)  

        # Parameters
        parameters = Parameters(0,0,0,0,0)
        # Motions
        motion = Motion.STOP
        
        self.status = 0

        print(TeleopControllerNode.INSTRUCTION)
        
    def getKey(self):          
        # Returns a single character from standard input
        key = ''
        if os.name == 'nt': # how it works on windows
            import msvcrt
            key = msvcrt.getch()
        else:
            import tty, termios, sys
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)                
        if ord(key) == 3: 
            quit() # handle ctrl+C        
        return key


    def control_loop(self, _):
        msg = Actuators()
        key = self.getKey()
        allowed_keys = {'w', 's', 'a', 'd', 'e', 'q', 'x'}

        if key == 'w':
            self.motion = Motion.FOREWARD
            print("Walking forward")

        elif key == 's':
            self.motion = Motion.BACKWARD
            print("Walking backward")

        elif key == 'a':
            self.motion = Motion.LEFT
            print("Turning left")

        elif key == 'd':
            self.motion = Motion.RIGHT
            print("Turning right")

        elif key == 'e':
            self.motion = Motion.JUMP
            print("Passing obstacles")

        elif key == 'q':
            self.motion = Motion.SLOW_JUMP
            print("Slowly passing obstacles")

        elif key == 'x':
            self.motion = Motion.STOP
            print("Stop")

        if key in allowed_keys:
            self.status = (self.status + 1) % 15
        
        if (self.status == 14):
            print(TeleopControllerNode.INSTRUCTION)
        
        self.parameters = getParameters(self.motion)

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
    


