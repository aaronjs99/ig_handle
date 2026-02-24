#!/usr/bin/env python3

import rospy
import os
import sys
import subprocess

def main():
    rospy.init_node('teensy_launcher', anonymous=True)
    
    port = rospy.get_param('~port', '/dev/serial/by-id/usb-Teensyduino_USB_Serial_13709860-if00')
    baud = rospy.get_param('~baud', 115200)
    
    if os.path.exists(port):
        rospy.loginfo("Teensy found at %s. Starting rosserial_python node...", port)
        # We use rosrun to launch the actual serial_node.py
        # This keeps the environment clean.
        cmd = ["rosrun", "rosserial_python", "serial_node.py", "_port:=" + port, "_baud:=" + str(baud)]
        
        try:
            # We use call here which blocks until the child exits.
            # This makes our launcher node stay alive as long as the serial node is alive.
            subprocess.call(cmd)
        except KeyboardInterrupt:
            pass
    else:
        rospy.logwarn("Teensy device NOT found at %s. Skipping rosserial node (Optional).", port)
        # Exit cleanly without error.

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
