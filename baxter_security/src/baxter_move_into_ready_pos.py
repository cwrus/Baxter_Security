#!/usr/bin/env python

import sys
import rospy
import baxter_interface
import baxter_external_devices
import movement
from kinect_test.msg import Coords

class moveIntoready"
    def __init__(self):
      # Create this ros node
      rospy.init_node("baxter_move_into_ready", anonymous=True)  

      # Subscribe to the kinect's coord publisher
      rospy.Subscriber("/baxter_security/kinect_lighter_coords", Coords, self.coordsCallback)

      rospy.spin()

  def coordsCallback(self, data)
    # Predetermined offsets from the kinect frame
    xoff = 0.899
    yoff = 0
    zoff = 0.7124
 
    xready = xoff + data.x 
    yready = yoff + data.y
    zready = data.z - zoff

    # Move to the desired position
    movement.moveto(xready, yready, zready)

if __name__ == '__main__':
    pf = PosFind()
