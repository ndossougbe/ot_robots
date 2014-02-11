#!/usr/bin/env python

import rospy

from publishers import LedController, DirectionController

if __name__ == '__main__':
    try:
        rospy.init_node('remote_controller')
        
        led = LedController().start()
        direction = DirectionController(False).start()

        rospy.spin()  # waits until rospy.is_shutdown() is true (Ctrl+C)
 
        print '\nTerminating the publishers...'
        for pub in (led, direction):
            pub.terminate()

        for pub in (led, direction):
            pub.join()

        print 'All ok.'

    except rospy.ROSInterruptException:
        pass