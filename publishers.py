import rospy

from threading import Thread

from kobuki_msgs.msg import Led
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from controller import DirectionalKeyListener


class ThreadedPublisher(object):

    def __init__(self, topic, type, frequency=1):
        self._pub = rospy.Publisher(topic, type)
        self._thread = Thread(target=self.loop)
        self._frequency = frequency
        self._running = False

    def loop(self):
        while self._running and not rospy.is_shutdown():
            self.update()
            rospy.sleep(self._frequency)

    def update(self):
        pass

    def terminate(self):
        print 'terminating '
        self._running = False

    def start(self):
        ''' Returns self for chaining '''
        self._running = True
        self._thread.start()
        return self

    def publish(self, *args):
        # rospy.loginfo(*args)
        self._pub.publish(*args)

    def join(self):
        self._thread.join()


class LedController(ThreadedPublisher):

    def __init__(self):
        super(LedController, self).__init__('/mobile_base/commands/led1', Led)
        self.color = 0

    def update(self):
        self.color = (self.color + 1)%4
        self.publish(self.color)


class DirectionController(ThreadedPublisher):

    LINEAR_SPD = 0.15
    ANGULAR_SPD = 0.5

    KEY_BINDINGS = {
        ord('a'): Twist(Vector3(LINEAR_SPD,0,0), 
                        Vector3(0,0,ANGULAR_SPD)),
        ord('z'): Twist(Vector3(LINEAR_SPD,0,0), 
                        Vector3(0,0,0)),
        ord('e'): Twist(Vector3(LINEAR_SPD,0,0), 
                        Vector3(0,0,-ANGULAR_SPD)),
        ord('q'): Twist(Vector3(0,0,0), 
                        Vector3(0,0,ANGULAR_SPD)),
        ord('s'): Twist(Vector3(-LINEAR_SPD,0,0),
                        Vector3(0,0,0)),
        ord('d'): Twist(Vector3(0,0,0), 
                        Vector3(0,0,-ANGULAR_SPD)),
        ord('w'): Twist(Vector3(-LINEAR_SPD,0,0),
                        Vector3(0,0,-ANGULAR_SPD)),
        ord('x'): Twist(Vector3(-LINEAR_SPD,0,0),
                        Vector3(0,0,ANGULAR_SPD)),
        ord(' '): Twist(Vector3(0,0,0), 
                        Vector3(0,0,0))
    }

    def __init__(self, target_sim=False):
        if target_sim: 
            topic = '/turtle1/cmd_vel' 
        else:
            topic = '/cmd_vel_mux/input/teleop'
        super(DirectionController, self).__init__(topic, Twist, 1/10.0)
        self.key_listener = DirectionalKeyListener(callback=self.on_key_pressed, 
                                                   direction_mappings=DirectionController.KEY_BINDINGS)

    def loop(self):
        ''' Replace this thread's loop with the listener's activation '''
        self.key_listener.activate()

    def on_key_pressed(self, direction, key, listener):
        ''' Listener style, to be used with DirectionalKeyListener '''
        self.publish(direction)
