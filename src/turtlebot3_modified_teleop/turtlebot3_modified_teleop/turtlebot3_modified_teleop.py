import os
import sys
import select
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name != 'nt':
    import termios
    import tty

# Constants for maximum velocities
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

# Set constant velocities
CONSTANT_LIN_VEL = 0.15
CONSTANT_ANG_VEL = 0.4

# Determine TurtleBot3 model
TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
if TURTLEBOT3_MODEL == 'burger':
    MAX_LIN_VEL = BURGER_MAX_LIN_VEL
    MAX_ANG_VEL = BURGER_MAX_ANG_VEL
else:
    MAX_LIN_VEL = WAFFLE_MAX_LIN_VEL
    MAX_ANG_VEL = WAFFLE_MAX_ANG_VEL

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w : move forward
a : turn left
d : turn right
x : move backward

Any other key : stop
CTRL-C to quit
"""

def get_key(settings):
    if os.name == 'nt':
        import msvcrt
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin) if os.name != 'nt' else None

    rclpy.init()
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=10))

    print(msg)

    try:
        while True:
            key = get_key(settings)
            twist = Twist()

            if key == 'w':
                twist.linear.x = min(CONSTANT_LIN_VEL, MAX_LIN_VEL)
            elif key == 'x':
                twist.linear.x = -min(CONSTANT_LIN_VEL, MAX_LIN_VEL)
            elif key == 'a':
                twist.angular.z = min(CONSTANT_ANG_VEL, MAX_ANG_VEL)
            elif key == 'd':
                twist.angular.z = -min(CONSTANT_ANG_VEL, MAX_ANG_VEL)
            else:
                # Stop the robot when no valid key is pressed
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            pub.publish(twist)

            if key == '\x03':  # CTRL-C
                break

    except Exception as e:
        print(e)

    finally:
        # Stop the robot on exit
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
