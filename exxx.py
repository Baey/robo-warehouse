#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move_robot():
    # Inicjalizacja węzła ROS
    rospy.init_node('move_robot_node', anonymous=True)
    
    # Tworzymy publishera, który będzie publikować wiadomości do tematu 'cmd_vel'
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Ustawienie częstotliwości publikacji (10 Hz)
    rate = rospy.Rate(10)
    
    # Tworzymy obiekt wiadomości typu Twist, aby ustawić prędkości robota
    move_cmd = Twist()

    # Prędkości: liniowa i kątowa
    move_cmd.linear.x = 0.5  # prędkość liniowa (przesuwanie do przodu)
    move_cmd.angular.z = 0.1  # prędkość kątowa (obrót w prawo)
    
    while not rospy.is_shutdown():
        # Publikujemy wiadomość do tematu 'cmd_vel'
        pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
