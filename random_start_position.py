#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

def set_random_position():
    # Parametry obszaru, w którym robot ma być umieszczony
    min_x = -5.0  # minimalna wartość dla osi X
    max_x = 5.0   # maksymalna wartość dla osi X
    min_y = -5.0  # minimalna wartość dla osi Y
    max_y = 5.0   # maksymalna wartość dla osi Y
    z_position = 0.1  # Wysokość robota nad ziemią (na poziomie gruntu w Gazebo)

    # Losowanie pozycji robota w zakresie
    random_x = random.uniform(min_x, max_x)
    random_y = random.uniform(min_y, max_y)

    # Tworzymy obiekt pozycji robota
    pose = Pose()
    pose.position.x = random_x
    pose.position.y = random_y
    pose.position.z = z_position

    # Konfiguracja stanu modelu robota
    model_state = ModelState()
    model_state.model_name = "robot_name"  # Zmień na nazwę swojego robota w Gazebo
    model_state.pose = pose
    model_state.twist.linear.x = 0
    model_state.twist.linear.y = 0
    model_state.twist.linear.z = 0
    model_state.twist.angular.x = 0
    model_state.twist.angular.y = 0
    model_state.twist.angular.z = 0

    # Czekamy na dostęp do usługi SetModelState
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        # Wywołanie usługi zmieniającej pozycję robota
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_model_state(model_state)
        rospy.loginfo("Pozycja robota została ustawiona na: X: %f, Y: %f", random_x, random_y)
    except rospy.ServiceException as e:
        rospy.logerr("Błąd przy wywoływaniu usługi: %s", e)

if __name__ == '__main__':
    rospy.init_node('random_position_setter')
    set_random_position()
