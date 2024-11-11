import rospy
import random
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel
from std_srvs.srv import Empty

def spawn_random_robot():
    rospy.init_node('spawn_random_robot_node')

    # Zainicjalizowanie serwisu do spawnowania modelu
    spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    
    # Wczytanie pliku SDF/URDF robota (np. robot.sdf) lub wstępnie zdefiniowanego modelu
    model_name = "robot_name"
    model_path = "/path/to/robot.sdf"  # Możesz użyć URDF lub SDF
    
    with open(model_path, 'r') as file:
        robot_model = file.read()

    # Losowanie współrzędnych dla robota w Gazebo
    x = random.uniform(-10, 10)  # Losowa pozycja X
    y = random.uniform(-10, 10)  # Losowa pozycja Y
    z = 0.5  # Stała wysokość nad ziemią
    
    # Utworzenie losowej pozycji
    random_pose = Pose()
    random_pose.position.x = x
    random_pose.position.y = y
    random_pose.position.z = z
    random_pose.orientation.x = 0
    random_pose.orientation.y = 0
    random_pose.orientation.z = 0
    random_pose.orientation.w = 1  # Domyślna orientacja

    # Spawnowanie robota w Gazebo
    spawn_service(model_name, robot_model, '', random_pose, '')

if __name__ == "__main__":
    try:
        spawn_random_robot()
    except rospy.ROSInterruptException:
        pass
