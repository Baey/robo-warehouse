# ROS 2 + Gazebo Harmonic docker

Repozytorium zawiera kod do wykorzystania przez studentów w celu realizacji zajęć z Systemów i Algorytmów Percepcji w Pojazdach Autonomicznych (SiAPwPA) w semestrze zimowym 2024/2025.

## Wymagania

- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [Nvidia Docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#container-device-interface-cdi-support)
- [VS Code devcontainer plugin](https://code.visualstudio.com/docs/devcontainers/containers#_quick-start-open-an-existing-folder-in-a-container)

> [!IMPORTANT]  
> System operacyjnym Ubuntu jest wymagany ze względu na obsługę GUI (wymaga innego podejścia przy wykorzystaniu Windowsa).

## Start

Otwórz VS Code w katalogu z projektem.
Przejdź do lewego dolnego rogu i kliknij niebieską ikonę z dwiema strzałkami skierowanymi do siebie. Z rozwijanego menu wybierz **"Open Folder in Container... ”** i poczekaj, aż docker się zbuduje. Może to potrwać do 10 minut przy wolniejszym połączeniu internetowym.

> [!TIP]
> Dla osób korzystających z Windowsa oraz WSL 2 przygotowano `Dockerfile.windows` oraz `compose.windows.yaml`. 

Po zalogowaniu się do dockera będzie on działał w sposób podobny do uruchamiania ROS na komputerze hosta. Wszystkie aplikacje GUI będą korzystać z domyślnego menedżera okien hosta, będziesz mieć również dostęp do wszystkich urządzeń na hoście, a także akceleracji GPU.
Docker ma preinstalowany [ROS 2 Humble](https://docs.ros.org/en/humble/Tutorials.html) i większość potrzebnych zależności oraz symulator [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/).

## Pierwsze uruchomienie

Dla osób, które nie miały doczynienia ze środowiskiem ROS 2 + Gazebo, zachęcam do przerobienia tutorialu: [Gazebo Tutorial](https://gazebosim.org/docs/harmonic/tutorials/). Pozwoli to zaznajomić się z tym środowiskiem i tworzyć w przyszłości zaawansowane symulacje.

Następnie pomocne będzie odpowiednia kontrola robotami w środowisku symulacyjnym, na dobry start proszę zaznajomić się z repozytorium: [Gazebo ROS 2 Control](https://github.com/ros-controls/gz_ros2_control/).

Na sam koniec pewnym podsumowaniem, a także praktycznym podejściem do tematu jest dostarczony od [Husariona](https://husarion.com/tutorials/ros2-tutorials/1-ros2-introduction/) tutorial dla ich kilku robotów.

> [!IMPORTANT] 
Należy pamiętać, aby po zbudowaniu wywołać komendę lub pracować w nowym terminalu:
>
> ```bash
> source ~/.bashrc
> ```
>
> W tym pliku dodane są już dwie ważne ścieżki:
>
> ```bash
> /opt/ros/$ROS_DISTRO/setup.bash
> /home/developer/ros2_ws/install/setup.bash
> ```

## Przykład
1. Zbuduj obszar roboczy wraz z simple_example package.  
2. Uruchom launcha `example.launch.py`, pokazujący, w jaki sposób należy połączyć Gazebo z ROS 2, aby możliwa była wzajemna komunikacja.


## Dodatkowe materiały
* [Getting Started](getting_started.md)
* [ROS 2 Command Cheat Sheet](cheatsheet.md)
* [ROS 2 Example packages in Python](example.md)
* [Bridge communication between ROS and Gazebo](ros_gz_bridge.md)

# READMI do Forka

## Spis treści

1. [Opis](#opis)
2. [Wymagania](#wymagania)
3. [Instalacja](#instalacja)
4. [Użycie](#użycie)

## Opis

TODO

## Wymagania

Wymagane oprogramowanie do uruchomienia programu:

- Python 3.10.12
- os-humble-slam-toolbox
- nav2_bringup


## Uruchomienie

> ```bash
> colcon build --symlink-install
> ```

> ```bash
> ros2 run robo_warehouse robo_warehouse_entry
> ```

Następnie w odobnym terminalu

> ```bash
> source install/setup.bash
> ```

> ```bash
> ros2 run tf2_tools view_frames
> ```

> ```bash
> ros2 run tf2_tools view_frames
> ```

> ```bash
> ros2 launch slam_toolbox online_async_launch.py
> ```

> ```bash
> ros2 launch nav2_bringup navigation_launch.py
> ```

## Użycie

sudo apt-get update
sudo apt install ros-humble-slam-toolbox ros-humble-rtabmap-ros
sudo apt install ros-humble-nav2-bringup


colcon build --symlink-install
source install/setup.bash
ros2 launch robo_warehouse minimal_launch_file.launch.py
