# READMI
## Spis treści

1. [Opis](#opis)
2. [Wymagania](#wymagania)
3. [Instalacja](#instalacja)
4. [Użycie](#użycie)

## Opis

Projekt jest symulacją działania dwóch robotów w magazynie, zaprojektowaną przy użyciu ROS2 + Gazebo. Celem robotów jest przewożenie paczek z jednego miejsca do drugiego w dynamicznie zmieniającym się środowisku. Roboty w tej symulacji zostały wyposażone w systemy SLAM (Simultaneous Localization and Mapping), które umożliwiają im rozpoznawanie otoczenia i mapowanie przestrzeni w czasie rzeczywistym. Dzięki temu roboty są w stanie skutecznie orientować się w przestrzeni magazynowej, planować najkrótszą ścieżkę do paczki oraz reagować na zmieniające się warunki otoczenia. Roboty komunikują się ze sobą, aby określić, który z nich jest bliżej paczki, a robot bliższy paczce podejmuje się jej transportu. Roboty obierają najkrótszą ścieżkę do celu, uwzględniając zmieniające się warunki otoczenia, takie jak przeszkody w przestrzeni magazynowej. System wykorzystuje algorytmy do planowania ścieżki, które pozwalają robotom na skuteczne i bezpieczne poruszanie się w otoczeniu pełnym dynamicznych przeszkód.

## Wymagania

Wymagane oprogramowanie do uruchomienia programu:

- Python 3.10.12

Przed uruchomieniem należy doinstalować odpodwiednie paczki:

> ```bash
> sudo apt-get update
> ```

> ```bash
> sudo apt install ros-humble-slam-toolbox ros-humble-rtabmap-ros
> ```

> ```bash
> ros2 sudo apt install ros-humble-nav2-bringup
> ```

## Uruchomienie

> ```bash
> colcon build --symlink-install
> ```

> ```bash
> source install/setup.bash
> ```

> ```bash
> ros2 launch robo_warehouse minimal_launch_file.launch.py
> ```

## Użycie

Aby zdadać robotowi docelową destynację należy po uruchomieniu okna klinknąc na przycisk "2D Goal Pose" i kliknąc na odpowiednie miejsce na planszy.
Następnie za pomoca strzałki można wybrać kierunek robota.

## Poruszanie robotem za pomocą klawiatury
w nowym terminalu:
> ```bash
> ros2 run ros_gz_bridge parameter_bridge /model/tugbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
> ```

w kolejnym nowym terminalu:
> ```bash
> source install/setup.bash
> ```
w konsoli wpisać t, a następnie można poruszać robotem klawiszami w,a,s,d



