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
W konsoli należy wpisać t, a następnie można poruszać robotem klawiszami w,a,s,d

## Implementacja - uwagi

#1. Realizacja SLAM-a

W projekcie wykorzystano narzędzie slam_toolbox, które oferuje szeroki zakres funkcjonalności do mapowania i lokalizacji. Mimo że konfiguracja teoretycznie była poprawna (poprawnie skonfigurowany topic z danymi skanów laserowych), narzędzie nie otrzymywało właściwych danych i publikowany topic map pozostawał pusty.

Analiza wykazała, że przyczyną problemów były nieprawidłowe transformacje (tf). Aby je rozwiązać:

- Dodano węzeł robot_state_publisher, odpowiedzialny za publikowanie drzew transformacji na podstawie plików URDF.
- Stworzono i skonfigurowano pliki tf_broadcaster.cpp, tf_listener.cpp oraz tracker.hpp, tak aby prawidłowo przekazywały informacje o położeniu i orientacji robota w przestrzeni.
  
Dzięki temu slam_toolbox zaczął poprawnie przetwarzać dane i publikować mapę w czasie rzeczywistym.

#2. Obsługa dwóch robotów

W ramach rozbudowy projektu próbowano dodać drugi egzemplarz robota. Głównym wyzwaniem okazały się zduplikowane nazwy topiców i framów (takich jak odom czy base_link), co prowadziło do konfliktów w publikowanych danych. Aby temu zapobiec, wprowadzono namespace-y, które w teorii powinny umożliwić równoległą pracę obu robotów.

Niestety, pomimo odpowiedniego przestrzegania nazewnictwa, slam_toolbox nie rozpoznawał poprawnych danych z drugiego robota:

- Dane były publikowane na jednym topicu, podczas gdy SLAM subskrybował inny.
- Pojawiał się błąd no odom frame, wskazujący na niepoprawne transformacje lub brak odpowiednich węzłów TF w drugim namespace.
  
Dodatkową trudnością był brak kompatybilności biblioteki gazebo_ros_control z używaną wersją Gazebo w momencie prób konfigurowania URDF pod kątem namespace-ów. To ograniczyło możliwość łatwego wdrożenia dwóch robotów w jednym środowisku symulacyjnym.

#3. Planowane, lecz niezrealizowane kroki
   
#3.1. Mapy łączone z wykorzystaniem map_merge

Zamierzano połączyć mapy generowane przez dwa niezależne egzemplarze robotów, korzystając z pakietu map_merge. Umożliwiłoby to uzyskanie jednej spójnej mapy środowiska, stworzonej na podstawie skanów z obu robotów.

#3.2. Scenariusze testowe z kooperacją robotów

Planowano stworzyć przykładowe zadania, w których oba roboty miałyby współpracować przy rozmieszczaniu paczek w magazynie. Projekt miał obejmować jedynie logikę i wyznaczanie tras do poszczególnych punktów, bez symulowania samych paczek.

