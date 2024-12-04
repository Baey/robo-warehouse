import xml.etree.ElementTree as ET
import random
import re
import os
def set_random_pos():
    file_name = "/home/developer/ros2_ws/src/robo_warehouse/worlds/tugbot_depot.sdf"

    with open(file_name, 'r', encoding='utf-8') as file:
        content = file.read()  
    pattern = r"<name>tugbot</name>"

    # Znajdź wszystkie wystąpienia i ich pozycje
    matches_thugbot = [(match.start(), match.group()) for match in re.finditer(pattern, content)]
    matches_pose = [(match.start(), match.group(1)) for match in re.finditer(r"<pose>(.*?)</pose>", content[matches_thugbot[0][0]:])]


    pos_pos = matches_pose[0][0] + matches_thugbot[0][0]
    substring = content[pos_pos:]

 
    # Wyrażenie regularne dopasowujące liczby float w tagu <pose>
    pattern = r"<pose>(-?[\d\.]+) (-?[\d\.]+)"
    # Funkcja zamieniająca liczby
    def replace_floats(match):
        # Generowanie losowych liczb float z przedziału 0-5
        random_float1 = f"{random.uniform(-5, 5):.6f}"
        random_float2 = f"{random.uniform(-5, 5):.6f}"
        # Zamiana dopasowanych liczb na losowe
        return f"<pose>{random_float1} {random_float2}"

    # Zamiana w tekście
    modified_text = re.sub(pattern, replace_floats, substring, count=1)
    with open(file_name, 'w', encoding='utf-8') as file:
        file.write(content[:pos_pos] + modified_text)
