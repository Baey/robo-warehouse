import xml.etree.ElementTree as ET
import random
import re
import os
def set_random_pos():
    # file_name = "./worlds/thugbot_depot.sdf"
    file_name = "/home/developer/ros2_ws/src/robo_warehouse/worlds/tugbot_depot.sdf"
    # tree = ET.parse(file_name)
    # root = tree.getroot()
    with open(file_name, 'r', encoding='utf-8') as file:
        content = file.read()  
    # Regex do znalezienia fraz "XX" i najbliższego słowa "pose"
    matches = []
    code = "<name>tugbot</name>"
    target = "<pose>"
    pattern = re.escape(code) + r"(.*?)\b" + re.escape(target) + r"\b"
    for match in re.finditer(pattern, content, re.DOTALL):
        position = match.start(1)  # Pozycja słowa "pose"
        matches.append((match.group(1), position))

    print(content)
    print(matches)
    # tree.write(file_name, encoding="utf-8", xml_declaration=True)
    
    # position_x = random.uniform(-5.0, 5.0)  # Random X between -5 and 5 meters
    # position_y = random.uniform(-5.0, 5.0)  # Random Y between -5 and 5 meters
set_random_pos()