import re

world_path = "/home/jakub/.simulation-gazebo/worlds/default.sdf"

with open(world_path, "r") as f:
    content = f.read()

content = content.rstrip()
assert content.endswith("</sdf>"), "Nieoczekiwany koniec pliku"
content = content[:-len("</sdf>")].rstrip()
assert content.endswith("</world>"), "Brak </world>"
content = content[:-len("</world>")]

zones = [
    ("landing_zone_1", 5, 0, "0.9 0.8 0.1"),   # żółta
    ("landing_zone_2", 10, 5, "0.2 0.4 0.9"),  # niebieska
    ("landing_zone_3", 15, 0, "0.6 0.2 0.8"),  # fioletowa
    ("landing_zone_4", 10, -5, "0.9 0.3 0.3"), # czerwona
]

zones_xml = ""
for name, x, y, color in zones:
    zones_xml += f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0.01 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>3 3 0.02</size></box></geometry>
          <material><ambient>{color} 0.8</ambient><diffuse>{color} 0.8</diffuse></material>
        </visual>
      </link>
    </model>
"""

new_content = content + zones_xml + "  </world>\n</sdf>\n"

with open(world_path, "w") as f:
    f.write(new_content)

print(f"Dodano {len(zones)} stref lądowania.")

