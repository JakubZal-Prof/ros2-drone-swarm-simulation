world_path = "/home/jakub/.simulation-gazebo/worlds/default.sdf"

with open(world_path, "r") as f:
    content = f.read()

# Usuń zamykające tagi z końca (dodamy je z powrotem po nowej zawartości)
content = content.rstrip()
assert content.endswith("</sdf>"), "Plik nie kończy się jak oczekiwano - sprawdź ręcznie!"
content = content[:-len("</sdf>")].rstrip()
assert content.endswith("</world>"), "Brak tagu </world> przed </sdf> - sprawdź ręcznie!"
content = content[:-len("</world>")]

# Wygeneruj drzewa: pień (cylinder) + korona (sfera), rozrzucone po obu stronach trasy (Y = 0..9)
trees_xml = ""
tree_positions = [
    (-1.5, 1.0), (1.5, 1.5), (-1.7, 4.0), (1.6, 4.5),
    (-1.4, 7.0), (1.8, 7.5), (-1.6, 8.5), (1.5, 2.5),
]

for i, (tx, ty) in enumerate(tree_positions):
    trees_xml += f"""
    <model name="tree_{i}">
      <static>true</static>
      <pose>{tx} {ty} 1.0 0 0 0</pose>
      <link name="link">
        <collision name="trunk_collision">
          <geometry><cylinder><radius>0.15</radius><length>2</length></cylinder></geometry>
        </collision>
        <visual name="trunk_visual">
          <geometry><cylinder><radius>0.15</radius><length>2</length></cylinder></geometry>
          <material><ambient>0.4 0.25 0.1 1</ambient><diffuse>0.4 0.25 0.1 1</diffuse></material>
        </visual>
        <visual name="crown_visual">
          <pose>0 0 1.3 0 0 0</pose>
          <geometry><sphere><radius>0.8</radius></sphere></geometry>
          <material><ambient>0.15 0.5 0.15 1</ambient><diffuse>0.15 0.5 0.15 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

# Strefa lądowania na końcu trasy (Y=9, na wysokości ziemi)
landing_xml = """
    <model name="landing_zone">
      <static>true</static>
      <pose>0 9 0.01 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>3 3 0.02</size></box></geometry>
          <material><ambient>0.1 0.8 0.2 0.8</ambient><diffuse>0.1 0.8 0.2 0.8</diffuse></material>
        </visual>
      </link>
    </model>
"""

new_content = content + trees_xml + landing_xml + "  </world>\n</sdf>\n"

with open(world_path, "w") as f:
    f.write(new_content)

print("Gotowe - dodano", len(tree_positions), "drzew i strefę lądowania.")
