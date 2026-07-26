import re

world_path = "/home/jakub/.simulation-gazebo/worlds/default.sdf"

with open(world_path, "r") as f:
    content = f.read()

# Znajdź każdy model (building_* lub tree_*) razem z jego <pose>, i przeskaluj
# drugą współrzędną (Y = "bok korytarza") tak, żeby był dalej od centrum (Y=0)
def widen(match):
    model_block = match.group(0)
    pose_match = re.search(r"<pose>([^<]+)</pose>", model_block)
    nums = pose_match.group(1).split()
    y = float(nums[1])
    # Przesuń dalej od zera, zachowując znak, z minimalnym marginesem 3.5m
    if y >= 0:
        new_y = max(y * 1.8, 3.5)
    else:
        new_y = min(y * 1.8, -3.5)
    nums[1] = str(round(new_y, 2))
    new_pose = f"<pose>{' '.join(nums)}</pose>"
    return model_block.replace(pose_match.group(0), new_pose)

pattern = r'<model name="(?:building|tree)_[^"]*">.*?</model>'
new_content = re.sub(pattern, widen, content, flags=re.DOTALL)

with open(world_path, "w") as f:
    f.write(new_content)

print("Poszerzono korytarz - budynki/drzewa odsunięte od trasy lotu.")
