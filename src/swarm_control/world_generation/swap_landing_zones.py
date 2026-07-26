import re

world_path = "/home/jakub/.simulation-gazebo/worlds/default.sdf"

with open(world_path, "r") as f:
    content = f.read()

def swap_xy(match):
    model_block = match.group(0)
    pose_match = re.search(r"<pose>([^<]+)</pose>", model_block)
    nums = pose_match.group(1).split()
    nums[0], nums[1] = nums[1], nums[0]
    new_pose = f"<pose>{' '.join(nums)}</pose>"
    return model_block.replace(pose_match.group(0), new_pose, 1)

# Zamień TYLKO strefy landing_zone_1..4 (nowe), nie ruszamy reszty świata
pattern = r'<model name="landing_zone_[1-4]">.*?</model>'
new_content = re.sub(pattern, swap_xy, content, flags=re.DOTALL)

with open(world_path, "w") as f:
    f.write(new_content)

print("Zamieniono osie X/Y tylko dla stref lądowania 1-4.")
