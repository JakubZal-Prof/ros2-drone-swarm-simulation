import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from px4_msgs.msg import VehicleLocalPosition

BAG_PATH = "formamation_full_mission1"

def read_positions(bag_path, topic_name):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    xs, ys, ts = [], [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, VehicleLocalPosition)
            xs.append(msg.x)
            ys.append(msg.y)
            ts.append(t / 1e9)
    return np.array(xs), np.array(ys), np.array(ts)

leader_x, leader_y, leader_t = read_positions(BAG_PATH, "/fmu/out/vehicle_local_position")
follower_x, follower_y, follower_t = read_positions(BAG_PATH, "/px4_1/fmu/out/vehicle_local_position")

t0 = min(leader_t[0], follower_t[0])
leader_t -= t0
follower_t -= t0

follower_x_interp = np.interp(leader_t, follower_t, follower_x)
follower_y_interp = np.interp(leader_t, follower_t, follower_y)
distance = np.sqrt((leader_x - follower_x_interp)**2 + (leader_y - follower_y_interp)**2)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

# --- Panel 1: mapa misji z trajektoriami ---

# Budynki (x, y, rozmiar_xy, kolor) - przeliczone z SDF (y,x -> x,y)
buildings = [
    (3.6, -2.5, 2, '#6b6b70'),
    (10.8, -2.5, 2, '#666a70'),
    (5.4, 2.5, 2, '#736f68'),
    (12.6, 2.5, 2, '#726f6b'),
]
for (bx, by, size, color) in buildings:
    rect = patches.Rectangle((bx - size/2, by - size/2), size, size, color=color, alpha=0.7)
    ax1.add_patch(rect)

# Drzewa (x, y) - przeliczone z SDF
trees = [
    (3.5, -1.5), (3.5, 1.5), (7.2, -1.7), (8.1, 1.6),
    (12.6, -1.4), (13.5, 1.8), (15.3, -1.6), (4.5, 1.5),
]
for (tx, ty) in trees:
    crown = plt.Circle((tx, ty), 0.8, color='#2d7d2d', alpha=0.6)
    ax1.add_patch(crown)

# 4 strefy lądowania (x, y, kolor) - przeliczone z SDF
landing_zones = [
    (5, 0, '#eab308', 'Strefa 1'),
    (10, 5, '#3b82f6', 'Strefa 2'),
    (15, 0, '#a855f7', 'Strefa 3'),
    (10, -5, '#ef4444', 'Strefa 4'),
]
for (zx, zy, color, label) in landing_zones:
    rect = patches.Rectangle((zx - 1.5, zy - 1.5), 3, 3, color=color, alpha=0.4)
    ax1.add_patch(rect)
    ax1.annotate(label, (zx, zy), ha='center', fontsize=8, fontweight='bold')

ax1.plot(leader_x, leader_y, label="Lider (dron 0)", color="#2563eb", linewidth=2)
ax1.plot(follower_x, follower_y, label="Podążający (dron 1)", color="#ea580c", linewidth=2)
ax1.scatter([leader_x[0]], [leader_y[0]], color='#2563eb', s=100, zorder=5)
ax1.scatter([follower_x[0]], [follower_y[0]], color='#ea580c', s=100, zorder=5)

ax1.set_xlabel("X [m]")
ax1.set_ylabel("Y [m]")
ax1.set_title("Misja: 4 lądowania w formacji, z przeszkodami")
ax1.legend(loc='upper left', fontsize=9)
ax1.axis('equal')
ax1.grid(True, alpha=0.3)

# --- Panel 2: utrzymanie formacji w czasie ---
ax2.plot(leader_t, distance, color="#16a34a", linewidth=2)
ax2.set_xlabel("Czas [s]")
ax2.set_ylabel("Odległość między dronami [m]")
ax2.set_title("Utrzymanie formacji podczas całej misji")
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("formation_analysis.png", dpi=150)
plt.show()

print(f"Lider: {len(leader_x)} punktów, Podążający: {len(follower_x)} punktów")
print(f"Średnia odległość: {np.mean(distance):.2f}m")
print(f"Odchylenie standardowe: {np.std(distance):.2f}m")
print(f"Min/Max odległość: {np.min(distance):.2f}m / {np.max(distance):.2f}m")
