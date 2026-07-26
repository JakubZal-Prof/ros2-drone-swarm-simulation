# ROS2 Drone Swarm Simulation

Symulacja roju dronów w ROS2 + PX4 SITL + Gazebo, napisana w C++. Dwa drony (lider i podążający) wykonują wspólną misję: lot w formacji przez środowisko z przeszkodami (budynki, drzewa), z lądowaniem w czterech różnych, wyznaczonych strefach.

## Stack technologiczny

- **ROS2 Humble**
- **PX4 Autopilot v1.16.2** (stabilne wydanie SITL)
- **Gazebo Harmonic**
- **C++** (rclcpp, px4_msgs)
- **Python** (analiza danych, generowanie świata symulacji)

## Architektura

- **Lider** — leci przez sekwencję 4 stref lądowania, wykonując w każdej pełny cykl: przelot → lądowanie → oczekiwanie → start
- **Podążający** — w czasie rzeczywistym śledzi pozycję lidera, utrzymując stały wektor przesunięcia względem niego (formacja)
- Komunikacja między dronami odbywa się przez natywne mechanizmy ROS2 (topiki z namespace'ami), bez dodatkowej infrastruktury

Logika lotu zaimplementowana jest jako maszyna stanów (`CRUISE_TO_ZONE → LANDING → HOLDING → TAKEOFF`), sterująca dronem przez PX4 offboard control mode.

## Struktura repozytorium
## Uruchomienie

### Wymagania wstępne
- Zainstalowane ROS2 Humble, PX4-Autopilot (branch v1.16.2), Gazebo Harmonic
- Zbudowany `micro-xrce-dds-agent`
- QGroundControl (wymagany do przejścia preflight checks w symulacji)

### Klonowanie (uwaga na submoduł)

```bash
git clone --recurse-submodules <adres-repo>
```

Jeśli repo zostało już sklonowane bez tej flagi:
```bash
git submodule update --init --recursive
```

### Podmiana świata symulacji

```bash
cp src/swarm_control/worlds/swarm_world.sdf ~/.simulation-gazebo/worlds/default.sdf
```

### Budowanie

```bash
colcon build
source install/setup.bash
```

### Start pełnej symulacji

```bash
ros2 launch swarm_control single_drone.launch.py
```

Uruchamia Gazebo, most PX4↔ROS2, obie instancje PX4 SITL oraz węzły sterujące dronami. Wymaga równolegle uruchomionego QGroundControl (łączy się automatycznie).

## Analiza lotu

Po nagraniu lotu (`ros2 bag record`), skrypt `analysis/plot_formation.py` generuje wykres trajektorii obu dronów na tle przeszkód i stref lądowania, wraz z wykresem utrzymania formacji w czasie.

![formation analysis](formation_analysis.png)

## Napotkane wyzwania techniczne

- **Niezgodność wersji PX4/px4_msgs** — konieczność dopasowania stabilnych wersji obu komponentów
- **Konwencja układu współrzędnych NED (PX4) vs ENU (Gazebo)** — wymaga świadomej zamiany osi X/Y przy projektowaniu świata symulacji względem trasy lotu
- **`target_system` w multi-vehicle** — każda instancja PX4 ma unikalny identyfikator systemu, wymagany do poprawnego adresowania komend
- **Odporność na timing startu** — logika retry przy uzbrajaniu drona, niezależna od dokładnego czasu inicjalizacji PX4
