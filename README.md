# 🤖 Robot Pathfinding Simulation

A comprehensive robotics project demonstrating autonomous navigation and maze exploration using three fundamental search algorithms: **Depth-First Search (DFS)**, **Breadth-First Search (BFS)**, and **A***.

## 🚀 Key Features
* **Supervisor-Robot Architecture:** A dedicated supervisor controller (`mypuck_supervisor.py`) handles the complex graph logic and pathfinding, while the robot controller (`mypuck_comms_only_controller.py`) focuses on sensor data acquisition.
* **Dynamic Algorithm Switching:** Swap between DFS, BFS, and A* instantly via Webots `controllerArgs` without modifying source code.
* **Multi-Sensor Integration:** The E-puck utilizes GPS for positioning, a Compass for orientation, and four custom distance sensors (`up`, `down`, `left`, `right`) for obstacle detection.
* **Target Detection:** The simulation identifies a specific target (a red sphere at coordinates $1.5, 1.5$) and triggers a sound alert upon arrival.
* **Automated Map Generation:** Upon reaching the target, the system generates a 2D visual representation of the explored maze using Python's `turtle` and `tkinter` libraries.

---

## 🛠 Tech Stack
* **Simulator:** Webots R2025a
* **Language:** Python 3.x
* **Libraries:** `collections` (defaultdict), `heapq` (A* priority queue), `struct` (binary data handling), `turtle` (map visualization)
* **Robot Model:** GCTronic E-puck

---

## 🧠 Algorithms Included

| Algorithm | Logic Type | Behavior |
| :--- | :--- | :--- |
| **DFS** | Stack-based | Deep exploration of one path before backtracking; standard for maze solving. |
| **BFS** | Queue-based | Level-order exploration; guaranteed to find the shortest path in an unweighted grid. |
| **A*** | Priority Queue (Heuristic) | Uses Euclidean distance to the target ($1.5, 1.5$) as a heuristic for optimized pathfinding. |

---

## ⚙️ How to Run

1.  **Open the World:** Launch `Simulation Environment.wbt` in Webots.
2.  **Configuration:** * Locate the **`MAINSUPERVISOR`** node in the Scene Tree.
    * In the **`controllerArgs`** field, enter your desired algorithm: `"DFS"`, `"BFS"`, or `"ASTAR"`.
    * Repeat this for the **`EPUCK1`** node's `controllerArgs`.
3.  **Simulation:** * Reset the simulation (⏪) and press Play (►).
    * The robot will explore the maze until it reaches the red sphere.
4.  **Result:** * The simulation will automatically pause.
    * A Turtle graphics window will open to display the generated map.
    * A file named `map.jpg` will be saved to the supervisor directory.

---

## 📂 Project Structure
* `Simulation Environment.wbt`: The 3D environment containing the maze and E-puck.
* `mypuck_supervisor.py`: The "brain" of the simulation. It maps the grid, runs search algorithms, and moves the robot.
* `mypuck_comms_only_controller.py`: The "eyes" of the robot. It transmits sensor states (obstacle present: True/False) to the supervisor.
* `map_generator.py`: A utility script that reads `Map_details.txt` and renders the maze walls and target location.
