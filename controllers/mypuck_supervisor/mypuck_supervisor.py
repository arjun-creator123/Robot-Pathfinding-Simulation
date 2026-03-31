"""mypuck_supervisor controller - MULTI-ALGO VERSION"""

from controller import Supervisor
import struct
from collections import defaultdict
import map_generator
import math
from time import sleep
import heapq 
import sys

# 1. READ CONTROLLER ARGUMENT
if len(sys.argv) > 1 and sys.argv[1].strip() != "":
    chosen_algo = sys.argv[1].strip().upper()
    if chosen_algo not in ["DFS", "BFS", "ASTAR", "A*"]:
        print(f"❌ ERROR: '{chosen_algo}' is not a valid algorithm.")
        sys.exit(1)
else:
    print("❌ ERROR: No algorithm specified in controllerArgs! Defaulting to DFS for safety.")
    chosen_algo = "DFS"

# 2. INITIALIZE SUPERVISOR
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

robot_node1 = supervisor.getFromDef("EPUCK1")
receiver1 = supervisor.getDevice("receiver1")
receiver1.enable(timestep)

trans_field1 = robot_node1.getField("translation")
rot_field1= robot_node1.getField("rotation")

right = (0,1,0,-1.57071) 
up = (0,1,0,0)
left = (0,1,0,1.57071)
down = (0,1,0,3.14142)
length_of_arena = 8

next_node1 = 0
current_node1 = 0
homenode1 = 0
first_run = True
flag = False

global_map_dict = {}
visited = [False for i in range(length_of_arena*length_of_arena)]
graph = defaultdict(set)

# 3. DATA STRUCTURES
class Stack():
    def __init__(self): self.stack = list()
    def push(self, item): self.stack.append(item)
    def pop(self): return self.stack.pop() if len(self.stack) > 0 else None

class BFSQueue():
    def __init__(self): self.queue = list()
    def push(self, item):
        if item not in self.queue: self.queue.append(item)
    def pop(self): return self.queue.pop(0) if len(self.queue) > 0 else None

class AStarQueue():
    def __init__(self):
        self.heap = []
        self.g_scores = {} 
    def push(self, item, f_score): heapq.heappush(self.heap, (f_score, item))
    def pop(self): return heapq.heappop(self.heap)[1] if len(self.heap) > 0 else None

# 4. PATHFINDING LOGIC
TARGET_X = 1.5
TARGET_Z = 1.5 # Fixed A* target coordinate

def dfs_next_node(current_node, robot_number):
    global visited, graph, data_struct_dict, homenode1
    data_struct_dict[robot_number].push(current_node)
    for node in graph[current_node]:
        if not visited[node]:
            if current_node == node: break
            return node
    if current_node == homenode1: return current_node
    data_struct_dict[robot_number].pop()
    return data_struct_dict[robot_number].pop()

def bfs_next_node(current_node, robot_number):
    global visited, graph, data_struct_dict, homenode1
    for node in graph[current_node]:
        if not visited[node]: data_struct_dict[robot_number].push(node)
    next_node = data_struct_dict[robot_number].pop()
    while next_node is not None and visited[next_node]:
        next_node = data_struct_dict[robot_number].pop()
    return homenode1 if next_node is None else next_node

def astar_next_node(current_node, robot_number):
    global visited, graph, data_struct_dict, homenode1, node_to_position
    if current_node not in data_struct_dict[robot_number].g_scores:
        data_struct_dict[robot_number].g_scores[current_node] = 0
    current_g = data_struct_dict[robot_number].g_scores[current_node]
    
    for node in graph[current_node]:
        if not visited[node]:
            g_score = current_g + 1
            if node not in data_struct_dict[robot_number].g_scores or g_score < data_struct_dict[robot_number].g_scores[node]:
                data_struct_dict[robot_number].g_scores[node] = g_score
                pos = node_to_position[node]
                h_score = math.hypot(pos[0] - TARGET_X, pos[2] - TARGET_Z)
                data_struct_dict[robot_number].push(node, g_score + h_score)
                
    next_node = data_struct_dict[robot_number].pop()
    while next_node is not None and visited[next_node]:
        next_node = data_struct_dict[robot_number].pop()
    return homenode1 if next_node is None else next_node

if chosen_algo == "BFS":
    print("🧠 Supervisor Engine: BFS")
    data_struct_dict = {1: BFSQueue()}
    next_node_generator = bfs_next_node
elif chosen_algo in ["ASTAR", "A*"]:
    print("🧠 Supervisor Engine: A*")
    data_struct_dict = {1: AStarQueue()}
    next_node_generator = astar_next_node
else:
    print("🧠 Supervisor Engine: DFS")
    data_struct_dict = {1: Stack()}
    next_node_generator = dfs_next_node

# 5. SHARED HELPER FUNCTIONS
def addEdge(graph,u,v): graph[u].add(v)

def get_closest_node(x, z, pos_dict):
    closest_key = None
    min_dist = float('inf')
    for key in pos_dict.keys():
        dist = math.hypot(key[0] - x, key[2] - z)
        if dist < min_dist:
            min_dist = dist
            closest_key = key
    return closest_key

def message_to_map_converion(X_pos,Z_pos,r_detail,u_detail,l_detail,d_detail):
    global global_map_dict
    surrounding = [r_detail,u_detail,l_detail,d_detail]
    global_map_dict[(X_pos,Z_pos)] = global_map_dict.get((X_pos,Z_pos),surrounding)

def write_map_to_file(target_pos):
    global global_map_dict
    file_handle = open("Map_details.txt", "w")
    for key,value in global_map_dict.items():
        sentence = "Position,{},{},Data,{},{},{},{}\n".format(str(key[0]),str(key[1]),str(value[0]),str(value[1]),str(value[2]),str(value[3]))
        file_handle.write(sentence)
    target_location = "Position,{},{},Data,{},{},{},{}\n".format(str(target_pos[0]),str(target_pos[1]),str(global_map_dict[(target_pos[0],target_pos[2])][0]),str(global_map_dict[(target_pos[0],target_pos[2])][1]),str(global_map_dict[(target_pos[0],target_pos[2])][2]),str(global_map_dict[(target_pos[0],target_pos[2])][3]))
    file_handle.close()

def graph_updation(current_node,r,u,l,d):
    global length_of_arena, graph
    max_nodes = 64
    if u and (current_node - length_of_arena >= 0) and (current_node - length_of_arena not in graph[current_node]): addEdge(graph, current_node, current_node - length_of_arena)
    if l and (current_node - 1 >= 0) and (current_node - 1 not in graph[current_node]): addEdge(graph, current_node, current_node - 1)
    if r and (current_node + 1 < max_nodes) and (current_node + 1 not in graph[current_node]): addEdge(graph, current_node, current_node + 1)
    if d and (current_node + length_of_arena < max_nodes) and (current_node + length_of_arena not in graph[current_node]): addEdge(graph, current_node, current_node + length_of_arena)

def next_direction(current_node,next_node):
    global length_of_arena, right, up, left, down
    if current_node+1 == next_node: return right
    elif current_node-length_of_arena == next_node: return up
    elif current_node-1 == next_node: return left
    elif current_node+length_of_arena == next_node: return down
    elif current_node==next_node: return down
    else: return down 

def conversion_between_node_and_position(length_of_arena,division=True):
    if division==True: length_of_arena = int(length_of_arena/2)
    positions_numbers = []
    for i in range(length_of_arena-1,-1,-1): positions_numbers.append(-float(str(i)+".5"))
    for i in range(0,length_of_arena): positions_numbers.append(float(str(i)+".5"))
    node_to_position = {}
    mid_positions = []
    for z in positions_numbers:
        for x in positions_numbers: mid_positions.append((x,0,z))
    for i in range(0,2*length_of_arena*2*length_of_arena): node_to_position[i] = node_to_position.get(i,mid_positions[i])
    position_to_node = dict([(value, key) for key, value in node_to_position.items()])
    return node_to_position,position_to_node

# 6. MAIN EXECUTION LOOP
(node_to_position,position_to_node) = conversion_between_node_and_position(length_of_arena)
message1 = [False, 0, 0, False, False, False, False]
timings = range(1,720)

for time in timings:
    while supervisor.getTime() < time:
        if supervisor.step(timestep) == -1: quit()
            
    if receiver1.getQueueLength()>0:
        received_data1 = receiver1.getBytes()
        message1 = struct.unpack("? f f ? ? ? ?",received_data1)
        
        closest_key1 = get_closest_node(message1[1], message1[2], position_to_node)
        current_node1 = position_to_node[closest_key1]
        
        if first_run and bool(graph):
            homenode1 = current_node1
            print("Starting Search from Homenode =", homenode1)
            first_run = False
            
        graph_updation(current_node1, message1[3], message1[4], message1[5], message1[6])
        message_to_map_converion(closest_key1[0], closest_key1[2], message1[3], message1[4], message1[5], message1[6])
        visited[current_node1] = True
        receiver1.nextPacket()
    
    if bool(graph):        
        next_node1 = next_node_generator(current_node1, 1)
        
        # Check if the target was found
        if message1[0]:
            print("🎯 TARGET FOUND! Generating map and stopping simulation.")
            target_pos = node_to_position[current_node1]
            
            # Prevent the extra wall from rendering by forcing clear paths
            global_map_dict[(target_pos[0], target_pos[2])] = [True, True, True, True]
            
            write_map_to_file(node_to_position[current_node1])
            
            # 1. Tell Webots to pause the simulation FIRST
            supervisor.simulationSetMode(0) 
            
            # 2. Generate the map (this opens the window)
            map_generator.generate_map()
            
            # 3. Keep the map open for 5 seconds (Change this number to whatever you want)
            sleep(5)
            
            # 4. Break the loop to cleanly shut down the controller!
            break
            
        # Check if maze is fully explored without finding target
        # CRITICAL FIX: Only trigger if we are at the homenode AND the queue is completely empty (next_node1 == homenode1)
        if current_node1 == homenode1 and next_node1 == homenode1 and flag and time > 4:
            print("🏁 Maze fully mapped. Target not found in reachable area.", flush=True)
            
            # Actually stop the simulation if it truly fails!
            supervisor.simulationSetMode(0)
            supervisor.step(timestep)
            import sys
            sys.exit(0)
            
        if not first_run: flag = True
        
        trans_field1.setSFVec3f(list(node_to_position[next_node1]))
        rot_field1.setSFRotation(list(next_direction(current_node1,next_node1)))
        robot_node1.resetPhysics()