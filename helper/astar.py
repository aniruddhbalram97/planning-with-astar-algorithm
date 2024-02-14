import math
import numpy as np
from queue import PriorityQueue

class AStar:
    def __init__(self, initial_node, goal_node, obstacle_map, step_size):
        self.initial_node = initial_node
        self.goal_node = goal_node
        self.obstacle_map = obstacle_map
        self.step_size = step_size
        
        self.open_list = PriorityQueue()
        self.close_list = np.zeros((500, 1200, 12), np.uint8)
        self.explored_list = []
        self.cost_of_node = {}
        self.parent_node = {}
        self.found_path = []
        
        self.open_list.put((0, self.initial_node))
        self.parent_node[self.initial_node] = None
        self.cost_of_node[self.initial_node] = 0
        return
    
    def move_neg_sixty(self, current_node):  
        next_node = []
        theta = -60
        next_x = int(round((current_node[0] + self.step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
        next_y = int(round((current_node[1] + self.step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
        next_orientation = theta + current_node[2]

        # For negative index in the visited matrix
        if(next_orientation < 0):
            next_orientation+=360
        next_orientation%=360
        
        next_node.append(next_x)
        next_node.append(next_y)
        next_node.append(next_orientation)
        return self.step_size, tuple(next_node)
    
    def move_neg_thirty(self, current_node):  
        next_node = []
        theta = -30
        next_x = int(round((current_node[0] + self.step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
        next_y = int(round((current_node[1] + self.step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
        next_orientation = theta + current_node[2]
        
        # For negative index in the visited matrix
        if(next_orientation < 0):
            next_orientation+=360
        next_orientation%=360
        
        next_node.append(next_x)
        next_node.append(next_y)
        next_node.append(next_orientation)
        return self.step_size, tuple(next_node)
    
    def move_zero(self, current_node):  
        next_node = []
        theta = 0
        next_x = int(round((current_node[0] + self.step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
        next_y = int(round((current_node[1] + self.step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
        next_orientation = theta + current_node[2]
        
        # For negative index in the visited matrix
        if(next_orientation < 0):
            next_orientation+=360
        next_orientation%=360
        
        next_node.append(next_x)
        next_node.append(next_y)
        next_node.append(next_orientation)
        return self.step_size, tuple(next_node)
    
    def move_pos_thirty(self, current_node):  
        next_node = []
        theta = 30
        next_x = int(round((current_node[0] + self.step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
        next_y = int(round((current_node[1] + self.step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
        next_orientation = theta + current_node[2]
        
        # For negative index in the visited matrix
        if(next_orientation < 0):
            next_orientation+=360
        next_orientation%=360
        
        next_node.append(next_x)
        next_node.append(next_y)
        next_node.append(next_orientation)
        return self.step_size, tuple(next_node)
    
    def move_pos_sixty(self, current_node):  
        next_node = []
        theta = 60
        next_x = int(round((current_node[0] + self.step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
        next_y = int(round((current_node[1] + self.step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
        next_orientation = theta + current_node[2]
        
        # For negative index in the visited matrix
        if(next_orientation < 0):
            next_orientation+=360
        next_orientation%=360
        
        next_node.append(next_x)
        next_node.append(next_y)
        next_node.append(next_orientation)
        return self.step_size, tuple(next_node)
    
    def valid_next_steps(self, current_node):
        valid_neighbors = []
        neg_sixty = self.move_neg_sixty(current_node)
        neg_thirty = self.move_neg_thirty(current_node)
        zero = self.move_zero(current_node)
        pos_thirty = self.move_pos_thirty(current_node)
        pos_sixty = self.move_pos_sixty(current_node)
        
        if((neg_sixty[1][0] - 5)< 0 or (neg_sixty[1][0] + 5) > 599 
        or (neg_sixty[1][1] - 5) < 0 or (neg_sixty[1][1] + 5) > 249
        or self.obstacle_map[neg_sixty[1][1] + 5][neg_sixty[1][0] + 5][0]==255
        or self.obstacle_map[neg_sixty[1][1] - 5][neg_sixty[1][0] + 5][0]==255
        or self.obstacle_map[neg_sixty[1][1] - 5][neg_sixty[1][0] - 5][0]==255
        or self.obstacle_map[neg_sixty[1][1] + 5][neg_sixty[1][0] - 5][0]==255
        or self.obstacle_map[neg_sixty[1][1] + 5][neg_sixty[1][0]][0]==255
        or self.obstacle_map[neg_sixty[1][1] - 5][neg_sixty[1][0]][0]==255
        or self.obstacle_map[neg_sixty[1][1]][neg_sixty[1][0] - 5][0]==255
        or self.obstacle_map[neg_sixty[1][1]][neg_sixty[1][0] - 5][0]==255
        or sum(self.obstacle_map[neg_sixty[1][1] + 5][neg_sixty[1][0] + 5])==765
        or sum(self.obstacle_map[neg_sixty[1][1] - 5][neg_sixty[1][0] + 5])==765
        or sum(self.obstacle_map[neg_sixty[1][1] - 5][neg_sixty[1][0] - 5])==765
        or sum(self.obstacle_map[neg_sixty[1][1] + 5][neg_sixty[1][0] - 5])==765
        or sum(self.obstacle_map[neg_sixty[1][1] + 5][neg_sixty[1][0]])==765
        or sum(self.obstacle_map[neg_sixty[1][1] - 5][neg_sixty[1][0]])==765
        or sum(self.obstacle_map[neg_sixty[1][1]][neg_sixty[1][0] - 5])==765
        or sum(self.obstacle_map[neg_sixty[1][1]][neg_sixty[1][0] - 5])==765
        ):
            print("-60 move invalid")
        else:
            valid_neighbors.append(neg_sixty)
            
        if((neg_thirty[1][0] - 5)< 0 or (neg_thirty[1][0] + 5) > 599 
        or (neg_thirty[1][1] - 5) < 0 or (neg_thirty[1][1] + 5) > 249
        or self.obstacle_map[neg_thirty[1][1] + 5][neg_thirty[1][0] + 5][0]==255
        or self.obstacle_map[neg_thirty[1][1] + 5][neg_thirty[1][0] - 5][0]==255
        or self.obstacle_map[neg_thirty[1][1] - 5][neg_thirty[1][0] + 5][0]==255
        or self.obstacle_map[neg_thirty[1][1] - 5][neg_thirty[1][0] - 5][0]==255
        or self.obstacle_map[neg_thirty[1][1] + 5][neg_thirty[1][0]][0]==255
        or self.obstacle_map[neg_thirty[1][1] - 5][neg_thirty[1][0]][0]==255
        or self.obstacle_map[neg_thirty[1][1]][neg_thirty[1][0] + 5][0]==255
        or self.obstacle_map[neg_thirty[1][1]][neg_thirty[1][0] - 5][0]==255
        or sum(self.obstacle_map[neg_thirty[1][1] + 5][neg_thirty[1][0] + 5])==765
        or sum(self.obstacle_map[neg_thirty[1][1] + 5][neg_thirty[1][0] - 5])==765
        or sum(self.obstacle_map[neg_thirty[1][1] - 5][neg_thirty[1][0] + 5])==765
        or sum(self.obstacle_map[neg_thirty[1][1] - 5][neg_thirty[1][0] - 5])==765
        or sum(self.obstacle_map[neg_thirty[1][1] + 5][neg_thirty[1][0]])==765
        or sum(self.obstacle_map[neg_thirty[1][1] - 5][neg_thirty[1][0]])==765
        or sum(self.obstacle_map[neg_thirty[1][1]][neg_thirty[1][0] + 5])==765
        or sum(self.obstacle_map[neg_thirty[1][1]][neg_thirty[1][0] - 5])==765
        ):
            print("-30 move invalid")
        else:
            valid_neighbors.append(neg_thirty)
            
        if((zero[1][0] - 5)< 0 or (zero[1][0] + 5) > 599 
        or (zero[1][1] - 5) < 0 or (zero[1][1] + 5) > 249
        or self.obstacle_map[zero[1][1] + 5][zero[1][0] + 5][0]==255
        or self.obstacle_map[zero[1][1] + 5][zero[1][0] - 5][0]==255
        or self.obstacle_map[zero[1][1] - 5][zero[1][0] + 5][0]==255
        or self.obstacle_map[zero[1][1] - 5][zero[1][0] - 5][0]==255
        or self.obstacle_map[zero[1][1]][zero[1][0] + 5][0]==255
        or self.obstacle_map[zero[1][1]][zero[1][0] - 5][0]==255
        or self.obstacle_map[zero[1][1] + 5][zero[1][0]][0]==255
        or self.obstacle_map[zero[1][1] - 5][zero[1][0]][0]==255     
        or sum(self.obstacle_map[zero[1][1] + 5][zero[1][0] + 5])==765
        or sum(self.obstacle_map[zero[1][1] + 5][zero[1][0] - 5])==765
        or sum(self.obstacle_map[zero[1][1] - 5][zero[1][0] + 5])==765
        or sum(self.obstacle_map[zero[1][1] - 5][zero[1][0] - 5])==765
        or sum(self.obstacle_map[zero[1][1]][zero[1][0] + 5])==765
        or sum(self.obstacle_map[zero[1][1]][zero[1][0] - 5])==765
        or sum(self.obstacle_map[zero[1][1] + 5][zero[1][0]])==765
        or sum(self.obstacle_map[zero[1][1] - 5][zero[1][0]])==765  
        ):
            print("0 move invalid")
        else:
            valid_neighbors.append(zero)
            
        if((pos_thirty[1][0] - 5)< 0 or (pos_thirty[1][0] + 5) > 599 
        or (pos_thirty[1][1] - 5) < 0 or (pos_thirty[1][1] + 5) > 249
        or self.obstacle_map[pos_thirty[1][1] + 5][pos_thirty[1][0] + 5][0]==255
        or self.obstacle_map[pos_thirty[1][1] - 5][pos_thirty[1][0] + 5][0]==255
        or self.obstacle_map[pos_thirty[1][1] + 5][pos_thirty[1][0] - 5][0]==255
        or self.obstacle_map[pos_thirty[1][1] - 5][pos_thirty[1][0] - 5][0]==255
        or self.obstacle_map[pos_thirty[1][1]][pos_thirty[1][0] + 5][0]==255
        or self.obstacle_map[pos_thirty[1][1]][pos_thirty[1][0] - 5][0]==255
        or self.obstacle_map[pos_thirty[1][1] + 5][pos_thirty[1][0]][0]==255
        or self.obstacle_map[pos_thirty[1][1] - 5][pos_thirty[1][0]][0]==255
        or sum(self.obstacle_map[pos_thirty[1][1] + 5][pos_thirty[1][0] + 5])==765
        or sum(self.obstacle_map[pos_thirty[1][1] - 5][pos_thirty[1][0] + 5])==765
        or sum(self.obstacle_map[pos_thirty[1][1] + 5][pos_thirty[1][0] - 5])==765
        or sum(self.obstacle_map[pos_thirty[1][1] - 5][pos_thirty[1][0] - 5])==765
        or sum(self.obstacle_map[pos_thirty[1][1]][pos_thirty[1][0] + 5])==765
        or sum(self.obstacle_map[pos_thirty[1][1]][pos_thirty[1][0] - 5])==765
        or sum(self.obstacle_map[pos_thirty[1][1] + 5][pos_thirty[1][0]])==765
        or sum(self.obstacle_map[pos_thirty[1][1] - 5][pos_thirty[1][0]])==765
        ):
            print("30 move invalid")
        else:
            valid_neighbors.append(pos_thirty)
            
        if((pos_sixty[1][0] - 5)< 0 or (pos_sixty[1][0] + 5) > 599 
        or (pos_sixty[1][1] - 5) < 0 or (pos_sixty[1][1] + 5) > 249
        or self.obstacle_map[pos_sixty[1][1] + 5][pos_sixty[1][0] + 5][0]==255
        or self.obstacle_map[pos_sixty[1][1] + 5][pos_sixty[1][0] - 5][0]==255
        or self.obstacle_map[pos_sixty[1][1] - 5][pos_sixty[1][0] - 5][0]==255
        or self.obstacle_map[pos_sixty[1][1] - 5][pos_sixty[1][0] + 5][0]==255
        or self.obstacle_map[pos_sixty[1][1] + 5][pos_sixty[1][0]][0]==255
        or self.obstacle_map[pos_sixty[1][1] - 5][pos_sixty[1][0]][0]==255
        or self.obstacle_map[pos_sixty[1][1]][pos_sixty[1][0] - 5][0]==255
        or self.obstacle_map[pos_sixty[1][1]][pos_sixty[1][0] + 5][0]==255  
        or sum(self.obstacle_map[pos_sixty[1][1] + 5][pos_sixty[1][0] + 5])==765
        or sum(self.obstacle_map[pos_sixty[1][1] + 5][pos_sixty[1][0] - 5])==765
        or sum(self.obstacle_map[pos_sixty[1][1] - 5][pos_sixty[1][0] - 5])==765
        or sum(self.obstacle_map[pos_sixty[1][1] - 5][pos_sixty[1][0] + 5])==765
        or sum(self.obstacle_map[pos_sixty[1][1] + 5][pos_sixty[1][0]])==765
        or sum(self.obstacle_map[pos_sixty[1][1] - 5][pos_sixty[1][0]])==765
        or sum(self.obstacle_map[pos_sixty[1][1]][pos_sixty[1][0] - 5])==765
        or sum(self.obstacle_map[pos_sixty[1][1]][pos_sixty[1][0] + 5])==765       
        ):
            print("60 move invalid")
        else:
            valid_neighbors.append(pos_sixty) 
        return valid_neighbors
    
    def run(self):
        while self.open_list:
            current_node = self.open_list.get()[1]
            self.close_list[int(round(current_node[1] * 2)),int(round(current_node[0] * 2)), int(round(current_node[2]/30))] = 2
            self.explored_list.append(current_node)
            if(math.sqrt((current_node[0] - self.goal_node[0])**2 + (current_node[1] - self.goal_node[1])**2) < 0.5 and 
            current_node[2] == self.goal_node[2]):
                print("\n.. GOAL REACHED ..\n")
                break
            else:
                valid_neighbors = self.valid_next_steps(current_node)
                for next_node in valid_neighbors:
                    closed = self.close_list[int(round(next_node[1][1] * 2)),int(round(next_node[1][0] * 2)), \
                        int(round(next_node[1][2]/30))]
                    # if the node is already present, ignore it
                    if(closed == 2):
                        print("visited: ", next_node)
                        continue    
                    else:
                        cost_to_next_node = next_node[0]
                        total_cost_to_node = self.cost_of_node[current_node] + cost_to_next_node
                        if(next_node[1] not in self.cost_of_node or total_cost_to_node < self.cost_of_node[next_node[1]]):
                            self.cost_of_node[next_node[1]] = total_cost_to_node
                            euc_dist = math.sqrt((next_node[1][0] - self.goal_node[0])**2 + (next_node[1][1] - self.goal_node[1])**2)
                            cost_with_heuristic = total_cost_to_node + euc_dist       
                            self.open_list.put((cost_with_heuristic, next_node[1]))
                            self.parent_node[next_node[1]] = current_node 
    
    def generate_path(self):
        end_node = self.goal_node
        while end_node is not None:
            self.found_path.append(end_node)
            end_node = self.parent_node[end_node]
        self.found_path.reverse()
        return self.found_path
    
    def get_explored_list(self):
        return self.explored_list
    
