import math
import numpy as np
from queue import PriorityQueue

class AStar:
    def __init__(self, initial_node, goal_node, obstacle_map):
        self.initial_node = initial_node
        self.goal_node = goal_node
        self.obstacle_map = obstacle_map
        
        self.open_list = PriorityQueue()
        self.close_list = np.zeros((500, 1200, 12), np.uint8)
        self.explored_list = []
        self.cost_of_node = []
        self.parent_node = []
        self.found_path = []
        
        self.open_list.put((0, self.initial_node))
        self.parent_node[self.initial_node] = None
        self.cost_of_node[self.initial_node] = 0
        return
    
    def move_neg_sixty(self, current_node, step_size):  
        next_node = []
        theta = -60
        next_x = int(round((current_node[0] + step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
        next_y = int(round((current_node[1] + step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
        next_orientation = theta + current_node[2]

        # For negative index in the visited matrix
        if(next_orientation < 0):
            next_orientation+=360
        next_orientation%=360
        
        next_node.append(next_x)
        next_node.append(next_y)
        next_node.append(next_orientation)
        return step_size, tuple(next_node)
    
    def move_neg_thirty(self, current_node, step_size):  
        next_node = []
        theta = -30
        next_x = int(round((current_node[0] + step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
        next_y = int(round((current_node[1] + step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
        next_orientation = theta + current_node[2]
        
        # For negative index in the visited matrix
        if(next_orientation < 0):
            next_orientation+=360
        next_orientation%=360
        
        next_node.append(next_x)
        next_node.append(next_y)
        next_node.append(next_orientation)
        return step_size, tuple(next_node)
    
    def move_zero(self, current_node, step_size):  
        next_node = []
        theta = 0
        next_x = int(round((current_node[0] + step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
        next_y = int(round((current_node[1] + step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
        next_orientation = theta + current_node[2]
        
        # For negative index in the visited matrix
        if(next_orientation < 0):
            next_orientation+=360
        next_orientation%=360
        
        next_node.append(next_x)
        next_node.append(next_y)
        next_node.append(next_orientation)
        return step_size, tuple(next_node)
    
    def move_pos_thirty(self, current_node, step_size):  
        next_node = []
        theta = 30
        next_x = int(round((current_node[0] + step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
        next_y = int(round((current_node[1] + step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
        next_orientation = theta + current_node[2]
        
        # For negative index in the visited matrix
        if(next_orientation < 0):
            next_orientation+=360
        next_orientation%=360
        
        next_node.append(next_x)
        next_node.append(next_y)
        next_node.append(next_orientation)
        return step_size, tuple(next_node)
    
    def move_pos_sixty(self, current_node, step_size):  
        next_node = []
        theta = 60
        next_x = int(round((current_node[0] + step_size * (math.cos((current_node[2] + theta) * (math.pi/180))))))
        next_y = int(round((current_node[1] + step_size * (math.sin((current_node[2] + theta) * (math.pi/180))))))
        next_orientation = theta + current_node[2]
        
        # For negative index in the visited matrix
        if(next_orientation < 0):
            next_orientation+=360
        next_orientation%=360
        
        next_node.append(next_x)
        next_node.append(next_y)
        next_node.append(next_orientation)
        return step_size, tuple(next_node)
    
    def valid_next_steps(self, current_node, step_size):
        valid_neighbors = []
        neg_sixty = self.move_neg_sixty(current_node, step_size)
        neg_thirty = self.move_neg_thirty(current_node, step_size)
        zero = self.move_zero(current_node, step_size)
        pos_thirty = self.move_pos_thirty(current_node, step_size)
        pos_sixty = self.move_pos_sixty(current_node, step_size)
        
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