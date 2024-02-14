class UserInput:
    def __init__(self, obstacle_map):
        self.goal_node = None
        self.initial_node = None
        self.step_size = None
        self.obstacle_map  = obstacle_map
        
    def enter_coordinates(self):
        x_initial = int(input("Enter initial x-coordinate "))
        y_initial = int(input("Enter initial y-coordinate "))
        x_goal = int(input("Enter goal x-coordinate "))
        y_goal = int(input("Enter goal y-coordinate "))
        orientation_initial = int(input("Enter initial orientation "))
        orientation_final = int(input("Enter goal orientation "))
        initial_node = (x_initial, self.obstacle_map.shape[0] - y_initial - 1, orientation_initial)
        goal_node = (x_goal, self.obstacle_map.shape[0] - y_goal - 1, orientation_final)
        return initial_node, goal_node
    
    def enter_step_size(self):
        self.step_size = int(input("Enter step-size (1 - 10): "))
        return self.step_size
    
    def check_valid_step_size(self):
        if(self.step_size > 10 or self.step_size < 1):
            print("\n Enter valid step size b/w 1 and 10 \n")
            return False
        else:
            return True
        
    def check_valid_entry(self):
        if(
        self.obstacle_map[self.initial_node[1]][self.initial_node[0]][0]==255 or \
        self.obstacle_map[self.goal_node[1]][self.goal_node[0]][0]==255 or \
        self.initial_node[0] < 0 or \
        self.initial_node[0] > 599 or \
        self.initial_node[1] < 0 or \
        self.initial_node[1] > 249 or \
        self.goal_node[0] < 0 or \
        self.goal_node[0] > 599 or \
        self.goal_node[1] < 0 or \
        self.goal_node[1] > 249 or \
        sum(self.obstacle_map[self.initial_node[1]][self.initial_node[0]])==765 or \
        sum(self.obstacle_map[self.goal_node[1]][self.goal_node[0]])==765
        ):
            print("The start point or end point is invalid. Either it is out of bounds or within obstacle space. Try Again!\n")
            return False
        else:
            print("Great! The start and goal points are valid")
            return True
        
    def take_input(self):
        self.initial_node, self.goal_node = self.enter_coordinates()
        while(not self.check_valid_entry()):
            self.initial_node, self.goal_node = self.enter_coordinates()
            
        self.step_size = self.enter_step_size()
        while(not self.check_valid_step_size):
            self.step_size = self.enter_step_size()
        return self.initial_node, self.goal_node, self.step_size