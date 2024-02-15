from helper.map import Map
from helper.user_input import UserInput
from helper.astar import AStar
from utils.plot import RecordVideo

if __name__=="__main__":
    map = Map(600, 250, 5, 75)
    map.generate_map()
    map.display_map()
    obstacle_map = map.get_map()
    
    input = UserInput(obstacle_map)
    initial_node, goal_node, step_size = input.take_input()
    
    astar = AStar(initial_node, goal_node, obstacle_map, step_size)
    astar.run()
    found_path = astar.generate_path()
    explored_list = astar.get_explored_list()
    
    record = RecordVideo(found_path, explored_list, obstacle_map, "./test_case")
    record.run()