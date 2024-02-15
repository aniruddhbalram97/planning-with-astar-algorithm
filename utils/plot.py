import cv2

class RecordVideo:
    def __init__(self, path, explored_list, map, file_path):
        self.path = path
        self.explored_list = explored_list
        self.map = map
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.path_video = cv2.VideoWriter(file_path + "/path_video.avi", self.fourcc, 60, (map.shape[1], map.shape[0]))
        self.explored_video = cv2.VideoWriter(file_path + "/explored_video.avi", self.fourcc, 60, (map.shape[1], map.shape[0]))
        
    def run(self):
        path_map = self.map.copy()
        for x, y, ori in self.path:
            path_map[y, x, 1] = 255
            cv2.circle(path_map, (x, y), 2, (0, 0, 255))
            self.path_video.write(path_map)
            
        explored_map = self.map.copy()
        for x, y, ori in self.explored_list:
            explored_map[y, x ,1] = 255
            self.explored_video.write(explored_map)
        return
    