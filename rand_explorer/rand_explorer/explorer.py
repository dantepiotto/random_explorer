#! /usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_simple_commander.costmap_2d import PyCostmap2D
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import random
import numpy as np
import time
from rclpy import qos
from std_msgs.msg import Empty

"""
Basic navigation demo to go to pose.
"""

# def Neighbours(mx,my,costmap):                          # Returns the 8-neighbourhood of the input point, given in map coordinates, as a list of [x,y] map coordinates clipped to the map boundaries
#     neighbours = [[max(mx-1,0),max(my-1,0)],[max(mx-1,0),my],[mx,min(my+1,costmap.size_y-1)],[mx,max(my-1,0)],[mx,my+1],[min(mx+1,costmap.size_x-1),max(my-1,0)],[min(mx+1,costmap.size_x-1),my],[min(mx+1,costmap.size_x-1),min(my+1,costmap.size_y-1)]]
#     return neighbours
# 
# def isFrontierPoint(goalpoint_generator,mx,my,costmap,rectangle):                     # Checks wheter a point is a frontier point by looking at the 8-neighbourhood. If a point in the neighbourhood is unexplored, returns true, else returns false
#     if not costmap.getCostXY(mx,my) < 100:
#         return False
#     for point in Neighbours(mx,my,costmap):
#         if not isInRectangle(goalpoint_generator, point[0],point[1],rectangle,costmap,costmap_data):
#             return False
# 
#         cost = costmap.getCostXY(point[0],point[1])
#         if cost == 255:
#             return True
#     return False
# 
# def isInRectangle(goalpoint_generator,mx,my,rectangle,costmap,costmap_data):
# 
#         rect_x1, rect_y1, rect_x2, rect_y2 = rectangle
#         # Converting rectangle boundaries to map coordinates
#         m_x1, m_y1 = costmap.worldToMap(rect_x1, rect_y1)
#         m_x2, m_y2 = costmap.worldToMap(rect_x2, rect_y2)
# 
#         # clip coordinates to map boundaries
#         m_x1 = goalpoint_generator.clip_mx(m_x1)
#         m_y1 = goalpoint_generator.clip_mx(m_y1)
#         m_x2 = goalpoint_generator.clip_mx(m_x2)
#         m_y2 = goalpoint_generator.clip_mx(m_y2)
# 
#         # Extract the relevant rectangle of the map (done on map converted to numpy matrix)
#         map_region = goalpoint_generator.costmap_data[m_y1:m_y2 +1, m_x1:m_x2+1]
# 
#         if m_x1 <= mx <= m_x2 and m_y1 <= my <= m_y2:
#             return True
#         return False
# 
# def NaiveFrontierSearch(goalpoint_generator,costmap,rectangle):                       # Returns a list of frontier points from a costmap stored as numpy array
#     #frontier_pts = [[x[0], x[1]] for x in np.argwhere(np.vectorize(lambda x: isFrontierPoint(goalpoint_generator, x[0], x[1], costmap, rectangle) if costmap[x[0], x[1]] < 100 else False)(np.indices(costmap.shape)))]
#     lambda x : isFrontierPoint(goalpoint_generator, x[0], x[1],costmap,rectangle)
#     return frontier_pts
# 
# def findClosestFrontier(frontier_pts, mx, my):
#     if not frontier_pts:
#         print("no Frontier points available")
#         return None
# 
#     frontier_array = np.array(frontier_pts)
#     reference_point = np.array([mx,my])
# 
#     distances = np.linalg.norm(frontier_array - reference_point, axis=1)
#     closest_index = np.argmin(distances)
# 
#     closest_point = frontier_pts[closest_index]
# 
#     return closest_point




class goalpoint_generator(Node):

    def __init__(self):
        super().__init__('random_explorer')
        self.costmap_ = None
        self.costmap_data = None
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback,qos.qos_profile_sensor_data)
        self.go_subscription = self.create_subscription(Empty, '/gopoint', self.generation_callback,10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.position_callback, 1)
        self.publisher_ = self.create_publisher(Empty, '/gopoint', 10)
        self.navigator = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0 # robot starting position is the origin of the reference frame
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.z = 1.0
        self.initial_pose.pose.orientation.w = 0.0
        self.x = None
        self.y = None
        self.state = 0 # State variable to indicate explore mode (0) or roam mode (1)
        self.navigator.setInitialPose(self.initial_pose)
        #self.navigator.waitUntilNav2Active()

    def map_callback(self, map):
        print("I am the map callback!")
        self.costmap_ = PyCostmap2D(map) 
        self.costmap_data = np.transpose(np.array(map.data, dtype=np.int8).reshape((map.info.height, map.info.width)))

    def position_callback(self, pose_msg):
        # print("I am the pose callback!")
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y

    def generate_random_coordinates(self,rectangle): # To be used only once the map is explored
        x1,y1,x2,y2 = rectangle
        while True:
            x = random.uniform(x1,x2)
            y = random.uniform(y1,y2)
            x0 = self.initial_pose.pose.position.x #for writeability/readability of code
            y0 = self.initial_pose.pose.position.y
            # keeping a log of the past 10 waypoints, in order to compute distance of potential waypoints with them to promote sufficient exploration
            past_positions = [[x0,y0],[x0,y0],[x0,y0],[x0,y0],[x0,y0],[x0,y0],[x0,y0],[x0,y0],[x0,y0],[x0,y0]]
            distances = np.linalg.norm(np.array(past_positions) - np.array([x,y]), axis=1)
            print("attempting random point with coords",x,y)
            if self.is_good_point(x,y) and np.min(distances) > 2:
                w = random.uniform(0,3.14)
                past_positions = [x,y] + past_positions[:-1]
                return x,y,w

    def generation_callback(self, msg):
        exploredness_thresh = 0.7 # TODO fix with actual map data
        xrange = 10
        yrange = 20
        rectangle = [-3,-2,3,2] # TODO fix with actual map data
        print("starting to generate a goalpoint")
        if self.state == 0:
            print("performing frontier detection ... \n")
            mx,my = self.costmap_.worldToMap(self.x,self.y)
            goal_point = self.findClosestFrontier(self.NaiveFrontierSearch(rectangle),mx,my)
            x,y = self.costmap_.mapToWorld(goal_point[0], goal_point[1])
            print(f"frontier point found with coords: {x} {y} ", )
            w = random.uniform(0,3.14)
        else:
            x,y,w = self.generate_random_coordinates(rectangle)
            print(f"Generated coordinates: X={x:.2f}, Y={y:.2f}, W={w:.2f}")
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.orientation.w = w
        self.navigator.goToPose(goal_pose)
        i= 0
        while not self.navigator.isTaskComplete():
            time.sleep(1)
            i+=1 
            if i == 10: 
                pass

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status or no result was obtained') # TODO regulate timeout for goal reach
        exploredness, exploredness_partial = self.get_exploredness(rectangle)
        print("exploredness value: ",exploredness)
        if exploredness > exploredness_thresh:
            self.state = 1                      # switch to roam mode if map is sufficiently explored
        msg = Empty()
        self.publisher_.publish(msg)

    

    def get_exploredness(self, rectangle):
        if self.costmap_ is None:
            print("Map data not available")
            time.sleep(1)
            return None
        rect_x1, rect_y1, rect_x2, rect_y2 = rectangle
        # Converting rectangle boundaries to map coordinates
        m_x1, m_y1 = self.costmap_.worldToMap(rect_x1, rect_y1)
        m_x2, m_y2 = self.costmap_.worldToMap(rect_x2, rect_y2)
        rectangle_pixel_area = (m_x2-m_x1+1)*(m_y2-m_y1 +1)

        # clip coordinates to map boundaries
        m_x1 = np.clip(m_x1, 0, self.costmap_.getSizeInCellsX()-1)
        m_y1 = np.clip(m_y1, 0, self.costmap_.getSizeInCellsY()-1)
        m_x2 = np.clip(m_x2, 0, self.costmap_.getSizeInCellsX()-1)
        m_y2 = np.clip(m_y2, 0, self.costmap_.getSizeInCellsY()-1)


        # Extract the relevant rectangle of the map (done on map converted to numpy matrix)
        map_region = self.costmap_data[m_y1:m_y2 +1, m_x1:m_x2+1]
        # Count points with value 255
        total_points = (m_x2-m_x1+1)*(m_y2-m_y1 +1)
        free_space_points = np.count_nonzero(map_region == -1)
        # calculate portion of uknown points
        exploredness_known = free_space_points/total_points
        exploredness = free_space_points/rectangle_pixel_area
        return exploredness, exploredness_known
        
    def is_good_point(self,x,y): 
        if self.costmap_ is None:
            print("Map data not available")
            time.sleep(1)
            return False
        mx, my = self.costmap_.worldToMap(x,y)
        # clip rw coordinates to limits of map 
        mx = np.clip(mx, 0, self.costmap_.getSizeInCellsX()-1)
        my = np.clip(my, 0, self.costmap_.getSizeInCellsY()-1)

        cost = self.costmap_.getCostXY(mx,my) 
        rectangle = (0.0,0.0,10.0,20.0) # TODO change rectangle coords
        if self.costmap_.getCostXY(mx,my) != 0 :
            print("point was cost higher than 0")
            return False
        else:
            return True

    ###############################################################################################
    #################              Frontier detection implementation              #################
    ###############################################################################################
        
    def Neighbours(self,mx,my):                          # Returns the 8-neighbourhood of the input point, given in map coordinates, as a list of [x,y] map coordinates clipped to the map boundaries
        neighbours = [[max(mx-1,0),max(my-1,0)],[max(mx-1,0),my],[max(mx-1,0),min(my+1,self.costmap_data.shape[1])],[mx,max(my-1,0)],[mx,min(my+1,self.costmap_data.shape[1])],[min(mx+1,self.costmap_data.shape[0]),max(my-1,0)],[min(mx+1,self.costmap_data.shape[0]),my],[min(mx+1,self.costmap_data.shape[0]),min(my+1,self.costmap_data.shape[1])]]
        return neighbours

    def isFrontierPoint(self,mx,my,rectangle):                     # Checks wheter a point is a frontier point by looking at the 8-neighbourhood. If a point in the neighbourhood is unexplored, returns true, else returns false
        if self.costmap_data[mx][my] != 0:
            return False
        # print("free space found")
        # print("neighbourhood: ", self.Neighbours(mx,my))
        for point in self.Neighbours(mx,my):
            # print(f"checking neighbour {point[0]} {point[1]}")
            if not self.isInRectangle(point[0],point[1],rectangle):
                # print("out of bounds")
                return False
            cost = self.costmap_data[point[0]][point[1]]
            if cost == -1:
                (f"found frontier point at coords {point[0]}{point[1]}")
                return True
            # else:
                #print(f"neighbour {point} not ")
        #print("free point in free neighbourhood")
        return False

    def isInRectangle(self,mx,my,rectangle):

            rect_x1, rect_y1, rect_x2, rect_y2 = rectangle
            # Converting rectangle boundaries to map coordinates
            m_x1, m_y1 = self.costmap_.worldToMap(rect_x1, rect_y1)
            m_x2, m_y2 = self.costmap_.worldToMap(rect_x2, rect_y2)

            # clip coordinates to map boundaries
            m_x1 = np.clip(m_x1, 0, self.costmap_.getSizeInCellsX()-1)
            m_y1 = np.clip(m_y1, 0, self.costmap_.getSizeInCellsY()-1)
            m_x2 = np.clip(m_x2, 0, self.costmap_.getSizeInCellsX()-1)
            m_y2 = np.clip(m_y2, 0, self.costmap_.getSizeInCellsY()-1)

            if m_x1 <= mx <= m_x2 and m_y1 <= my <= m_y2:
                return True
            return False

    def NaiveFrontierSearch(self,rectangle):                       # Returns a list of frontier points from a costmap stored as numpy array
        #frontier_pts = [[x[0], x[1]] for x in np.argwhere(np.vectorize(lambda x: isFrontierPoint(goalpoint_generator, x[0], x[1], costmap, rectangle) if costmap[x[0], x[1]] < 100 else False)(np.indices(costmap.shape)))]
        # lambda x : isFrontierPoint(x[0], x[1],costmap,rectangle)
        costmap = self.costmap_data
        boolean_frontier_array = np.zeros_like(costmap, dtype=bool)
        # print(f"costmap dimensions: x: {self.costmap_.getSizeInCellsX()} y: {self.costmap_.getSizeInCellsY()}")
        # print(f"costmap_data dimensions: x: {costmap.shape[0]} y: {costmap.shape[1]}")
        for x in range(costmap.shape[0]):
            # print(f"x axis: iteration number {x} out of {self.costmap_.getSizeInCellsX()}")
            for y in range(costmap.shape[1]):
                # print(f"y axis: iteration number {y} out of {self.costmap_.getSizeInCellsY()}")
                #print(f"point with coordinates {x},{y} has value {costmap[x][y]}")
                boolean_frontier_array[x,y] = self.isFrontierPoint(x,y,rectangle)
        frontier_pts = np.transpose(np.nonzero(boolean_frontier_array))
        print("frontier size: ", frontier_pts.shape)
        
        return frontier_pts

    def findClosestFrontier(self,frontier_pts, mx, my):
        print("frontier points: ", frontier_pts)

        reference_point = np.array([mx,my])
        threshold = 20

        distances = np.linalg.norm(frontier_pts - reference_point, axis=1)
        print("distance vector: ", distances)
        print("distance vector length: ", len(distances))
        above_thresh_idx = np.where(distances > threshold)[0]
        if len(above_thresh_idx) == 0:
            print("no values above the threshold")
            return None
        closest_idx = np.argmin(distances[above_thresh_idx])
        original_idx = above_thresh_idx[closest_idx]

        closest_point = frontier_pts[original_idx]

        return closest_point
        
        
def main():
    rclpy.init()

    generator = goalpoint_generator()
    rclpy.spin(generator)

    generator.navigator.lifecycleShutdown()
    generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
