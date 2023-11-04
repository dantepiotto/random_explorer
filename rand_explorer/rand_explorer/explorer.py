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
from nav_msgs.msg import OccupancyGrid
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

def Neighbours(mx,my,costmap):                          # Returns the 8-neighbourhood of the input point, given in map coordinates, as a list of [x,y] map coordinates clipped to the map boundaries
    neighbours = [[max(mx-1,0),max(my-1,0)],[max(mx-1,0),my],[mx,min(my+1,costmap.size_y-1)],[mx,max(my-1,0)],[mx,my+1],[min(mx+1,costmap.size_x-1),max(my-1,0)],[min(mx+1,costmap.size_x-1),my],[min(mx+1,costmap.size_x-1),min(my+1,costmap.size_y-1)]]
    return neighbours

def isFrontierPoint(mx,my,costmap,rectangle):                     # Checks wheter a point is a frontier point by looking at the 8-neighbourhood. If a point in the neighbourhood is unexplored, returns true, else returns false
    for point in Neighbours(mx,my,costmap):
        if not isInRectangle(point[0],point[1],rectangle):
            return False

        cost = costmap.getCostXY(point[0],point[1])
        if cost == 255:
            return True
    return False

def isInRectangle(mx,my,rectangle):

        rect_x1, rect_y1, rect_x2, rect_y2 = rectangle
        # Converting rectangle boundaries to map coordinates
        m_x1, m_y1 = self.costmap_.worldToMap(rect_x1, rect_y1)
        m_x2, m_y2 = self.costmap_.worldToMap(rect_x2, rect_y2)

        # clip coordinates to map boundaries
        m_x1 = self.clip_mx(m_x1)
        m_y1 = self.clip_mx(m_y1)
        m_x2 = self.clip_mx(m_x2)
        m_y2 = self.clip_mx(m_y2)

        # Extract the relevant rectangle of the map (done on map converted to numpy matrix)
        map_region = self.costmap_data[m_y1:m_y2 +1, m_x1:m_x2+1]

        if m_x1 <= mx <= m_x2 and m_y1 <= my <= m_y2:
            return True
        return False

def NaiveFrontierSearch(costmap):                       # Returns a list of frontier points from a costmpa stored as numpy array
    frontier_pts = [[x[0], x[1]] for x in np.argwhere(np.vectorize(lambda x: isFrontierPoint(x[0], x[1], costmap, rectangle) if costmap[x[0], x[1]] < 100 else False)(np.indices(costmap.shape)))]
    return frontier_pts

def findClosestFrontier(frontier_pts, mx, my):
    if not frontier_pts:
        print("no Frontier points available")
        return None

    frontier_array = np.array(frontier_points)
    reference_point = np.array([mx,my])

    distances = np.linalg.norm(frontier_array - reference_point, axis=1)
    closest_index = np.argmin(distances)

    closest_point = frontier_points[closest_index]

    return closest_point


class goalpoint_generator(Node):

    def __init__(self):
        super().__init__('random_explorer')
        self.costmap_ = None
        self.costmap_data = None
        self.subscription = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.map_callback,qos.qos_profile_sensor_data)
        self.subscription = self.create_subscription(Empty, '/gopoint', self.generation_callback,1)
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.position_callback, 1)
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
        self.navigator.waitUntilNav2Active()

    def map_callback(self, map):
        print("I am the map callback!")
        self.costmap_ = PyCostmap2D(map) 
        self.costmap_data = np.array(map.data, dtype=np.int8).reshape((map.info.height, map.info.width))

    def position_callback(self, pose_msg):
        print("I am the pose callback!")
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
        rectangle = [-5,-1,15,19] # TODO fix with actual map data
        print("starting to generate a goalpoint")
        if self.state == 0:
            print("performing frontier detection ... \n")
            goal_point = findClosestFrontier(NaiveFrontierSearch(self.costmap_data),self.x,self.y)
            print("frontier point found with coords: ", goal_point)
            x,y = goal_point[0], goal_point[1]
            w = random.uniform(0,3.14)
        else:
            x,y,w = self.generate_random_coordinates(xrange,yrange)
            print(f"Generated coordinates: X={x:.2f}, Y={y:.2f}, W={w:.2f}")
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = w
        self.navigator.goToPose(goal_pose)
        i= 0
        while not self.navigator.isTaskComplete():
            time.sleep(1)
            i+=1 
            if i == 30: 
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
        exploredness, exploredness_partial = get_exploredness(rectangle)
        if exploredness > exploredness_thresh:
            self.state = 1                      # switch to roam mode if map is sufficiently explored


    
    def clip_mx(self,mx): #clip x coordinate to map boundaries
        mx = max(0, min(mx, self.costmap_.size_x - 1))
        return mx

    def clip_my(self,my): #clip y coordinate to map boundaries
        my = max(0, min(my, self.costmap_.size_y - 1))
        return my

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
        m_x1 = self.clip_mx(m_x1)
        m_y1 = self.clip_mx(m_y1)
        m_x2 = self.clip_mx(m_x2)
        m_y2 = self.clip_mx(m_y2)

        # Extract the relevant rectangle of the map (done on map converted to numpy matrix)
        map_region = self.costmap_data[m_y1:m_y2 +1, m_x1:m_x2+1]
        # Count points with value 255
        total_points = (m_x2-m_x1+1)*(m_y2-m_y1 +1)
        free_space_points = np.count_nonzero(map_region == 255)
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
        mx = self.clip_mx(mx)
        my = self.clip_my(my)
        cost = self.costmap_.getCostXY(mx,my) 
        rectangle = (0.0,0.0,10.0,20.0) # TODO change rectangle coords
        if distances
        if self.costmap_.getCostXY(mx,my) != 0 :
            print("point was cost higher than 0")
            return False
        else:
            return True
        
        
def main():
    rclpy.init()

    generator = goalpoint_generator()
    rclpy.spin(generator)

    navigator.lifecycleShutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
