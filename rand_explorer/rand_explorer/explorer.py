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

from geometry_msgs.msg import PoseStamped
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


class goalpoint_generator(Node):

    def __init__(self):
        super().__init__('random_explorer')
        self.costmap_ = None
        self.costmap_data = None
        self.subscription = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.map_callback,qos.qos_profile_sensor_data)
        self.subscription = self.create_subscription(Empty, '/gopoint', self.generation_callback,1)
        self.navigator = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0 # TODO change initial pose NEED MAP INFORMATION (might need to fix day before competition)
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.z = 1.0
        self.initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()

    def map_callback(self, map):
        print("I am the callback!")
        self.costmap_ = PyCostmap2D(map) 
        self.costmap_data = np.array(map.data, dtype=np.int8).reshape((map.info.height, map.info.width))

    def generate_random_coordinates(self,xrange,yrange):
        while True:
            x = random.uniform(0,2.0)
            y = random.uniform(0,2.0)
            x0 = self.initial_pose.pose.position.x #for writeability of code
            y0 = self.initial_pose.pose.position.y
            # keeping a log of the past 10 waypoints, in order to compute distance of potential waypoints with them to promote sufficient exploration
            past_positions = [[x0,y0],[x0,y0],[x0,y0],[x0,y0],[x0,y0],[x0,y0],[x0,y0],[x0,y0],[x0,y0],[x0,y0]]
            distances = np.linalg.norm(np.array(past_positinos) - np.array([x,y]), axis=1)
            print("attempting point with coords",x,y)
            if self.is_good_point(x,y):
                w = random.uniform(0,3.14)
                past_positions = [x,y] + past_positions[:-1]
                return x,y,w

    def generation_callback(self, msg):
        xrange = 10
        yrange = 20
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        print("starting to generate a goalpoint")
        x,y,w = self.generate_random_coordinates(xrange,yrange)
        print(f"Generated coordinates: X={x:.2f}, Y={y:.2f}, W={w:.2f}")
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = w
        self.navigator.goToPose(goal_pose)
        result = self.navigator.getResult()
        i= 0
        while not self.navigator.isTaskComplete():
            time.sleep(1)
            i+=1 
            if i == 30: 
                pass

        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!') # TODO regulate timeout for goal reach

    def is_in_known_obst(self,mx,my): # TODO
        if mx == my:# write conditions to check whether coords are in a known obstacle
            return True
        return False
    
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
        # convert map to numpy matrix to get appropriate results 

        # clip coordinates to map boundaries
        m_x1 = clip_mx(m_x1)
        m_y1 = clip_mx(m_y1)
        m_x2 = clip_mx(m_x2)
        m_y2 = clip_mx(m_y2)

        # Extract the relevant rectangle of the map
        map_region = self.costmap_data[m_y1:m_y2 +1, m_x1:m_x2+1]
        # Count points with value 255
        total_points = (m_x2-m_x1+1)*(m_y2-m_y1 +1)
        free_space_points = np.count_nonzero(map_region == 255)
        # calculate portion of uknown points
        exploredness = free_space_points/total_points

        return exploredness
        
    def is_good_point(self,x,y): # TODO
        if self.costmap_ is None:
            print("Map data not available")
            time.sleep(1)
            return False
        mx, my = self.costmap_.worldToMap(x,y)
        # TODO in case of stupid coordinates IDEA: clip to rw coordinates limits of map 
        mx = clip_mx(mx)
        my = clip_my(my)
        cost = self.costmap_.getCostXY(mx,my) 
        rectangle = (0.0,0.0,10.0,20.0) # TODO change rectangle coords
        if self.is_in_known_obst(mx,my):
            print("point was in known obstacle")
            return False
        elif self.get_exploredness(rectangle) < 0.0: # TODO set value once you have more map info
            print("map is unexplored, checking if point is uknown")
            return cost == 255
        elif self.costmap_.getCostXY(mx,my) != 0 :
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
