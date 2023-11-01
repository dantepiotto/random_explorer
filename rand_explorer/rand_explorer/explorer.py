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
        # self.map_data = None
        # self.map_resolution = None
        # self.map_width = None
        # self.map_height = None
        super().__init__('random_explorer')
        self.costmap_ = None
        self.costmap_data = None
        self.subscription = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.map_callback,qos.qos_profile_sensor_data)
        self.subscription = self.create_subscription(Empty, '/gopoint', self.generation_callback,1)
        self.navigator = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0 # TODO change initial pose
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.z = 1.0
        self.initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()

    def map_callback(self, map):
        print("I am the callback!")
        # self.map_data = np.array(map.data, dtype=np.int8).reshape((map.info.height, map.info.width))
        # self.map_resolution = map.info.resolution
        # self.map_width = map.info.width
        # self.map_height = map.info.height
        self.costmap_ = PyCostmap2D(map) # TODO fix costmap is not properly read probs  ##### Think this works right now
        self.costmap_data = np.array(map.data, dtype=np.int8).reshape((map.info.height, map.info.width))

    def generate_random_coordinates(self,xrange,yrange):
        while True:
            x = random.uniform(0,2.0)
            y = random.uniform(0,2.0)
            #x = 1.1 
            #y = 1.0
            print("attempting point with coords",x,y)
            if self.is_good_point(x,y):
                w = random.uniform(0,3.14)
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
        m_x1 = max(0, min(m_x1, self.costmap_.size_x - 1))
        m_y1 = max(0, min(m_y1, self.costmap_.size_y - 1))
        m_x2 = max(0, min(m_x2, self.costmap_.size_x - 1))
        m_y2 = max(0, min(m_y2, self.costmap_.size_y - 1))

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
        # TODO in case of stupid coordinates
        cost = self.costmap_.getCostXY(mx,my) 
        rectangle = (1.0,1.0,3.0,3.0) # TODO change rectangle coords
        if self.is_in_known_obst(mx,my):
            print("point was in known obstacle")
            return False
        elif self.get_exploredness(rectangle) < 0.0 : # TODO set value once you have more map info
            print("map is unexplored, checking if point is uknown")
            return cost == 255
        elif self.costmap_.getCostXY(mx,my) != 0 :
            print("point was cost higher than 0")
            return False
        else:
            return True
        
        
def main():
    rclpy.init()

    #node = rclpy.create_node('explorer_node')
    generator = goalpoint_generator()
    # map_subscription = node.crate_subscription(OccupancyGrid, '/map', generator.map_callback,10)
    


    # navigator = BasicNavigator()

    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.0 # TODO change initial pose
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.orientation.z = 1.0
    # initial_pose.pose.orientation.w = 0.0
    # navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    # navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    #####################################
    # RANDOM CHOICE OF NEXT POINT
    #####################################
    # xrange = 10
    # yrange = 20
    # goal_pose = PoseStamped()
    # goal_pose.header.frame_id = 'map'
    # goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    rclpy.spin(generator)
    # rclpy.spin_once(generator,timeout_sec=None)
    # try:
    #     while True:
    #         x,y,w = generate_random_coordinates(xrange,yrange,generator)
    #         print(f"Generated coordinates: X={x:.2f}, Y={y:.2f}, W={w:.2f}")
    #         goal_pose.pose.position.x = x
    #         goal_pose.pose.position.y = y
    #         goal_pose.pose.orientation.w = w
    #         navigator.goToPose(goal_pose)
    #         result = navigator.getResult()
    #         i= 0
    #         while not navigator.isTaskComplete():
    #             time.sleep(1)
    #             i+=1 
    #             if i == 30: 
    #                 pass

    #         if result == TaskResult.SUCCEEDED:
    #             print('Goal succeeded!')
    #         elif result == TaskResult.CANCELED:
    #             print('Goal was canceled!')
    #         elif result == TaskResult.FAILED:
    #             print('Goal failed!')
    #         else:
    #             print('Goal has an invalid return status!') # TODO regulate timeout for goal reach
    #         print("about to spin")
    #         rclpy.spin_once(generator)
    # except KeyboardInterrupt:
    #     pass
    # Go to our demos first goal pose

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)
    

    # i = 0
    # while not navigator.isTaskComplete():
    #     ################################################
    #     #
    #     # Implement some code here for your application!
    #     #
    #     ################################################

    #     # Do something with the feedback
    #     i = i + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and i % 5 == 0:
    #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #               + ' seconds.')

    #         # Some navigation timeout to demo cancellation
    #         # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #         #     navigator.cancelTask()

    #         # Some navigation request change to demo preemption
    #         # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
    #         #     goal_pose.pose.position.x = -3.0
    #         #     navigator.goToPose(goal_pose)

    # Do something depending on the return code
    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
