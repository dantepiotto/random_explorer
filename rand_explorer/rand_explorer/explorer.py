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
from rclpy.duration import Duration
import random
import numpy as np

"""
Basic navigation demo to go to pose.
"""
def generate_random_coordinates(xrange,yrange,goalpoint_generator):
    while True:
        x = random.uniform(0,xrange)
        y = random.uniform(0,yrange)
        if goalpoint_generator.is_good_point(x,y):
            w = random.uniform(0,3.14)
            return x,y,w


class goalpoint_generator:

    def ___init___(self):
        # self.map_data = None
        # self.map_resolution = None
        # self.map_width = None
        # self.map_height = None
        self.costmap_ = None
        self.costmap_data = None
        self.subscription = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.map_callback,10)
    def map_callback(self, map):
        # self.map_data = np.array(map.data, dtype=np.int8).reshape((map.info.height, map.info.width))
        # self.map_resolution = map.info.resolution
        # self.map_width = map.info.width
        # self.map_height = map.info.height
        self.costmap_ = PyCostmap2D(map) # TODO fix costmap is not properly read probs  ##### Think this works right now
        self.costmap_data = np.array(map.data, dtype=np.int8).reshape((map.info.height, map.info.width))

    def is_in_known_obst(self,mx,my): # TODO
        if mx == my:# write conditions to check whether coords are in a known obstacle
            return True
        return False
    
    def get_exploredness(self, rectangle):
        if self.costmap_ is None:
            print("Map data not available")
            return None
        rect_x1, rect_y1, rect_x2, rect_y2 = rectangle
        # Converting rectangle boundaries to map coordinates
        m_x1, m_y1 = self.costmap_.worldToMapValidated(rect_x1, rect_y1)
        m_x2, m_y2 = self.costmap_.worldToMapValidated(rect_x2, rect_y2)
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
            return False
        mx, my = self.costmap_.worldToMapValidated(x,y)
        cost = self.costmap_.getCostXY(mx,my) 
        rectangle = (1.0,1.0,3.0,3.0)
        if self.is_in_known_obst(mx,my):
            return False
        elif self.costmap_.getCostXY(mx,my) == 254:
            return False
        elif self.get_exploredness(rectangle) < 0.7 : # TODO set value once you have more map info
            return cost == 255
        else:
            return True
        
        

             


    
    


def main():
    rclpy.init()

    node = rclpy.create_node('explorer_node')
    generator = goalpoint_generator()
    # map_subscription = node.crate_subscription(OccupancyGrid, '/map', generator.map_callback,10)
    


    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0 # TODO change initial pose
    initial_pose.pose.position.y = 0
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    #####################################
    # RANDOM CHOICE OF NEXT POINT
    #####################################
    xrange = 10
    yrange = 20
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    try:
        while True:
            x,y,w = generate_random_coordinates(xrange,yrange,generator)
            print(f"Generated coordinates: X={x:.2f}, Y={y:.2f}, W={w:.2f}")
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.w = w
            navigator.goToPose(goal_pose)
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!') # TODO regulate timeout for goal reach
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
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
