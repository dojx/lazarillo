#!/usr/bin/env python

import rospy
# ROS mensajes
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose2D
from lazarillo.node import Node
from nav_msgs.msg import OccupancyGrid
import tf
# Otros
import numpy as np
import cv2
from queue import PriorityQueue

class AStarServer():
    def __init__(self):
        self.sub = rospy.Subscriber('/lazarillo/goal', Pose2D, self.find_path)
        self.pub = rospy.Publisher('/lazarillo/path', Path, queue_size=1)

        self.grid = []
        self.x_off = 0
        self.y_off = 0
        self.res = 0
        self.img = []
        self.map = []
        self.kernel = 11

        self.listener = tf.TransformListener()

        self.get_map()

    def find_path(self, data):
        (trans, _) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))

        st_x = trans[0]
        st_y = trans[1]
        st_row, st_col = self.odom_2_pxl(st_x, st_y)
        ed_row, ed_col = self.odom_2_pxl(data.x, data.y)

        start = self.grid[st_row][st_col]
        end = self.grid[ed_row][ed_col]

        count = 0
        open_set = PriorityQueue()
        open_set.put((0, count, start)) # f-score, count, node
        came_from = {}
        g_score = {node: float("inf") for row in self.grid for node in row}
        g_score[start] = 0
        f_score = {node: float("inf") for row in self.grid for node in row}
        f_score[start] = self.heuristic(start, end)
        open_set_hash = {start}
        path = []

        while not open_set.empty():
            current = open_set.get()[2]
            open_set_hash.remove(current)

            if current == end:
                path.append(current)
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                break

            for neighbor, corner in current.neighbors:
                temp_g_score = g_score[current] + (14 if corner else 10)

                if temp_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] = temp_g_score + self.heuristic(neighbor, end)
                    if neighbor not in open_set_hash:
                        count += 1
                        open_set.put((f_score[neighbor], count, neighbor))
                        open_set_hash.add(neighbor)

        self.publish_path(path)
        # self.show_map(start, end, path)

    def publish_path(self, path):
        p = Path()
        p.header.frame_id = "map"

        for node in reversed(path):
            pose = PoseStamped()
            row, col = node.get_pos()
            x, y = self.pxl_2_odom(row, col)
            pose.pose.position.x = x
            pose.pose.position.y = y
            p.poses.append(pose)

        self.pub.publish(p)

    def heuristic(self, a, b):
        x1, y1 = a.get_pos()
        x2, y2 = b.get_pos()
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2) * 10

    def get_map(self):
        msg = rospy.wait_for_message('/rtabmap/grid_map', OccupancyGrid, timeout=30)
        self.x_off = msg.info.origin.position.x
        self.y_off = msg.info.origin.position.y
        self.res = msg.info.resolution

        print(self.x_off, self.y_off)

        aux = np.array(msg.data, dtype=np.uint8).reshape((msg.info.height, msg.info.width))
        aux[aux == 100] = 255
        kernel = np.ones((self.kernel, self.kernel), np.uint8)
        img = cv2.dilate(aux, kernel, iterations=1)
        self.make_grid(img) 
        self.img = img   
        self.map = aux  

    def make_grid(self, img):
        rows, cols = img.shape
        grid = []
        for i in range(rows):
            grid.append([])
            for j in range(cols):
                node = Node(i, j)
                if img[i, j]:
                    node.make_obstacle()
                grid[i].append(node)

        for row in grid:
            for node in row:
                node.update_neighbors(grid)

        self.grid = grid

    def show_map(self, start, end, path):
        inv_img = cv2.bitwise_not(self.img)
        img_rgb = cv2.cvtColor(inv_img, cv2.COLOR_GRAY2BGR)

        mask = self.img - self.map
        img_rgb[mask == 255] = (150, 0, 150)

        for node in path:
            img_rgb[node.get_pos()][:] = (0, 255, 0)

        img_rgb[start.get_pos()][:] = (255, 0, 0) # Blue
        img_rgb[end.get_pos()][:] = (0, 0, 255) # Red

        scale = 3
        big = cv2.resize(np.flipud(img_rgb), (0,0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST) 
        cv2.imshow('image', big)
        cv2.waitKey(0) 
        cv2.destroyAllWindows() 

    def odom_2_pxl(self, x, y):
        row = round((y - self.y_off) / self.res)
        col = round((x - self.x_off) / self.res)   
        return row, col

    def pxl_2_odom(self, row, col):
        x = self.x_off + (col * self.res)
        y = self.y_off + (row * self.res)
        return x, y

if __name__ == "__main__":
    rospy.init_node('a_star_server')
    server = AStarServer()
    try:
        rospy.spin()
    except Exception as e:
        print(e)   