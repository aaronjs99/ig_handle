#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import threading
import numpy as np

class OdomPlotter:
    def __init__(self):
        rospy.init_node('odom_plotter', anonymous=True)

        self.mocap_topic = rospy.get_param('~mocap_topic', '/motive/rigid_body_1/pose')
        self.dlio_topic = rospy.get_param('~dlio_topic', '/state/odometry')

        self.mocap_data = {'x': [], 'y': []}
        self.dlio_data = {'x': [], 'y': []}

        self.mocap_origin = None
        self.dlio_origin = None

        self.lock = threading.Lock()

        # Subscribers
        rospy.Subscriber(self.mocap_topic, PoseStamped, self.mocap_callback)
        rospy.Subscriber(self.dlio_topic, Odometry, self.dlio_callback)

        # Plotting setup
        self.fig, self.ax = plt.subplots()
        self.line_mocap, = self.ax.plot([], [], 'r-', label='Mocap')
        self.line_dlio, = self.ax.plot([], [], 'b-', label='DLIO')
        self.ax.legend()
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Real-time Odometry Comparison')
        self.ax.grid(True)
        self.ax.axis('equal')

    def mocap_callback(self, msg):
        with self.lock:
            x = msg.pose.position.x
            y = msg.pose.position.y

            if self.mocap_origin is None:
                self.mocap_origin = (x, y)
            
            self.mocap_data['x'].append(x - self.mocap_origin[0])
            self.mocap_data['y'].append(y - self.mocap_origin[1])

    def dlio_callback(self, msg):
        with self.lock:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            if self.dlio_origin is None:
                self.dlio_origin = (x, y)

            self.dlio_data['x'].append(x - self.dlio_origin[0])
            self.dlio_data['y'].append(y - self.dlio_origin[1])

    def update_plot(self, frame):
        with self.lock:
            self.line_mocap.set_data(self.mocap_data['x'], self.mocap_data['y'])
            self.line_dlio.set_data(self.dlio_data['x'], self.dlio_data['y'])

            # Rescale axes
            all_x = self.mocap_data['x'] + self.dlio_data['x']
            all_y = self.mocap_data['y'] + self.dlio_data['y']
            
            if all_x and all_y:
                min_x, max_x = min(all_x), max(all_x)
                min_y, max_y = min(all_y), max(all_y)
                
                margin = 1.0
                self.ax.set_xlim(min_x - margin, max_x + margin)
                self.ax.set_ylim(min_y - margin, max_y + margin)

        return self.line_mocap, self.line_dlio

    def run(self):
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)
        plt.show()

if __name__ == '__main__':
    try:
        plotter = OdomPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
