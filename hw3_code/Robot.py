import itertools
import numpy as np
from matplotlib import pyplot as plt
from numpy.core.fromnumeric import size
from shapely.geometry import Point, LineString


def lines_colliding(line1,line2):
    p1 = line1[0]
    p2 = line1[1]
    p3 = line2[0]
    p4 = line2[1]
    uA = ((p4[0]-p3[0])*(p1[1]-p3[1]) - (p4[1]-p3[1])*(p1[0]-p3[0])) / ((p4[1]-p3[1])*(p2[0]-p1[0]) - (p4[0]-p3[0])*(p2[1]-p1[1]))
    uB = ((p2[0]-p1[0])*(p1[1]-p3[1]) - (p2[1]-p1[1])*(p1[0]-p3[0])) / ((p4[1]-p3[1])*(p2[0]-p1[0]) - (p4[0]-p3[0])*(p2[1]-p1[1]))
    return uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1

class Robot(object):
    
    def __init__(self):

        # define robot properties
        self.links = np.array([80.0,70.0,40.0,40.0])
        self.dim = len(self.links)

        # robot field of fiew (FOV) for inspecting points, from [-np.pi/6, np.pi/6]
        self.ee_fov = np.pi/3

        # visibility distance for the robot's end-effector. Farther than that, the robot won't see any points.
        self.vis_dist = 60.0

    def compute_distance(self, prev_config, next_config):
        '''
        Compute the euclidean distance betweeen two given configurations.
        @param prev_config Previous configuration.
        @param next_config Next configuration.
        '''
        # TODO: Task 2.2

        return np.linalg.norm(np.array(prev_config) - np.array(next_config))

    def compute_forward_kinematics(self, given_config):
        '''
        Compute the 2D position (x,y) of each one of the links (including end-effector) and return.
        @param given_config Given configuration.
        '''
        # TODO: Task 2.2

        positions = [(0,0)]
        link_angle = 0.0  # Initialize link angle
        for angle in given_config:
            # Calculate the x and y coordinates of the end of the link
            x = np.cos(link_angle) * self.links[len(positions)-1] + positions[-1][0]
            y = np.sin(link_angle) * self.links[len(positions)-1] + positions[-1][1]
            positions.append((x, y))
            # Update the link angle for the next link
            link_angle += angle

        return positions[1:]

    def compute_ee_angle(self, given_config):
        '''
        Compute the 1D orientation of the end-effector w.r.t. world origin (or first joint)
        @param given_config Given configuration.
        '''
        ee_angle = given_config[0]
        for i in range(1,len(given_config)):
            ee_angle = self.compute_link_angle(ee_angle, given_config[i])

        return ee_angle

    def compute_link_angle(self, link_angle, given_angle):
        '''
        Compute the 1D orientation of a link given the previous link and the current joint angle.
        @param link_angle previous link angle.
        @param given_angle Given joint angle.
        '''
        if link_angle + given_angle > np.pi:
            return link_angle + given_angle - 2*np.pi
        elif link_angle + given_angle < -np.pi:
            return link_angle + given_angle + 2*np.pi
        else:
            return link_angle + given_angle
        
    def validate_robot(self, robot_positions):
        '''
        Verify that the given set of links positions does not contain self collisions.
        @param robot_positions Given links positions.
        '''
        # TODO: Task 2.2
        
        links = [ pos for pos in robot_positions]

        # Iterate through each pair of adjacent links and check for intersection
        for i in range(len(links) - 3):
            for j in range(i + 2, len(links) - 1):
                line1 = [links[i], links[i + 1]]
                line2 = [links[j], links[j + 1]]
                if lines_colliding(line1,line2):
                    return False  # Collision detected
        return True
