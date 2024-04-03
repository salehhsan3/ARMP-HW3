import numpy as np
from RRTTree import RRTTree
import time

class RRTMotionPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        # TODO: Task 2.3
    
        self.tree.add_vertex(self.planning_env.start)
        while not self.tree.is_goal_exists(self.planning_env.goal):
            goal_bias = np.random.random()
            x_limit, y_limit = self.planning_env.xlimit, self.planning_env.ylimit
            if goal_bias < self.goal_prob: # correct way??
                random_state = self.planning_env.goal
            else:
                random_state = np.array([np.random.uniform(x_limit[0], x_limit[1]), np.random.uniform(y_limit[0], y_limit[1])])
            nearest_state_idx, nearest_state = self.tree.get_nearest_config(random_state)
            new_state = self.extend(nearest_state, random_state)
            if self.planning_env.state_validity_checker(new_state) and self.planning_env.edge_validity_checker(nearest_state, new_state):
                self.tree.add_vertex(new_state)
                self.tree.add_edge(nearest_state_idx, self.tree.get_idx_for_state(new_state), self.planning_env.compute_distance(nearest_state, new_state))
            
        # print total path cost and time
        curr_idx = self.tree.get_idx_for_state(self.planning_env.goal)
        start_idx = self.tree.get_idx_for_state(self.planning_env.start)
        while curr_idx != start_idx:
            plan.append(self.tree.vertices[curr_idx].state)
            curr_idx = self.tree.edges[curr_idx]

        # Add the start state to the plan.
        plan.append(self.planning_env.start)
        plan.reverse()
        
        # print total path cost and time
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

        return np.array(plan)

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps in the configuration space.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 2.3

        cost = 0
        for i in range(1, len(plan)):
            cost += self.planning_env.compute_distance(plan[i-1],plan[i])
        return cost

    def extend(self, near_config, rand_config):
        '''
        Compute and return a new configuration for the sampled one.
        @param near_config The nearest configuration to the sampled configuration.
        @param rand_config The sampled configuration.
        '''
        # TODO: Task 2.3

        n = 19 # a changeable parameter for step-size
        if self.ext_mode == "E1" or self.planning_env.compute_distance(near_config, rand_config) < n:
            return rand_config
        dist = self.planning_env.compute_distance(near_config, rand_config)
        normed_direction = (rand_config - near_config) / dist # normed vector
        new_state = near_config + (n * normed_direction)
        return new_state
    