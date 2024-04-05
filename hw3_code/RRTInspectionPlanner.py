import numpy as np
from RRTTree import RRTTree
import time

class RRTInspectionPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob, coverage):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env, task="ip")

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.coverage = coverage

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        # TODO: Task 2.4

        # Add the start point of the env to the plan 
        self.tree.add_vertex(self.planning_env.start, self.planning_env.get_inspected_points(self.planning_env.start))
        while self.tree.max_coverage < self.coverage:
            #goal_bias = np.random.random()
            #if goal_bias < self.goal_prob: 
            #    random_config = self.tree.vertices[self.tree.max_coverage_id].config
            #else:
            
            # Generate a random config to visit
            random_config = np.array([np.random.uniform(-np.pi,np.pi) for _ in range(self.planning_env.robot.dim)])

            # Get from the tree the nearest state index to the generated point
            goal_bias = np.random.random()
            if goal_bias < self.goal_prob: 
                nearest_state_idx, nearest_config = self.tree.max_coverage_id, self.tree.vertices[self.tree.max_coverage_id].config
            else:
                nearest_state_idx, nearest_config = self.tree.get_nearest_config(random_config)
            new_config = self.extend(nearest_config, random_config)

            # Ensure the new config is legal and the edge exists
            if not (self.planning_env.config_validity_checker(new_config) or not (self.planning_env.edge_validity_checker(nearest_config, new_config))):
                continue
            
            # Calculate the new state (combine the inspection points)
            nearest_inspection_points = self.tree.vertices[nearest_state_idx].inspected_points
            new_inspection_points = self.planning_env.get_inspected_points(new_config)
            new_inspection_points = self.planning_env.compute_union_of_points(new_inspection_points, nearest_inspection_points)

            # Add the vertex to the tree
            new_id = self.tree.add_vertex(new_config, new_inspection_points)
            self.tree.add_edge(nearest_state_idx, new_id, self.planning_env.robot.compute_distance(nearest_config, new_config))

        # print total path cost and time
        curr_idx = self.tree.max_coverage_id
        start_idx = self.tree.get_root_id()

        while curr_idx != start_idx:
            plan.append(self.tree.vertices[curr_idx].config)
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
        # TODO: Task 2.4

        cost = 0
        for i in range(1, len(plan)):
            cost += self.planning_env.robot.compute_distance(plan[i-1],plan[i])
        return cost

    def extend(self, near_config, rand_config):
        '''
        Compute and return a new configuration for the sampled one.
        @param near_config The nearest configuration to the sampled configuration.
        @param rand_config The sampled configuration.
        '''
        # TODO: Task 2.4

        n = 19 # a changeable parameter for step-size
        if self.ext_mode == "E1" or self.planning_env.robot.compute_distance(near_config, rand_config) < n:
            return rand_config
        dist = self.planning_env.robot.compute_distance(near_config, rand_config)
        normed_direction = (rand_config - near_config) / dist # normed vector
        new_state = near_config + (n * normed_direction)
        return new_state

    