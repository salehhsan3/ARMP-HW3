import numpy as np
import argparse
from MapEnvironment import MapEnvironment
from RRTMotionPlanner import RRTMotionPlanner
from RRTInspectionPlannerNoBias import RRTInspectionPlanner
import time
import json
import matplotlib.pyplot as plt
import concurrent.futures


def plot_performance(goal_bias, avg_costs, avg_times, extension, task, coverage):
    plt.figure(figsize=(10, 6))

    # Plot average cost of path
    plt.subplot(2, 1, 1)
    plt.scatter(avg_times, avg_costs, marker='o')
    plt.xlabel('Average Time')
    plt.ylabel('Average Cost of Path')
    plt.title(f'Performance of RRTMotionPlanner with Goal Bias = {goal_bias}')
    plt.grid(True)

    plt.tight_layout()
    if task == 'mp':
        plt.savefig(f'mp-task\\bias={goal_bias}\\{extension}\\performance_plot_bias={goal_bias}_{extension}.png')
    else:
        plt.savefig(f'ip-task-no-bias\\{extension}\\coverage={coverage}\\performance_plot_{extension}_coverage={coverage}.png')

def run_with_timeout(planner, timeout):
    with concurrent.futures.ThreadPoolExecutor() as executor:
        future = executor.submit(planner.plan)
        try:
            plan = future.result(timeout=timeout)
            return plan
        except concurrent.futures.TimeoutError:
            print("Planner execution timed out.")
            return None

def main(args):
    goal_biases = [0.05, 0.2]
    extensions = ['E1', 'E2']
    coverages = [0.5, 0.75]
    all_combinations = [(goal_bias, ext, coverage) for goal_bias in goal_biases for ext in extensions for coverage in coverages]
    num_executions, succesful_runs = 10, 0

    for coverage, ext in [(coverage, ext) for coverage in coverages for ext in extensions]: # for inspection planning without bias
        print(coverage, ext)
        if coverage == 0.5 and ext == 'E1':
            continue # already computed
        if coverage == 0.5 and ext == 'E2':
            continue # already computed
        # if coverage == 0.75 and ext == 'E1':
        #     continue # algorithm doesnt converge!
        # if coverage == 0.75 and ext == 'E2':
        #     continue # algorithm doesnt converge!
        costs = []
        times = []

        for _ in range(num_executions):
            
            planning_env = MapEnvironment(json_file=args.map, task=args.task)

            if args.task == 'mp':
                planner = RRTMotionPlanner(planning_env=planning_env, ext_mode=ext, goal_prob=bias)
                save_dir = f'mp-task\\bias={bias}\\{ext}'
            elif args.task == 'ip':
                planner = RRTInspectionPlanner(planning_env=planning_env, ext_mode=ext, goal_prob=0.05, coverage=coverage)
                save_dir = f'ip-task-no-bias\\{ext}\\coverage={coverage}'
            else:
                raise ValueError('Unknown task option: %s' % args.task);

            start_time = time.time()
            
            plan = run_with_timeout(planner,timeout=1000) # dont allow runs for more than 1000 seconds
            execution_time = time.time() - start_time
            if plan != None:
                cost_of_path = planner.compute_cost(plan)
                planner.planning_env.visualize_plan(plan, save_dir)

                costs.append(cost_of_path)
                times.append(execution_time)
                succesful_runs += 1
            else:
                costs.append(0)
                times.append(0)

        avg_cost = sum(costs) / succesful_runs
        avg_time = sum(times) / succesful_runs
        if args.task == 'ip':
            plot_performance(0.05, costs, times, ext, 'ip', coverage)
        else:
            plot_performance(bias, costs, times, ext, 'mp', None)
        print(f"Goal Bias: {0.05 * 100}%")
        print(f"Average Cost of Path: {avg_cost}")
        print(f"Average Execution Time: {avg_time} seconds")

        output_data = {
            # "goal_bias": bias,
            "average_cost_of_path": avg_cost,
            "average_execution_time": avg_time
        }
        if args.task == 'mp':
            output_file = f"mp-task\\bias={bias}\\{ext}\\output_bias_{args.task}_bias={bias}_ext={ext}.json"
        else:
            output_file = f"ip-task-no-bias\\{ext}\\coverage={coverage}\\output_bias_coverage={coverage}_{args.task}_ext={ext}.json"
        with open(output_file, "w") as f:
            json.dump(output_data, f)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('-map', '--map', type=str, default='map_mp.json', help='Json file name containing all map information')
    parser.add_argument('-task', '--task', type=str, default='mp', help='choose from mp (motion planning) and ip (inspection planning)')
    parser.add_argument('-ext_mode', '--ext_mode', type=str, default='E1', help='edge extension mode')
    parser.add_argument('-goal_prob', '--goal_prob', type=float, default=0.05, help='probability to draw goal vertex')
    parser.add_argument('-coverage', '--coverage', type=float, default=0.5, help='percentage of points to inspect (inspection planning)')
    args = parser.parse_args()
    main(args)
