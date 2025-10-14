import math
import bisect
import shutil
from datetime import datetime

from IPython.core.profileapp import list_profiles_in
import cpmpy as cp
from cpmpy import Model, SolverLookup
from cpmpy.exceptions import NotSupportedError
from cpmpy.solvers import CPM_ortools
from cpmpy.solvers.solver_interface import SolverInterface, ExitStatus
import time
import csv
import os
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from cpmpy.tools import GridSearchTuner, ParameterTuner
from utils.costant import FLASHLIGHT_CLIPPED, FLASHLIGHT_SCREWS
from utils.utility import ensure_directory_exists


class Tester:

    def __init__(self,problem_no_symmetries,problem_symmetries,timeout):
        self.problem_no_symmetries = problem_no_symmetries
        self.problem_symmetries = problem_symmetries
        self.timeout = timeout
        self.encountered_timeout_symmetries = False
        self.encountered_timeout_no_symmetries = False
        self.time_symmetries = []
        self.time_no_symmetries = []


    def test(self,range_item,params=None):
        for i in range(1,range_item):
            self.problem_no_symmetries.add_items_to_build(FLASHLIGHT_SCREWS, 1)
            self.problem_no_symmetries.add_items_to_build(FLASHLIGHT_CLIPPED, 1)
            self.problem_symmetries.add_items_to_build(FLASHLIGHT_SCREWS, 1)
            self.problem_symmetries.add_items_to_build(FLASHLIGHT_CLIPPED, 1)
        
            # Test symmetries problem
            self._test_problem(self.problem_symmetries, self.encountered_timeout_symmetries,
                              self.time_symmetries, params)

            # Test no-symmetries problem
            self._test_problem(self.problem_no_symmetries, self.encountered_timeout_no_symmetries,
                              self.time_no_symmetries, params)
        self.save_comparison_csv()
    
    def _test_problem(self, problem, timeout_flag, time_list, params):
        """Helper method to test a problem and handle timeout logic."""
        if not timeout_flag:
            problem.model_problem()
            solved, time, status = problem.solve(solver='ortools', params=params, timeout=self.timeout)
            timeout_flag = (status != ExitStatus.OPTIMAL)
            if timeout_flag:
                time_list.append(self.timeout)
            else:
                time_list.append(time)
        else:
            time_list.append(self.timeout)
        
        # Update the timeout flag in the calling scope
        if problem == self.problem_symmetries:
            self.encountered_timeout_symmetries = timeout_flag
        else:
            self.encountered_timeout_no_symmetries = timeout_flag
    
    def save_comparison_csv(self, output_dir="results", filename="comparisons.csv"):
        """
        Save the comparison results to a CSV file.
        
        Args:
            output_dir (str): Directory to save the CSV file (default: "results")
            filename (str): Name of the CSV file (default: "comparisons.csv")
        """
        # Create output directory if it doesn't exist
        ensure_directory_exists(output_dir)
        
        # Prepare data for CSV
        csv_data = []
        max_length = max(len(self.time_symmetries), len(self.time_no_symmetries))
        
        for i in range(max_length):
            # Since we add 2 items per iteration (FLASHLIGHT_SCREWS + FLASHLIGHT_CLIPPED), 
            # the actual number of items is (i + 1) * 2
            row = {
                'number_of_items': (i + 1) * 2,
                'time_symmetries': self.time_symmetries[i] if i < len(self.time_symmetries) else '',
                'time_no_symmetries': self.time_no_symmetries[i] if i < len(self.time_no_symmetries) else ''
            }
            csv_data.append(row)
        
        # Write to CSV file
        filepath = os.path.join(output_dir, filename)
        with open(filepath, 'w', newline='', encoding='utf-8') as csvfile:
            fieldnames = ['number_of_items', 'time_symmetries', 'time_no_symmetries']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            writer.writerows(csv_data)
        
        print(f"Comparison results saved to: {filepath}")
    
    def plot_comparison(self, output_dir="results", filename="comparison_plot.png", 
                       figsize=(12, 8), bar_width=0.35):
        """
        Create a bar chart comparing symmetries vs no-symmetries solving times.
        
        Args:
            output_dir (str): Directory to save the plot (default: "results")
            filename (str): Name of the plot file (default: "comparison_plot.png")
            figsize (tuple): Figure size (width, height) in inches (default: (12, 8))
            bar_width (float): Width of the bars (default: 0.35)
        """
        # Create output directory if it doesn't exist
        ensure_directory_exists(output_dir)
        
        # Prepare data
        max_length = max(len(self.time_symmetries), len(self.time_no_symmetries))
        # Since we add 2 items per iteration (FLASHLIGHT_SCREWS + FLASHLIGHT_CLIPPED), 
        # the actual number of items is iteration * 2
        items = [i * 2 for i in range(1, max_length + 1)]
        
        # Pad shorter list with None values for proper plotting
        time_sym = self.time_symmetries + [None] * (max_length - len(self.time_symmetries))
        time_no_sym = self.time_no_symmetries + [None] * (max_length - len(self.time_no_symmetries))
        
        # Create figure and axis
        fig, ax = plt.subplots(figsize=figsize)
        
        # Set up bar positions using actual item values
        x = np.array(items)
        
        # Create bars
        bars1 = ax.bar(x - bar_width/2, time_sym, bar_width, 
                      label='With Symmetries', alpha=0.8, color='skyblue')
        bars2 = ax.bar(x + bar_width/2, time_no_sym, bar_width,
                      label='Without Symmetries', alpha=0.8, color='lightcoral')
        
        # Customize the plot
        ax.set_xlabel('Number of Items', fontsize=12)
        ax.set_ylabel('Solving Time (seconds)', fontsize=12)
        ax.set_title('Symmetries vs No-Symmetries Solving Time Comparison', fontsize=14, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(items)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Add value labels on bars
        def add_value_labels(bars):
            for bar in bars:
                if bar.get_height() is not None:
                    height = bar.get_height()
                    ax.annotate(f'{height:.2f}',
                              xy=(bar.get_x() + bar.get_width() / 2, height),
                              xytext=(0, 3),  # 3 points vertical offset
                              textcoords="offset points",
                              ha='center', va='bottom', fontsize=8)
        
        add_value_labels(bars1)
        add_value_labels(bars2)
        
        # Adjust layout to prevent label cutoff
        plt.tight_layout()
        
        # Save the plot
        filepath = os.path.join(output_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"Comparison plot saved to: {filepath}")
        
        # Show the plot
        plt.show()
        
        return filepath
    
    @staticmethod
    def plot_from_csv(csv_filepath, output_dir="results", filename="comparison_plot_from_csv.png", 
                     figsize=(12, 8), bar_width=0.35):
        """
        Create a bar chart comparison plot from a previously saved CSV file.
        
        Args:
            csv_filepath (str): Path to the CSV file to read
            output_dir (str): Directory to save the plot (default: "results")
            filename (str): Name of the plot file (default: "comparison_plot_from_csv.png")
            figsize (tuple): Figure size (width, height) in inches (default: (12, 8))
            bar_width (float): Width of the bars (default: 0.35)
        """
        # Create output directory if it doesn't exist
        ensure_directory_exists(output_dir)
        
        # Read CSV data
        items = []
        time_sym = []
        time_no_sym = []
        
        try:
            with open(csv_filepath, 'r', newline='', encoding='utf-8') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    items.append(int(row['number_of_items']))
                    # Handle empty values
                    time_sym.append(float(row['time_symmetries']) if row['time_symmetries'] else None)
                    time_no_sym.append(float(row['time_no_symmetries']) if row['time_no_symmetries'] else None)
        except FileNotFoundError:
            print(f"Error: CSV file not found at {csv_filepath}")
            return None
        except Exception as e:
            print(f"Error reading CSV file: {e}")
            return None
        
        # Create figure and axis
        fig, ax = plt.subplots(figsize=figsize)
        
        # Set up bar positions using actual item values
        x = np.array(items)
        
        # Create bars
        bars1 = ax.bar(x - bar_width/2, time_sym, bar_width, 
                      label='With Symmetries', alpha=0.8, color='skyblue')
        bars2 = ax.bar(x + bar_width/2, time_no_sym, bar_width,
                      label='Without Symmetries', alpha=0.8, color='lightcoral')
        
        # Customize the plot
        ax.set_xlabel('Number of Items', fontsize=12)
        ax.set_ylabel('Solving Time (seconds)', fontsize=12)
        ax.set_title('Symmetries vs No-Symmetries Solving Time Comparison', fontsize=14, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(items)
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Add value labels on bars
        def add_value_labels(bars):
            for bar in bars:
                if bar.get_height() is not None:
                    height = bar.get_height()
                    ax.annotate(f'{height:.2f}',
                              xy=(bar.get_x() + bar.get_width() / 2, height),
                              xytext=(0, 3),  # 3 points vertical offset
                              textcoords="offset points",
                              ha='center', va='bottom', fontsize=8)
        
        add_value_labels(bars1)
        add_value_labels(bars2)
        
        # Adjust layout to prevent label cutoff
        plt.tight_layout()
        
        # Save the plot
        filepath = os.path.join(output_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        print(f"Comparison plot from CSV saved to: {filepath}")
        
        # Show the plot
        plt.show()
        
        return filepath

class BatchTuner:

    def __init__(self,params,default_vals):
        self.list_problems = []
        self.params = params
        self.default_vals = default_vals

    def add_problem(self,problem):
        self.list_problems.append(problem)

    def tune(self, max_tries=100, grid=False, time_limit=3600):
        if grid:
            tuner_grid = GridSearchTuner("ortools", self.list_problems)
            best_param = tuner_grid.tune(max_tries=max_tries, time_limit=time_limit)
        else:
            tuner_param = ParameterTuner("ortools", self.list_problems)
            best_param = tuner_param.tune(max_tries=max_tries, time_limit=time_limit)
        self.save_best_params(best_param)
    
    def save_best_params(self, best_params, output_dir="results", filename="best_tuning_params.json"):
        """
        Save the best tuning parameters to a JSON file.
        
        Args:
            best_params (dict): Dictionary containing the best tuning parameters
            output_dir (str): Directory to save the JSON file (default: "results")
            filename (str): Name of the JSON file (default: "best_tuning_params.json")
        """
        # Create output directory if it doesn't exist
        ensure_directory_exists(output_dir)
        
        # Write to JSON file
        filepath = os.path.join(output_dir, filename)
        # Convert all numpy.int64 -> Python int
        clean_params = {k: int(v) if isinstance(v, np.integer) else v
                        for k, v in best_params.items()}

        with open(filepath, 'w', encoding='utf-8') as jsonfile:
            json.dump(clean_params, jsonfile, indent=2, ensure_ascii=False)
        
        print(f"Best tuning parameters saved to: {filepath}")
        return filepath

class FWI:

    def __init__(self, jobshop, top_k, solutions_table=None,
                 solutions_storage=None, solution_counter=None):
        self.model = jobshop.m
        self.variables = jobshop.variables
        self.jobshop = jobshop
        self.top_k = top_k
        self.objectives_names = self.jobshop.objectives_names
        self.default_values = jobshop.default_values
        self.default_batches = self.create_batches_weights(self.default_values,
                                                      [self.variables[name] for name in self.objectives_names])
        
        # App integration
        self.solutions_table = solutions_table
        self.solutions_storage = solutions_storage
        self.solution_counter = solution_counter

    def generate_solution_id(self):
        """Generate a unique solution ID"""
        if self.solution_counter is not None:
            self.solution_counter[0] += 1
            return f"{self.solution_counter[0]:03d}"
        return "FWI_001"

    def save_solution_images(self, solution_id):
        """Save topology and solution images with solution ID"""
        if not self.jobshop:
            return None, None
            
        # Create solutions directory if it doesn't exist
        solutions_dir = "./solutions"
        os.makedirs(solutions_dir, exist_ok=True)
        
        # Copy topology image
        topology_src = "./images/topology.jpg"
        topology_dst = f"{solutions_dir}/topology_{solution_id}.jpg"
        if os.path.exists(topology_src):
            shutil.copy2(topology_src, topology_dst)
        
        # Copy solution image
        solution_src = "./images/sol.jpg"
        solution_dst = f"{solutions_dir}/solution_{solution_id}.jpg"
        if os.path.exists(solution_src):
            shutil.copy2(solution_src, solution_dst)
        
        return topology_dst, solution_dst

    def store_solution_data(self, solution_id, makespan, resources,conv_belts,employees, robots, topology_path, solution_path):
        """Store solution data with associated image paths"""
        if self.solutions_storage is not None:
            self.solutions_storage[solution_id] = {
                'id': solution_id,
                'makespan': makespan,
                'resources': resources,
                'conv_belts': conv_belts,
                'employees': employees,
                'robots': robots,
                'topology_path': topology_path,
                'solution_path': solution_path,
                'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }

    def add_solution_to_table(self, solution_id, makespan, resources,trasnport, employees, robots):
        """Add a new solution to the table (thread-safe)"""
        if self.solutions_table is not None:
            try:
                print(f"FWI: Adding solution to table: {solution_id}, {makespan}, {resources}, {trasnport}, {employees}, {robots}")
                # Use after() to schedule the table update on the main thread
                self.solutions_table.after(0, lambda: self.solutions_table.insert("", "end", values=(solution_id, makespan, resources, trasnport, employees, robots)))
                print("FWI: Successfully scheduled solution for table")
            except Exception as e:
                print(f"FWI: Error adding solution to table: {e}")

    def create_batches_weights(self,default_value, objectives):
        batch_weights = []
        times = math.floor((len(objectives)) / len(default_value))
        for i in range(times):
            batch_weights.append(default_value)
        remaining = ((len(objectives)) % len(default_value))
        if remaining > 0:
            batch_weights.append(default_value[-remaining:])
        return batch_weights

    def start_fwi(self):
        objectives = [self.variables[name] for name in self.objectives_names]
        #solve the problem one time

        
        #if self.model.solve():
        if self.general_solve(self.model, self.default_batches, objectives,[], store_solution=True):
            non_dominated_solution = [objective.value() for objective in objectives]
            non_dominated_solutions = [non_dominated_solution]
            depth = -1
            to_fix = [objectives[i].value() for i in range(len(objectives) - 2)]    #generate a list containing the value
                                                                                    # than can be fixed

            non_dominated_solutions, to_continue = self.fwi_method(self.model, objectives,
                                                                   self.variables,non_dominated_solution,
                                                                   non_dominated_solutions,depth,
                                                                   to_fix, self.top_k)


    def general_solve(self,model,batches_weights,objectives,new_constraints, store_solution=False):
        counter = 0
        constraints = []
        model_ortools = SolverLookup.get('ortools', model)
        for cons in new_constraints:
            model_ortools += cons
        for i in range(len(batches_weights)):
            batch = batches_weights[i]
            obj = np.sum(np.array(batch) * np.array(objectives[counter:(len(batch) + counter)]))
            model_ortools.minimize(obj)
            if model_ortools.solve():
                for obj in objectives[counter:(len(batch) + counter)]:
                    constraints.append(obj == obj.value())
                model_ortools += constraints
                counter += len(batch)
            else:
                return False
        
        # If this is a new solution and we should store it
        if store_solution and self.jobshop:
            self._store_new_solution()
        
        return True

    def _store_new_solution(self):
        """Store a new solution found by FWI"""
        try:
            # Generate solution ID
            solution_id = self.generate_solution_id()
            
            # Create Gantt chart for this solution
            if self.jobshop and hasattr(self.jobshop, 'make_gantt'):
                try:
                    self.jobshop.make_gantt(folder="./images/sol.jpg")
                    self.jobshop.make_ws_updated(location="./images/topology.jpg")
                except Exception as e:
                    print(f"FWI: Error creating Gantt chart: {e}")
            
            # Save images
            topology_path, solution_path = self.save_solution_images(solution_id)
            
            # Get solution data
            makespan = None
            if hasattr(self.jobshop, "makespan") and getattr(self.jobshop.makespan, "value", None):
                try:
                    v = self.jobshop.makespan.value()
                    makespan = int(v) if v is not None else None
                except Exception:
                    makespan = None
            
            ms_text = str(makespan) if makespan is not None else "-"
            
            # Get resource data (you may need to adapt this based on your problem structure)
            resources = 0
            employees = 0
            robots = 0
            transport = 0
            
            try:
                if hasattr(self.jobshop, "resource_count"):
                    resources = self.jobshop.resource_count.value() if hasattr(self.jobshop.resource_count, 'value') else 0
                if hasattr(self.jobshop, "humans_used"):
                    employees = int(self.jobshop.humans_used.value()) if hasattr(self.jobshop.humans_used, 'value') else 0
                if hasattr(self.jobshop, "robot_used"):
                    robots = int(self.jobshop.robot_used.value()) if hasattr(self.jobshop.robot_used, 'value') else 0
                if hasattr(self.jobshop, "robot_used"):
                    robots = int(self.jobshop.robot_used.value()) if hasattr(self.jobshop.robot_used, 'value') else 0
                if hasattr(self.jobshop, "tr_count"):
                    transport = int(self.jobshop.tr_count.value()) if hasattr(self.jobshop.tr_count, 'value') else 0
            except Exception as e:
                print(f"FWI: Error getting resource data: {e}")
            
            # Store solution data
            self.store_solution_data(solution_id, ms_text, resources,transport,
                                     employees, robots, topology_path, solution_path)
            
            # Add to table
            self.add_solution_to_table(solution_id, ms_text, resources,transport, employees, robots)
            
            print(f"FWI: Stored new solution {solution_id}")
            
        except Exception as e:
            print(f"FWI: Error storing new solution: {e}")


    def fwi_method(self, model, objectives, variables, current_solution, non_dominated_solutions, depth, to_fix, top_k):
        '''

        Args:
            cache_constraints: cache containing the constraints
            objectives: list of the objectives ordered according to the preferences
            current_solution: solution that I want to fix, worsen, improve
            non_dominated_solutions: list of non-dominated solutions
            depth: current level in the tree
            to_fix: list of the objectives, containing the fixed values
            top_k: int indicating how many solutions must be returned
            weights: weights of the objectives

        Returns:
           non_dominated_solutions: list of non-dominated solutions
        '''
        depth +=1
        #reach the lowest level of the tree
        if depth < len(objectives) - 2:
            non_dominated_solutions, to_continue = self.fwi_method(model, objectives, variables,
                                                                   current_solution, non_dominated_solutions,
                                                                   depth, to_fix, top_k)
            if not to_continue:
                return non_dominated_solutions, to_continue


        table_1 = self.make_new_table(non_dominated_solutions, current_solution, depth)
        table_worse_up_to = self.make_table_worse(table_1,current_solution[depth])
        i=0
        #iterate throughout the table
        while len(non_dominated_solutions) < top_k and i<len(table_worse_up_to)-1:
            new_constraints = []
            clause_fix = []
            for d in range(depth):
                clause_fix.append(objectives[d] == to_fix[d])
            clause_fix = cp.all(clause_fix)
            new_constraints.append(clause_fix)

            clause_fwi = []
            #worsen the objective, >= if the fixed objective value for the solution in the table is <
            fixed_values = [inner_list[-1] for inner_list in table_worse_up_to[i:]]
            if any((current_solution[:depth] == fixed) for fixed in fixed_values):
                clause_fwi.append(objectives[depth] >= table_worse_up_to[i][0])
            else:
                clause_fwi.append(objectives[depth] >= table_worse_up_to[i][0])
            worse_up_to_index = self.find_next_worse_index(table_worse_up_to,i)
            #the objective that I want to worsen, can be worsened up to...
            clause_fwi.append(objectives[depth] < table_worse_up_to[worse_up_to_index][0])
            new_constraints.append(cp.all(clause_fwi))
            table_improve = self.make_table_improve(table_1, table_worse_up_to[i][0])
            new_constraints.append(self.make_clause_fwi(objectives, depth, table_improve))
            batch_weights = self.create_batches_weights(self.default_values, objectives[depth:])


            if self.general_solve(model, batch_weights, objectives[depth:],
                                  new_constraints, store_solution=True):
                if depth < len(objectives) - 2:
                    for k in range(depth, len(objectives) - 2):
                        to_fix[k] = objectives[k].value()
                non_dominated_solution = [objective.value() for objective in objectives]
                non_dominated_solutions.append(non_dominated_solution)
                current_solution = non_dominated_solution

                if depth < len(objectives) - 2:
                    #if I am not in the lowest level, reach it
                    non_dominated_solutions, to_continue = self.fwi_method(model, objectives,
                                                                                       variables,current_solution,
                                                                                       non_dominated_solutions,depth,
                                                                                       to_fix, top_k)
                    if not to_continue:
                        return non_dominated_solutions, to_continue

                table_1 = self.make_new_table(non_dominated_solutions, current_solution, depth)
                table_worse_up_to = self.make_table_worse(table_1,current_solution[depth])
                i=0
            else:
                #if it is not unsatisfiable, skip solutions until the 'worse up to' one
                i=worse_up_to_index
        return non_dominated_solutions, True

    def make_clause_fwi(self, objectives, depth, table_2):
        '''

        Args:
            objectives: list of the objectives, ordered accoring preferences
            depth: current level of the tree
            table_2: table improving, generated by make_table_improve

        Returns:
            clause_fwi: clause for improving objectives (now the disjunctive method is used)
        '''
        clause_fwi = self.make_classic_disjunction(table_2, objectives[depth + 1:])
        return clause_fwi



    def make_classic_disjunction(self, table,objectives):
        '''

        Args:
            table: table_improve
            objectives:

        Returns:
            disjunction_classic: conjunction of disjunction (disjunctive method)
        '''
        disjunction_classic = []
        for row in table:
            part_disjunction = []
            for index_obj in range(len(objectives)):
                part_disjunction.append(objectives[index_obj] < row[index_obj])
            disjunction_classic.append(cp.any(part_disjunction))
        return cp.all(disjunction_classic)

    def make_table_worse(self, table,value):
        '''

        Args:
            table: table containing solutions with only the worsening part and the improving part,
                   plus the value of the objectives for the fixed part
            value: value of the objective that I want to worsen

        Returns:
            worse_up_to: table containing only the objective that we want to worsen,
                         plus the value of the objective for the fixed part

        '''
        column = [[el[0],el[-1]] for el in table]
        #added infinity at the end
        column.append([int(1e10),[]])
        #the table will contain only the values that are worsen wrt to value, since the table is already ordered,
        #I just need to get the rows after value
        column_for_index = [el[0] for el in table]
        worse_up_to = column[column_for_index.index(value):]
        return worse_up_to

    def find_next_worse_index(self, table,i):
        '''

        Args:
            table: table for worsening part
            i: current position in the table
        Returns:
            index_worse_up_to: index pointing the next worse value
                               ex: [15,15,15,20,infinity], if i = 0, index_worse_up_to = 3
        '''
        for index in range(i+1,len(table)):
            if table[index][0]!=table[i][0]:
                index_worse_up_to = index
                break
        return index_worse_up_to

    def make_new_table(self, solutions, current_sol, depth):
        '''
        Args:
            solutions: list containing all the non-dominated solutions
            current_sol: solution that we want to fix-worse-improve
            depth: current depth of the tree structure
            weights: list of weights

        Returns:
            dominate_sols: list of solutions without the fixed part.
                           At the end of each solution, there is also the objective value for the fixed part.
                           2 filters are applied (in the following ordering):
                                1: only the solution that dominate current_sol in the fixed part are added.
                                    ex: [10 50] 20 40 [sol in solutions]\
                                        [12 40] 30 100 [current_sol]
                                        since [10 50] do not dominate [12 40] I am not going to add [20 40] in dominate_sols
                                2: deleted dominated solutions by considering only the worsening part and the improving part
                                    ex: [9 15]   20 30
                                        [10 10]  30 40 [this will be deleted, is dominated by the first one,
                                                        without considering the fixed part in brackets]
        '''
        dominate_sols = []
        for sol in solutions:
                # find all solution that dominate current_sol in the fixed part, these will be put, first filtering
                if not any(current_sol[i] < sol[i] for i in range(depth)):
                    row = sol[depth:]
                    if depth == 0:
                        #if i am in the fist level of the tree, I do not have any fixed value
                        obj_fixed = []
                    else:
                        obj_fixed = [sol[i] for i in range(depth)]
                    #add as last element of the solution, the value of the objective for the fixed part
                    row.append(obj_fixed)
                    #add the solution in dominate_sols, ordered
                    index = bisect.bisect_left(dominate_sols, row)
                    dominate_sols.insert(index, row)
                    i = index
                    # delete all those solutions that are dominated, second filtering
                    while i < len(dominate_sols) - 1:
                        sol_1 = dominate_sols[i][:-1]
                        sol_2 = dominate_sols[i + 1][:-1]
                        if all(x <= y for x, y in zip(sol_1, sol_2)):
                            dominate_sols.pop(i + 1)
                            i -= 1
                        i += 1
        return dominate_sols

    def make_table_improve(self, table, worsen):
        '''

        Args:
            table: table from make_new table
            worsen: starting value for worsening

        Returns:
            table_improve: the table improvement contains the value that we want to improve using the disjunctive method
                           the solutions that are considered are those that have a better or equal value wrt worsen
                           1 filtering is applied
                                1: deleted dominated solutions by considering only the improving part

        '''
        table_improve = []
        for sol in table:
            #I take those solutions that have a better or equal value for the worsening part wrt worsen
            if sol[0] <= worsen:
                #add only the part that we want to improve, deleted worsening part and fixed objective value
                table_improve.append(sol[1:-1])
        table_improve.sort()
        i = 0
        while i < len(table_improve) - 1:
            #delete dominated solutions by considering the improving part
            sol_1 = table_improve[i]
            sol_2 = table_improve[i + 1]
            if all(x <= y for x, y in zip(sol_1, sol_2)):
                table_improve.pop(i + 1)
                i -= 1
            i += 1
        return table_improve



    def make_dictionary_from_table(self, table):
        dictionary = {}
        if len(table[0])==1:
            dictionary = table[0][0] #there only one element
        else:
            for inner_list in table:
                current_dict = dictionary
                for i, value in enumerate(inner_list):
                    if i == len(inner_list) - 2:
                        if value not in current_dict:
                            current_dict[value] = inner_list[-1]
                        break
                    else:
                        if value not in current_dict:
                            current_dict[value] = {}
                        current_dict = current_dict[value]
        return dictionary

