from IPython.core.profileapp import list_profiles_in

from cpmpy import Model, SolverLookup
from cpmpy.exceptions import NotSupportedError
from cpmpy.solvers.solver_interface import SolverInterface, ExitStatus
import time
import csv
import os
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

from cpmpy.tools import GridSearchTuner, ParameterTuner
from utility.costant import FLASHLIGHT_CLIPPED, FLASHLIGHT_SCREWS
from utility.utility import ensure_directory_exists


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

