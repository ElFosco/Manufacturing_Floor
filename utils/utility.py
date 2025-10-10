import os
from pathlib import Path

from matplotlib import pyplot as plt

from cpmpy.tools import ParameterTuner, GridSearchTuner


def solve_all_and_gantt(problem, solver='ortools', outdir="symmetries", max_solutions=None):
    """
    Enumerate all solutions of the *current* CPMpy model (problem.m),
    call problem.make_gantt() for each solution, and save under outdir/.

    Args:
        problem: your FJTransportProblem (already modeled via model_problem())
        solver:  solver name passed to problem.m.solve(...)
        outdir:  directory to save gantt outputs
        max_solutions: optional cap on number of solutions (None = all)

    Notes:
        - We build a 'nogood' after each solution using the current values of
          key decision vars (assignments + times) to exclude it.
        - If there are infinitely many time-symmetric variants in other models,
          your makespan objective already pushes to t=0 in your case, so this is safe.
    """
    Path(outdir).mkdir(parents=True, exist_ok=True)

    # A small helper to safely pull a value from a var (handles None)
    def _val(v):
        vv = v.value()
        return int(vv) if vv is not None else None

    # Collect a stable set of decision variables that uniquely determine a solution.
    # Using assignments + start times is usually enough (and safe).
    def collect_vars():
        vars_list = []

        # Processing assignment booleans
        vars_list += list(problem.x_h.values())
        vars_list += list(problem.x_a.values())

        # Transport choice booleans
        vars_list += list(problem.zT.values())

        # Operation start times
        vars_list += list(problem.S.values())

        # Transport start times
        vars_list += list(problem.S_T.values())

        # If you want even stricter nogoods, include OP_flag/H_flag and E/E_T:
        vars_list += list(problem.OP_flag.values())
        vars_list += list(problem.H_flag.values())
        vars_list += list(problem.E.values())
        vars_list += list(problem.E_T.values())

        # Also the makespan (ties down objective-equal solutions)
        vars_list.append(problem.makespan)

        # Filter out any None (shouldn't happen) and duplicates
        # (CPMpy vars are hashable; set() is fine)
        return list(dict.fromkeys([v for v in vars_list if v is not None]))

    # Build the initial variable list once (it won't change structurally)
    vars_list = collect_vars()

    sol_idx = 0
    while True:
        # Solve
        if not problem.m.solve(solver=solver):
            # No further solution
            if sol_idx == 0:
                print("No solution found.")
            else:
                print(f"All done. Enumerated {sol_idx} solution(s).")
            break

        # Found a solution -> export Gantt
        sol_idx += 1
        # Try to call your Gantt function with a meaningful filename if supported
        out_png = os.path.join(outdir, f"solution_{sol_idx:04d}.png")
        try:
            # If your make_gantt accepts a path/filename, pass it; otherwise it may save internally
            problem.make_gantt(out_png)
            print(f"Gantt saved: {out_png}")
        except TypeError:
            # Fallback: if make_gantt() takes no args, call it and hope it saves by itself
            problem.make_gantt()
            print(f"Gantt generated for solution {sol_idx} (filename not provided).")
        except Exception as e:
            print(f"Warning: make_gantt failed on solution {sol_idx}: {e}")

        # Stop if reached cap
        if max_solutions is not None and sol_idx >= max_solutions:
            print(f"Reached max_solutions={max_solutions}.")
            break

        # Build a nogood to exclude the current solution:
        #    sum( var == current_value ) <= N-1
        # This forces at least one var to take a different value next time.
        equals_literals = []
        for v in vars_list:
            vv = _val(v)
            # If some value is None (shouldn't happen after a successful solve), skip it
            if vv is None:
                continue
            equals_literals.append(v == vv)

        if not equals_literals:
            # Nothing to block? then we can't enumerate further safely
            print("No literals to build a nogood; stopping enumeration.")
            break

        problem.m += (sum(equals_literals) <= len(equals_literals) - 1)

def tune(problem,max_tries=100,grid=False,time_limit=60):
    if grid:
        tuner_grid = GridSearchTuner("ortools", problem)
        best_param = tuner_grid.tune(max_tries=max_tries,time_limit=time_limit)
    else:
        tuner_param = ParameterTuner("ortools", problem)
        best_param = tuner_param.tune(max_tries=max_tries,time_limit=time_limit)
    print('Tuning done')
    return best_param


def ensure_directory_exists(directory_path):
    """
    Create directory if it doesn't exist.

    Args:
        directory_path (str): Path to the directory to create
    """
    Path(directory_path).mkdir(parents=True, exist_ok=True)

def plot_comparison_sym_csv(csv_filepath, output_dir="results", filename="comparison_plot_from_csv.png",
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
    bars1 = ax.bar(x - bar_width / 2, time_sym, bar_width,
                   label='With Symmetries', alpha=0.8, color='skyblue')
    bars2 = ax.bar(x + bar_width / 2, time_no_sym, bar_width,
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

    def load_tuning_params(json_filepath):
        """
        Load tuning parameters from a JSON file.

        Args:
            json_filepath (str): Path to the JSON file to read

        Returns:
            dict: Dictionary containing the tuning parameters
        """
        try:
            with open(json_filepath, 'r', encoding='utf-8') as jsonfile:
                params = json.load(jsonfile)
            print(f"Tuning parameters loaded from: {json_filepath}")
            return params
        except FileNotFoundError:
            print(f"Error: JSON file not found at {json_filepath}")
            return None
        except json.JSONDecodeError as e:
            print(f"Error parsing JSON file: {e}")
            return None
        except Exception as e:
            print(f"Error reading JSON file: {e}")
            return None