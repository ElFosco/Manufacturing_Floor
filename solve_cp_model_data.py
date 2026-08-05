import pandas as pd
import json
import os
import glob
import time
import argparse
from utils.jobshop_model_final import FJTransportProblemFinal

TIMING_COLUMNS = [
    'task_decomposition_s',
    'native_model_build_s',
    'solve_call_wall_s',
]


def solve_problem(json_file, weights, timeout, symmetry_breaking, custom_bound, solver, use_no_overlap=False, nadir_points=None):
    problem = FJTransportProblemFinal(custom_bound=custom_bound, json_file=json_file, symmetry_breaking=symmetry_breaking, use_no_overlap=use_no_overlap)

    decomposition_start = time.perf_counter()
    problem.make_model()
    decomposition_end = time.perf_counter()

    objectives = ['makespan', 'workstations', 'employees', 'robots', 'employee_time']
    weights_dict = dict(zip(objectives, weights))

    # Pass nadir_points and solver
    elapsed, status, objs, solve_timings = problem.solve(
        objectives=weights_dict,
        timeout=timeout,
        solver=solver,
        nadir_points=nadir_points,
        return_timings=True,
    )

    timings = {
        'task_decomposition_s': decomposition_end - decomposition_start,
        **solve_timings,
    }
    return elapsed, status, objs, timings

def main():
    parser = argparse.ArgumentParser(description="Solve problems from cp_model_data with user preferences.")
    parser.add_argument("-s", "--symmetry", action="store_true", help="Use symmetry breaking constraints")
    parser.add_argument("-l", "--lowerbound", action="store_true", help="Use custom lower bound (custom_bound)")
    parser.add_argument("-n", "--no_overlap", action="store_true", help="Use NoOverlap constraints instead of Cumulative")
    parser.add_argument("-v", "--solver", type=str, default="ortools", help="Solver to use (e.g., 'ortools', 'gurobi', 'z3')")
    parser.add_argument("-t", "--type", type=str, help="Type of training data to process (e.g., '1', '2', or '3'). If omitted, all types are processed.")
    args = parser.parse_args()

    # Load all user preferences
    try:
        user_preferences_full = pd.read_csv('data/user_preferences.csv')
        # Select indices 5-14 and 30-39
        target_indices = list(range(5, 15)) + list(range(30, 40))
        user_preferences = user_preferences_full.iloc[target_indices]
        print(f"Loaded {len(user_preferences)} targeted user preferences (indices 5-14 and 30-39).")
    except Exception as e:
        print(f"Error loading user_preferences.csv: {e}")
        return

    # Find JSON files in cp_model_data, optionally filtering by type
    if args.type:
        pattern = f'data/cp_model_data/data_training_{args.type}_*.json'
        print(f"Filtering for data type: {args.type}")
    else:
        pattern = 'data/cp_model_data/*.json'
    
    json_files = sorted(glob.glob(pattern))
    if not json_files:
        print(f"No JSON files found matching pattern: {pattern}")
        return
    print(f"Found {len(json_files)} JSON files to process.")

    # Ensure output directory exists
    output_dir = f'data/cp_model_results/{args.solver}'
    os.makedirs(output_dir, exist_ok=True)

    timeout = 600 # 10 minutes timeout for each solve

    for json_file in json_files:
        filename = os.path.basename(json_file).replace('.json', '')
        # Output to data/cp_model_results
        sym_str = "sym" if args.symmetry else "nosym"
        bound_str = "lb" if args.lowerbound else "nolb"
        overlap_str = "nooverlap" if args.no_overlap else "cumulative"
        output_csv = os.path.join(output_dir, f'solution_{filename}_{args.solver}_{sym_str}_{bound_str}_{overlap_str}.csv')
        
        # Remove existing file to start fresh (overwrite)
        if os.path.exists(output_csv):
            print(f"\nOutput file {output_csv} exists. Overwriting...")
            os.remove(output_csv)

        print(f"\n===== Solving for {json_file} (Solver: {args.solver}, Symmetry: {args.symmetry}, Custom Bound: {args.lowerbound}, Model: {overlap_str}) =====")

        for i, (index, row) in enumerate(user_preferences.iterrows()):
            weights = row.values
            actual_index = target_indices[i]
            
            print(f"  Preference {actual_index} weights={weights.tolist()}...")
            
            try:
                # Pass args.solver to solve_problem
                elapsed, status, objs, timings = solve_problem(json_file, weights, timeout, args.symmetry, args.lowerbound, args.solver, use_no_overlap=args.no_overlap)
                print(
                    f"    - Status: {status}, Time: {elapsed:.2f}s, "
                    f"Decomposition: {timings['task_decomposition_s']:.2f}s, "
                    f"Native build: {timings['native_model_build_s']:.2f}s, "
                    f"Solve call: {timings['solve_call_wall_s']:.2f}s"
                )
                result = {
                    'user_index': actual_index,
                    'weights': weights.tolist(),
                    'status': status,
                    'elapsed': elapsed,
                    'objectives': objs,
                    **timings,
                }
            except Exception as e:
                print(f"    - Error solving preference {actual_index}: {e}")
                result = {
                    'user_index': actual_index,
                    'weights': weights.tolist(),
                    'status': "ERROR",
                    'elapsed': 0,
                    'objectives': {},
                    **{col: 0 for col in TIMING_COLUMNS},
                }
            
            # Save result immediately after each solve (append to the freshly cleared file)
            df_result = pd.DataFrame([result])
            file_exists = os.path.isfile(output_csv)
            df_result.to_csv(output_csv, mode='a', index=False, header=not file_exists)
        
        print(f"Results for {filename} finalized in {output_csv}")

if __name__ == "__main__":
    main()
