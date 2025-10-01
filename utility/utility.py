import os
from pathlib import Path

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

def tune_list(problem,max_tries=100,time_limit=60,all_params=None,defaults=None):
    tuner_param = GridSearchTuner("ortools", model=problem,all_params=all_params,defaults=defaults)
    best_param = tuner_param.tune_list(max_tries=max_tries,time_limit=time_limit)
    print('Tuning done')
    return best_param

