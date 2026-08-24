import argparse
import hashlib
import time
from pathlib import Path

import gurobipy as gp
from gurobipy import GRB


DEFAULT_MPS_FILES = [
    "data/cp_model_results/gurobi/models/data_training_1_3_user_31_sym_lb_cumulative.mps",
    "data/cp_model_results/gurobi/models/data_training_1_3_user_31_sym_lb_nooverlap.mps",
]


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def status_name(status_code):
    names = {
        GRB.OPTIMAL: "OPTIMAL",
        GRB.INFEASIBLE: "INFEASIBLE",
        GRB.INF_OR_UNBD: "INF_OR_UNBD",
        GRB.UNBOUNDED: "UNBOUNDED",
        GRB.CUTOFF: "CUTOFF",
        GRB.ITERATION_LIMIT: "ITERATION_LIMIT",
        GRB.NODE_LIMIT: "NODE_LIMIT",
        GRB.TIME_LIMIT: "TIME_LIMIT",
        GRB.SOLUTION_LIMIT: "SOLUTION_LIMIT",
        GRB.INTERRUPTED: "INTERRUPTED",
        GRB.NUMERIC: "NUMERIC",
        GRB.SUBOPTIMAL: "SUBOPTIMAL",
        GRB.INPROGRESS: "INPROGRESS",
        GRB.USER_OBJ_LIMIT: "USER_OBJ_LIMIT",
    }
    return names.get(status_code, f"UNKNOWN_{status_code}")


def solve_mps(path, time_limit, threads, log):
    start = time.perf_counter()
    model = gp.read(str(path))
    read_s = time.perf_counter() - start

    model.Params.OutputFlag = 1 if log else 0
    if time_limit is not None:
        model.Params.TimeLimit = time_limit
    if threads is not None:
        model.Params.Threads = threads

    solve_start = time.perf_counter()
    model.optimize()
    solve_s = time.perf_counter() - solve_start

    result = {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
        "variables": model.NumVars,
        "constraints": model.NumConstrs,
        "nonzeros": model.NumNZs,
        "read_s": read_s,
        "solve_s": solve_s,
        "gurobi_runtime_s": model.Runtime,
        "status": status_name(model.Status),
        "solutions": model.SolCount,
        "objective": None,
        "objective_bound": None,
        "mip_gap": None,
    }

    if model.SolCount > 0:
        result["objective"] = model.ObjVal
    if model.IsMIP:
        result["objective_bound"] = model.ObjBound
        if model.SolCount > 0:
            result["mip_gap"] = model.MIPGap

    return result


def print_result(result):
    print(f"\n{result['path']}")
    print(f"  status: {result['status']}")
    print(f"  solutions: {result['solutions']}")
    print(f"  objective: {result['objective']}")
    print(f"  objective_bound: {result['objective_bound']}")
    print(f"  mip_gap: {result['mip_gap']}")
    print(
        "  model_size: "
        f"{result['variables']} vars, "
        f"{result['constraints']} constraints, "
        f"{result['nonzeros']} nonzeros"
    )
    print(f"  read_s: {result['read_s']:.3f}")
    print(f"  solve_s: {result['solve_s']:.3f}")
    print(f"  gurobi_runtime_s: {result['gurobi_runtime_s']:.3f}")
    print(f"  size_bytes: {result['size_bytes']}")
    print(f"  sha256: {result['sha256']}")


def print_checksum_groups(paths):
    groups = {}
    for path in paths:
        groups.setdefault(sha256_file(path), []).append(path)

    print("\nChecksum groups")
    for digest, group in groups.items():
        print(f"  {digest}")
        for path in group:
            print(f"    {path}")

    if len(groups) == 1 and len(paths) > 1:
        print("\nAll supplied files are byte-identical.")
    elif len(paths) > 1:
        print("\nSupplied files are not all byte-identical.")


def main():
    parser = argparse.ArgumentParser(
        description="Read and solve Gurobi MPS files, and report checksums."
    )
    parser.add_argument(
        "mps_files",
        nargs="*",
        default=DEFAULT_MPS_FILES,
        help="MPS files to verify. Defaults to the user 31 cumulative and nooverlap files.",
    )
    parser.add_argument(
        "--time-limit",
        type=float,
        default=600.0,
        help="Gurobi time limit per MPS file in seconds.",
    )
    parser.add_argument(
        "--threads",
        type=int,
        default=None,
        help="Optional Gurobi thread limit.",
    )
    parser.add_argument(
        "--checksums-only",
        action="store_true",
        help="Only compare SHA-256 checksums; do not optimize.",
    )
    parser.add_argument(
        "--log",
        action="store_true",
        help="Show Gurobi optimizer output.",
    )
    args = parser.parse_args()

    paths = [Path(p) for p in args.mps_files]
    missing = [path for path in paths if not path.exists()]
    if missing:
        raise FileNotFoundError("Missing MPS file(s): " + ", ".join(map(str, missing)))

    print_checksum_groups(paths)
    if args.checksums_only:
        return

    for path in paths:
        print_result(solve_mps(path, args.time_limit, args.threads, args.log))


if __name__ == "__main__":
    main()
