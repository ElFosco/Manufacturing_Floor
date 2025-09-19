#!/usr/bin/env python3
"""
Corrected version of the transport model setup
"""

from utility.jobshop_model_transport import FJTransportProblem
from utility.costant import HUMAN_JOBS_TIME, WS_KITTING, WS_BUILDING_1, WS_BUILDING_2, WS_PALLETTING, FLASHLIGHT_SCREWS

# Create the problem
problem = FJTransportProblem(ILP_formulation=False, ILP_opt=False, symmetry_breaking=True)

# Set horizon (maximum time)
problem.horizon = 100

# Add human worker
problem.add_human()

# Set durations
problem.set_dur_hum(HUMAN_JOBS_TIME)
problem.set_dur_arm({
    'K1': 15, 'K2': 8, 'B1': 25, 'B2': 18, 'P': 12
})

# Add workstations
id_kit = problem.add_workstation(WS_KITTING)
id_grip = problem.add_workstation(WS_BUILDING_1)
id_gs = problem.add_workstation(WS_BUILDING_2)
id_pal = problem.add_workstation(WS_PALLETTING)

# Add transport connections
problem.add_transport(id_kit, id_grip)  # Kitting -> Assembly (Grip)
problem.add_transport(id_grip, id_gs)   # Assembly (Grip) -> Assembly (Grip & Screw)
problem.add_transport(id_gs, id_pal)    # Assembly (Grip & Screw) -> Packing

# Set conveyor transport time
problem.set_t_conv(3)

# Add items to build
problem.add_items_to_build(FLASHLIGHT_SCREWS, 1)

# Setup and solve the problem
print("Setting up the problem...")
problem.model_problem()

print("Solving...")
solution = problem.solve()

if solution:
    print(f"Solution found! Makespan: {problem.makespan.value()}")
    
    # Create visualizations
    print("Creating workstation topology...")
    problem.make_ws_topology()
    
    print("Creating Gantt chart...")
    problem.make_gantt()
    
    print("Done!")
else:
    print("No solution found!")
