#!/usr/bin/env python3
"""
Test script to debug the transport model setup
"""

from utility.jobshop_model_transport import FJTransportProblem
from utility.costant import HUMAN_JOBS_TIME, WS_KITTING, WS_BUILDING_1, WS_BUILDING_2, WS_PALLETTING, FLASHLIGHT_SCREWS

# Create the problem
problem = FJTransportProblem(ILP_formulation=False, ILP_opt=False, symmetry_breaking=True)

# Set horizon
problem.horizon = 100

try:
    print("Adding human worker...")
    problem.add_human()
    
    print("Setting human durations...")
    problem.set_dur_hum(HUMAN_JOBS_TIME)
    
    print("Setting arm durations...")
    problem.set_dur_arm({
        'K1': 15, 'K2': 8, 'B1': 25, 'B2': 18, 'P': 12
    })
    
    print("Adding workstations...")
    id_kit = problem.add_workstation(WS_KITTING)
    id_grip = problem.add_workstation(WS_BUILDING_1)
    id_gs = problem.add_workstation(WS_BUILDING_2)
    id_pal = problem.add_workstation(WS_PALLETTING)
    
    print(f"Workstation IDs: kit={id_kit}, grip={id_grip}, gs={id_gs}, pal={id_pal}")
    
    print("Adding transport connections...")
    problem.add_transport(id_kit, id_grip)
    problem.add_transport(id_grip, id_gs)
    problem.add_transport(id_gs, id_pal)
    
    print("Setting conveyor transport time...")
    problem.set_t_conv(3)
    
    print("Adding items to build...")
    problem.add_items_to_build(FLASHLIGHT_SCREWS, 1)
    
    print("Setting up the problem...")
    problem.model_problem()
    
    print("Solving...")
    solution = problem.solve()
    
    if solution:
        print(f"Solution found! Makespan: {problem.makespan.value()}")
    else:
        print("No solution found!")
        
except Exception as e:
    print(f"Error occurred: {e}")
    import traceback
    traceback.print_exc()
