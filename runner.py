import json

import time

from model_mf import MFModelCP, MFModelILP, MFSModelILP, MFSModelCP

with open("data/model.json") as json_file:
    data = json.load(json_file)



# model_cp = MFModelCP(data)
#
# for solver in ['ortools']:
#     start = time.time()
#     if model_cp.solve(solver=solver,time_limit=60):
#         print(model_cp.get_makespan())
#         end = time.time()
#         print(f'Solving for {solver} with CP formulation: {end-start} seconds')
#     else:
#         print(f'Timeout reached for {solver} with CP formulation')
#
#
# model_ilp = MFModelILP(data)
#
# for solver in ['gurobi','ortools','exact','choco']:
#     start = time.time()
#     if model_ilp.solve(solver=solver,time_limit=60):
#         print(model_ilp.get_makespan())
#         end = time.time()
#         print(f'Solving for {solver} with ILP formulation: {end-start} seconds')
#     else:
#         print(f'Timeout reached for {solver} with ILP formulation')



model_cp= MFSModelILP(data)

for solver in ['ortools']:
    start = time.time()
    if model_cp.solve(solver=solver,time_limit=30):
        print(model_cp.get_makespan())
        end = time.time()
        print(f'Solving for {solver} with CP formulation: {end-start} seconds')
    else:
        print(f'Solving for {solver} with CP formulation: {end - start} seconds')
        print(f'Timeout reached for {solver} with CP formulation')


