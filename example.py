import gurobipy as gp
from gurobipy import GRB

print("Gurobi version:", gp.gurobi.version())

m = gp.Model("toy")

x = m.addVar(vtype=GRB.INTEGER, lb=0, ub=10, name="x")
y = m.addVar(vtype=GRB.INTEGER, lb=0, ub=10, name="y")

m.addConstr(x + y >= 5)
m.addConstr(x <= 3)

m.setObjective(x + 2 * y, GRB.MINIMIZE)
m.optimize()

print("status:", m.Status)
print("objective:", m.ObjVal)
print("x:", x.X)
print("y:", y.X)
