import cpmpy as cp


model = cp.Model()
model += ~cp.all([False])

model.solve()