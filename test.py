import cpmpy as cp



x = cp.boolvar(shape=(3))
y = cp.boolvar(shape=(3))



ha = cp.max(x)
hb = cp.max(y)
both_h = ha & hb



