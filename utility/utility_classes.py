from cpmpy import Model, SolverLookup
from cpmpy.exceptions import NotSupportedError
from cpmpy.solvers.solver_interface import SolverInterface
import time

class SequentialModel(Model):
    """
    A composite model that holds multiple cpmpy.Model objects and
    solves them sequentially when solve() is called.

    solve() returns a list of per-model result dicts in order.
    """
    def __init__(self):
        super().__init__()
        self.submodels = []
        self.cpm_status = []

    def add_model(self, m: Model):
        self.submodels.append(m)


    def solve(self, solver=None, time_limit=None, **kwargs):
        """ Send the model to a solver and get the result.

            Run :func:`SolverLookup.solvernames() <cpmpy.solvers.SolverLookup.solvernames>` to find out the valid solver names on your system. (default: None = first available solver)

        Arguments:
            solver (string or a name in SolverLookup.solvernames() or a SolverInterface class (Class, not object!), optional):
                name of a solver to use.
            time_limit (list, optional): list of time limits in seconds for each isntance to be solved


        Returns:
            bool: the computed output:

            - True      if a solution is found (not necessarily optimal, e.g. could be after timeout)
            - False     if no solution is found
        """
        self.cpm_status = [None for _ in self.submodels]
        ret_list = []

        if kwargs and solver is None:
            raise NotSupportedError("Specify the solver when using kwargs, since they are solver-specific!")
        for index in range(len(self.submodels)):
            if isinstance(solver, SolverInterface):
                # for advanced use, call its constructor with this model
                s = solver(self.submodels[index])
            else:
                s = SolverLookup.get(solver,self.submodels[index])
            # call solver
            ret = s.solve(time_limit=time_limit[index], **kwargs)
            print('Solved submodel', index)
            # store CPMpy status (s object has no further use)
            self.cpm_status[index] = s.status()
            ret_list.append(ret)
        return ret_list
