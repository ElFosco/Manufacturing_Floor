from utility.visualization import visulize_schedule


class Nimbus:
    def __init__(self, mfModel):
        self.mfModel = mfModel
        self.sub_objectives = mfModel.get_sub_objectives()
        self.no_objectives = mfModel.get_no_objectives()
        self.utopian_points = []
        self.optimal_points = []
        self.nadir_points = []

    def get_objective_names(self):
        return self.mfModel.get_objective_names()

    def solve(self):
        self.mfModel.solve(solver='ortools',time_limit=60)
        location = visulize_schedule(self.mfModel)
        obj_values = [el.value() for el in self.sub_objectives]
        return location,obj_values


    def compute_optimal_utopian_nadir_points(self):

        for i in range(len(self.no_objectives)):
            for_optimal = [1 if j == i else 0 for j in range(len(self.no_objectives))]
            for_nadir = [-1 if j == i else 0 for j in range(len(self.no_objectives))]

            objectives = self.mfModel.define_new_objective_function(for_optimal)
            self.mfModel.solve()
            self.optimal_points.append(objectives[i].value())
            self.utopian_points.append(objectives[i].value() - 1e-3)

            objectives = self.mfModel.define_new_objective_function(for_nadir)
            self.mfModel.solve()
            self.nadir_points.append(objectives[i].value())

    # def solve_1(self):
    #
    # def solve_2(self):
    #
    # def solve_3(self):
    #
    # def solve_4(self):