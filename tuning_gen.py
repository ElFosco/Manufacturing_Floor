from utils.jobshop_model_transport import FJTransportProblem
from utils.costant import *
from utils.utility_classes import BatchTuner


params = {  #'optimize_with_core' : [False, True],
            'search_branching': [0,1,2,3,4,5,6],
            'boolean_encoding_level' : [0,1,2,3],
            'linearization_level': [0, 1, 2],
            #'core_minimization_level' : [0,1,2],
            'cp_model_probing_level': [0, 1, 2, 3],
            'cp_model_presolve' : [False, True],
            # 'clause_cleanup_ordering' : [0,1],
            # 'binary_minimization_algorithm' : [0,1,2,3,4],
            # 'minimization_algorithm' : [0,1,2,3],
            'use_phase_saving' : [False, True],
            'symmetry_level': [1, 2, 3, 4],
            # 'use_dynamic_precedence_in_cumulative': [False, True],
            # 'use_overload_checker_in_cumulative':  [False, True],
            # 'use_timetable_edge_finding_in_cumulative': [False, True],
            # 'use_disjunctive_constraint_in_cumulative': [False, True]
        }

defaults = {
            #'optimize_with_core': False,
            'search_branching': 2,
            'boolean_encoding_level': 1,
            'linearization_level': 0,
            #'core_minimization_level': 2,
            'cp_model_probing_level': 2,
            'cp_model_presolve': True,
            # 'clause_cleanup_ordering': 0,
            # 'binary_minimization_algorithm': 1,
            # 'minimization_algorithm': 2,
            'use_phase_saving': True,
            'symmetry_level': 2,
            # 'use_dynamic_precedence_in_cumulative': False,
            # 'use_overload_checker_in_cumulative':  False,
            # 'use_timetable_edge_finding_in_cumulative': False,
            # 'use_disjunctive_constraint_in_cumulative': True
        }

tuner = BatchTuner(params=params,default_vals=defaults)


#first problem
problem = FJTransportProblem(symmetry_breaking=True)
problem.add_human()
problem.set_dur_hum(HUMAN_JOBS_TIME)
problem.add_robot()
problem.set_dur_robot(ROBOT_JOBS_TIME)

id_kit_1 = problem.add_workstation(WS_KITTING)
id_kit_2 = problem.add_workstation(WS_KITTING)
id_grip = problem.add_workstation(WS_BUILDING_1)
id_gs = problem.add_workstation(WS_BUILDING_2)
id_gs_2 = problem.add_workstation(WS_BUILDING_2)
id_pal = problem.add_workstation(WS_PALLETTING)
id_pal_2 = problem.add_workstation(WS_PALLETTING)

problem.add_transport(id_kit_1, id_grip)
problem.add_transport(id_grip, id_gs)
problem.add_transport(id_gs, id_pal)
problem.add_transport(id_kit_2, id_gs_2)
problem.add_transport(id_gs_2, id_pal_2)

problem.set_t_conv(3)
problem.add_items_to_build(FLASHLIGHT_CLIPPED,4)
problem.add_items_to_build(FLASHLIGHT_SCREWS,4)
problem.model_problem()

tuner.add_problem(problem.m)


#second problem
problem = FJTransportProblem(symmetry_breaking=True)
problem.add_human()
problem.set_dur_hum(HUMAN_JOBS_TIME)

id_kit_1 = problem.add_workstation(WS_KITTING)
id_kit_2 = problem.add_workstation(WS_KITTING)
id_kit_3 = problem.add_workstation(WS_KITTING)
id_grip = problem.add_workstation(WS_BUILDING_1)
id_gs = problem.add_workstation(WS_BUILDING_2)
id_grip_2 = problem.add_workstation(WS_BUILDING_1)
id_pal = problem.add_workstation(WS_PALLETTING)
id_pal_2 = problem.add_workstation(WS_PALLETTING)

problem.add_transport(id_kit_1, id_grip)
problem.add_transport(id_grip, id_gs)
problem.add_transport(id_gs, id_pal)
problem.add_transport(id_kit_2, id_grip_2)
problem.add_transport(id_gs_2, id_pal_2)
problem.add_transport(id_kit_3, id_grip_2)

problem.set_t_conv(3)
problem.add_items_to_build(FLASHLIGHT_CLIPPED,5)
problem.add_items_to_build(FLASHLIGHT_SCREWS,5)
problem.model_problem()

tuner.add_problem(problem.m)

#third problem
problem = FJTransportProblem(symmetry_breaking=True)
problem.add_human()
problem.set_dur_hum(HUMAN_JOBS_TIME)
problem.add_robot()
problem.set_dur_robot(ROBOT_JOBS_TIME)

id_kit_1 = problem.add_workstation(WS_KITTING)
id_grip = problem.add_workstation(WS_BUILDING_1)
id_gs = problem.add_workstation(WS_BUILDING_2)
id_gs_2 = problem.add_workstation(WS_BUILDING_2)
id_pal = problem.add_workstation(WS_PALLETTING)
id_pal_2 = problem.add_workstation(WS_PALLETTING)

problem.add_transport(id_kit_1, id_grip)
problem.add_transport(id_grip, id_gs)
problem.add_transport(id_gs, id_pal)
problem.add_transport(id_gs_2, id_pal_2)

problem.set_t_conv(3)
problem.add_items_to_build(FLASHLIGHT_CLIPPED,5)
problem.add_items_to_build(FLASHLIGHT_SCREWS,5)
problem.model_problem()

tuner.add_problem(problem.m)


#fourth problem
problem = FJTransportProblem(symmetry_breaking=True)
problem.add_human()
problem.set_dur_hum(HUMAN_JOBS_TIME)
problem.add_robot()
problem.set_dur_robot(ROBOT_JOBS_TIME)

id_kit_1 = problem.add_workstation(WS_KITTING)
id_kit_2 = problem.add_workstation(WS_KITTING)
id_grip = problem.add_workstation(WS_BUILDING_1)
id_gs = problem.add_workstation(WS_BUILDING_2)
id_pal = problem.add_workstation(WS_PALLETTING)
id_pal_2 = problem.add_workstation(WS_PALLETTING)

problem.add_transport(id_kit_1, id_grip)
problem.add_transport(id_grip, id_pal)
problem.add_transport(id_kit_2, id_gs)
problem.add_transport(id_gs, id_pal_2)

problem.set_t_conv(3)
problem.add_items_to_build(FLASHLIGHT_CLIPPED,5)
problem.add_items_to_build(FLASHLIGHT_SCREWS,5)
problem.model_problem()

tuner.add_problem(problem.m)


problem = FJTransportProblem(symmetry_breaking=True)
problem.add_human()
problem.set_dur_hum(HUMAN_JOBS_TIME)
problem.add_robot()
problem.set_dur_robot(ROBOT_JOBS_TIME)

id_kit_1 = problem.add_workstation(WS_KITTING)
id_kit_2 = problem.add_workstation(WS_KITTING)
id_grip = problem.add_workstation(WS_BUILDING_1)
id_gs = problem.add_workstation(WS_BUILDING_2)
id_gs_2 = problem.add_workstation(WS_BUILDING_2)
id_pal = problem.add_workstation(WS_PALLETTING)
id_pal_2 = problem.add_workstation(WS_PALLETTING)

problem.add_transport(id_kit_1, id_grip)
problem.add_transport(id_grip, id_pal)
problem.add_transport(id_grip, id_gs_2)
problem.add_transport(id_gs_2, id_pal_2)
problem.add_transport(id_kit_2, id_gs)
problem.add_transport(id_gs, id_pal_2)

problem.set_t_conv(3)
problem.add_items_to_build(FLASHLIGHT_CLIPPED,5)
problem.add_items_to_build(FLASHLIGHT_SCREWS,5)
problem.model_problem()

tuner.add_problem(problem.m)


tuner.tune(max_tries=None, verbose=2)
