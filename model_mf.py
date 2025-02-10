import itertools
from collections import defaultdict
import cpmpy as cp
from abc import ABC, abstractmethod

import numpy as np

from utils import get_key_dict


class MFModel(ABC):

  def get_dv(self):
    return self.start,self.duration

  def get_makespan(self):
    return max(np.concatenate([self.start[dv].value() + self.duration[dv].value() for key, dv in self.dict_product_dv.items()]))

  def get_jobs(self):
    return self.jobs

  def get_info(self):
    return self.info


  def define_dicts(self):
    index_dv = 0
    dict_state_job = defaultdict(list)
    dict_product_dv = defaultdict(list)
    info = {}

    for job, job_info in self.jobs.items():
      id = job_info['__id__']
      resource = job_info['properties']['Resource']['value']
      action = job_info.get('properties', {}).get('State', {}).get('value', None)
      product_instance = job_info['properties']['Product_instance']['value']
      job_name = job_info['name']
      if resource == 'Cell_0_Staubli':
        dict_state_job[action].append(index_dv)
      info[id] = [index_dv, resource, product_instance, action, job_name]
      dict_product_dv[product_instance].append(index_dv)
      index_dv += 1

    return info, index_dv, dict_state_job, dict_product_dv

  def define_ub(self):
    ub = 0
    for job, job_info in self.jobs.items():
      val = job_info['properties'][f'{job}_duration']['value']
      if val != None:
        ub += val
    return ub

  def define_dv(self,):
    start = cp.intvar(0, self.ub, shape=self.index_dv, name="start")
    duration = cp.intvar(0, self.ub, shape=self.index_dv, name="duration")
    return start, duration

  def add_constraints(self):
    self.add_fixed_duration()
    for constr, constr_info in self.constraint.items():
      if constr_info['__type__'] == 'HasNoOverlap':
        self.add_no_overlap(constr,constr_info)
      elif constr_info['__type__'] == 'IsRightAfter':
        self.add_right_after(constr_info)
      elif constr_info['__type__'] == 'IsSynced':
        self.add_synced(constr_info)
      elif constr_info['__type__'] == 'IsRightBefore':
        self.add_right_before(constr_info)
    self.add_obj_function()

  def add_fixed_duration(self):
    for job, job_info in self.jobs.items():
      val = job_info['properties'][f'{job}_duration']['value']
      if val != None:
        self.model += (self.duration[self.info[job_info['__id__']][0]] == val)


  def add_right_after(self,constr_info):
    first_subject = self.info[constr_info['first']['__id__']][0]
    second_subject = self.info[constr_info['second']['__id__']][0]
    self.model += ((self.start[second_subject] + self.duration[second_subject]) == self.start[first_subject])

  def add_right_before(self,constr_info):
    first_subject = self.info[constr_info['first']['__id__']][0]
    second_subject = self.info[constr_info['second']['__id__']][0]
    self.model += ((self.start[first_subject] + self.duration[first_subject]) == self.start[second_subject])

  def add_synced(self,constr_info):
    first_subject = self.info[constr_info['first']['__id__']][0]
    second_subject = self.info[constr_info['second']['__id__']][0]
    self.model += (self.start[first_subject] == self.start[second_subject])
    self.model += (self.duration[first_subject] == self.duration[second_subject])


  def solve(self,solver,time_limit=60):
    return self.model.solve(solver=solver,time_limit=time_limit)


  @abstractmethod
  def add_obj_function(self):
    pass

  @abstractmethod
  def add_no_overlap(self,constr,constr_info):
    pass


class MFModelCP(MFModel):
  def __init__(self, data):
    self.jobs = data['nodes']
    self.constraint = data['edges']
    self.ub = self.define_ub()
    self.info, self.index_dv, self.dict_state_job,self.dict_product_dv = self.define_dicts()
    self.start,self.duration = self.define_dv()
    self.model = cp.Model()
    self.dict_resource_job = defaultdict(set)
    self.add_constraints()

  def add_constraints(self):
    super().add_constraints()
    self.add_cumulative_constraint()


  def add_no_overlap(self,constr,constr_info):
    first_subject = self.info[constr_info['first']['__id__']][0]
    resource = self.info[constr_info['first']['__id__']][1]
    second_subject = self.info[constr_info['second']['__id__']][0]
    second_resource = self.info[constr_info['second']['__id__']][1]
    self.dict_resource_job[resource].add(first_subject)
    self.dict_resource_job[second_resource].add(second_subject)

  def add_cumulative_constraint(self):
    for resource, jobs_same_res in self.dict_resource_job.items():
      start_no_overlap = [self.start[i] for i in jobs_same_res]
      duration_no_overlap = [self.duration[i] for i in jobs_same_res]
      end_no_overlap = [self.start[i] + self.duration[i] for i in jobs_same_res]
      self.model += cp.Cumulative(start_no_overlap, duration_no_overlap, end_no_overlap,demand=1,capacity=1)


  def add_obj_function(self):
    obj = []
    for key, dv in self.dict_product_dv.items():
      obj.append(self.start[dv] + self.duration[dv])
    self.model.minimize(cp.max(obj))





class MFSModelCP(MFModelCP):


  def __init__(self, data,max_switches=20,duration_switches=40):
    self.max_switches = max_switches
    self.duration_switches = duration_switches
    self.jobs = data['nodes']
    self.constraint = data['edges']
    self.ub = self.define_ub()
    self.start_transition, self.duration_transition = self.define_states_dv()
    self.info, self.index_dv, self.dict_state_job, self.dict_product_dv = self.define_dicts()
    self.start, self.duration = self.define_dv()
    self.model = cp.Model()
    self.dict_resource_job = defaultdict(set)
    self.add_constraints()


  def add_constraints(self):
    super().add_constraints()
    self.add_constraints_state()


  def define_states_dv(self):
    start_transition = cp.intvar(0, self.ub, shape=self.max_switches,name='start_transition')
    duration_transition = cp.intvar(0, self.duration_switches, shape=self.max_switches,name='duration_transition')
    return start_transition,duration_transition


  def add_constraints_state(self):
    states_staubli = len(self.dict_state_job.keys())
    jobs_staubli = sum(len(lst) for lst in self.dict_state_job.values())
    list_jobs_id = [item for lst in self.dict_state_job.values() for item in lst]


    T = cp.boolvar(shape=(states_staubli, states_staubli, self.max_switches))  # ok
    P = cp.intvar(0, self.max_switches, shape=jobs_staubli)  # ok
    M = cp.boolvar(shape=(self.max_switches, jobs_staubli))  # ok


    # Only one switch is feasible at a time #15                         #OK
    for n in range(self.max_switches):
      self.model += (sum(T[trans_1, trans_2, n] for trans_1, trans_2 in
                         itertools.product(range(states_staubli), repeat=2)) <= 1)

    # Continuity of the switches #16                                     #OK
    for n in range(self.max_switches - 1):
      for given_state in range(states_staubli):
        self.model += (sum(T[any_state_1, given_state, n] for any_state_1 in range(states_staubli)) == sum(
          T[given_state, any_state_2, n + 1] for any_state_2 in range(states_staubli)))

    # After how many transitions an operation is executed #17.1         #OK
    for job in range(jobs_staubli):
      id_job = list_jobs_id[job]
      self.model += (sum(self.start_transition[n] < self.start[id_job] for n in range(self.max_switches)) == (P[job]))

    # 17.2                                                               #ok
    for job in range(jobs_staubli):
      self.model += (sum(M[n, job] for n in range(self.max_switches)) == 1)

    # # #18                                                               #ok
    for job in range(jobs_staubli):
      self.model += (sum(n * M[n, job] for n in range(self.max_switches)) == P[job])

    # # #19                                                               #ok
    for job in range(jobs_staubli):
      for n in range(self.max_switches):
        resource_job = get_key_dict(self.dict_state_job, list_jobs_id[job])
        index_resource = sorted(list(self.dict_state_job.keys())).index(resource_job)
        self.model += (M[n, job] <= sum(T[state, index_resource, n] for state in range(states_staubli)))

    # #20                                                                 #ok
    for n in range(self.max_switches):
      self.model += (self.duration_transition[n] == sum(T[trans_1, trans_2, n] * self.duration_switches for trans_1, trans_2 in
                                                        itertools.product(range(states_staubli), repeat=2)))


    # #21
    for n in range(self.max_switches - 1):
      self.model += self.start_transition[n] < self.start_transition[n + 1]



  def add_cumulative_constraint(self):
    for resource, jobs_same_res in self.dict_resource_job.items():
      start_no_overlap = [self.start[i] for i in jobs_same_res]
      duration_no_overlap = [self.duration[i] for i in jobs_same_res]
      end_no_overlap = [self.start[i]+self.duration[i] for i in jobs_same_res]
      if resource == 'Cell_0_Staubli':
        #added states
        start_no_overlap_transition = [el for el in self.start_transition]
        duration_no_overlap_transition = [el for el in self.duration_transition]
        end_no_overlap_transition = [self.start_transition[index] + self.duration_transition[index]
                                     for index in range(self.max_switches)]
        #added jobs
        start_no_overlap = start_no_overlap + start_no_overlap_transition
        duration_no_overlap = duration_no_overlap + duration_no_overlap_transition
        end_no_overlap = end_no_overlap + end_no_overlap_transition

      self.model += cp.Cumulative(start_no_overlap, duration_no_overlap, end_no_overlap,demand=1,capacity=1)


class MFModelILP(MFModel):

  def __init__(self, data):
    self.jobs = data['nodes']
    self.constraint = data['edges']
    self.ub = self.define_ub()
    self.info, self.index_dv, self.dict_state_job,self.dict_product_dv = self.define_dicts()
    self.start,self.duration = self.define_dv()
    self.model = cp.Model()
    self.dict_resource_job = defaultdict(set)
    self.add_constraints()

  def add_constraints(self):
    self.add_def_precendes()
    super().add_constraints()


  def add_def_precendes(self):
    self.precedences = cp.boolvar(shape=(self.index_dv, self.index_dv), name="precedences")
    for (comb_1, comb_2) in (itertools.combinations(range(self.index_dv), 2)):
      self.model += (self.precedences[comb_1, comb_2] + self.precedences[comb_2, comb_1] == 1)
      self.model += ((self.start[comb_2] + self.duration[comb_2] - self.ub * self.precedences[comb_1, comb_2])
                     <= self.start[comb_1])

  def add_no_overlap(self,constr,constr_info):
    first_subject = self.info[constr_info['first']['__id__']][0]
    resource = self.info[constr_info['first']['__id__']][1]
    second_subject = self.info[constr_info['second']['__id__']][0]
    second_resource = self.info[constr_info['second']['__id__']][1]
    self.model += ((self.start[first_subject] + self.duration[first_subject] - self.ub * self.precedences[second_subject, first_subject]) <=
              self.start[second_subject])
    self.dict_resource_job[resource].add(first_subject)
    self.dict_resource_job[second_resource].add(second_subject)

  def add_obj_function(self):
    makespan = cp.intvar(0,self.ub,name="makespan")
    for i in range(self.index_dv):
        self.model += (self.start[i] + self.duration[i] <= makespan)
    self.model.minimize(makespan)




class MFSModelILP(MFModelILP):

  def __init__(self, data,max_switches=20,duration_switches=40):
    self.max_switches = max_switches
    self.duration_switches = duration_switches
    self.jobs = data['nodes']
    self.constraint = data['edges']
    self.ub = self.define_ub()
    self.info, self.index_dv, self.dict_state_job, self.dict_product_dv = self.define_dicts()
    self.start, self.duration = self.define_dv()
    self.model = cp.Model()
    self.dict_resource_job = defaultdict(set)
    self.add_constraints()


  def define_dv(self,):
    start = cp.intvar(0, self.ub, shape=self.index_dv+self.max_switches, name="start")
    duration = cp.intvar(0, self.ub, shape=self.index_dv+self.max_switches, name="duration")
    return start, duration



  def add_def_precendes(self):
    #we can reduce this to the jobs running in the same resource
    self.precedences = cp.boolvar(shape=(self.index_dv+self.max_switches, self.index_dv+self.max_switches),
                                  name="precedences")

    for (comb_1, comb_2) in (itertools.combinations(range(self.index_dv+self.max_switches), 2)):
      self.model += (self.precedences[comb_1, comb_2] + self.precedences[comb_2, comb_1] == 1)
      self.model += ((self.start[comb_2] + self.duration[comb_2] - self.ub * self.precedences[comb_1, comb_2])
                     <= self.start[comb_1])


  def add_constraints(self):
    super().add_constraints()
    self.add_constraints_state()


  def add_constraints_state(self):
    states_staubli = len(self.dict_state_job.keys())
    jobs_staubli = sum(len(lst) for lst in self.dict_state_job.values())
    list_jobs_id = [item for lst in self.dict_state_job.values() for item in lst]


    T = cp.boolvar(shape=(states_staubli, states_staubli, self.max_switches))  # ok
    P = cp.intvar(0, self.max_switches, shape=jobs_staubli)  # ok
    M = cp.boolvar(shape=(self.max_switches, jobs_staubli))  # ok


    # Only one switch is feasible at a time #15                         #OK
    for n in range(self.max_switches):
      self.model += (sum(T[trans_1, trans_2, n] for trans_1, trans_2 in
                         itertools.product(range(states_staubli), repeat=2)) <= 1)

    # Continuity of the switches #16                                     #OK
    for n in range(self.max_switches - 1):
      for given_state in range(states_staubli):
        self.model += (sum(T[any_state_1, given_state, n] for any_state_1 in range(states_staubli)) == sum(
          T[given_state, any_state_2, n + 1] for any_state_2 in range(states_staubli)))

    # After how many transitions an operation is executed #17.1         #OK
    for job in range(jobs_staubli):
      id_job = list_jobs_id[job]
      self.model += (sum(self.precedences[self.index_dv+n,id_job] for n in range(self.max_switches)) == (P[job]))

    # 17.2                                                               #ok
    for job in range(jobs_staubli):
      self.model += (sum(M[n, job] for n in range(self.max_switches)) == 1)

    # # #18                                                               #ok
    for job in range(jobs_staubli):
      self.model += (sum(n * M[n, job] for n in range(self.max_switches)) == P[job])

    # # #19                                                               #ok
    for job in range(jobs_staubli):
      for n in range(self.max_switches):
        resource_job = get_key_dict(self.dict_state_job, list_jobs_id[job])
        index_resource = sorted(list(self.dict_state_job.keys())).index(resource_job)
        self.model += (M[n, job] <= sum(T[state, index_resource, n] for state in range(states_staubli)))

    # #20                                                                 #ok
    for n in range(self.max_switches):
      self.model += (self.duration[self.index_dv+n] == sum(T[trans_1, trans_2, n] * self.duration_switches for trans_1, trans_2 in
                                                        itertools.product(range(states_staubli), repeat=2)))

    # #21
    for n in range(self.max_switches - 1):
      self.model += self.start[self.index_dv+n] < self.start[self.index_dv+n+1]


    for resource, jobs_same_res in self.dict_resource_job.items():
      if resource == 'Cell_0_Staubli':
        for sub in jobs_same_res:
          for switch in range(self.max_switches):
            self.model += ((self.start[sub] + self.duration[sub] - self.ub * self.precedences[self.index_dv + switch, sub])
                           <= self.start[self.index_dv + switch])











