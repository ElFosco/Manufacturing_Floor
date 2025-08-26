from abc import ABC, abstractmethod
import random


class Item(ABC):

    def __init__(self):
        self.dict_instance = None

    @abstractmethod
    def define_action_time_constraints(self):
        pass

    @abstractmethod
    def define_action_time(self):
        pass

    def get_dict_instance(self):
        return self.dict_instance

    def define_get_from_source(self,dict,time):
        dict['get from source']= ['source', time, []]
        return dict

    def define_transport_to_cell(self, dict, time):
        dict['transport to cell'] = ['transport', time, [['RightAfter','get from source']]]
        return dict

    def define_transport_to_sink(self, dict, time, constraints):
        dict['transport to sink'] = ['transport', time, constraints]
        return dict

    def define_drop_to_sink(self, dict, time):
        dict['drop to sink'] = ['sink', time, [['RightAfter','transport to sink']]]
        return dict

    def define_action_no_binpick(self,dict,time,action_name,constraints_staubli):
        dict[f'staubli {action_name}'] = ['staubli', time, constraints_staubli]
        return dict

    def define_action_with_binpick(self,dict,time_b,time_s,action_name,constraints_binpick,constraints_staubli):
        dict[f'binpick {action_name}'] = ['binpick', time_b, constraints_binpick]
        dict[f'staubli {action_name}'] = ['staubli', time_s, constraints_staubli]
        return dict


class Engine(Item):

    def __init__(self):
        self.dict_action_time = self.define_action_time()
        self.dict_instance = self.define_action_time_constraints()


    def define_action_time(self):
        dict_action_time = {
            'get from source' : [10],
            'transport to cell' : [18],
            'add motor cover' : [33],
            'binpick M5' : [320],
            'add M5' : [192],
            'add feeder': [152],
            'binpick M3': [160],
            'add M3': [96],
            'transport to sink': [18],
            'drop to sink': [10],
        }
        return dict_action_time

    def define_action_time_constraints(self):
        dict_constraints = {}
        dict_constraints = self.define_get_from_source(dict_constraints,self.dict_action_time['get from source'])
        dict_constraints = self.define_transport_to_cell(dict_constraints,self.dict_action_time['transport to cell'])
        dict_constraints = self.define_action_no_binpick(dict_constraints,self.dict_action_time['add motor cover'],
                                                         'add motor cover',
                                                         [['RightAfter','transport to cell']])
        dict_constraints = self.define_action_with_binpick(dict_constraints,
                                                           self.dict_action_time['binpick M5'],
                                                           self.dict_action_time['add M5'],'add M5',
                                                           [[]],
                                                           [['RigthAfter','binpick M5']])
        dict_constraints = self.define_action_no_binpick(dict_constraints, self.dict_action_time['add feeder'],
                                                         'add feeder',
                                                         [['After', 'add M5']])
        dict_constraints = self.define_action_with_binpick(dict_constraints,
                                                           self.dict_action_time['binpick M3'],
                                                           self.dict_action_time['add M3'],'add M3',
                                                           [[]],
                                                           [['RigthAfter','binpick M3'],['After','add feeder']])
        dict_constraints = self.define_transport_to_sink(dict_constraints,self.dict_action_time['transport to sink'],
                                                         [['After','add M3']])
        dict_constraints = self.define_drop_to_sink(dict_constraints,self.dict_action_time['drop to sink'])

        return dict_constraints

class LightSwitch(Item):

    def __init__(self):
        self.dict_action_time = self.define_action_time()
        self.dict_instance = self.define_action_time_constraints()

    def define_action_time(self):
        dict_action_time = {
            'get from source' : [5],
            'transport to cell' : [10],

            'binpick BasePlate': [10],
            'add BasePlate': [15],

            'binpick Spring1': [15],
            'add Spring1': [10],

            'binpick Spring2': [15],
            'add Spring2': [12],

            'binpick PCB': [20],
            'add PCB': [15],

            'binpick Lid': [10],
            'add Lid': [25],

            'click Lid': [5],

            'binpick Button': [20],
            'add Button': [10],

            'check Switch': [20],
            'remove Switch': [30],

            'transport to sink': [10],
            'drop to sink': [10],
        }
        return dict_action_time


    def define_action_time_constraints(self):
        dict_constraints = {}
        dict_constraints = self.define_get_from_source(dict_constraints, self.dict_action_time['get from source'])
        dict_constraints = self.define_transport_to_cell(dict_constraints, self.dict_action_time['transport to cell'])

        dict_constraints = self.define_action_with_binpick(dict_constraints,
                                                           self.dict_action_time['binpick BasePlate'],
                                                           self.dict_action_time['add BasePlate'], 'add BasePlate',
                                                           [[]],
                                                           [['RigthAfter', 'binpick BasePlate'],
                                                            ['After', 'transport to cel']])

        dict_constraints = self.define_action_with_binpick(dict_constraints,
                                                           self.dict_action_time['binpick BasePlate'],
                                                           self.dict_action_time['add BasePlate'], 'add BasePlate',
                                                           [[]],
                                                           [['RigthAfter', 'binpick BasePlate'],
                                                            ['After', 'transport to cel']])

        dict_constraints = self.define_action_with_binpick(dict_constraints,
                                                           self.dict_action_time['binpick Spring1'],
                                                           self.dict_action_time['add Spring1'], 'add Spring1',
                                                           [[]],
                                                           [['RigthAfter', 'binpick Spring1'],
                                                            ['After', 'add BasePlate']])

        dict_constraints = self.define_action_with_binpick(dict_constraints,
                                                           self.dict_action_time['binpick Spring2'],
                                                           self.dict_action_time['add Spring2'], 'add Spring2',
                                                           [[]],
                                                           [['RigthAfter', 'binpick Spring2'],
                                                            ['After', 'add Spring1']])

        dict_constraints = self.define_action_with_binpick(dict_constraints,
                                                           self.dict_action_time['binpick PCB'],
                                                           self.dict_action_time['add PCB'], 'add PCB',
                                                           [[]],
                                                           [['RigthAfter', 'binpick PCB'],
                                                            ['After', 'add Spring2']])

        dict_constraints = self.define_action_with_binpick(dict_constraints,
                                                           self.dict_action_time['binpick Lid'],
                                                           self.dict_action_time['add Lid'], 'add Lid',
                                                           [[]],
                                                           [['RigthAfter', 'binpick Lid'],
                                                            ['After', 'add PCB']])

        dict_constraints = self.define_action_no_binpick(dict_constraints, self.dict_action_time['click Lid'],
                                                         'click Lid',[['After', 'add Lid']])

        dict_constraints = self.define_action_with_binpick(dict_constraints,
                                                           self.dict_action_time['binpick Button'],
                                                           self.dict_action_time['add Button'], 'add Button',
                                                           [[]],
                                                           [['RigthAfter', 'binpick Button'],
                                                            ['After', 'click Lid']])

        dict_constraints = self.define_action_no_binpick(dict_constraints, self.dict_action_time['check Switch'],
                                                         'check Switch',[['After', 'add Button']])
        dict_constraints = self.define_action_no_binpick(dict_constraints, self.dict_action_time['remove Switch'],
                                                         'remove Switch',[['RightAfter', 'check Switch']])


        dict_constraints = self.define_transport_to_sink(dict_constraints, self.dict_action_time['transport to sink'],
                                                         [['After', 'remove switch']])
        dict_constraints = self.define_drop_to_sink(dict_constraints, self.dict_action_time['drop to sink'],[['After','remove Switch']])

        return dict_constraints







