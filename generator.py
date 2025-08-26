import copy
import random



class Generator():

    def __init__(self,dict_no_items,dict_item):
        '''
        :param dict_no_items:
        dictionary containing as key the name of the item and the no of items that have to be created
        :param dict_action_time:
        dictionary containing as key the name of the item and as item a subdicitonary with and a list containing the possible
        time value that it can take, randomly selected
        '''
        self.dict_no_items = dict_no_items
        self.dict_item = dict_item
        self.output_dict = dict()


    def generate_constraints(self):
        self.dict_item_actions = {}
        for item in self.dict_no_items:
            for no_item in self.dict_no_items[item]:
                name_item = f'{item}_{no_item}'
                constraints = copy.deepcopy(self.dict_item_constraints[item])
                self.output_dict[name_item] = constraints
        return self.output_dict


