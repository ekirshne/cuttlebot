#import needed libraries
import numpy as np
from random import random as rand
import csv
import datetime

#Cognition class for defining cognitive abilities
class Cognition():
    #Cognition class constructor
    def __init__(self, color_dict, learning_rate, softmax_beta, discount_factor, data_path):
        #save the robot cognition parameters
        self.learning_rate = learning_rate #0.2
        self.softmax_beta = softmax_beta #1.0
        self.discount_factor = discount_factor #0.9
        self.data_path = data_path.joinpath("Q_Tables.csv")

        # Initialize the Q_table
        self.q_table = dict()
        for color in color_dict:
            self.q_table[color] = {
                'RUN': 0,
                'APPROACH': 0
            }
        self.q_table['NONE'] = {
            'FORWARD' : 0,
            'BACK' : 0
        }

        # Create the CSV file and record learning rate and the discount factor info
        #self.timestamp = '{:%Y_%m_%d_%H_%M_%S}'.format(datetime.datetime.now())
        self.write_data_to_csv()


    def write_data_to_csv(self):
        timestamp_string = str(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        with open(self.data_path, 'a', newline='') as file:
            writer = csv.writer(file)
            data = [
                [timestamp_string, 'APPROACH', 'RUN'],
                ['BLUE', self.q_table['BLUE']['APPROACH'], self.q_table['BLUE']['RUN']],
                ['GREEN', self.q_table['GREEN']['APPROACH'], self.q_table['GREEN']['RUN']],
                [] # Add an empty line in the CSV file between each Q table update
            ]
            writer.writerows(data)


    def print_q_table(self) -> None:
        '''Prints the current Q-table.

        This function prints the values associated with each state-action pair in the Q-table.
        The Q-table is a dictionary where keys represent states and values represent dictionaries
        of actions and their corresponding Q-values. The 'NONE' state is excluded from printing.'''

        print("=== CURRENT Q-TABLE ===")
        for state in self.q_table.keys():
            if state == 'NONE':
                continue
            print(f"{state}:")
            for action in self.q_table[state].keys():
                print(f"\t{action}: {self.q_table[state][action]}")
        print("==========")




    def _softmax(self, np_value_list: np.ndarray) -> np.ndarray:
        '''Computes the softmax function for a list of values.'''

        #referencing alvas & Trevor Merrifield on stack overflow: https://stackoverflow.com/questions/34968722/how-to-implement-the-softmax-function-in-python
        #solution used for numerical stability: e^x / sum(e^x) = e^(x-x_max) / sum(e^(x-x_max))
        adjusted_values = self.softmax_beta*np_value_list
        exponential_vector = np.exp(adjusted_values - np.max(adjusted_values))
        return exponential_vector/(exponential_vector.sum())




    def get_state_with_largest_punishment(self, colors_in_view: list) -> str: #str | None:
        '''Returns the color with the largest punishment according to the Q-table.

        This function iterates through the colors in the robot's view and checks 
        their corresponding Q-values in the Q-table. It identifies the color with 
        the lowest Q-value among the 'RUN' and 'APPROACH' actions, which indicates 
        the largest punishment. If a color is not present in the Q-table, it initializes 
        its Q-values to zero. If multiple colors have the same lowest Q-value, the function 
        selects the first encountered color. If no colors are present, None is returned.'''

        #error check for passing an empty list
        if len(colors_in_view) == 0:
            return None
        
        #Initiallize the most_punishing_color and min_value to None
        most_punishing_color = None
        min_value = None
        #loop through each color
        for color in colors_in_view:
            #try to access the color from the q_table
            if color in self.q_table:
                #get the min value from approach and run
                current_color_min_q_value = min(self.q_table[color]["RUN"], self.q_table[color]["APPROACH"])
                #update the most punishing color if the possible punishment is lower than before
                if min_value == None:
                   min_value = current_color_min_q_value 
                   most_punishing_color = color
                elif current_color_min_q_value < min_value:
                    most_punishing_color = color
                    min_value = current_color_min_q_value
            #if color does not exist in q_table, then add it in
            else:
                self.q_table[color] = {
                    'RUN' : 0, 
                    'APPROACH' : 0
                }
                #update the most_punishing color if it is currently positive
                if most_punishing_color == None or min_value > 0:
                    most_punishing_color = color
                    min_value = 0
        #return the most_punishing color
        return most_punishing_color




    def get_action(self, state: str) -> str:
        '''Determines the action to take based on the given state.

        This function retrieves the Q-values associated with the 'RUN' and 'APPROACH' actions 
        for the given state from the Q-table. If the state is not present in the Q-table, it 
        initializes its Q-values to zero. It then computes the softmax probabilities for these 
        Q-values and selects an action based on these probabilities. The function returns either 
        'RUN' or 'APPROACH' as the chosen action.'''

        try:
            run_approach_q_value_list = np.array([self.q_table[state]['RUN'], self.q_table[state]['APPROACH']])
        #if state does not exist in q_table, then add it in
        except:
            self.q_table[state] = {
                'RUN' : 0, 
                'APPROACH' : 0
            }
            run_approach_q_value_list = np.array([self.q_table[state]['RUN'], self.q_table[state]['APPROACH']])
        
        run_approach_probabilities = self._softmax(run_approach_q_value_list) # Action selection policy
        random_value = rand()
        print(f"probs: {run_approach_probabilities}")
        print(f"val: {random_value}")
        if random_value < run_approach_probabilities[0]:
            return 'RUN'
        else:
            return 'APPROACH'
    



    def get_new_state(self, state: str, action: str) -> str:
        '''Determines the new state based on the current state and the chosen action.'''
        new_state = 'NONE'
        return new_state
    



    def get_max_q_at_state(self, state: str) -> float: #float | None:
        '''Returns the maximum Q-value for a given state.

        This function retrieves the Q-values associated with different actions for the given state 
        from the Q-table and returns the maximum Q-value among them.'''

        #get the q_table row of actions for a given state
        q_action_dict = self.q_table[state]
        #declare the max_q value
        max_q = None
        #loop through all actions
        for action in q_action_dict.keys():
            #make the first q value the max value
            if max_q == None:
                max_q = q_action_dict[action]
            #reassign the value for larger q values found
            elif max_q < q_action_dict[action]:
                max_q = q_action_dict[action]
        #return the max_q value found
        return max_q
    

    def update_q_table(self, state: str, action: str, reward: float, new_state: str) -> None:
        '''Updates the Q-table based on the Q-learning algorithm.

        This function updates the Q-value of a given state-action pair in the Q-table 
        using the Q-learning update rule. The Q-learning update rule calculates the 
        updated Q-value based on the reward received, the discount factor, the learning 
        rate, and the maximum possible Q-value for the new state.'''

        #q-learning update rule: use max possible q value of actions at the current state
        print(f"Updating Q-Table: {state}, {action}")
        self.q_table[state][action] += self.learning_rate * (reward + self.discount_factor * self.get_max_q_at_state(new_state) - self.q_table[state][action]) # Q = Q + lr * (reward + discount * max{Q_next} - Q) https://huggingface.co/blog/deep-rl-q-part2

        print(f"\tChanged to {self.q_table[state][action]}")

        # Record the Q values into the CSV file 
        self.write_data_to_csv()