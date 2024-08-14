import csv
import time
from uuid import uuid1

class EventTracker():
    ''' Use to track events and to write them to a csv file
            addEvent() : (str) action to track, (any) content
            writeOut(): Adds last event and writes to csv
    '''
    def __init__(self, csvfile="eggs.csv") -> None:
        self.__events = []
        self.csvfile = csvfile
        self.__actions = {}
        self.__user = self.__set_user()
    
    def __set_user(self):
        return uuid1()

    def addEvent(self, action, content):
        newEvent = self.__newEvent(action, content)
        self.__events.append(newEvent)

    def __set_action_counter(self, action):
        # Using a dict to track, as application interaction are short
        if action not in self.__actions:
            self.__actions[action] = 1
        else:
            self.__actions[action] += 1
        return self.__actions[action]

    def __newEvent(self, action, content):
        __time = time.time()
        __action = action
        __action_counter = self.__set_action_counter()
        __content = content
        __user = self.__user
        event = [__time, __action, __action_counter, __content, __user]
        return event
    
    def __write_to_csv(self):
        with open(self.csvfile, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(self.__events)

    def writeOut(self):
        self.addEvent("End_Interaction", 1)
        self.__write_to_csv()