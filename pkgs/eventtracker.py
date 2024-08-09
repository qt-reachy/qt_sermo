import csv
import time
from hashlib import sha256

class EventTracker():
    ''' Use to track events and to write them to a csv file
            addEvent() : (str) action to track, (any) content
            writeOut(): Adds last event and writes to csv
    '''
    def __init__(self, csvfile="eggs.csv") -> None:
        self.__events = []
        self.csvfile = csvfile
        self.__runtime_start = time.time()
        self.__id = self.__set_id()
    
    def __set_id(self):
        return sha256(str(self.__runtime_start).encode('utf-8')).hexdigest()

    def addEvent(self, action, content):
        self.__events.append(self.__newEvent(action, content))
    
    def __newEvent(self, action, content):
        event = [time.time(), action, content, self.__id]
        return event
    
    def __write_to_csv(self):
        with open(self.csvfile, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(self.__events)

    def writeOut(self):
        self.addEvent("End_Interaction", 1)
        self.__write_to_csv()