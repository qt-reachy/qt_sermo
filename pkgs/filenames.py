# class to create filenames

import datetime
import os


class TimestampFilename():
    ''' Class to create a filename at timestamp
        Set with set_filename()
    '''
    def __init__(self, location, extension) -> None:
        self.location = location
        self.extension = extension
        self.timestamp = self.set_timestamp()
        self.filename = self.set_filename()
        
    @property
    def get_location(self):
        return self.location
    
    @property
    def get_extension(self):
        return self.extension
    
    @property
    def get_timestmap(self):
        return self.timestamp
    
    @property
    def get_filename(self):
        return self.filename
        
    def datetime_to_integer(self, dt):
        # datetime to YYYMMddHHmmss
        # 1970-01-01 01:30:45 -> 19700101013045
        return int(dt.strftime("%Y%m%d%H%M%S"))
    
    def set_timestamp(self):
        return self.datetime_to_integer(datetime.datetime.now())
    
    def set_filename(self):
        return os.path.expanduser(f"~{self.location}{self.timestamp}{self.extension}")