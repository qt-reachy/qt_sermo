import wave
import rospy
import threading
from audio_common_msgs.msg import AudioData

# Service message import
from qt_sermo.srv import *

class Recorder():
    def __init__(self, filename) -> None:
        self.filename = filename
        self.__recording = False
        self.__await = True
        self.__running = True

    def handle_record(self, data):
        """ Handles sermo_record.srv
            No bool handling in terminal, so we send int64
                (int64) 1 = Start : 0 = Stop / Exit
        """
        if data.action == 1:
            self.__await = False
            self.__recording = True
        elif data.action == 0 and self.__await == False:
            self.__recording = False
        # Exit before recording
        elif data.action == 0 and self.__await == True:
            self.__recording == False
            self.__await == False
            self.__running = False
            self.filename = False
        else:
            pass
        
        return True


    def channel_callback(self, msg, wf):
        """ Respeaker writeframes callback function """
        wf.writeframes(msg.data)

    def __recorder(self) -> None:
        while self.__await:
            rospy.sleep(0.5)
        if self.__running == True:
            try:
                self.wf = wave.open(self.filename, 'wb')
                self.wf.setframerate(16000)
                self.wf.setnchannels(1)
                self.wf.setsampwidth(2)
                self.__service = rospy.Subscriber('/qt_respeaker_app/channel0', AudioData, self.channel_callback, self.wf)
                while self.__recording:
                    rospy.loginfo('recording...')
                    rospy.sleep(0.5)
                self.wf.close()
                self.__service.unregister()
                self.__running = False
            
            except Exception as e:
                print(e)

    def record(self):
        """ Fires of a new thread with a recorder 
            Opens a sermo_record service that handles recorder
        """
        start = threading.Thread(target=self.__recorder)

        start.start()

        # Start sermo_record service
        service = rospy.Service(
        'qt_robot/sermo_recorder', sermo_record, self.handle_record
            )
        rospy.loginfo('Service sermo recorder open')
        
        # Wait until the recorder is done then remove service
        while self.__running:
            rospy.sleep(0.5)

        service.shutdown("Done")
        rospy.loginfo('Service sermo recorder closed')

        return self.filename
