import wave
import rospy
import threading
from audio_common_msgs.msg import AudioData

from qt_sermo.srv import *

class Record():
    def __init__(self, filename) -> None:
        self.filename = filename
        self._recording = True

    def channel_callback(self, msg, wf):
        wf.writeframes(msg.data)

    def timed_record(self, t):
        ''' Simple record for t time '''
        try:
            wf = wave.open(self.filename, 'wb')
            wf.setframerate(16000)
            wf.setnchannels(1)
            wf.setsampwidth(2)
            service = rospy.Subscriber('/qt_respeaker_app/channel0', AudioData, self.channel_callback, wf)
            rospy.sleep(t)
            wf.close()
            service.unregister()
        except Exception as e:
            print(e)

    def break_record(self, data):
        if data.action == 0:
            self.__recording = data.action
            rospy.loginfo("stopped recording")
            return True
        else:
            return False

    def ham_record(self) -> None:
        self.__recording = True
        try:
            self.wf = wave.open(self.filename, 'wb')
            self.wf.setframerate(16000)
            self.wf.setnchannels(1)
            self.wf.setsampwidth(2)
            self.__service = rospy.Subscriber('/qt_respeaker_app/channel0', AudioData, self.channel_callback, self.wf)
            service = rospy.Service(
                'qt_robot/sermo_recorder', sermo_record, self.break_record
                )
            while self.__recording:
                rospy.loginfo('recording...')
                rospy.sleep(0.5)
            self.wf.close()
            self.__service.unregister()
            service.shutdown("Done")
        
        except Exception as e:
            print(e)

    def record(self):
        start = threading.Thread(target=self.__record)
        stop = threading.Thread(target=self.break_record)
        
        start.start()
        stop.start()
        
        start.join()
        stop.join()

        return self.filename

