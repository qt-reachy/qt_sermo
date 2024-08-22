# ROS Respeaker recording class

import wave
import rospy
import threading
from audio_common_msgs.msg import AudioData

class Record():
    def __init__(self, filename) -> None:
        # Required values for Respeaker and Whisper
        self.filename = filename
        self._recording = True

    def channel_callback(self, msg, wf):
        wf.writeframes(msg.data)

    def timed_record(self, t):
        ''' Simple record for t time '''
        try:
            rospy.init_node('audio_record')
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

    def break_record(self) -> None:
        while True:
            val = input('"q" and ENTER to stop')
            if val == "q":
                break
        self.__recording = False


    def __record(self):
        try:
            self.wf = wave.open(self.filename, 'wb')
            self.wf.setframerate(16000)
            self.wf.setnchannels(1)
            self.wf.setsampwidth(2)
            self.__service = rospy.Subscriber('/qt_respeaker_app/channel0', AudioData, self.channel_callback, self.wf)
            while self.__recording:
                rospy.spin()
            self.wf.close()
            self.__service.unregister()
        except Exception as e:
            print(e)

    def record(self):
        rospy.init_node('audio_record')
        start = threading.Thread(target=self.__record)
        stop = threading.Thread(target=self.break_record)
        self.__recording = True
        start.start()
        stop.start()
        stop.join()

