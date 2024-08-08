# ROS Respeaker recording class

import wave
import rospy
import asyncio
from audio_common_msgs.msg import AudioData

class Record():
    def __init__(self, filename) -> None:
        self.wf = wave.open(filename, 'wb')
        # Required values for Respeaker and Whisper
        self.wf.setframerate(16000)
        self.wf.setnchannels(1)
        self.wf.setsampwidth(2)
        self.recording = 0
        self.timeout = 0

    def channel_callback(self, msg, wf):
        wf.writeframes(msg.data)
    
    def set_timeout(self, t):
        self.timeout = t

    async def stop_record(self):
        try:
            await asyncio.sleep(self.timeout)
            self.recording = 0
            self.wf.close()
        except Exception as e:
            print(e)
        print("Stopped recording")

    async def start_record(self):
        try:
            self.recording = 1
            print("recording...")
            while self.recording:
                rospy.sleep(0.1)
                await self.stop_record()
        except Exception as e:
            print(e)

    def timed_record(self, t):
        ''' Records using ROS Respeaker for t seconds '''
        self.set_timeout(t=t)
        try:
            rospy.init_node('audio_record')
            service = rospy.Subscriber('/qt_respeaker_app/channel0', AudioData, self.channel_callback, self.wf)
            loop = asyncio.new_event_loop()
            tasks = [
                loop.create_task(self.start_record()),
            ]
            loop.run_until_complete(asyncio.wait(tasks))
            loop.close()
            service.unregister()
        except Exception as e:
            return e