#!/usr/bin/env python3
import wave
import rospy
import datetime
import time
import os
import subprocess
from qt_robot_interface.srv import *
from qt_gesture_controller.srv import gesture_play
from audio_common_msgs.msg import AudioData
from pkgs.synchronizer import TaskSynchronizer
from pkgs.llm import LLM


AUDIO_RATE = 16000
AUDIO_CHANNELS = 1
AUDIO_WIDTH = 2

# Debugging parameters
TALK = 0
PRINT_COMMAND = 0


def channel_callback(msg, wf):
    wf.writeframes(msg.data)

def datetime_to_integer(dt):
    # 1970-01-01 01:30:45 -> 19700101013045
    return int(dt.strftime("%Y%m%d%H%M%S"))



# main
if __name__ == '__main__':
    
    rospy.loginfo("Started Whispercpp")


    # define ros services
    speechConfig = rospy.ServiceProxy('/qt_robot/speech/config', speech_config)
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
    emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)

    # ceate an instance of TaskSynchronizer
    ts = TaskSynchronizer()

    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/gesture/play')
    rospy.wait_for_service('/qt_robot/emotion/show')
    


    speechConfig("en-US", 0, 0)

    if TALK == True:
        results = ts.sync([
            (0, lambda: speechSay('Hey! My name is QT, and I am a robot')),
            (0, lambda: gesturePlay('QT/happy', 0))
        ])
        print('talkText and gesturePlay finished.')

    print("Started whisper cpp test")

    START_KEY = 's'
    STOP_KEY = 'q'

    timestamp = datetime_to_integer(datetime.datetime.now())
    filename = os.path.expanduser(f"~/kristoffer/conversations/{timestamp}.wav")

    whispercpp_path = os.path.expanduser("~/catkin_ws/src/whispercpp/src/whisper.cpp/")



    # call the relevant service
    rospy.init_node('audio_record')
    
    wf = wave.open(filename, 'wb')
    wf.setnchannels(AUDIO_CHANNELS)
    wf.setsampwidth(AUDIO_WIDTH)
    wf.setframerate(AUDIO_RATE)

    start = time.time()

    speechSay('#CAT#')

    rospy.Subscriber('/qt_respeaker_app/channel0', AudioData, channel_callback, wf)
    print("recording...")
    while not rospy.is_shutdown():
        rospy.sleep(1)
        if time.time() > start + 5:
            print("Stopped recording")
            rospy.signal_shutdown("end")
            wf.close()
    

    # command = str(f"{whispercpp_path}main -m {whispercpp_path}models/ggml-small.en.bin {whispercpp_path}samples/jfk.wav")
    command = str(f"{whispercpp_path}main -m {whispercpp_path}models/ggml-small.en.bin {filename}")
    
    if PRINT_COMMAND == True:
        print("Commanding whisper.cpp: ", command)

    p = subprocess.run(command, shell=True, capture_output=True, text=True)
    
    print("I heard the following:")
    # print(p.stdout)
    # print(p.stderr) # detailed report

    eggs = str(p.stdout.strip()).split("]")[1]

    print(eggs)

    newChat = LLM("192.168.2.120:11434", "user", "llama3", "You are a robot called QT")
    newChat.chat(eggs)

    print("Done")