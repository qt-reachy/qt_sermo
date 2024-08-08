#!/usr/bin/env python3
import rospy
import time
import subprocess
import re

from qt_robot_interface.srv import *
from qt_gesture_controller.srv import gesture_play

from pkgs.synchronizer import TaskSynchronizer
from pkgs.record import Record
from pkgs.llm import LLM
from pkgs.filenames import TimestampFilename

import env

# Variables
START_KEY = 's'
STOP_KEY = 'q'
EXTENSION = ".wav"
ROLE = "user"
MODEL = "qt" # "llama3.1"
WHISPERLOCATION = env.WhispercppLocation
LOCATION = env.LocalConversationLocation
HOST = env.LocalOllamaHost


# main
if __name__ == '__main__':
    
    rospy.loginfo("Started interaction")
    start = time.time()

    # define ros services
    speechConfig = rospy.ServiceProxy('/qt_robot/speech/config', speech_config)
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
    emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)

    # ceate an instance of TaskSynchronizer

    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/gesture/play')
    rospy.wait_for_service('/qt_robot/emotion/show')
    rospy.wait_for_service('/qt_robot/speech/say')

    filename = TimestampFilename(location=LOCATION, extension=EXTENSION).set_filename()

    recording = Record(filename=filename)
    speechSay('#GOAT#')
    recording.timed_record(5)

    # recording = str(f"{WHISPERLOCATION}main -m {WHISPERLOCATION}models/ggml-small.en.bin {WHISPERLOCATION}samples/jfk.wav")
    recording = str(f"{WHISPERLOCATION}main -m {WHISPERLOCATION}models/ggml-small.en.bin {filename}")
    # print("ASR: ", recording)
    p = subprocess.run(recording, shell=True, capture_output=True, text=True)
    print("I heard the following:")
    # print(p.stdout)
    # print(p.stderr) # detailed report
    prompt = str(p.stdout.strip()).split("]")[1]
    print(prompt)

    # Make new chat and querry LLM
    newChat = LLM(HOST, ROLE, MODEL)
    reply = newChat.prompt(prompt)

    # fix reply
    reply = reply


    speechConfig("en-US", 0, 0)

    ts = TaskSynchronizer()
    results = ts.sync([
            (0, lambda: speechSay(reply)),
            (0, lambda: gesturePlay('QT/Dance/Dance-1-3', 0))
        ])


    print("Finished interaction")