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
from pkgs.eventtracker import EventTracker

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
CSV = env.LogCsvFile


def clean_output(text):
    text = re.sub("[\(\[\*].*?[\)\]\*]", "", text)
    text = text.replace(",", " ")
    text = text.replace("'", " ")
    final_text = re.sub(" +", " ", text)
    return final_text


# main
if __name__ == '__main__':
    

    # define ros services
    speechConfig = rospy.ServiceProxy('/qt_robot/speech/config', speech_config)
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
    emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)

    
    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/gesture/play')
    rospy.wait_for_service('/qt_robot/emotion/show')
    rospy.wait_for_service('/qt_robot/speech/say')

    # init classes
    filename = TimestampFilename(location=LOCATION, extension=EXTENSION).set_filename()
    tracker = EventTracker(csvfile=CSV)
    recording = Record(filename=filename)

    rospy.loginfo("Started interaction")

    try:
        tracker.addEvent("Init", 1)
        speechSay('#GOAT#')
        tracker.addEvent("recording", 1)
        recording.timed_record(5)
        tracker.addEvent("recording", 0)

        recording = str(f"{WHISPERLOCATION}main -m {WHISPERLOCATION}models/ggml-small.en.bin {filename}")
        tracker.addEvent("Whisper_interferance", filename)
        p = subprocess.run(recording, shell=True, capture_output=True, text=True)
        tracker.addEvent("Whisper_runtime", 1)

        # Clean up Whisper output to a prompt
        prompt = str(p.stdout.strip()).split("]")[1]
        
        # Make new chat and querry LLM
        newChat = LLM(HOST, ROLE, MODEL)
        tracker.addEvent("Ollama_prompt", prompt)
        response = newChat.prompt(prompt)
        tracker.addEvent("Ollama_reply", response)


        # Config speech to english
        speechConfig("en-US", 0, 0)

        # fix reply for SpeechSay
        say = clean_output(response)
        tracker.addEvent("SpeechSay", say)
        # Do some movement and say llm response
        ts = TaskSynchronizer()
        results = ts.sync([
                (0, lambda: speechSay(say)),
                (0, lambda: gesturePlay('QT/Dance/Dance-1-3', 0))
            ])
        
    except Exception as e:
        tracker.addEvent("error", e.args)
        print(e)


    tracker.writeOut()
    rospy.loginfo("Finished interaction")