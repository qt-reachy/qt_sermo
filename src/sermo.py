#!/usr/bin/env python3

import rospy
import subprocess
import re

from qt_robot_interface.srv import *
from qt_gesture_controller.srv import gesture_play

from synchronizer import TaskSynchronizer
from record import Record
from llm import LLM
from eventtracker import EventTracker
from filenames import TimestampFilename

import env as env

from qt_sermo.srv import *

# Variables
ROLE = "user"
MODEL = "qt" # "llama3.1"
WHISPERLOCATION = env.WhispercppLocation
LOCATION = env.LocalConversationLocation
HOST = env.LocalOllamaHost
CSV = env.LogCsvFile
EXTENSION = ".wav"


def clean_output(text):
    text = re.sub("[\(\[\*].*?[\)\]\*]", "", text)
    text = text.replace(",", " ")
    text = text.replace("'", " ")
    final_text = re.sub(" +", " ", text)
    return final_text

def chat(msg):
    # rospy.loginfo(msg.service == "start")
    tracker = EventTracker(csvfile=CSV)
    try:
        tracker.start()

        while True:
            filename = TimestampFilename(location=LOCATION, extension=EXTENSION).set_filename()
            recorder = Record(filename)
            speechSay('#GOAT#')
            tracker.addEvent("recording_start", ROLE)
            recorder.ham_record()
            tracker.addEvent("recording_done", filename)
            inference_command = str(f"{WHISPERLOCATION}main -m {WHISPERLOCATION}models/ggml-small.en.bin {filename}")
            tracker.addEvent("asr_inference_start", inference_command)
            p = subprocess.run(inference_command, shell=True, capture_output=True, text=True)
            tracker.addEvent("asr_inference_done", p.stdout)
            # Clean up Whisper output to a prompt
            prompt = re.sub("[\(\[].*?[\)\]]", "", p.stdout)
            if any(ext in prompt for ext in ["bye", "goodbye"]):
                tracker.addEvent("llm_prompt_start", prompt)
                tracker.addEvent("llm_prompt_done", "Goodbye")
                
                gesture = 'QT/bye'
                say = "Nice talking to you, goodbye"
                tracker.addEvent("robot_gesture", gesture)
                tracker.addEvent("robot_start", say)
                ts = TaskSynchronizer()
                results = ts.sync([
                        (0, lambda: speechSay(say)),
                        (0, lambda: gesturePlay(gesture, 0))
                    ])
                tracker.addEvent("robot_done", True)
                break
            else:
                # Make new chat and querry LLM
                newChat = LLM(HOST, ROLE, MODEL)
                tracker.addEvent("llm_prompt_start", prompt)
                response = newChat.prompt(prompt)
                tracker.addEvent("llm_prompt_done", response)


                # Config speech to english
                speechConfig("en-US", 0, 0)

                # fix reply for SpeechSay
                say = clean_output(response)
                gesture = 'QT/Dance/Dance-1-3'
                tracker.addEvent("robot_gesture", gesture)
                tracker.addEvent("robot_start", say)
                # Do some movement and say llm response
                ts = TaskSynchronizer()
                results = ts.sync([
                        (0, lambda: speechSay(say)),
                        (0, lambda: gesturePlay(gesture, 0))
                    ])
                
            tracker.addEvent("robot_done", results)

    except Exception as e:
        tracker.addEvent("error", e.args)
        print(e)


    tracker.stop()

    return True


# main
if __name__ == '__main__':
    rospy.init_node('qt_sermo')
    rospy.loginfo("qt_sermo_node started!")
    

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

    service = rospy.Service(
        'qt_robot/sermo_chat', sermo_chat, chat
    )

    rospy.spin()
    

    rospy.loginfo("Finished interaction")
