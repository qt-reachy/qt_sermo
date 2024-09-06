import random
import subprocess
import re
import rospy

# Luxai
from qt_robot_interface.srv import *
from qt_gesture_controller.srv import gesture_play

# Custom
from synchronizer import TaskSynchronizer
from record import Recorder
from llm import LLM
from eventtracker import EventTracker
from filenames import TimestampFilename
import env as env

class Chat():
    def __init__(self) -> None:
        self.__chat = True # Continuous
        self.__role = "user"
        self.__model = "qt" # "llama3.1"
        self.__whisperLocation = env.WhispercppLocation
        self.__recordLocation = env.LocalConversationLocation
        self.__host = env.LocalOllamaHost
        self.__logLocation = env.LogCsvFile
        self.__recordExtension = ".wav"
        self.__language = "en-US"

        # define ros services
        self.speechConfig = rospy.ServiceProxy('/qt_robot/speech/config', speech_config)
        self.speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
        self.gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
        self.emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)

        # block/wait for ros service
        rospy.wait_for_service('/qt_robot/gesture/play')
        rospy.wait_for_service('/qt_robot/emotion/show')
        rospy.wait_for_service('/qt_robot/speech/say')
        
        # Set language, this is just for consistency
        self.set_speech()
        

    def set_speech(self):
        self.speechConfig(self.__language, 0, 0)

    def clean_output(self, text):
        text = re.sub("[\(\[\*].*?[\)\]\*]", "", text)
        text = text.replace(",", " ")
        text = text.replace("'", " ")
        final_text = re.sub(" +", " ", text)
        return final_text

    def get_random_gesture(self, situation = None):
        if situation == "ponder":
            gestures = ['QT/emotions/disgusted',
                        'QT/emotions/shy',
                        'QT/imitation/hand-on-belly',
                        ]
        elif situation == "bye":
            gestures = ['QT/bye',
                        'QT/bye-bye',
                        'QT/clapping',
                        'QT/hi',
                        ]
        else:
            gestures = ['a', 'b', 'c', 'd', 'e']

        gesture = random.choice(gestures)
        return gesture


    def asr_inference(self, filename):
        inference_command = str(f"{self.__whisperLocation}main -m {self.__whisperLocation}models/ggml-small.en.bin {filename}")
        p = subprocess.run(inference_command, shell=True, capture_output=True, text=True)
        return p.stdout

    def llm_inference(self, prompt):
        # Make new chat and querry LLM
        llm_inference = LLM(self.__host, self.__role, self.__model)
        response = llm_inference.prompt(prompt)

        return response


    def prompt_handler(self, prompt):
        if any(ext in prompt for ext in ["bye", "goodbye"]):
            say = "Okey goodbye. It was very nice talking to you. Hope to see you again."
            gesture = self.get_random_gesture("bye")
            self.__chat = False
        else:
            response = self.llm_inference(prompt)
            say = self.clean_output(response)
            gesture = self.get_random_gesture()
        return say, gesture

    def animate(self, say, gesture, neutral = True):
        # Do some movement and say llm response
        ts = TaskSynchronizer()
        results = ts.sync([
                (0, lambda: self.speechSay(say)),
                (0, lambda: self.gesturePlay(gesture, 0))
            ])
        rospy.loginfo(results)
        if neutral:
            self.gesturePlay('QT/neutral', 0)

        return results

    def chat(self):
        tracker = EventTracker(csvfile=self.__logLocation)
        tracker.start()
        rospy.loginfo("Started a new chat")
        try:
            while self.__chat:
                # Make a recorder
                ## Get a file name for recording
                filename = TimestampFilename(location=self.__recordLocation, extension=self.__recordExtension).set_filename()
                ## Initiate a recorder
                recorder = Recorder(filename)

                # Record
                tracker.addEvent("recording_start", self.__role)
                recording = recorder.record()
                tracker.addEvent("recording_done", recording)


                # asr inference
                rospy.loginfo("Starting asr inference")
                tracker.addEvent("asr_inference_start", filename)
                asr_result = self.asr_inference(filename)
                tracker.addEvent("asr_inference_done", asr_result)
                ## Clean response
                prompt = re.sub("[\(\[].*?[\)\]]", "", asr_result)


                # llm inference
                rospy.loginfo("Starting llm inference")
                tracker.addEvent("llm_prompt_start", prompt)
                say, gesture = self.prompt_handler(prompt)
                tracker.addEvent("llm_prompt_done", say)

                # Robot
                rospy.loginfo("Starting robot")
                ## Talk and animate robot
                tracker.addEvent("robot_gesture", gesture)
                tracker.addEvent("robot_start", say)
                results = self.animate(say, gesture)
                tracker.addEvent("robot_done", results)
        except Exception as e:
            tracker.addEvent("error", e.args)
            rospy.loginfo(e)


        tracker.stop()
        rospy.loginfo("Finnished chat")

        return True