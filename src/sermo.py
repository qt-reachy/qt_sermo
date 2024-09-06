import rospy
import threading
from chat import Chat

def sermo_handler(msg):
    """ QT runs Python 3.8, switches introduced 3.10 """
    
    rospy.loginfo(msg.service)

    userChat = Chat()

    if msg.service == "intro":
        intro(userChat)
    elif msg.service == "chat":
        run = run = threading.Thread(target=userChat.chat)
        run.start()
        return False
    elif msg.service == "image_gen":
        apologise(userChat)
    elif msg.service == "exposition":
        apologise(userChat)
    elif msg.service == "exit":
        pass
    else:
        apologise(userChat)

    return True

def intro(userChat) -> None:
    say = "I am QT the robot. You can talk to me by prompting my LLM system if you pick the chat program. Use the buttons to to start and stop prompting and when you want to stop chatting simply prompt goodbye."
    gesture = 'QT/show_QT'
    userChat.animate(say=say, gesture=gesture)


def apologise(userChat) -> None:
    say = "I am sorry, that function is not available yet"
    gesture = 'QT/peekaboo'
    userChat.animate(say, gesture)





