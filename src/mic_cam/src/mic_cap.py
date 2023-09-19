#!/usr/bin/env python3

import speech_recognition as sr
import rospy

from std_msgs.msg import *

r = sr.Recognizer()

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, fuel_ask)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def fuel_ask(data):
    try:
        with sr.Microphone(device_index=5) as source:
            rospy.loginfo("Please say something... ")
            audio = r.listen(source, phrase_time_limit=5)  # Listen for up to 5 seconds
            text = r.recognize_google(audio, language='th-TH')
            print("You said:", text)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")

device = ['HDA Intel PCH: CX8200 Analog (hw:0,0)', 'HDA Intel PCH: HDMI 0 (hw:0,3)',
        'HDA Intel PCH: HDMI 1 (hw:0,7)', 'HDA Intel PCH: HDMI 2 (hw:0,8)',
        'HDA Intel PCH: HDMI 3 (hw:0,9)', 'HDA Intel PCH: HDMI 4 (hw:0,10)',
        'RØDE NT-USB Mini: USB Audio (hw:1,0)', 'sysdefault', 'hdmi', 'samplerate', 'speexrate', 'pulse', 'upmix', 'vdownmix', 'default']


if __name__ == "__main__":
    listener()






 


            
    