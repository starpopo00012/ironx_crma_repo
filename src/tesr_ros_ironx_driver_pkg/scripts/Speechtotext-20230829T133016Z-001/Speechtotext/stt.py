#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
import speech_recognition as sr
# from gtts import gTTS
# import os

# tts=gTTS(text='เติมน้ำมันอะไรดีคะ',lang='th')
# tts.save('AskforOil.mp3')

# file = "AskforOil.mp3"
# print("Play mp3")
# os.system("mpg123 "+file)

pub = rospy.Publisher('chatter', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz

print(sr.__version__)
print(sr.Microphone.list_microphone_names()) #print all the microphones connected to your machine

def fuel_type():
    r = sr.Recognizer()
    with sr.Microphone(device_index=5) as source:
        print("Please say something... ")
        audio = r.listen(source)  # Listen for up to 5 seconds
        text = r.recognize_google(audio, language='th-TH')
        print("You said:", text)
    
device = ['HDA Intel PCH: CX8200 Analog (hw:0,0)', 'HDA Intel PCH: HDMI 0 (hw:0,3)',
          'HDA Intel PCH: HDMI 1 (hw:0,7)', 'HDA Intel PCH: HDMI 2 (hw:0,8)',
          'HDA Intel PCH: HDMI 3 (hw:0,9)', 'HDA Intel PCH: HDMI 4 (hw:0,10)',
          'RØDE NT-USB Mini: USB Audio (hw:1,0)', 'sysdefault', 'hdmi', 'samplerate', 'speexrate', 'pulse', 'upmix', 'vdownmix', 'default']

def main():
    fuel_type()
    print(device[6]) 

if __name__ == "__main__":
    fuel_type()
        