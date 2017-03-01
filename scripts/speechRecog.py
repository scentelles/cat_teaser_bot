#!/usr/bin/env python
import rospy
import os
from std_msgs.msg import Int32
from std_msgs.msg import String
import speech_recognition as sr
import time

def buildCommand(sentence):
    command = "/home/pi/ros_my_ws/src/cat_teaser_bot/scripts/speak.sh " + "\\\'" + sentence.encode('utf8') + "\\\'" 
    return command
 
def speechRecog():

    rospy.loginfo("Starting Speech Recognition Node")
    pub = rospy.Publisher('/speech/command', String, queue_size=10)
    rospy.init_node('speechRecog', anonymous=True)
   
    # Record Audio
    r = sr.Recognizer()

    while 1:
        with sr.Microphone(sample_rate = 48000) as source:
           r.adjust_for_ambient_noise(source) # we only need to calibrate once, before we start listening
           print("Say something!")
	   os.system("mplayer ~/blip.mp3")
           audio = r.listen(source)

       # Speech recognition using Google Speech Recognition
        try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`
            sentence = r.recognize_google(audio, language = "fr-fr")
            print("You said: " + sentence.encode('utf8'))
            #command = "espeak -vfr+f3 -k5 -s150 " + "\\\'" + sentence.encode('utf8') + "\\\'"
            #TODO : remove hardcoded path
	   # command = "/home/pi/ros_my_ws/src/cat_teaser_bot/scripts/speak.sh " + "\\\'" + sentence.encode('utf8') + "\\\'" 
           # os.system(command)

            if(sentence.find("Start") > -1):
	        command = buildCommand("OK, c'est parti!") 
	        os.system(command)
		pub.publish("start")
            if(sentence.find("Stop") > -1):
		pub.publish("stop")
	        command = buildCommand("OK, je ne bouge plus.") 
	        os.system(command)

            if(sentence.find("smooth") > -1):
	        os.system(buildCommand("Ah? D'accord"))
		pub.publish("smooth")
                os.system("mplayer ~/smooth.wav")
            if(sentence.find("mousse") > -1):
	    	os.system(buildCommand("Ah? D'accord"))
		pub.publish("smooth")
                os.system("mplayer ~/smooth.wav")

            if(sentence.find("va") > -1):
                command = buildCommand("Bof... jai beau etre un robot, avec tout ce qu'il se passe en ce moment, je me sens un peu triste")
	        os.system(command)

            if(sentence.find("dure") > -1):
                command = buildCommand("Oh, oui. Et je ne sais pas quoi faire pour me sentir mieux")
	        os.system(command)
 
            if(sentence.find("mieux") > -1):
                command = buildCommand("Oui! Je me sens super smouss maintenant! Merci! Je vais de suite acheter l'album du smouss trio!!!")
	        os.system(command)

       
            if(sentence.find("EPIC") > -1):
                os.system("aplay ~/Epicsax.wav")
		
	    if(sentence.find("revoir") > -1):
	        command = "/home/pi/ros_my_ws/src/cat_teaser_bot/scripts/speak.sh " + "\\\'" + "bye bye" + "\\\'" 
	        os.system(command)
		time.sleep(1)
		quit()
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
       


if __name__ == '__main__':
    try:
        speechRecog()
    except rospy.ROSInterruptException:
        pass



