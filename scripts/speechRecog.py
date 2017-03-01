#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import speech_recognition as sr



 
def speechRecog():

    rospy.loginfo("Starting Speech Recognition Node")
    pub = rospy.Publisher('/command/move', Int32, queue_size=10)
    rospy.init_node('speechRecog', anonymous=True)
   
    # Record Audio
    r = sr.Recognizer()

    while 1:
        with sr.Microphone(sample_rate = 48000) as source:
           r.adjust_for_ambient_noise(source) # we only need to calibrate once, before we start listening
           print("Say something!")
           audio = r.listen(source)

       # Speech recognition using Google Speech Recognition
        try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`
            sentence = r.recognize_google(audio, language = "fr-fr")
            print("You said: " + sentence)
            #command = "espeak -vfr+f3 -k5 -s150 " + "\\\'" + sentence.encode('utf8') + "\\\'"
            command = "./speak.sh " + "\\\'" + sentence.encode('utf8') + "\\\'" 
            os.system(command)
       
            if(sentence.find("EPIC") > -1):
                os.system("aplay ~/Epicsax.wav")
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
       


if __name__ == '__main__':
    try:
        speechRecog()
    except rospy.ROSInterruptException:
        pass



