#!/usr/bin/env python
import os

import rospy

from std_msgs.msg import String

def handle_audio_request(audio):
   print "Scheduling playback"
   return 
   
   
def renderAudio(data):
   audioID = data.data
   
   if(audioID == "EPIC"):
      rospy.loginfo(rospy.get_caller_id() + 'Starting playback of  %s', data.data)
      os.system("aplay ~/Epicsax.wav")

def audioRenderer():
   rospy.init_node('audioRenderer', anonymous=False)
   print "Ready to playback audio request"
   
   
   
   rospy.Subscriber('/audio/command', String, renderAudio)

   rospy.spin()
    
       
if __name__ == "__main__":
   audioRenderer()
