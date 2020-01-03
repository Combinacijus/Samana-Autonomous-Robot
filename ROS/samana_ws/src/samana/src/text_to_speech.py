#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import pyttsx3
import time


class TTS:
    def __init__(self):
        self.queue = []  # String lists
        self.reading = False
        self.max_queue_size = 15
        self.wpm = 220  # NOTE: tunnign
        self.wpm_fast = 330

        self.tts = pyttsx3.init()
        
        self.tts.setProperty('voice', "english-us")
        # 200WPM normal rate upto 300WPM quite normal and might be possible upto 350WPM
        # self.tts.setProperty('rate', self.wpm_fast)
        self.tts.say("Send it!")
        self.tts.runAndWait()
        self.tts.setProperty('rate', self.wpm)

    def init_ros(self):
        # Init ROS node
        rospy.init_node("text_to_speech")
        rospy.Subscriber("text_to_speech", String, self.tts_cb)

        while not rospy.is_shutdown():  # So it would exit on ctrl+C
            self.read_queue()
            time.sleep(0.1) # Decreses CPU usage A LOT
        
        # Probably never called
        rospy.spin() 

    def tts_cb(self, txt):
        '''
            Save text from /text_to_speech topic to queue
        '''
        self.queue.append(txt.data)

    def read_queue(self):
        '''
            Reads one message from queue and removes it from the queue
        '''
        if self.reading:  # If already reading return
            return

        self.reading = True

        while len(self.queue) != 0:
            # Pop overflowing queue
            while len(self.queue) > self.max_queue_size + 1:
                self.queue.pop(0)

            # Notify if queue overflowed and popping at faster rate
            if len(self.queue) > self.max_queue_size:
                self.tts.setProperty('rate', self.wpm_fast)
                self.queue[0] = "pop " + self.queue[0]

            # Make speech faster if queue is fuller # NOTE: tunning
            self.tts.setProperty('rate', self.wpm + len(self.queue) * 13)
            
            # Say what is in the queue
            self.tts.say(self.queue[0])
            self.tts.runAndWait()
            self.queue.pop(0)

        self.reading = False


if __name__ == "__main__":
    try:
        tts = TTS()  # Start ROS node and all callbacks
        tts.init_ros()
    except rospy.ROSInterruptException:
        pass
