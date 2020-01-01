#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import pyttsx3


class TTS:
    def __init__(self):
        self.queue = []  # String lists
        self.reading = False
        self.max_queue_size = 15
        self.wpm = 240  # NOTE: tunnign
        self.wpm_fast = 400

        self.tts = pyttsx3.init()
        # 200WPM normal rate upto 300WPM quite normal and might be possible upto 350WPM
        self.tts.setProperty('rate', self.wpm_fast)
        self.tts.say("yup p")
        self.tts.runAndWait()
        self.tts.setProperty('rate', self.wpm)

        # Init ROS node
        rospy.init_node("text_to_speech")
        rospy.Subscriber("text_to_speech", String, self.tts_cb,)

        while True:
            self.read_queue()

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
    except rospy.ROSInterruptException:
        pass
