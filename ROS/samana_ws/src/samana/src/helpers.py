#!/usr/bin/env python
'''
Gintaras Grebliunas
combinacijus@gmail.com

Helper code
'''

import rospy
from std_msgs.msg import String


class IsFresh:
    '''
        Class for tracking if variables/etc are fresh (updated recently)
    '''

    def __init__(self, timeout, prefix, audio=True):
        '''
            @param timeout:  timeout in seconds
            @param prefix:   prefix text for publishing "{prefix} fresh/stale" on state change
        '''
        self.timeout = rospy.Duration(timeout)
        self.last_updated = rospy.Time(0)
        self.fresh = True
        self.prefix = prefix
        self.audio = audio
        self.audio_pub = rospy.Publisher('text_to_speech', String, queue_size=5)
        self.last_state = self.fresh

    def updated(self):
        '''
            Updates last_updated time. 
            Publishes "{prefix} fresh" on state change to fresh
        '''
        self.last_updated = rospy.Time.now()
        if self.audio and self.fresh is not True:
            rospy.logwarn("REFRESHED {}".format(self.prefix))
            self.audio_pub.publish("{} fresh".format(self.prefix))
        self.fresh = True

    def is_fresh(self):
        '''
            Check is fresh and updates fresh state
            Publishes "{prefix} stale" on state change to stale
        '''
        if rospy.Time.now() - self.last_updated > self.timeout:
            if self.audio and self.fresh is not False and self.last_updated != rospy.Time(0):
                rospy.logerr("STALED {}:  dt: {}  timeout: {}".format(self.prefix, rospy.Time.now() - self.last_updated, self.timeout))
                self.audio_pub.publish("{} stale".format(self.prefix))
            self.fresh = False

            return False
        else:
            return True

    def changed(self):
        '''
            Returns true if state change between this function calls
        '''
        fresh = self.is_fresh()

        if self.last_state is not True and fresh is True:
            self.last_state = True
            return True

        if self.last_state is not False and fresh is False:
            self.last_state = False
            return True

        return False


class BashRunner:
    """
        Runs bash command in order.
        If delay is specified puts command into new thread and sleeps for delay period until executing next command.
        Example usage:
        cmd = BashRunner()
        cmd.add("pwd", delay=3)
        cmd.add("ls -l")
        cmd.run()
    """

    import os
    import threading
    import time

    def __init__(self):
        self.cmd = []
        self.delay = []

    def add(self, cmd, delay=-1):
        self.cmd.append(cmd)
        self.delay.append(delay)

    def run(self):
        """
        Runs bash command in order.
        If delay is specified puts command into new thread and sleeps for delay period until executing next command.
        """
        
        for cmd, delay in zip(self.cmd, self.delay):
            if delay == -1:
                print("Run command: {}".format(cmd))
                self.os.system(cmd)
            else:
                self.threading.Thread(target=self.os.system, args=(cmd,)).start()
                print("Sleep for {} sec".format(delay))
                self.time.sleep(delay)

    def __str__(self):
        s = ""
        for c, d in zip(self.cmd, self.delay):
            s += "{}  | DELAY={}\n".format(c, d)
        
        return s