#!/usr/bin/env python

# import ROS headers
import rospy
import actionlib
from std_msgs import String

# import PAL Robotics custom headers
from pal_interaction_msgs.msg import TtsAction, TtsGoal

# import hri action message
from sciroc_hri.msg import HRIAction, HRIFeedback, HRIResult

# import dialogflow packages
import os
import dialogflow
from google.api_core.exceptions import InvalidArgument
from dialogflow_ros import DialogflowClient, DialogflowRequest
from dialogflow_ros.msg import *
import time
from dialogflow_ros.msg import GetOrderAction
import actionlib
from std_msgs.msg import Empty

client=None

class HRI:
    def __init__(self, name):
        rospy.loginfo("Human Robot Interaction Node has Started")
        self._feedback = HRIFeedback()
        self._result = HRIResult()
        self._action_name = name
        rospy.Subscriber("/dialogflow_text", String, self.transcript_cb)
        self.dc = DialogflowClient()
        self.dr = DialogflowRequest()
        self.transcript = " "

        # Initialize the hri_action_server to listen for goal from client
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            HRIAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()

        self.text = ""

    def transcript_cb(self, msg):
        self.transcript = msg

    def call_dialogflow(self, text):
        final_order_str=client()
        
        indx=final_order_str.find(":")
        final_order_str=final_order_str[indx+1:]

        order_list=final_order_str.split()

        return order_list

    def say_something(self):
        client = actionlib.SimpleActionClient("tts/goal", TtsAction)
        rospy.loginfo("Waiting for Server")
        client.wait_for_server()
        rospy.loginfo("Reached Server")
        goal = TtsGoal()
        goal.rawtext.text = self.text
        goal.rawtext.lang_id = "en_GB"
        client.send_goal(goal)
        client.wait_for_result()
        res = client.get_result()

    # def tts_event(self, event):
    #     return {
    #         1: "TTS_EVENT_INITIALIZATION",
    #         2: "TTS_EVENT_SHUTDOWN",
    #         4: "TTS_EVENT_SYNCHRONIZATION",
    #         8: "TTS_EVENT_FINISHED_PLAYING_UTTERANCE",
    #         16: "TTS_EVENT_MARK",
    #         32: "TTS_EVENT_STARTED_PLAYING_WORD",
    #         64: "TTS_EVENT_FINISHED_PLAYING_PHRASE",
    #         128: "TTS_EVENT_FINISHED_PLAYING_SENTENCE",
    #     }[event]

    # def tts_feedbackCb(self, feedback):
    #     print("event type: " + self._tts_event(feedback.event_type))
    #     print("timestamp: " + str(feedback.timestamp))
    #     print("current word: " + feedback.text_said)
    #     print("next word: " + feedback.next_word)
    #     print("-")

    # def say_something(self):
    #     client = actionlib.SimpleActionClient("tts_to_soundplay", TtsAction)
    #     rospy.loginfo("Waiting for Server")
    #     client.wait_for_server()
    #     rospy.loginfo("Reached Server")
    #     goal = TtsGoal()
    #     goal.rawtext.text = self.text
    #     goal.rawtext.lang_id = "en_GB"
    #     client.send_goal(goal, feedback_cb=self.tts_feedbackCb)
    #     client.wait_for_result()
    #     res = client.get_result()
    #     print("text: " + res.text)
    #     print("warning/error msgs: " + res.msg)
    #     print("---")
    #     return True

    def execute_cb(self, goal):
        if goal.mode == 0:
            # Announce Text
            self.text = goal.text
            self.say_something()

        elif goal.mode == 1:
            self.text_to_be_analysed = "Take Order"
            response = self.call_dialogflow()
            # TODO this is the only place dialogflow is needed
            # Take Orderù
            self._result.result=True
            self._result.order_list=response

        elif goal.mode == 2:
            # Greet
            self.text = "Hello there!,  How are you today?"
            self.say_something()

        elif goal.mode == 3:
            # Take Item
            self.text = "Please can you put the items on my tray!,  i will wait for fifteen seconds and then i leave to delevery the order "
            self.say_something()
            time.sleep(15)

        elif goal.mode == 4:
            self.text = "Please take the drinks that you've order from  my tray,  I will wait for fifteen seconds for you to do that"
            self.say_something()
            time.sleep(15)
            # Drop Item

        rospy.loginfo("%s: Succeeded" % self._action_name)
        self._as.set_succeeded(self._result)


if __name__ == "__main__":

    rospy.init_node("sciroc_hri")
    g=HRI()
    client = actionlib.SimpleActionClient('requested_by_hri', GetOrderAction)
    rospy.spin()