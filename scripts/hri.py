#!/usr/bin/env python

# import ROS headers
import rospy
import actionlib

# import PAL Robotics custom headers
from pal_interaction_msgs.msg import TtsAction, TtsGoal


class HRI:
    def __init__(self):
        rospy.loginfo("Human Robot Interaction Node has Started")
        # Initialize the hri_action_server to listen for goal from client
        self.text = ""

    def tts_event(self, event):
        return {
            1: "TTS_EVENT_INITIALIZATION",
            2: "TTS_EVENT_SHUTDOWN",
            4: "TTS_EVENT_SYNCHRONIZATION",
            8: "TTS_EVENT_FINISHED_PLAYING_UTTERANCE",
            16: "TTS_EVENT_MARK",
            32: "TTS_EVENT_STARTED_PLAYING_WORD",
            64: "TTS_EVENT_FINISHED_PLAYING_PHRASE",
            128: "TTS_EVENT_FINISHED_PLAYING_SENTENCE",
        }[event]

    def tts_feedbackCb(self, feedback):
        print("event type: " + self._tts_event(feedback.event_type))
        print("timestamp: " + str(feedback.timestamp))
        print("current word: " + feedback.text_said)
        print("next word: " + feedback.next_word)
        print("-")

    def say_something(self):
        client = actionlib.SimpleActionClient("tts_to_soundplay", TtsAction)
        rospy.loginfo("Waiting for Server")
        client.wait_for_server()
        rospy.loginfo("Reached Server")
        goal = TtsGoal()
        goal.rawtext.text = self.text
        goal.rawtext.lang_id = "en_GB"
        client.send_goal(goal, feedback_cb=self.tts_feedbackCb)
        client.wait_for_result()
        res = client.get_result()
        print("text: " + res.text)
        print("warning/error msgs: " + res.msg)
        print("---")


if __name__ == "__main__":

    rospy.init_node("sciroc_hri")

