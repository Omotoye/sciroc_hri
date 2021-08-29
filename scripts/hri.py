#!/usr/bin/env python

# import ROS headers
import rospy
import actionlib

# import PAL Robotics custom headers
from pal_interaction_msgs.msg import TtsAction, TtsGoal

# import hri action message
from sciroc_hri.msg import HRIAction, HRIFeedback, HRIResult

# import dialogflow packages
import os
import dialogflow
from google.api_core.exceptions import InvalidArgument


class HRI:
    def __init__(self, name):
        rospy.loginfo("Human Robot Interaction Node has Started")
        self._feedback = HRIFeedback()
        self._result = HRIResult()
        self._action_name = name
        self.text_to_be_analysed = ""

        # setting up dialogflow
        os.environ[
            "GOOGLE_APPLICATION_CREDENTIALS"
        ] = "gentle-proton-252714-066b7ef02309.json"

        self.DIALOGFLOW_PROJECT_ID = "gentle-proton-252714"
        self.DIALOGFLOW_LANGUAGE_CODE = "en"
        self.SESSION_ID = "sciroc_hri"
        self.session_client = dialogflow.SessionsClient()
        self.session = self.session_client.session_path(
            self.DIALOGFLOW_PROJECT_ID, self.SESSION_ID
        )

        # Initialize the hri_action_server to listen for goal from client
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            HRIAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()

        self.text = ""

    def call_dialogflow(self):
        text_input = dialogflow.types.TextInput(
            text=self.text_to_be_analysed, language_code=self.DIALOGFLOW_LANGUAGE_CODE
        )
        query_input = dialogflow.types.QueryInput(text=text_input)
        try:
            response = self.session_client.detect_intent(
                session=self.session, query_input=query_input
            )
        except InvalidArgument:
            raise
        return response

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
        return True

    def execute_cb(self, goal):
        if goal.mode == 0:
            # Announce Text
            self.text = goal.text
            self._result.result = self.say_something()

        elif goal.mode == 1:
            self.text_to_be_analysed = "Take Order"
            response = self.call_dialogflow()
            # Take Order
            pass

        elif goal.mode == 2:
            # Greet
            pass

        elif goal.mode == 3:
            # Take Item
            pass

        elif goal.mode == 4:
            # Drop Item
            pass

        rospy.loginfo("%s: Succeeded" % self._action_name)
        self._as.set_succeeded(self._result)


if __name__ == "__main__":

    rospy.init_node("sciroc_hri")
