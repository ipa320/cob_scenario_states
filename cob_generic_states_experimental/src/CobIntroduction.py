#!/usr/bin/python
import roslib
roslib.load_manifest('cob_generic_states_experimental')
import rospy
import smach
import smach_ros

import random

from simple_script_server import *
sss = simple_script_server()

class CobIntroductionInit(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
			
    def execute(self, userdata):
        handle_base = sss.init("base")
        handle_torso = sss.init("torso")
        handle_tray = sss.init("tray")
        handle_sdh = sss.init("sdh")
        handle_arm = sss.init("arm")
        handle_head = sss.init("head")

        if handle_base.get_error_code() != 0:
            return "failed"
        if handle_torso.get_error_code() != 0:
            return "failed"
        if handle_tray.get_error_code() != 0:
            return "failed"
        if handle_sdh.get_error_code() != 0:
            return "failed"
        if handle_arm.get_error_code() != 0:
            return "failed"
        if handle_head.get_error_code() != 0:
            return "failed"

        return "succeeded"

class CobIntroductionRecover(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
			
    def execute(self, userdata):
        handle_base = sss.recover("base")
        handle_torso = sss.recover("torso")
        handle_tray = sss.recover("tray")
        handle_sdh = sss.recover("sdh")
        handle_arm = sss.recover("arm")
        handle_head = sss.recover("head")

        if handle_base.get_error_code() != 0:
            return "failed"
        if handle_torso.get_error_code() != 0:
            return "failed"
        if handle_tray.get_error_code() != 0:
            return "failed"
        if handle_sdh.get_error_code() != 0:
            return "failed"
        if handle_arm.get_error_code() != 0:
            return "failed"
        if handle_head.get_error_code() != 0:
            return "failed"

        return "succeeded"

class CobIntroductionPrepare(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
			
    def execute(self, userdata):
        handle_torso = sss.move("torso", "home",False)
        handle_tray = sss.move("tray", "down",False)
        handle_sdh = sss.move("sdh", "home",False)
        handle_arm = sss.move("arm", "folded",False)
        handle_head = sss.move("head", "front",False)

        handle_torso.wait()
        handle_tray.wait()
        handle_sdh.wait()
        handle_arm.wait()
        handle_head.wait()

        if handle_torso.get_error_code() != 0:
            return "failed"
        if handle_tray.get_error_code() != 0:
            return "failed"
        if handle_sdh.get_error_code() != 0:
            return "failed"
        if handle_arm.get_error_code() != 0:
            return "failed"
        if handle_head.get_error_code() != 0:
            return "failed"

        return "succeeded"

class CobIntroduction(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'])
			
    def execute(self, userdata):
        handle_torso = sss.move("torso","nod",False)
        handle_say = sss.say(["Hello, nice to meet you. My name is Care-O-bot. I am a mobile service robot build by Fraunhofer I. P. A., in Stuttgart and I am designed as a household assistant. My job is to help for example elderly people to stay longer at home, so that they do not have to go to a care facility."],False)
        handle_torso.wait()
        sss.move("torso","left")
        rospy.sleep(1)
        sss.move("torso","right")
        rospy.sleep(1)
        sss.move("torso","left")
        rospy.sleep(1)
        sss.move("torso","right")
        rospy.sleep(1)
        sss.move("torso","home")
        handle_say.wait()
        
        sss.say(["I will show you some of my capabilities in a second."])
        
        # sound and led
        sss.say(["First of all, I can speak to you and change colors. This way I can express my mood and intention."])

        sss.set_light("yellow")
        handle_say = sss.say(["I normally light up in yellow if I am moving some of my hardware components, so please pay attention."], False)
        handle_say.wait()

        sss.set_light("blue")
        handle_say = sss.say(["Blue means that I am heavily thinking which means I am for example calculating a collision free path."])
        handle_say.wait()

        sss.set_light("green")
        handle_say = sss.say(["If I light up in green, everything is fine and I am ready to be at your service."])
        handle_say.wait()
        rospy.sleep(1)

        # tray
        sss.set_light("yellow")
        handle_tray = sss.move("tray", "up", False)
        sss.say(["I have a tray that can be used to receive or hand over objects from people."])
        handle_tray.wait()
        sss.set_light("green")

        # head and cameras
        sss.say(["With my torso I can perform gestures like nodding."],False)
        sss.move("torso", "nod")
        sss.say(["or shaking"])
        sss.move("torso","shake")
        sss.say(["In my head there are cameras. I use them to recognize people."])

        # seek attention
        sss.move("torso","left")
        sss.say(["Hey there on the left!"])
        rospy.sleep(1)
        sss.say(["Keep your eyes focused on me!"])
        rospy.sleep(2)

        sss.move("torso","home", False)
        sss.say(["I can also flip my cameras to the back side, see."])
        sss.move("head","back")
        sss.say(["On the backside I can use my cameras to detect objects."])

        # arm
        sss.say(["Oh, what is that?"])
        sss.say(["I have an arm and a gripper on the back side."])
        sss.set_light("yellow")
        handle_arm = sss.move("arm",["intermediateback", "intermediatefront"], False)
        sss.move("head","front", False)
        sss.move("torso","front_left", False)
        sss.say(["Lets move them to the front so that you can see them. My functional design follows a two side interaction concept: Normally I use the arm on the backside to manipulate objects and use the tray to safely hand over objects to humans on the front side."])
        handle_arm.wait()
        sss.set_light("green")

        # gripper
        handle_sdh = sss.move("sdh","cylopen",False)
        sss.move("torso","front_right", False)
        handle_say = sss.say(["If I want to grasp an object I can use my gripper. My gripper is very flexible it can adapt to various objects. I can grasp box shaped or cylindrical objects."],False)
        handle_sdh.wait()
        sss.move("sdh","cylclosed")
        sss.move("sdh","home")
        handle_say.wait()

        sss.say(["My gripper can also hold spherical objects like a ball or an apple."], False)
        sss.move("sdh","spheropen")
        sss.move("torso","home", False)
        sss.move("sdh","spherclosed")
        sss.move("sdh","home",False)

        # arm back
        sss.set_light("yellow")
        sss.say(["Are you already impressed? Wait, let me go on."], False)
        handle_arm = sss.move("arm",["intermediatefront", "intermediateback", "folded"], False)
        sss.move("tray","down", False)

        # base
        sss.say(["I have an omnidirectional base which allows me to turn on the spot."], False)
        sss.move_base_rel("base",[0, 0, 1.57])

        sss.say(["I can also move forward and backwards"], False)
        sss.move_base_rel("base",[0.1, 0, 0])
        sss.move_base_rel("base",[-0.1, 0, 0])

        sss.say(["and sidewards. This helps a lot if I get you a drink in your home. Using my safety laser scanners in the base I can safely navigate between humans."], False)
        sss.move_base_rel("base",[0, 0.1, 0])
        sss.move_base_rel("base",[0, -0.1, 0])
        sss.move_base_rel("base",[0, 0, -1.57])
        sss.set_light("green")

        # please take joystick
        sss.move("torso","left", False)
        sss.say(["Me and my brothers are in use as a research platform all over the world. If you are interessted you can rent or buy me for your own research."])
        sss.move("torso","front_right", False)
        sss.say(["Now you know who I am and what I can do, so its your turn. Please take the joystick to drive me around and have fun!"])
        sss.move("torso",["nod"])

        return 'succeeded'

class Explore(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
								outcomes=['finished','failed'])
        with self:

            smach.StateMachine.add('COB_INTRODUCTION_PREPARE',CobIntroductionPrepare(),
                                   transitions={'succeeded':'COB_INTRODUCTION',
                                                'failed':'failed'})

            smach.StateMachine.add('COB_INTRODUCTION',CobIntroduction(),
                                   transitions={'succeeded':'finished',
                                                'failed':'failed'})

















class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:
            smach.StateMachine.add('STATE',Explore(),
                transitions={'finished':'ended',
                            'failed':'ended'})

if __name__=='__main__':
    rospy.init_node('cob_introduction')
    sm = SM()
    sis = smach_ros.IntrospectionServer('SM', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
