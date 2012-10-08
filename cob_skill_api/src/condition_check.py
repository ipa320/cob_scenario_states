#!/usr/bin/env python

#################################################################
##\file
#
# \note
# Copyright (c) 2012 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: Care-O-bot Research
# \note
# ROS package name: cob_skill_api
#
# \author
# Author: Thiago de Freitas Oliveira Araujo, 
# email:thiago.de.freitas.oliveira.araujo@ipa.fhg.de
# \author
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: August 2012
#
# \brief
# Definition of the conditions checks for the Skill API
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see < http://www.gnu.org/licenses/>.
#
#################################################################

import roslib
roslib.load_manifest('cob_skill_api')
import rospy
import smach
import smach_ros
from actionlib import *
from actionlib.msg import *
from diagnostic_msgs.msg import DiagnosticArray

import tf
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion

from simple_script_server import *
sss = simple_script_server()

from kinematics_msgs.srv import *
from sensor_msgs.msg import *
from move_base_msgs.msg import *

from abc_conditioncheck import ConditionCheck

import rostopic

class ConditionCheck(ConditionCheck):

    def __init__(self, checkType = "pre_check"):
        
        self.state = self.create_state()
        self.state.execute = self.execute
        
        self.checkType = checkType
        self.checks = rospy.get_param(self.checkType)
        self.robot_name = rospy.get_param('robot_name')
        
        if rospy.has_param('required_components') and rospy.has_param('optional_components'):
            self.required_components = rospy.get_param('required_components')
            self.optional_components = rospy.get_param('optional_components')
            self.full_components = self.required_components + " " + self.optional_components
        
        elif rospy.has_param('required_components'):
            self.required_components = rospy.get_param('required_components')
            self.optional_components = ""
            self.full_components = self.required_components
        
        elif rospy.has_param('optional_components'):
            self.optional_components = rospy.get_param('optional_components')
            self.required_components = ""
            self.full_components = self.optional_components
        
        else:
            self.required_components = ""
            self.optional_components =""
            self.full_components = ""
        
        self.required_components = self.required_components.split()
        self.optional_components = self.optional_components.split()
        self.full_components = self.full_components.split()
        
        self.tfL = tf.TransformListener()
        
        self.result = "failed"
        
        self.component_check = {}
    	self.iters = 30 # trials to get component on diagnostics message
        self.diagnostics_subscriber = rospy.Subscriber('diagnostics', DiagnosticArray, self.diagnostics_callback)
        
        rospy.wait_for_message('diagnostics', DiagnosticArray, timeout=10)
        
    ####################################################################
    # function: create_state()
    # Main routine of the State Machine
    ####################################################################
    
    def create_state(self, outcomes=['success','failed'], input_keys=['pose'], output_keys = ['pose']):
             
        return smach.State(outcomes, input_keys, output_keys)
    
    ####################################################################
    # function: execute()
    # Main routine of the State Machine
    ####################################################################
    
    def execute(self, userdata):
    
        for check in self.checks:
            
            
            getattr(self, check.keys()[0])(check, userdata)
            
            if (self.result == "failed"):
                rospy.logerr("Check failure on <<%s>>"%check)
                return self.result
           
            
        # announce ready
        if ("sound" in self.full_components) and (self.result == "success"):
            sss.say(["Ready."])
        
        return self.result
    
    #####################################################################
    # function: joint_configuration_check()
    # This function is responsible for checking the 
    # Joint configuration of the Robot before performing the skill action
    #####################################################################
    
    def joint_configuration_check_js(self, params, userdata): # from parameter server, get names and supposed states
    
        rospy.loginfo("<<joint_configuration_check_js>>")
        rospy.loginfo("This is part of the <<%s>>", self.checkType)
        
        for item in params.values()[0]:
            
            joint_names = item['joint_names']
            joint_states = item['joint_states']
            aw_error = item['allowed_error']
            
            self.joint_configuration_check(joint_names,joint_states, aw_error)
            
            if(self.result=="failed"):
                return
    
    
    def joint_configuration_check_ss(self, params, userdata): # get names and states from script server
    
        rospy.loginfo("<<joint_configuration_check_ss>>")
        
        for item in params.values()[0]:
            component = item['component']
            configuration = item['configuration']
            
            ss_names_path = "/script_server/" + component + "/joint_names"
            ss_values_path = "/script_server/" + component + "/" + configuration
            
            if rospy.has_param(ss_names_path):
                joint_names = rospy.get_param(ss_names_path)
            else:
                self.result ="failed"
                rospy.logerr("There is no " + ss_names_path + " on parameters server." )
                return
            
            if rospy.has_param(ss_values_path):
                joint_states = rospy.get_param(ss_values_path)[0]
            else:
                self.result = "failed"
                rospy.logerr("There is no " + ss_values_path + " on parameters server." )
                return
            
            aw_error = item['allowed_error']
            
            self.joint_configuration_check(joint_names,joint_states, aw_error)
    
    def joint_configuration_check(self, joint_names, joint_states, aw_error):
    
        joints = zip(joint_names, joint_states)
        
         ##################
         # This is for avoiding the need for redefining the yaml file all time
        comp_names = []
        for m in joint_names:

            comp_names.append(m[:m.index("_")])

        for c in comp_names:
            if (c not in self.full_components):
                rospy.loginfo("TEST SKIPPED...") 
                rospy.loginfo("There is a definition to check the component <<" + c + ">> that was not defined as required")
                rospy.loginfo("Please remove this check from the configuration file.")
                self.result = "success"
                return
            
        ###################
        
        jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)

        #TODO: Try to ellaborate a better approach to this:
        # Description: this avoids that the state machine hangs on the robot when the joint states is not available
        trials = 30
        while joint_names[0] not in jointsMsg.name:
            if (trials==0):
                rospy.logerr("Exceeded maximum amount of trials for waiting for <<" + joint_names[0] + ">> on /joint_states messages")
                rospy.loginfo("Please remove it from the configuration file, if not necessary for running the robot skill.")
                self.result = "failed"
                return
            trials-=1
         # END   

            jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState)

        try:
        
            for name, state in joints:
            
                rospy.loginfo("Checking the <<%s>> joint"%name)              
                
                value = jointsMsg.position[jointsMsg.name.index(name)]
                
                assert abs(value - state) <= aw_error, "Error on the Joint Position for the <<%s>>"%name
        
        except AssertionError,e:
            self.result = "failed"
            rospy.logerr("<<Error Message>>:%s"%e)
            rospy.logerr("at joint_configuration_check")
            return
        
        
        self.result = "success"
    
    ####################################################################
    # function: component_ready_check()
    # This function is responsible for checking the initial conditions
    # provided by the user for proper operation of the Robot
    #####################################################################
    
    def component_ready_check(self, params, userdata): #Simplified due to the current simulation possibilities
        
        for item in params.values()[0]:  
            
            try:
                print "COMPONENT STATUS", self.component_check
                while (self.iters>0):
                    pass
                component_present = item in self.component_check    
                assert component_present == True, "Component not found on scan trials on the diagnostics message"    
                    
                rospy.loginfo("Started to check if all components are ready")
                    
                status = self.component_check[item]["status"]

                assert status == 0, "<<" + item +">>" + " component is not ready yet."
   
            except AssertionError,e:
                self.result = "failed"
                rospy.loginfo(item + " not found")
                rospy.logerr("<<Error Message>>:%s"%e)
                rospy.logerr("at:%s"%params)
                return

        rospy.loginfo("All Necessary components are present.") 
        self.result = "success"
    
    
    ####################################################################
    # function: action_check()
    # This function is responsible for checking the availability of an 
    # action
    #####################################################################
    
    
    def action_check(self, params, userdata):
    
        rospy.loginfo("Checking <<actions>>")
        result_summary = {}
        self.result = "success"        
        for item in params.values()[0]:
        
            action_type = item['action_type']
            action_name = item['action_name']
            
            rospy.loginfo("Now checking <<%s>>", action_name)
            rospy.loginfo("Of type <<%s>>", action_type)
            
            #************
            # Description: Get topic type and uses that for dynamically importing the required modules
            imp_action = rostopic.get_topic_type("/"+action_name+"/goal", blocking=True)
            imp_action = imp_action[0].split("/")[0]
            imp_action += ".msg"
            
            mod = __import__(imp_action, fromlist=[action_type]) # from move_base_msg.msg
            cls = getattr(mod, action_type) # import MoveBaseAction
            #****************************
            ac_client = actionlib.SimpleActionClient(action_name, cls)
            
            result = ac_client.wait_for_server(rospy.Duration(5))
            result_summary[action_name] = result # this make a summary for the results for all the actions checked
            
        if result == False:
            self.result = "failed"

        rospy.loginfo("Result of the Actions Check")
        rospy.loginfo(result_summary)    
        rospy.loginfo("Finished Checking <<actions>>")
    
    
    ####################################################################
    # function: pose_check()
    # This function is responsible for checking if the Robot position
    # is convenient for performing the skill action
    ####################################################################
    
    def pose_check(self, params, userdata):
    
        try:
            rospy.loginfo("Checking the Robot <<Pose>>")
            rospy.loginfo("This is part of the <<%s>>", self.checkType)
            
            for item in params.values()[0]:
            
                reference_frame = item['reference_frame']
                target_frame = item['target_pose']['frame_id']
                
                ##################
                # This is for avoiding the need for redefining the yaml file all time
                comp_name = reference_frame.strip("/")
                comp_name = comp_name[:comp_name.index("_")]
                
                if (comp_name not in self.full_components):
                    rospy.loginfo("TEST SKIPPED...") 
                    rospy.loginfo("There is a definition to check the pose of the component <<" + comp_name + ">>, but it is not listed as required")
                    rospy.loginfo("Please remove this check from the configuration file, if testing a new skill.")
                    continue
                
                ###################
                
                mes = "Checking the " + reference_frame + " against the " + target_frame
                
                rospy.loginfo(mes)
                
                self.tfL.waitForTransform(target_frame, reference_frame, rospy.Time(), rospy.Duration(20.0))
                
                (trans,rot) = self.tfL.lookupTransform(target_frame, reference_frame, rospy.Time(0))
                
                angles = euler_from_quaternion(rot)
                
                xy_goal_tolerance = item['allowed_position_error']
                yaw_goal_tolerance = item['allowed_orientation_error']
                
                if (item['target_pose']['position'] == "userdata"): # kept this for analyzing user defined goals(post_check)
                         
                    assert abs(trans[0] - userdata.pose[0]) <= xy_goal_tolerance, "Error on the X axis position"
                    assert abs(trans[1] - userdata.pose[1]) <= xy_goal_tolerance, "Error on the Y axis position"
                    assert abs(angles[2] - userdata.pose[2]) <= yaw_goal_tolerance, "Error on the Angle"
                     
                    rospy.set_param("position_exists", trans)
                    rospy.set_param("orientation_exists", angles)
                    
                else:
                
                    for pos in range(len(trans)):
                        
                        ########## TODO HACK FIXME: WARNING: HACK FOR PRE_CHECK USING THE CURRENT POSITION
                        # Description: Sets the current translation using the last received goal
                        if(rospy.has_param("position_exists") and target_frame == "/map"):
                            item['target_pose']['position'][pos] = rospy.get_param("position_exists")[pos]
                        ########## HACK END
                        
                        messageX = "Position " + (str)(pos) + ":" +  " Real Position: " +  (str)(trans[pos]) + ", Target Position: " + \
                            (str)(item['target_pose']['position'][pos]) + ", Tolerance: "+ (str)(xy_goal_tolerance)
                        
                         
                        assert abs(trans[pos] - item['target_pose']['position'][pos]) <= xy_goal_tolerance, "Error on the position %s"%messageX
                        
                        rospy.loginfo(messageX)
                    
                    for ori in range(len(angles)):
                        
                        ########## TODO HACK FIXME: WARNING: HACK FOR PRE_CHECK USING THE CURRENT ORIENATION
                        # Description: Sets the current orientation using the last received goal 
                        if(rospy.has_param("orientation_exists") and target_frame == "/map" ):
                            item['target_pose']['orientation'][ori] = rospy.get_param("orientation_exists")[ori]
                        ########## HACK END
                        
                        messageA = "Orientation " + (str)(ori) + ":" + " Real Orientation: " + (str)(angles[ori]) + ", Target Orientation: " + \
                            (str)(item['target_pose']['orientation'][ori]) + ", Tolerance: "+ (str)(yaw_goal_tolerance)
                        
                        assert abs(angles[ori] - item['target_pose']['orientation'][ori]) <= yaw_goal_tolerance, "Error on the orientation %s"%messageA
                        
                        rospy.loginfo(messageA)
            
        except AssertionError,e:
            self.result = "failed"
            rospy.logerr("<<Error Message>>:%s"%e)
            rospy.logerr("at pose_check")
            return
        
        
        self.result = "success"
    
    
    ####################################################################
    # function: init_components()
    # Initializes robot components according to the components available
    # for the Robot
    ####################################################################
    
    def init_components(self, params, userdata):
    
        
        if "sound" in self.full_components:
            sss.say(["Preparing."],False)
        # bring robot into the starting state
        
        for item in params.values()[0]:
        
            comp_name = item.keys()[0]
            param_name = item.values()[0]
            
            if comp_name in self.full_components:
                handler = sss.move(comp_name, param_name, False)
                handler.wait()
            else:
                rospy.loginfo("Tried to initialize the component <<" + comp_name + ">> that is not declared as required.")
                rospy.loginfo("Please, to avoid this message, remove the component from this test definition.")
            
        # wait for all movements to be finished
        # announce ready
        
        if "sound" in self.full_components:
            sss.say(["Ready."])
        
        sss.sleep(1)
        
        rospy.loginfo("Components successfully initialized.")
        
        self.result = "success"
    
    def diagnostics_callback(self, msg):
        if(self.iters >0):
            self.iters -= 1
		
        self.component_check[msg.status[0].name] = {"status": -1}   
        self.component_check[msg.status[0].name]["status"] = msg.status[0].level
