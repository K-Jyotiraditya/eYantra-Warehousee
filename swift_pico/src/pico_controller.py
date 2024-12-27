#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/pid_error			/throttle_pid
						/pitch_pid
						/roll_pid
					
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

'''
# Theme:            Warehouse Drone
# Author:           Kalepu Jyotiraditya
# Filename:         pico_controller
# Functions:        disarm,arm,whycon_callback,altitude_set_pid,roll_set_pid,pitch_set_pid,
					pid,constrain_throttle,constrain_values,main

# Global variables: self.drone_position, self.setpoint, self.whycon_setpoint, self.cmd, self.pid_error,
# 					 self.Kp, self.Ki, self.Kd, self.out_roll, self.out_pitch, self.out_throttle,
# 					 self.prev_error, self.current_error, self.integral_error, self.sample_time, 
# 					self.command_pub, self.pid_error_pub, and self.timer
'''


# Importing the required libraries

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node



class Swift_Pico(Node):
	def __init__(self):
		super().__init__('pico_controller')  # initializing ros node with name pico_controller

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0, 0.0, 0.0]

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [2, 2, 19]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

		# Declaring a cmd of message type swift_msgs and initializing values
		self.whycon_setpoint=self.setpoint

		self.cmd = SwiftMsgs()
		self.pid_error= PIDError()
		self.cmd.rc_roll = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_throttle = 1500

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [0.03*200 , 0.03*200 , 0.03*1280]
		self.Ki = [0.00004*5 , 0.00004*5 , 0.0004*170]
		self.Kd = [0.06*350 , 0.06*350 , 0.06*850]

	

		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.out_roll=0.0
		self.out_pitch=0.0
		self.out_throttle=0.0
		# These will store the PID output values for roll, pitch, and throttle.


	
		self.prev_error = [0,0,0]				#[pitch, roll, throttle]
		self.current_error = [0,0,0]
		self.integral_error = [0,0,0]
		# These are for storing the current and previous errors (for proportional and derivative control) 
        # and the integral of the error (for integral control).





		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit.
	
		self.sample_time = 0.060  # in seconds

		# Publishing /drone_command, /pid_error
		self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
		self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

		#------------------------Add other ROS 2 Publishers here-----------------------------------------------------
	

		# Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
		self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
		self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
		self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
		self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)


		#------------------------Add other ROS Subscribers here-----------------------------------------------------
		
		self.arm()  # ARMING THE DRONE

		# Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)
		self.timer = self.create_timer(self.sample_time, self.pid)

	def disarm(self):
		self.cmd.rc_roll = 1000
		self.cmd.rc_yaw = 1000
		self.cmd.rc_pitch = 1000
		self.cmd.rc_throttle = 1000
		self.cmd.rc_aux4 = 1000
		self.command_pub.publish(self.cmd)
		

	def arm(self):
		self.disarm()
		self.cmd.rc_roll = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_throttle = 1500
		self.cmd.rc_aux4 = 2000
		self.command_pub.publish(self.cmd)  # Publishing /drone_command


	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self, msg):
		self.drone_position[0] = msg.poses[0].position.x 
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z 

		# Callback function that gets executed when /whycon publishes the drone's position.
        # It updates the drone's current position in [x, y, z].


	
		#---------------------------------------------------------------------------------------------------------------


	# Callback function for /throttle_pid
	# This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
	def altitude_set_pid(self, alt):
		# self.Kp[2] = alt.kp * 0.03  # This is just for an example. You can change the ratio/fraction value accordingly
		# self.Ki[2] = alt.ki * 0.0004
		# self.Kd[2] = alt.kd * 0.06
		pass

	def roll_set_pid(self, roll):
		# self.Kp[0] = roll.kp * 0.03  # This is just for an example. You can change the ratio/fraction value accordingly
		# self.Ki[0] = roll.ki * 0.00004
		# self.Kd[0] = roll.kd * 0.06
		pass

	def pitch_set_pid(self, pitch):
		# self.Kp[1] = pitch.kp * 0.03  # This is just for an example. You can change the ratio/fraction value accordingly
		# self.Ki[1] = pitch.ki * 0.00004
		# self.Kd[1] = pitch.kd * 0.06
		pass




	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum




		print("Drone_setpoint = ",self.setpoint)
		print("Drone_position = ",self.drone_position)

		# Step 1: Calculate current errors for [x, y, z] (roll, pitch, throttle) based on the difference 
        # between setpoints and current drone position.
		self.current_error = [(self.setpoint[0] - self.drone_position[0]),
							  (self.setpoint[1] - self.drone_position[1]) , 
							  (self.setpoint[2] - self.drone_position[2])] 


		# Step 2: Update the integral error by adding the current error.
		self.integral_error = [(self.integral_error[0] + self.current_error[0]) , 
						 	   (self.integral_error[1] + self.current_error[1]) , 
							   (self.integral_error[2] + self.current_error[2])]

		 # Step 3: Constrain the integral error for throttle (z-axis) to prevent windup.
		self.integral_error[2] = self.constrain_throttle()


		# Step 4: Calculate PID outputs for throttle, roll, and pitch based on the errors.
		self.out_throttle = self.Kp[2]*self.current_error[2] + (self.Kd[2]*(self.current_error[2]-self.prev_error[2]))/self.sample_time  + self.Ki[2]*self.integral_error[2]

		self.out_roll = self.Kp[0]*self.current_error[0] + (self.Kd[0]*(self.current_error[0]-self.prev_error[0]))/self.sample_time  + self.Ki[0]*self.integral_error[0]

		self.out_pitch = self.Kp[1]*self.current_error[1] + (self.Kd[1]*(self.current_error[1]-self.prev_error[1]))/self.sample_time  + self.Ki[1]*self.integral_error[1]
 

		print("Kp = ",self.Kp)
		print("Kd = ",self.Kd)
		print("Ki = ",self.Ki)
		print("Output_throttle = ",self.out_throttle)
		print("Output_roll = ",self.out_roll)
		print("Output_pitch = ",self.out_pitch)

        # Step 5: Update the drone commands by applying the calculated PID output values.
		self.cmd.rc_roll = 1500 + int(self.out_roll)
		self.cmd.rc_yaw = 1500 
		self.cmd.rc_pitch = 1500 - int(self.out_pitch)
		self.cmd.rc_throttle = 1500 - int(self.out_throttle)
		self.cmd.rc_aux4 = 2000


		print("Non_constraint_Roll=",self.cmd.rc_roll)
		print("Non_constraint_Pitch",self.cmd.rc_pitch )
		print("Non_constraint_Throttle=",self.cmd.rc_throttle)

        # Step 6: Constrain the final command values to stay within valid ranges.

		self.cmd.rc_roll,self.cmd.rc_pitch,self.cmd.rc_throttle=self.constrain_value()

		print("current_error=",self.current_error)
		print("Position_error_Z",self.current_error[2] )

		print("constraint_Roll=",self.cmd.rc_roll)
		print("constraint_Pitch",self.cmd.rc_pitch )
		print("constraint_Throttle=",self.cmd.rc_throttle)
		print("Integral error_throttle=",self.integral_error)
		print("Constraint Integral error_throttle=",self.integral_error)
		print(" ")

        # Step 7: Publish the calculated commands to the /drone_command topic.
		self.command_pub.publish(self.cmd)


		# Step 8: Store current errors as previous errors for the next iteration.
		self.prev_error=self.current_error

		# Step 9: Publish the current PID errors (for roll, pitch, throttle) to the /pid_error topic.
		self.pid_error.roll_error=self.current_error[1]
		self.pid_error.pitch_error=self.current_error[0]
		self.pid_error.throttle_error= self.current_error[2]
		self.pid_error_pub.publish(self.pid_error)

	def constrain_throttle(self):
		 # Constrains the integral error for throttle to avoid excessive accumulation (anti-windup).
		constraint_integral=max(-370, min(self.integral_error[2], -300))
		return constraint_integral


	def constrain_value(self):
		# Constrains the final command values for roll, pitch, and throttle to be within valid ranges.
		roll_constraint=max(1000, min(self.cmd.rc_roll, 1600))
		pitch_constraint=max(1000, min(self.cmd.rc_pitch, 1600))
		throttle_constraint=max(1000, min(self.cmd.rc_throttle, 1685))
		return roll_constraint, pitch_constraint,throttle_constraint


def main(args=None):
	rclpy.init(args=args)
	swift_pico = Swift_Pico()
	rclpy.spin(swift_pico)
	swift_pico.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()








