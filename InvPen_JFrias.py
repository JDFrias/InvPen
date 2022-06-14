#Project: Inverted Pendulum
#Class: EEL5171 - group 1
#Author: Julian Frias
#Description: This Project is a rough Simulation for an Inverted Pendulum using PID Controllers sourced from the following Libraries
#numpy, opencv-python, math, time, matplotlib, and control

import numpy as np,cv2,math,time,matplotlib.pyplot as plt,sys
from control.matlab import *

class Cart:
	def __init__(self,x,mass,sim_size):
		self.x = x  
		self.y = int(0.6*sim_size) 		
		self.mass = mass
		self.color = (255,0,0)				#bgr(,,) 0-255 for intenstity. I chose blue for FIU

class Pendulum:
	def __init__(self,length,theta,ball_mass):
		self.length = length
		self.theta = theta
		self.ball_mass = ball_mass		
		self.color = (0,223,255)			#bgr(,,) 0-255 for intesity. I chose Gold for FIU

def simulation_build(sim_size,cart,pendulum):
	# This function constructs the simulation enviroment
	display_length = pendulum.length * 100
	initial_matrix = np.zeros((sim_size,sim_size,3),np.uint8)

	# This creates a origin horizon line
	cv2.line(initial_matrix,(0,int(0.6 * sim_size)),(sim_size,int(0.6 * sim_size)),(255,255,255),2)

	# Constructing the cart
	cv2.rectangle(initial_matrix,(int(cart.x) + 30,cart.y + 20),(int(cart.x) - 30,cart.y - 20),cart.color,-1)

	# Contructing the pendulum	
	pendulum_x_endpoint = int(cart.x - (display_length) * math.sin(pendulum.theta))
	pendulum_y_endpoint = int(cart.y - (display_length) * math.cos(pendulum.theta))
	cv2.line(initial_matrix,(int(cart.x),cart.y),(pendulum_x_endpoint,pendulum_y_endpoint),pendulum.color,4)
	cv2.circle(initial_matrix,(pendulum_x_endpoint,pendulum_y_endpoint),6,(255,255,255),-1)
	
	#Instantiating the simulation window
	cv2.imshow('EEL5171 - PID Controlled Inverted Pendulum',initial_matrix)
	cv2.waitKey(5) #required after imshow(); in milliseconds so that you can ctrl+c out of the simulation

def calc_controlInput(cart,pendulum,time_delta,error,previous_error,integral,g):
	# Using PID to find control inputs

	Kp = -100 #Proportional(present)
	Kd = -15 #Derivative(future)	
	Ki = -10 #Integral(Past)

	derivative = (error - previous_error) / time_delta #maintains rate of change 

	integral += error * time_delta #maintains sum of total input over time

	F = (Kp * error) + (Kd * derivative) + (Ki * integral) #Here is critical as it produces the Force for the actuated signal of this control system
	return F,integral



def run_controlInput(cart,pendulum,F,time_delta,x_tminus2,theta_d,theta_tminus2,previous_time_delta,g):
	# Here we calculate the second derivative of Theta also known as Angular Velocity
	theta_dd = (((cart.mass + pendulum.ball_mass) * g * math.sin(pendulum.theta)) + (F * math.cos(pendulum.theta)) - (pendulum.ball_mass * ((theta_d)**2.0) * pendulum.length * math.sin(pendulum.theta) * math.cos(pendulum.theta))) / (pendulum.length * (cart.mass + (pendulum.ball_mass * (math.sin(pendulum.theta)**2.0)))) 
	
	# Here we calculate the second derivative of X also known as Linear Acceleration of the Cart
	x_dd = ((pendulum.ball_mass * g * math.sin(pendulum.theta) * math.cos(pendulum.theta)) - (pendulum.ball_mass * pendulum.length * math.sin(pendulum.theta) * (theta_d**2)) + (F)) / (cart.mass + (pendulum.ball_mass * (math.sin(pendulum.theta)**2)))
	
	#Here was aggregate the calculated x and theta of the system using the newly derived linear acceleration and angular velocity
	cart.x += ((time_delta**2) * x_dd) + (((cart.x - x_tminus2) * time_delta) / previous_time_delta)
	
	pendulum.theta += ((time_delta**2)*theta_dd) + (((pendulum.theta - theta_tminus2)*time_delta)/previous_time_delta)

def calc_error(pendulum):
	# iteratively defining the error
	previous_error = (pendulum.theta % (2 * math.pi)) - 0
	if previous_error > math.pi:
		previous_error = previous_error - (2 * math.pi)
	return previous_error

def build_graphs(times,errors,theta,force,x):
	# This function plots all the graphs for the state variables over time 
	
	plt.subplot(4, 1, 3)
	plt.plot(times,force,'-b')
	plt.ylabel('Force')
	plt.xlabel('Time')

	plt.subplot(4, 1, 4)
	plt.plot(times,x,'-b')
	plt.ylabel('X')
	plt.xlabel('Time')
	
	plt.subplot(4, 1, 1)
	plt.plot(times,errors,'-b')
	plt.ylabel('Error')
	plt.xlabel('Time')

	plt.subplot(4, 1, 2)
	plt.plot(times,theta,'-b')
	plt.ylabel('Theta')
	plt.xlabel('Time')

	plt.show()




#Now we will show the instance module driver

def main():
	# Initializing environment variables for the simulation
	massBall = 1.0	#m
	massCart = 5.0	#M
	g = 9.81
	errors = []
	force = []
	theta = []
	times = []
	x = []
	sim_size = 1000
	sim_time = 30		#here we can change the time duration for the simulation
	previous_timestamp = time.time()
	end_time = previous_timestamp + sim_time

	# Initializing cart and pendulum objects
	cart = Cart(int(0.2 * sim_size),massCart,sim_size)
	pendulum = Pendulum(1,-1,massBall)		#Here we can initialize the Theta for the simulation

	# Initializing calculation variables needed for the simulation
	theta_d = 0
	theta_tminus1 = theta_tminus2 = pendulum.theta
	x_tminus1 = x_tminus2 = cart.x
	previous_error = calc_error(pendulum)
	integral = 0
	previous_time_delta = 0
	
	# The simulation will run for the time set in main() line 
	while time.time() <= end_time:		
		current_timestamp = time.time()
		time_delta = (current_timestamp - previous_timestamp)
		error = calc_error(pendulum)
		if previous_time_delta != 0:
			theta_d = (theta_tminus1 - theta_tminus2 ) / previous_time_delta				
			x_dot = (x_tminus1 - x_tminus2) / previous_time_delta
			F,intergral = calc_controlInput(cart,pendulum,time_delta,error,previous_error,integral,g)
			run_controlInput(cart,pendulum,F,time_delta,x_tminus2,theta_d,theta_tminus2,previous_time_delta,g)
			
			# Aggregating the data into arrays for the graphs
			errors.append(error)
			force.append(F)
			theta.append(pendulum.theta)
			times.append(current_timestamp)
			x.append(cart.x)
				
		# Calling the simulation wrapper in Opencv-python// also here we update the variables
		simulation_build(sim_size,cart,pendulum)
		previous_time_delta = time_delta
		previous_timestamp = current_timestamp
		previous_error = error
		theta_tminus2 = theta_tminus1
		theta_tminus1 = pendulum.theta
		x_tminus2 = x_tminus1
		x_tminus1 = cart.x

	build_graphs(times,errors,theta,force,x)

if __name__ == "__main__":
	main()