import time

def PID(roll, pitch, yaw, f):
	#Define the global variables to prevent them from dying and resetting to zero, each time a function call occurs. Some of these variables 		may be redundant.
	global kp_roll, ki_roll, kd_roll, kp_pitch, ki_pitch, kd_pitch, kp_yaw, ki_yaw, kd_yaw, prevErr_roll, prevErr_pitch, prevErr_yaw, pMem_roll, pMem_yaw, pMem_pitch, iMem_roll, iMem_pitch, iMem_yaw, dMem_roll, dMem_pitch, dMem_yaw, flag, setpoint, sampleTime
	
        kp_roll = 50
	ki_roll = 50
	kd_roll = 50
	kp_pitch = kp_roll
	ki_pitch = ki_roll
	kd_pitch = kd_roll
	kp_yaw = 0.1
	ki_yaw = 0
	kd_yaw = 0
	flag = 0
	
        sampleTime = 0
	reference = 0
        
        #errors in degrees
	err_pitch = float(pitch)*(180 / 3.141592653) - reference 
 	err_roll = float(roll)*(180 / 3.141592653) - reference
	err_yaw = float(yaw)*(180/3.14159263) - reference
	currTime = time.time()
	
        if flag == 0:
		prevTime = 0
		prevErr_roll = 0
		prevErr_pitch = 0
		prevErr_yaw = 0
		pMem_roll = 0
		pMem_pitch = 0
		pMem_yaw = 0
		iMem_roll = 0
		iMem_pitch = 0
		iMem_yaw = 0
		dMem_roll = 0
		dMem_pitch = 0
		dMem_yaw = 0
		flag = 1
	
        #small changes can be approximated to derivative element (for calculating D parameter in PID)
        dTime = currTime - prevTime
	dErr_pitch = err_pitch - prevErr_pitch
	dErr_roll = err_roll - prevErr_roll
	dErr_yaw = err_yaw - prevErr_yaw
	
	if(dTime >= sampleTime):
		#Kp*e(t)
		pMem_roll = kp_roll * err_roll
		pMem_pitch = kp_pitch * err_pitch
		pMem_yaw = kp_yaw * err_yaw
		
		#integral(e(t))
		iMem_roll += err_pitch * dTime
		iMem_pitch += err_roll * dTime
		iMem_yaw += err_yaw * dTime
	        
                #extreme boundary cases
		if(iMem_roll > 100): iMem_roll = 100
		if(iMem_roll < -100): iMem_roll = -100
		if(iMem_pitch > 100): iMem_pitch = 100
		if(iMem_pitch < -100): iMem_pitch = -100
		if(iMem_yaw > 100): iMem_yaw = 100
		if(iMem_yaw < -100): iMem_yaw = 100
		
		#derivative(e(t))
		dMem_roll = dErr_roll / dTime
		dMem_pitch = dErr_pitch / dTime
		dMem_yaw = dErr_yaw / dTime
	
	#update changing variables
        prevTime = currTime
	prevErr_roll = err_roll
	prevErr_pitch = err_pitch
	prevErr_yaw = err_yaw
	
	#assemble the output of PID controller
	output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll
	output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
	output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw 
	
	#motor mixing algorithm
        #br in my code is fr in gazebo's world
	esc_br = 1500 + output_roll + output_pitch - output_yaw
	#bl in my code is br in gazebo's world
	esc_bl = 1500 + output_roll - output_pitch + output_yaw
	#fl in my code is bl in gazebo's world
	esc_fl = 1500 - output_roll - output_pitch - output_yaw
	#fr in my code is fl in gazebo's world
	esc_fr = 1500 - output_roll + output_pitch + output_yaw
	
	#bound condition
        if(esc_br > 2000): esc_br = 2000
	if(esc_bl > 2000): esc_bl = 2000
	if(esc_fr > 2000): esc_fr = 2000
	if(esc_fl > 2000): esc_fl = 2000
	
	if(esc_br < 150): esc_br = 150
	if(esc_bl < 150): esc_bl = 150
	if(esc_fr < 150): esc_fr = 150
	if(esc_fl < 150): esc_fl = 150
	
	#Map the esc values to motor values
	br_motor_vel = ((esc_br - 1500)/25) + 50
	bl_motor_vel = ((esc_bl - 1500)/25) + 50
	fr_motor_vel = ((esc_fr - 1500)/25) + 50
	fl_motor_vel = ((esc_fl - 1500)/25) + 50
	
        #get published to gazebo
        f.data = [fr_motor_vel,-fl_motor_vel,bl_motor_vel, -br_motor_vel]
	
	#Returns final v object, and respective errors
	return f, err_roll, err_pitch, err_yaw
