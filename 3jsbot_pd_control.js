//////////////////////////////////////////////////
/////     MOTION CONTROL ROUTINES 
//////////////////////////////////////////////////

// CS148: add PD controller here
	function robot_pd_control(){

		for(x in robot.joints){
			var active = robot.joints[x];
			//		active.servo.desired = active.angle+0.1;
			var kp = 0.1;
			var tau = kp*(active.servo.desired-active.angle);
		
//			console.log(tau);
		
			active.control = tau;
		
//			active.angle += active.servo.gain;
		
		
			var joint_quaternion = quaternion_from_axisangle(active.angle, active.axis);
		
			var normalized_quaternion = quaternion_normalize(joint_quaternion);
		
			var quaternion_rotate_matrix = quaternion_to_rotation_matrix(normalized_quaternion);

			active.DOF.rotate = quaternion_rotate_matrix;
		
			//active.servo.gain = 0;
			//active.control = 0;
		}
	}