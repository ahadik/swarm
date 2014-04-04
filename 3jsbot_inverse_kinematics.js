//////////////////////////////////////////////////
/////     INVERSE KINEMATICS 
/////     Resolved-rate IK with geometric jacobian
//////////////////////////////////////////////////

// CS148: generate joint controls to move robot to move robot endeffector to target location

/*
CS148: reference code has functions for:

robot_inverse_kinematics
iterate_inverse_kinematics
*/



function robot_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {
    // compute joint angle controls to move location on specified link to Cartesian location
    if (update_ik) {
       	iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos);
        endeffector_geom.visible = true;
        target_geom.visible = true;
        

    }
    else {
        endeffector_geom.visible = false;
        target_geom.visible = false;
    }
    update_ik = false;

}

function getOn(joint, endeffector_global_pos){
	var joint = robot.joints[joint];
	var end_x = endeffector_global_pos[0][3];
	var end_y = endeffector_global_pos[1][3];
	var end_z = endeffector_global_pos[2][3];
	
	var xyz_mult = [[joint.origin.xyz[0]],[joint.origin.xyz[1]],[joint.origin.xyz[2]],[1]];
	var global_xyz = multiply_matrices(joint.xform,xyz_mult);
	
	var o_sub = [end_x-joint.xform[0][3],end_y-joint.xform[1][3],end_z-joint.xform[2][3]];
	
	return o_sub;
}

function getJacobianVector (zi, o_sub){

	var vector_cros = vector_cross(zi,o_sub);

	var jacobianVector = [[vector_cros[0]],[vector_cros[1]],[vector_cros[2]],[zi[0]],[zi[1]],[zi[2]]];
	return jacobianVector;
}

function pseudo_inverse(jacobian, target_pos, global_end){
	alpha=0.7;
	var jacobianTrans = column_matrix_transpose(jacobian);
	var jacobianRow = column_to_row(jacobian);
	
	var dx = new Array(6);
	dx[0]=[target_pos[0]-global_end[0][3]];
	dx[1]=[target_pos[1]-global_end[1][3]];
	dx[2]=[target_pos[2]-global_end[2][3]];
	dx[3]=[0];
	dx[4]=[0];
	dx[5]=[0];

	console.log(jacobianRow);
	console.log(jacobianTrans);
	

	var pseudoInverseMult = numeric.dot(jacobianRow, jacobianTrans);
	

	
	var pseudoInverse = numeric.inv(pseudoInverseMult);
	
	var jacobiansMult = numeric.dot(jacobianTrans, pseudoInverse);
	var matrix = numeric.dot(jacobiansMult, dx);
	var finalMat = scalar_mult(matrix,alpha);
	
	return finalMat;
	
}


function jacobian_transpose(jacobian, target_pos, global_end){
var alpha = 0.1;
var jacobianTrans = column_matrix_transpose(jacobian);

var dx = new Array(6);
dx[0]=[target_pos[0]-global_end[0][3]];
dx[1]=[target_pos[1]-global_end[1][3]];
dx[2]=[target_pos[2]-global_end[2][3]];
dx[3]=[0];
dx[4]=[0];
dx[5]=[0];

var pseudo = numeric.dot(jacobianTrans,dx);

var pseudo_scaled = scalar_mult(pseudo,alpha);
console.log("adf");
console.log(pseudo);



return pseudo_scaled;

}

function getZi(joint){
	var joint = robot.joints[joint];
	var zeroed_xform = joint.xform;
	zeroed_xform[0][3]=0;
	zeroed_xform[1][3]=0;
	zeroed_xform[2][3]=0;
	var mult_axis = [[joint.axis[0]],[joint.axis[1]],[joint.axis[2]],[1]];
	var zi = multiply_matrices(zeroed_xform,mult_axis);
	console.log("xform");
	console.log(zeroed_xform);
	
	var zi_return = [zi[0][0],zi[1][0],zi[2][0]];
	
	return zi_return;
}

function applyPseudo(pseudo, joint){
	var workingJoint = robot.joints[joint];
	
	console.log(workingJoint);
	console.log(joint);
	console.log(workingJoint.name);
	var count = 0;
	
	while(workingJoint.parent!="base"){
		workingJoint.servo.desired=pseudo[count][0];

		count++;
		workingJoint=robot.joints[robot.links[workingJoint.parent].parent];
	}
	workingJoint.servo.desired=pseudo[count][0];
	
	
}

function iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos){

		var endeffector_global_pos = multiply_matrices(robot.joints[endeffector_joint].xform, [[1,0,0,endeffector_local_pos[0]],[0,1,0,endeffector_local_pos[1]],[0,0,1,endeffector_local_pos[2]],[0,0,0,endeffector_local_pos[3]]]);
        
        simpleApplyMatrix(endeffector_geom, matrix_2Darray_to_threejs(endeffector_global_pos));
        simpleApplyMatrix(target_geom, matrix_2Darray_to_threejs([[1,0,0,target_pos[0]],[0,1,0,target_pos[1]],[0,0,1,target_pos[2]],[0,0,0,target_pos[3]]]));

	var jacobian = new Array(4);
	var jacobianVec;
		
	var workingJoint = robot.joints[endeffector_joint];
	var count = 0;
	while (workingJoint.parent!="base"){
		var zi = getZi(workingJoint.name);
		var o_sub = getOn(workingJoint.name, endeffector_global_pos);

				
		jacobianVec = getJacobianVector (zi, o_sub);
		jacobian[count]=jacobianVec;
		workingJoint = robot.joints[robot.links[workingJoint.parent].parent];
		count++;
	}
	
	zi = getZi(workingJoint.name, endeffector_global_pos);
	o_sub = getOn(workingJoint.name, endeffector_global_pos);
	jacobianVec = getJacobianVector (zi, o_sub);
	count++;		
	jacobian[3]=jacobianVec;

	
var deltatheta = jacobian_transpose(jacobian, target_pos, endeffector_global_pos);

	//var deltatheta = pseudo_inverse(jacobian, target_pos, endeffector_global_pos);

	applyPseudo(deltatheta,endeffector_joint);
	robot_pd_control();

	
}