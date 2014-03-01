//////////////////////////////////////////////////
/////     FORWARD KINEMATICS
//////////////////////////////////////////////////

// CS148: compute and draw robot kinematics (.xform matrix for each link)
// CS148: compute and draw robot heading and lateral vectors for base movement in plane
// matrix_2Darray_to_threejs converts a 2D JavaScript array to a threejs matrix
//   for example: var tempmat = matrix_2Darray_to_threejs(link.xform);
// simpleApplyMatrix transforms a threejs object by a matrix
//   for example: simpleApplyMatrix(link.geom,tempmat);

/*
CS148: reference code has functions for:

robot_forward_kinematics
traverse_forward_kinematics_link
traverse_forward_kinematics_joint
compute_and_draw_heading
*/


function set_robot_parts(){
	var x,tempmat;
    for (x in robot.joints) {

        // give the joint its name as an id
        robot.joints[x].name = x;
		
		var tx = robot.joints[x].origin.xyz[0];
		var ty = robot.joints[x].origin.xyz[1];
		var tz = robot.joints[x].origin.xyz[2];
		
        // CONSTRUCT KINEMATIC HIERARCHY

        //Instantiate a rotation matrix for the joint
        var rotateMatrix = make_rotation_matrix(robot.joints[x].origin.rpy[0],robot.joints[x].origin.rpy[1],robot.joints[x].origin.rpy[2]);
//        console.log(rotateMatrix);
        //Instantiate a transformation matrix for the joint
        var translationMatrix = [
        	[1,0,0,tx],
        	[0,1,0,ty],
        	[0,0,1,tz],
        	[0,0,0,1]
        ];
        
        var parentXForm = robot.links[robot.joints[x].parent].xform;
        
        var xForm = multMultiMatrices([parentXForm,translationMatrix,rotateMatrix]);
        
        robot.joints[x].xform = xForm;
        robot.links[robot.joints[x].child].xform = xForm;

        linkmat = matrix_2Darray_to_threejs(robot.links[robot.joints[x].child].xform);
        
        simpleApplyMatrix(robot.links[robot.joints[x].child].geom, linkmat);
        
		//Apply transformation matrix
		tempmat = matrix_2Darray_to_threejs(robot.joints[x].xform);
		simpleApplyMatrix(robot.joints[x].geom,tempmat);
        
        
    }
}