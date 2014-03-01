//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

matrix_multiply
matrix_transpose
vector_normalize
vector_cross
generate_identity
generate_translation_matrix
generate_rotation_matrix_X
generate_rotation_matrix_Y
generate_rotation_matrix_Z
*/



	//Make an identity matrix of the desired size
	function makeIdentityMatrix(size){
		var idenMatrix = new Array();
		for(var y = 0; y<size; y++){
			idenMatrix[y]=new Array();
			for(var x = 0; x<size; x++){
				if(x===y){
					idenMatrix[y][x]=1;
				}else{
					idenMatrix[y][x]=0;
				}
			}
		}
		return idenMatrix;
	}
	
	//Take an array of matrices and multiply them together in left to right (first to last) order
	function multMultiMatrices(matrices){
		
		var multiedMatrix = matrices[0];
		for(var x = 1; x<matrices.length;x++){
			multiedMatrix = multiply_matrices(multiedMatrix,matrices[x]);
		}
		return multiedMatrix;
	}
	
	/*
	INPUT: Two matrices in row form, that is arrays of arrays as rows
	OUTPUT: A single matrix of the multiplication of the two input matrices again in row form
	*/
	function multiply_matrices(mat1,mat2){
		//Instantiate variables representing height and width of the output matrix, multMatrices
		var height = mat1.length;
		var width = mat2[0].length;
		var multMatrices = new Array();
		
		//Create empty arrays for all rows of the multiplied matrix
		for(var a = 0; a<height;a++){
			multMatrices[a]=[];
		}

		//For each entry of the output matrix
		for (var x = 0; x<width; x++){
			for (var y = 0; y<height; y++){
				//Instantiate a variable for holding the tallied value for this index
				var val=0;
				//Iterate over all row of the output matrix
				for (var i = 0; i<height; i++){
					//add the multiplication of the two entries of interest to the val tally
					val = val+mat1[y][i]*mat2[i][x];
				}
				//Set the val tally
				multMatrices[y][x]=val;
			}
		}
		
		return multMatrices;
	}
	
	/*
	INPUT: values for roll, pitch, yaw
	OUTPUT: a single matrix that is the rotation matrix for the input values
	*/
	function make_rotation_matrix(roll,pitch,yaw){
		//Construct the three rotational matrices
		var xrot = [
			[1,0,0,0],
			[0, Math.cos(roll), -Math.sin(roll),0],
			[0, Math.sin(roll), Math.cos(roll),0],
			[0,0,0,1]
		];
		var yrot = [
			[Math.cos(pitch), 0, Math.sin(pitch),0],
			[0,1,0,0],
			[-Math.sin(pitch),0,Math.cos(pitch),0],
			[0,0,0,1]
		];
		var zrot = [
			[Math.cos(yaw), -Math.sin(yaw),0,0],
			[Math.sin(yaw), Math.cos(yaw), 0,0], 
			[0,0,1,0],
			[0,0,0,1]
		];
		
		var xyRotation = multiply_matrices(xrot,yrot);
		var totalRotation = multiply_matrices(xyRotation,zrot);
		return totalRotation;
	}