
//////////////////////////////////////////////////
/////     MAIN FUNCTION CALLS
//////////////////////////////////////////////////
var epsilon = .2;
var isConnected = false;
numParticles = 10;
particleArray = [];
particlePartners = [];

animateBool = true;

// main animation loop maintained by threejs
animate();


//////////////////////////////////////////////////
/////     INITIALIZATION FUNCTION DEFINITONS
//////////////////////////////////////////////////

function setAnimate(){
	if(animateBool){
		animateBool = false;
	}else{
		animateBool = true;
	}
}

function colorPath(connectA, connectB){
  var workingNode = connectA;
  var path=[];
  
  while(workingNode.parent!=null){
    path.unshift(workingNode);
    workingNode=workingNode.parent;
  }

  workingNode = connectB;
  console.log(connectB);
  while(workingNode.parent!=null){
    path.push(workingNode);
    workingNode=workingNode.parent;
  }

  draw_highlighted_path(path);
}

function findRandom(){
	var x = (Math.random()*6)-1;
	var y = (Math.random()*6)-1;

	var vertex = [y,x];

	while(collision_test(vertex)){
		var x = Math.random()*5;
		var y = Math.random()*5;
		vertex = [y,x];
	}

	return vertex;
}




//neighbor is a vertex object, desired is just the configuration points
function extend(point, desired, epsilon){

	var xDiff = desired[0]-point[0];
	var yDiff = desired[1]-point[1];

	//Create a new vertex a distance of epsilon away from the neighbor
	var theta = Math.atan(yDiff/xDiff);

	var newY = Math.sin(theta)*epsilon+point[1];
	var newX = Math.cos(theta)*epsilon+point[0];
	
	if((yDiff<0)&&(xDiff<0)){
		newY = -Math.sin(theta)*epsilon+point[1];
		newX = -Math.cos(theta)*epsilon+point[0];
	}
	
	if((xDiff<0)&&(yDiff>0)){
		newY = -Math.sin(theta)*epsilon+point[1];
		newX = -Math.cos(theta)*epsilon+point[0];
	}

	var newVertex = [newX,newY];

	return newVertex;
}

function getDistance(pointA, pointB){
	var dist = Math.sqrt(Math.pow((pointA[0]-pointB[0]),2)+Math.pow((pointA[1]-pointB[1]),2));

	return dist;
}


function rrt_planning_iteration() {
  if(!isConnected){
    //console.log("Not connected");
	//get a random vertex
	var vertex = findRandom();
	var treeA;
	var treeB;


	if(currTree==0){
		treeA = globalTrees[0];
		treeB = globalTrees[1];
		currTree=1;
	}else if(currTree==1){
		treeA = globalTrees[1];
		treeB = globalTrees[0];
		currTree=0;
	}

	//find the nearest neighbor on tree A to the randomly found vertex
	var nearestNeighbor = nearest_neighbor(vertex,treeA);


	//extend treeA towards the random vertex one step
	//Nearest neighbor is an object, vertex is just points
  //console.log("single extend");
	//rrt_extend(treeA, nearestNeighbor, vertex,false);

	//draw_2D_configuration(treeA.vertices[treeA.newest].vertex);
	//Find the vertex on treeB that is closest to the newly created vertex on treeB
	var nearestB = nearest_neighbor(treeA.vertices[treeA.newest].vertex,treeB);

	//While extension can proceed without collision, call rrt_extend extending treeA. If extend returns true, update nearestNeighbor to be the recently added element.

  var extendBool = true;

  var count = 0;
  while(extendBool){
    //console.log("extending");

    if (rrt_iterate && (Date.now()-cur_time > 10)) {

        if(count == 10){break;}
        count++;
        extendBool = rrt_extend(treeB, nearestB, treeA.vertices[treeA.newest],true);
        if(isConnected){break;}
        if(extendBool){
          nearestB=treeB.vertices[treeB.newest];
          //draw_2D_configuration(nearestB.vertex);
        }


        // update time marker for last iteration update
        cur_time = Date.now();
   }
   if(isConnected){break;}
	}
	if(isConnected){
		//connection has been made
    rrt_iterate=false;
		return true;
	}else{
		// connection has not been made
		return false;
	}
}
}


function generatePoints(){
	particleArray.length = numParticles;
	
	for(i=0; i<particleArray.length; i++){
		var coordX = Math.random()*8-2;
		var coordY = Math.random()*8-2;
		
		particleArray[i]=[coordX,coordY];
	}
	
}

function paintPoints(){
	for(particle in particleArray){
		draw_2D_configuration(particleArray[particle]);
		var partnerA = Math.floor(Math.random()*numParticles);
		var partnerB = Math.floor(Math.random()*numParticles);
		
		while((partnerA==particle)||(partnerB==particle)||(partnerA==partnerB)){
			partnerA = Math.floor(Math.random()*numParticles);
			partnerB = Math.floor(Math.random()*numParticles);
		}
		
		var partner = [partnerA,partnerB];
		particlePartners[particle] = partner;
	}
}

function paintEdges(){
	for(particle in particleArray){
		draw_2D_edge_configurations(particleArray[particle],particleArray[particlePartners[particle][0]], "#000000");
		draw_2D_edge_configurations(particleArray[particle],particleArray[particlePartners[particle][1]], "#000000");
	}
}

function hideEdges(){
	clearFrame();
	paintPoints();
}

function matrixMult(mat1,mat2){
		//Instantiate variables representing height and width of the output matrix, multMatrices
		var height = mat1.length;
		var width = mat2[0].length;
		var mat1Width = mat1[0].length;
		var multMatrices = new Array();
		
		//Create empty arrays for all rows of the multiplied matrix
		for(var a = 0; a<height;a++){
			multMatrices[a]=[];
		}

		//For each entry of the output matrix
		for (var y = 0; y<height; y++){

			for (var x = 0; x<width; x++){
							
				//Instantiate a variable for holding the tallied value for this index
				var val=0;
				//Iterate over all row of the output matrix
				for (var i = 0; i<mat1Width; i++){

					/*
					console.log("HERE");
					console.log(x);
					console.log(y);
					console.log(i);
				*/
					//add the multiplication of the two entries of interest to the val tally
					val = val+mat1[y][i]*mat2[i][x];
				}
				//Set the val tally
				multMatrices[y][x]=val;
			}
		}
		
		return multMatrices;
	}

function solveEquilateral(leftPoint, rightPoint, dir){
	var normalizedEndeffector = [[rightPoint[0]-leftPoint[0]], [rightPoint[1]-leftPoint[1]]];
	var radianRotation = 60*Math.PI/180*dir;
	var rotationMatrix = [[Math.cos(radianRotation), -Math.sin(radianRotation)],[Math.sin(radianRotation), Math.cos(radianRotation)]];
	var rotatedVector = matrixMult(rotationMatrix,normalizedEndeffector);
	var goalPoint = [rotatedVector[0][0]+leftPoint[0], rotatedVector[1][0]+leftPoint[1]];
	return goalPoint;
}

function getClosest(target, pointA, pointB){
	var distA = getDistance(target, pointA);
	var distB = getDistance(target, pointB);
	if(distA<distB){
		return pointA;
	}else{
		return pointB;
	}
}

function findEqual(main){

	var mainPoint = particleArray[main];
	var pointA = particleArray[particlePartners[main][0]];
	var pointB = particleArray[particlePartners[main][1]];
	
	//draw_2D_edge_configurations(mainPoint, pointA, "#000000");
	//draw_2D_edge_configurations(mainPoint, pointB, "#000000");

	var distance = getDistance(pointA, pointB);
	
	console.log(distance);
	
	//left is point of rotation (reference) and right is endeffector
	var leftPoint;
	var rightPoint;
	
	if(pointA[0]<pointB[0]){
		leftPoint=pointA;
		rightPoint=pointB;
	}else{
		leftPoint=pointB;
		rightPoint=pointA;
	}
	
	var solutionA = solveEquilateral(leftPoint, rightPoint, 1);
	var solutionB = solveEquilateral(leftPoint, rightPoint, -1);
	
	var targetPoint = getClosest(mainPoint, solutionA, solutionB);
	
	//draw_2D_edge_configurations(leftPoint,targetPoint, "#DD4000");
	//draw_2D_edge_configurations(rightPoint,targetPoint, "#DD4000");
	//draw_2D_edge_configurations(leftPoint,rightPoint, "#DD4000");
	
	return calcStep(mainPoint, targetPoint);
}

function calcStep(point, target){
	var error = getDistance(point, target);
	var scalingFactor = .1;
	var distance = error*scalingFactor;
	
	return extend(point, target, distance);
}

function init() {

	generatePoints();
	paintPoints();

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();


}


//////////////////////////////////////////////////
/////     ANIMATION AND INTERACTION LOOP
//////////////////////////////////////////////////

function animate() {

    // alternative to using setInterval for updating in-browser drawing
    // this effectively request that the animate function be called again for next draw
    // http://learningwebgl.com/blog/?p=3189
    requestAnimationFrame( animate );

	if(animateBool){
		var newParPos = [];
	    newParPos.length = particleArray.length;
	
	    for (particle in particleArray){	
		    var goal = findEqual(particle);
		    newParPos[particle] = goal;
	    }
	    
	    particleArray = newParPos;
	    
	    clearFrame();
	    paintPoints();
	    paintEdges();	
	}

}


//////////////////////////////////////////////////
/////     CANVAS DRAWING SUPPORT ROUTINES
//////////////////////////////////////////////////

function draw_robot_world() {

    // draw start and goal configurations
    c = document.getElementById("myCanvas");
    ctx = c.getContext("2d");
    ctx.fillStyle = "#0000FF";
    ctx.fillRect((q_init[0]*100+200)-5,(q_init[1]*100+200)-5,10,10);
    ctx.fillStyle = "#00FF00";
    ctx.fillRect((q_goal[0]*100+200)-5,(q_goal[1]*100+200)-5,10,10);

    // draw robot's world
    for (j=0;j<range.length;j++) {
        ctx.fillStyle = "#FF0000";
        ctx.fillRect((range[j][0][0]*100+200),(range[j][1][0]*100+200),(range[j][0][1]-range[j][0][0])*100,(range[j][1][1]-range[j][1][0])*100);
    }

}

function clearFrame(){
	ctx.clearRect(0,0,800,800);
}

function draw_2D_configuration(q) {
    // draw location of 2D configuration on canvas
    c = document.getElementById("myCanvas");
    ctx = c.getContext("2d");
    ctx.fillStyle = "#8888AA";
    ctx.fillRect((q[0]*100+200)-3,(q[1]*100+200)-3,6,6);
}

function draw_2D_edge_configurations(q1,q2, style) {
    // draw line between locations of two 2D configurations on canvas
    c = document.getElementById("myCanvas");
    ctx = c.getContext("2d");
    ctx.strokeStyle = style;
    ctx.beginPath();
    ctx.moveTo(q1[0]*100+200,q1[1]*100+200);
    ctx.lineTo(q2[0]*100+200,q2[1]*100+200);
    ctx.stroke();
}

function draw_highlighted_path(path) {
    ctx = c.getContext("2d");
    ctx.strokeStyle="#0000FF";
    ctx.lineWidth=4;
    ctx.beginPath();
    for (i=1;i<path.length;i++) {
        ctx.moveTo(path[i-1].vertex[0]*100+200,path[i-1].vertex[1]*100+200);
        ctx.lineTo(path[i].vertex[0]*100+200,path[i].vertex[1]*100+200);
    }
    ctx.stroke();
}

//////////////////////////////////////////////////
/////     COLLISION SUPPORT ROUTINES
//////////////////////////////////////////////////

function set_planning_scene() {

    // obstacles specified as a range along [0] (x-dimension) and [1] y-dimension
    range = []; // global variable

    // world boundary
    /*
    range[0] = [ [-1.1,5.1],[-1.1,-1] ];
    range[1] = [ [-1.1,5.1],[5,5.1] ];
    range[2] = [ [-1.1,-1],[-1.1,5.1] ];
    range[3] = [ [5,5.1],[-1.1,5.1] ];
*/
/*  misc stuff with narrow opening
*/
/*
    range[4] = [ [1,2],[1,2] ];
    range[5] = [ [3,3.3],[1,4] ];
    range[6] = [ [0.6,0.7],[0.4,0.7] ];
    range[7] = [ [3.7,3.9],[-0.8,5] ];
*/
/*  narrow path 1
    range[4] = [ [1,3],[4,5] ];
    range[5] = [ [1,3],[-1,2] ];
    range[6] = [ [1,1.95],[2,3.8] ];
*/

/*  narrow path 2
    range[4] = [ [1,3],[4,5] ];
    range[5] = [ [1,3],[-1,2] ];
    range[6] = [ [1,1.9],[2,3.8] ];
    range[7] = [ [2.1,3],[2.2,4] ];
*/

/*  three compartments
    range[4] = [ [1,1.3],[4,5] ];
    range[5] = [ [1,1.3],[-1,3.5] ];
    range[6] = [ [2.7,3],[-1,0] ];
    range[7] = [ [2.7,3],[.5,5] ];
*/

}

function collision_test(q) {

    // return no collision, if no collision detected with any obstacle
    return false;
}


