
//////////////////////////////////////////////////
/////     MAIN FUNCTION CALLS
//////////////////////////////////////////////////
var epsilon = .2;
var isConnected = false;
numParticles = 10;
particleArray = [];
particlePartners = [];
// initialize threejs scene, user input, and robot kinematics
init();

// main animation loop maintained by threejs
animate();


//////////////////////////////////////////////////
/////     INITIALIZATION FUNCTION DEFINITONS
//////////////////////////////////////////////////

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

//parent is an object, child is just a vertex
function tree_add_vertex(parent,child, tree){

	//Add the new child to the tree
	tree.vertices[tree.newest+1] = {};
	tree.vertices[tree.newest+1].vertex = child;
	tree.vertices[tree.newest+1].edges = [];
	tree.vertices[tree.newest+1].parent = parent;
	
	draw_2D_edge_configurations(parent.vertex,tree.vertices[tree.newest+1].vertex);

	//add the new child as a child of the parent
	parent.edges[parent.edges.length] = tree.vertices[tree.newest+1];
	tree.newest++;
}

function connect_trees(vertexA,vertexB){
	vertexA.connection = vertexB;
	vertexB.connection = vertexA;
}



//neighbor is a vertex object, desired is just the configuration points
function rrt_extend(tree, neighbor, desiredIn, connectBool){

  var desired;
  if(connectBool){
    desired = desiredIn.vertex;
  }else{
    desired = desiredIn;
  }

	var distance = getDistance(neighbor.vertex, desired);



	if((distance<epsilon)&&connectBool){

		isConnected = true;
		rrt_iterate=false;
		connect_trees(neighbor,desiredIn);
    console.log("CONNECTED");
    colorPath(neighbor,desiredIn);
		return false;
	}


	var xDiff = desired[0]-neighbor.vertex[0];
	var yDiff = desired[1]-neighbor.vertex[1];



	//Create a new vertex a distance of epsilon away from the neighbor

	var theta = Math.atan(yDiff/xDiff);

  var newY = Math.sin(theta)*epsilon+neighbor.vertex[1];
  var newX = Math.cos(theta)*epsilon+neighbor.vertex[0];

  if((yDiff<0)&&(xDiff<0)){
    newY = -Math.sin(theta)*epsilon+neighbor.vertex[1];
    newX = -Math.cos(theta)*epsilon+neighbor.vertex[0];
  }


  if((xDiff<0)&&(yDiff>0)){
    newY = -Math.sin(theta)*epsilon+neighbor.vertex[1];
    newX = -Math.cos(theta)*epsilon+neighbor.vertex[0];
  }

	var newVertex = [newX,newY];

	//If the new vertex is a collision, return false
	if(collision_test(newVertex)){
		return false;
	}else{
		tree_add_vertex(neighbor,newVertex, tree);
		return true;
	}



}

function getDistance(pointA, pointB){
	var dist = Math.sqrt(Math.pow((pointA[0]-pointB[0]),2)+Math.pow((pointA[1]-pointB[1]),2));

	return dist;
}

function nearest_neighbor(q,tree){
	var nearest = [Infinity,Infinity];
	var nearestDist = Infinity;

	for (vertex in tree.vertices){
		var testDist = getDistance(q, tree.vertices[vertex].vertex);
		if(testDist<nearestDist){
			nearestDist = testDist;
			nearest = tree.vertices[vertex];
		}
	}
	return nearest;
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

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];
    tree.vertices[0].parent = null;

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function findPartners(){
	
}

function init() {

	particleArray.length = numParticles;
	
	for(i=0; i<particleArray.length; i++){
		var coordX = Math.random()*8-2;
		var coordY = Math.random()*8-2;
		
		particleArray[i]=[coordX,coordY];
	}
	
	for(particle in particleArray){
		draw_2D_configuration(particleArray[particle]);
		var partnerA = Math.floor(Math.random()*numParticles);
		var partnerB = Math.floor(Math.random()*numParticles);
		while(partnerA==partnerB){
			partnerB = Math.floor(Math.random()*numParticles);	
		}
		var partner = [partnerA,partnerB];
		particlePartners[particle] = partner;
	}
	
	for(particle in particleArray){
		
		draw_2D_edge_configurations(particleArray[particlePartners[particle][0]],particleArray[particlePartners[particle][1]]);
	}
	

    // specify start and goal configurations
    q_start_config = [-2,-2];
    q_goal_config = [6,6];
    q_init = q_start_config;
    q_goal = q_goal_config;

    globalTreeA = tree_init(q_start_config);
    globalTreeB = tree_init(q_goal_config);

    globalTrees = [globalTreeA, globalTreeB];
    currTree = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // set the world for the planner (stored as "range" global variable)
    set_planning_scene();

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

    //draw_robot_world();

    // specify rrt algorithm to use for planning
    rrt_alg = 1;  // 0: basic rrt, 1: rrt_connect

    // make sure the rrt iterations are not running faster than animation update
    if (rrt_iterate && (Date.now()-cur_time > 10)) {
      if(!isConnected){
        console.log("iterating");
        rrt_planning_iteration();
      }
      // update time marker for last iteration update
      cur_time = Date.now();

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

function draw_2D_edge_configurations(q1,q2) {
    // draw line between locations of two 2D configurations on canvas
    c = document.getElementById("myCanvas");
    ctx = c.getContext("2d");
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


