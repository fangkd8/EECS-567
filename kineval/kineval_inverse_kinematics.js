
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {
    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }/*
    else if ((kineval.params.update_ik)||(kineval.params.ik_orientation_included)) {
        kineval.iterateIK_with_angle(endeffector_target_world, endeffector_joint, endeffector_position_local);
    }*/

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

   // update time from start of trial
   cur_time = new Date();
   kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

   // get endeffector Cartesian position in the world
   endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

   // compute distance of endeffector to target
   kineval.params.trial_ik_random.distance_current = Math.sqrt(
           Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
           + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
           + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

   // if target reached, increment scoring and generate new target location
   // KE 2 : convert hardcoded constants into proper parameters
   if (kineval.params.trial_ik_random.distance_current < 0.01) {
       kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
       kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
       kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
       kineval.params.trial_ik_random.targets += 1;
       textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
   }

}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {
    if ((kineval.params.ik_pseudoinverse == false)&&(kineval.params.ik_orientation_included == false)){
        var nend = endeffector_joint;
        var pos = matrix_multiply(robot.joints[nend].xform, endeffector_position_local);
        var pos = matrix_transpose(pos); pos.length = pos.length-1;
        //position of endeffector;
        var chain = [];
        var idx = nend;
        while (robot.joints[idx].parent !== base.name){
            var prc = robot.joints[idx].parent;
            idx = robot.links[prc].parent;
            chain.push(robot.joints[idx]);
        } // Chain from endeffector to Base.    

        var target = [];
        target = endeffector_target_world.position;
        target = matrix_transpose(target); target.length = target.length-1;
        var delta_x = [];
        delta_x = vector_minus(target, pos);
        delta_x = matrix_transpose(delta_x);
        //vector towards the target. 
        
        var jack = [];
        jack = jacobian_trans(chain,pos);
        qspace = matrix_multiply(jack, delta_x);
        //q_space;  
        
        for (var iv=0;iv<chain.length;iv++){
            chain[iv].control = chain[iv].control + kineval.params.ik_steplength*qspace[iv][0];
        }
    }

	else if(kineval.params.ik_pseudoinverse){
		var nendx = endeffector_joint;
		var posx = matrix_multiply(robot.joints[nendx].xform, endeffector_position_local);
        posx = matrix_transpose(posx); posx.length = posx.length-1;
        //position of endeffector;
        var chain = [];
        var idx = nendx;
        while (robot.joints[idx].parent !== base.name){
            var prc = robot.joints[idx].parent;
            idx = robot.links[prc].parent;
            chain.push(robot.joints[idx]);
        } // Chain from endeffector to Base.    

        var targetx = [];
        targetx = endeffector_target_world.position;
        targetx = matrix_transpose(targetx); targetx.length = targetx.length-1;
        var delta_x2 = [];
        delta_x2 = vector_minus(targetx, posx);
        delta_x2 = matrix_transpose(delta_x2);
		
		var jack = [];
		jack2 = jacobian_trans(chain, posx);
		//console.log(jack);
		
		var suso = [];
		suso = matrix_transpose(jack2); //suso is the Jacobian matrix, jack is J^T.
		var qs = [];
		//console.log(suso);
		
		if (suso.length >= jack2.length){
			//var baka = [];
			baka = matrix_multiply(jack2,suso);
			baka = matrix_inverse(baka);
			baka = matrix_multiply(baka, jack2);
		}
		else if (suso.length < jack2.length){
			//var baka = [];
			baka = matrix_multiply(suso,jack2);
			baka = matrix_inverse(baka);
			baka = matrix_multiply(jack2,baka);
		}
        
		qs = matrix_multiply(baka, delta_x2);
		
		for (var ib=0;ib<chain.length;ib++){
            chain[ib].control = chain[ib].control + kineval.params.ik_steplength*qs[ib][0];
        }

	}
	
    else if(kineval.params.ik_orientation_included){
        var nend1 = endeffector_joint;
        var pos1 = matrix_multiply(robot.joints[nend1].xform, endeffector_position_local);
        pos1 = matrix_transpose(pos1); pos1.length = pos1.length-1;  
        var pos2 = get_euler_angle(nend1);
        
		for (var i=3;i<6;i++){
            pos1[i] = pos2[i-3];
        } 
        var chain1 = [];
        var idx1 = nend1;
        while (robot.joints[idx1].parent !== base.name){
            var prc1 = robot.joints[idx1].parent;
            idx1 = robot.links[prc1].parent;
            chain1.push(robot.joints[idx1]);
        }
        var target1 =[];
        target1 = endeffector_target_world.position;
        target1 = matrix_transpose(target1); target1.length = target1.length-1;
        var target2 =[];
        target2 = endeffector_target_world.orientation;
        var tg = [];
        for (var iq=0;iq<3;iq++){
            tg[iq] = target1[iq];
        }
        for (var ix=0;ix<3;ix++){
            tg[ix+3] = target2[ix];
        }

        //tg is the target with both position and euler angle. row vector, 6 entries.

        //var delta_x1 = [];
        delta_x1 = vector_minus(tg, pos1);
        delta_x1 = matrix_transpose(delta_x1);
        //vector towards the target. 
        jack1 = jacobian_trans_6(chain1,pos1);
        qspace1 = matrix_multiply(jack1, delta_x1);
        for (var iv=0;iv<chain1.length;iv++){
            chain1[iv].angle += kineval.params.ik_steplength*qspace1[iv][0];
        }
    }
    // STENCIL: implement inverse kinematics iteration
}


function axis_position(x){
    var ax = [];
    for (var i=0; i<4; i++){
        ax[i] = [];
    }
    ax[0][0] = robot.joints[x].axis[0];
    ax[1][0] = robot.joints[x].axis[1];
    ax[2][0] = robot.joints[x].axis[2];
    ax[3][0] = 1;
    var T =[];
    T = robot.joints[x].xform;
    var world = [];
    world = matrix_multiply(T, ax);
    world = matrix_transpose(world);
    world.length = world.length - 1;
    return world;
}

function joint_position(x){
    var T = [];
    T = robot.joints[x].xform;
    var world = [];
    world = matrix_multiply(T,[[0],[0],[0],[1]]);
    world = matrix_transpose(world);
    world.length = world.length - 1;
    return world;
}

function vector_minus(a,b){
    var c =[];
    for (var i=0;i<a.length;i++){
        c[i] = a[i] - b[i];
    }
    return c;
}

function get_euler_angle(x){
    var lis1 = []
    var rmat = [];
    rmat = robot.joints[x].xform;
    
    var R11, R12, R13, R23, R33;
    R11 = rmat[0][0];
    R12 = rmat[0][1];
    R13 = rmat[0][2];
    R23 = rmat[1][2];
    R33 = rmat[2][2];
    //console.log(R13);
    var r,p,y;
    r = Math.atan(-R23/R33);
    p = Math.asin(R13);
    y = Math.atan(-R12/R11);
    //console.log(p);
    lis1[0] = r;
    lis1[1] = p;
    lis1[2] = y;
	
    return lis1;
}

function jacobian_trans(chain,p){
    var jaco = [];
    for (var i=0; i<chain.length; i++){
        jaco[i] = [];
    }
    for (var j=0; j<chain.length; j++){
        if (chain[j].type == 'prismatic') {
            xc = chain[j].name;
            jaco[j] = axis_position(xc);
        }
        else {
            xc = chain[j].name;
            z1 = axis_position(xc);
            o1 = joint_position(xc);
            o = vector_minus(p,o1);
            jaco[j] = vector_cross(z1, o);
        }
    }
    return jaco;
    // This jacobian is after being tranposed.
}

function jacobian_trans_6(chain, p){
    var jaco = [];
    for (var i=0; i<chain.length; i++){
        jaco[i] = [];
    }
    for (var j=0; j<chain.length; j++){
        if (chain[j].type == 'prismatic'){
            xd = chain[j].name;
            jaco[j] = axis_position(xd);
            jaco[j][3] = 0;
            jaco[j][4] = 0;
            jaco[j][5] = 0;
        }
        else {
            xd = chain[j].name;
            z2 = axis_position(xd);
            o2 = joint_position(xd);
            var pe = [];
            pe[0] = p[0];
            pe[1] = p[1];
            pe[2] = p[2];
            ox = vector_minus(pe,o2);
            jaco[j] = vector_cross(z2,ox);
            var jaco1 = [];
            jaco1 = z2;
            jaco[j][3] = jaco1[0];
            jaco[j][4] = jaco1[1];
            jaco[j][5] = jaco1[2];
        }
    }
    return jaco;
}