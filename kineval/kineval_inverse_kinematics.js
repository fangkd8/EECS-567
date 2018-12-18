
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
    }

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
    chain = [];
    var idx = endeffector_joint;
    i=0;
    while (robot.joints[idx].parent != base.name){
        chain.unshift(idx);
        var prc = robot.joints[idx].parent;
        idx = robot.links[prc].parent
    }
    chain.unshift(idx);

    end_p = matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_position_local);
    orientation = get_euler_angle(endeffector_joint);
    target = [];//6-dim vector in delta_x calculation.
    pos = [];//6-dim
    target_1 = [];
    if (!kineval.params.ik_orientation_included){
        for(var i=0; i<3; i++){
            target[i] = endeffector_target_world.position[i][0];
            pos[i] = end_p[i][0];
            target_1[i] = endeffector_target_world.position[i][0];
        }
        for(var i=3; i<6; i++){
            target[i] = 0;
            pos[i] = 0;
        }           
    }
    else if(kineval.params.ik_orientation_included){
        for(var i=0; i<3; i++){
            target[i] = endeffector_target_world.position[i][0];
            pos[i] = end_p[i][0];
        }
        for(var i=3; i<6; i++){
            target[i] = endeffector_target_world.orientation[i-3];
            pos[i] = orientation[i-3];
        }
    }

    end_pp = end_p; var trash = end_pp.pop();
    if (!kineval.params.ik_orientation_included)
        delta_x1 = vector_minus(target_1, end_pp); delta_x1 = matrix_transpose(delta_x1);
    
    delta_x = vector_minus(target, pos); delta_x = matrix_transpose(delta_x);

    J = jacobian_matrix(chain, pos);
    J1 = Jacobian_3_matrix(chain, pos);
    //J1 is 3-N matrix.

    if ((!kineval.params.ik_orientation_included)&&(kineval.params.ik_pseudoinverse)){
        if (!kineval.params.ik_pseudoinverse) inv_J = matrix_transpose(J1);
        else if (kineval.params.ik_pseudoinverse){
            if (J1.length >= J1[0].length){
                JT1 = matrix_transpose(J1);
                inv_J = matrix_multiply(JT1, J1);
                //inv_J = numeric.inv(inv_J);
                inv_J = matrix_inverse(inv_J);
                inv_J = matrix_multiply(inv_J, JT1);
            }
            else if (J1.length < J1[0].length){
                JT1 = matrix_transpose(J1);
                inv_J = matrix_multiply(J1, JT1);
                //inv_J = numeric.inv(inv_J);
                inv_J = matrix_inverse(inv_J);
                inv_J = matrix_multiply(JT1, inv_J);
            }
        } 
        q = matrix_multiply(inv_J, delta_x1);  
    }
    else if ((!kineval.params.ik_orientation_included)&&(!kineval.params.ik_pseudoinverse)){
        inv_J = matrix_transpose(J);
        q = matrix_multiply(inv_J, delta_x);
    }
    else if (kineval.params.ik_orientation_included){
        if (!kineval.params.ik_pseudoinverse) inv_J = matrix_transpose(J);
        else if (kineval.params.ik_pseudoinverse){
            if (J.length >= J[0].length){
                JT = matrix_transpose(J);
                inv_J = matrix_multiply(JT, J);
                //inv_J = numeric.inv(inv_J);
                inv_J = matrix_inverse(inv_J);
                inv_J = matrix_multiply(inv_J, JT);
            }
            else if (J.length < J[0].length){
                JT = matrix_transpose(J);
                inv_J = matrix_multiply(J, JT);
                //inv_J = numeric.inv(inv_J);
                inv_J = matrix_inverse(inv_J);
                inv_J = matrix_multiply(JT, inv_J);
            }
        } 
        q = matrix_multiply(inv_J, delta_x);
    }

    for (var i=0; i<chain.length; i++){
        robot.joints[chain[i]].control += kineval.params.ik_steplength*q[i][0];
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
    
    var r,p,y;
    r = Math.asin(rmat[2][1]);
    p = Math.atan2(-rmat[2][0],rmat[2][2]);
    y = Math.atan2(-rmat[0][1],rmat[1][1]);

    lis1[0] = r;
    lis1[1] = p;
    lis1[2] = y;
	
    return lis1;
}

function jacobian_matrix(chain, pos){
    var J = [[],[],[],[],[],[]];
    for (var i=0; i<chain.length; i++){
        // joint to world frame;
        var joint_world = [];
        joint_world = matrix_multiply(robot.joints[chain[i]].xform, [[0],[0],[0],[1]]);
        // axis in world
        var axis = axis_position(chain[i]);
        var end = [
        [pos[0] - joint_world[0][0]],
        [pos[1] - joint_world[1][0]],
        [pos[2] - joint_world[2][0]],
        ];
        var k0 = [joint_world[0],joint_world[1],joint_world[2]];
        var k = vector_minus(axis, k0);
        if (robot.joints[chain[i]].type == 'prismatic'){
            J[0][i] = k[0];
            J[1][i] = k[1];
            J[2][i] = k[2];
            J[3][i] = 0;
            J[4][i] = 0;
            J[5][i] = 0;
        }
        else{
            var v1 = vector_cross(k,end);
            J[0][i] = v1[0];
            J[1][i] = v1[1];
            J[2][i] = v1[2];
            J[3][i] = k[0];
            J[4][i] = k[1];
            J[5][i] = k[2];
        }
    }
    return J;
}

function Jacobian_3_matrix(chain, pos){
    var J = [[],[],[]];
    for (var i=0; i<chain.length; i++){
        // joint to world frame;
        var joint_world = [];
        joint_world = matrix_multiply(robot.joints[chain[i]].xform, [[0],[0],[0],[1]]);
        // axis in world
        var axis = axis_position(chain[i]);
        var end = [
        [pos[0] - joint_world[0][0]],
        [pos[1] - joint_world[1][0]],
        [pos[2] - joint_world[2][0]],
        ];
        var k0 = [joint_world[0],joint_world[1],joint_world[2]];
        var k = vector_minus(axis, k0);
        if (robot.joints[chain[i]].type == 'prismatic'){
            J[0][i] = k[0];
            J[1][i] = k[1];
            J[2][i] = k[2];
        }
        else{
            var v1 = vector_cross(k,end);
            J[0][i] = v1[0];
            J[1][i] = v1[1];
            J[2][i] = v1[2];
        }
    }
    return J;
}