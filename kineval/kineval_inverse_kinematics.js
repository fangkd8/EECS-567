
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

    // STENCIL: see instructor for random time trial code
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {
    if (kineval.params.ik_orientation_included == false){
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
        
        jack = jacobian_trans(chain,pos);
        qspace = matrix_multiply(jack, delta_x);
        //q_space;  

        for (var iv=0;iv<chain.length;iv++){
            chain[iv].control = chain[iv].control + kineval.params.ik_steplength*qspace[iv][0];
        }
    }

    else if(kineval.params.ik_orientation_included){
        var nend1 = endeffector_joint;
        var pos1 = matrix_multiply(robot.joints[nend1].xform, endeffector_position_local);
        pos1 = matrix_transpose(pos1); pos1.length = pos1.length-1;
        for (var i=3;i<6;i++){
            pos1[i] = robot.joints[nend1].origin.rpy[i-3];
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
        for (var i=0;i<3;i++){
            tg[i] = target1[i];
        }
        for (var i=0;i<3;i++){
            tg[i+3] = target2[i];
        }

        //tg is the target with both position and euler angle. row vector, 6 entries.

        //var delta_x1 = [];
        delta_x1 = vector_minus(tg, pos1);
        delta_x1 = matrix_transpose(delta_x1);
        //vector towards the target. 
        jack1 = jacobian_trans(chain1,pos1);
        qspace1 = matrix_multiply(jack1, delta_x1);
        for (var iv=0;iv<chain1.length;iv++){
            chain1[iv].control = chain1[iv].control + kineval.params.ik_steplength*qspace1[iv][0];
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

function jacobian_trans(chain,p){
    if (p.length == 3){
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
    }
    else if (p.length == 6){
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
                o2 = joint_position(xc);
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
    }
    return jaco;
    // This jacobian is after being tranposed.
}