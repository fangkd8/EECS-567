
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 
    if (robot.links_geom_imported == true) {
        var offset_xform = [];
        offset_xform = matrix_multiply(generate_rotation_matrix_X(-Math.PI/2),generate_rotation_matrix_Z(-Math.PI/2));
        
    }
    else if (robot.links_geom_imported == undefined) {
        var offset_xform = [];
        offset_xform = [
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,0],
        [0,0,0,1],
        ];
    }
    kineval.buildFKTransforms = 1;
    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }
    robot_heading = [];
    robot_lateral = [];
    for (i=0;i<4;i++){
        robot_heading[i] = [];
        robot_lateral[i] = [];
        robot_lateral[i][0] = 0;
        robot_heading[i][0] = 0;
    }
    var onset_xform = [];
        onset_xform = [
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,0],
        [0,0,0,1],
        ];
    robot_heading[2][0]=1;
    robot_lateral[0][0]=1;
    robot_heading[3][0]=1;
    robot_lateral[3][0]=1;

    t0 = robot.origin.rpy[0];
    t1 = robot.origin.rpy[1];
    t2 = robot.origin.rpy[2];
    var t = [];
    t = robot.origin.xyz; 
    traverseFKBase(t[0],t[1],t[2],t0,t1,t2,onset_xform);
    robot_heading = matrix_multiply(base.xform,robot_heading);
    robot_lateral = matrix_multiply(base.xform,robot_lateral);
    traverseFKBase(t[0],t[1],t[2],t0,t1,t2,offset_xform);
    rlinks = new Array();

    if (robot.links_geom_imported == undefined){
        for (x in robot.joints){
            for (i=0;i<base.children.length;i++){
                if (robot.joints[x].name == base.children[i]){
                    robot.joints[x].q = quaternion_from_axisangle(robot,x);
                    robot.joints[x].mat = quaternion_to_rotation_matrix(robot.joints[x].q);                
                    robot.joints[x].xform = matrix_multiply(base.xform,traverseFKJoint(x,onset_xform));
                    child = robot.joints[x].child;
                    traverseFKLink(robot.links[child],robot.joints[x]);
                    robot.links[child].xform = matrix_multiply(robot.links[child].xform,robot.joints[x].mat);
                    rlinks.push(robot.links[child]);
                }
            }
        }

        for (i=0;i<rlinks.length;i++){
            if (rlinks[i].children.length!==0){
                ex = rlinks[i].children.length;
                for (j=0;j<ex;j++){
                    nam = rlinks[i].children[j];
                    parent = rlinks[i].name;
                    robot.joints[nam].xform = matrix_multiply(robot.links[parent].xform,traverseFKJoint(nam,onset_xform)); 
                    robot.joints[nam].q = quaternion_from_axisangle(robot,nam);
                    chi = robot.joints[nam].child;
                    robot.joints[nam].mat = quaternion_to_rotation_matrix(robot.joints[nam].q);
                    traverseFKLink(robot.links[chi],robot.joints[nam]);
                    robot.links[chi].xform = matrix_multiply(robot.links[chi].xform,robot.joints[nam].mat);
                    rlinks.push(robot.links[chi]);
                }
            }
        }
    }
    
    else {
        for (x in robot.joints){
            for (i=0;i<base.children.length;i++){
                if (robot.joints[x].name == base.children[i]){
                    robot.joints[x].xform = matrix_multiply(base.xform,traverseFKJoint(x,onset_xform));
                    child = robot.joints[x].child;
                    traverseFKLink(robot.links[child],robot.joints[x]);
                    if (robot.joints[x].type == 'revolute'||robot.joints[x].type == 'continuous'){
                        robot.joints[x].q = quaternion_from_axisangle(robot,x);
                        robot.joints[x].mat = quaternion_to_rotation_matrix(robot.joints[x].q);
                        robot.links[child].xform = matrix_multiply(robot.links[child].xform,robot.joints[x].mat); 
                    }
                    else if (robot.joints[x].type == 'prismatic'){
                        var t=[];
                        if (robot.joints[x].angle > robot.joints[x].limit.upper){
                            robot.joints[x].angle = robot.joints[x].limit.upper;
                        }
                        else if (robot.joints[x].angle < robot.joints[x].limit.lower){
                            robot.joints[x].angle = robot.joints[x].limit.lower;
                        }
                        t = vector_multi(robot.joints[x].angle,robot.joints[x].axis);
                        var mt =[];
                        mt = generate_translation_matrix(t[0],t[1],t[2]);
                        robot.links[child].xform = matrix_multiply(robot.links[child].xform,mt);
                    }              
                    rlinks.push(robot.links[child]);
                }
            }
        }

        for (i=0;i<rlinks.length;i++){
            if (rlinks[i].children.length!==0){
                ex = rlinks[i].children.length;
                for (j=0;j<ex;j++){
                    nam = rlinks[i].children[j];
                    parent = rlinks[i].name;
                    robot.joints[nam].xform = matrix_multiply(robot.links[parent].xform,traverseFKJoint(nam,onset_xform)); 
                    chi = robot.joints[nam].child;
                    traverseFKLink(robot.links[chi],robot.joints[nam]);
                    if (robot.joints[nam].type == 'revolute'||robot.joints[nam].type == 'continuous'){
                        robot.joints[nam].q = quaternion_from_axisangle(robot,nam);
                        robot.joints[nam].mat = quaternion_to_rotation_matrix(robot.joints[nam].q);
                        robot.links[chi].xform = matrix_multiply(robot.links[chi].xform,robot.joints[nam].mat);
                    }
                    else if (robot.joints[nam].type == 'prismatic'){
                        var t=[];
                        if (robot.joints[nam].angle > robot.joints[nam].limit.upper){
                            robot.joints[nam].angle = robot.joints[nam].limit.upper;
                        }
                        else if (robot.joints[nam].angle < robot.joints[nam].limit.lower){
                            robot.joints[nam].angle = robot.joints[nam].limit.lower;
                        }
                        t = vector_multi(robot.joints[nam].angle,robot.joints[nam].axis);
                        var mt =[];
                        mt = generate_translation_matrix(t[0],t[1],t[2]);
                        robot.links[chi].xform = matrix_multiply(robot.links[chi].xform,mt);
                    }
                    rlinks.push(robot.links[chi]);
                }
            }
        }
    }

    // STENCIL: implement kineval.buildFKTransforms();
}
function traverseFKBase(a,b,c,r1,r2,r3,off){
    for (x in robot.links){
        if (robot.links[x].name == robot.base){
            base = robot.links[x];
            base.xform = generate_identity(a,b,c,r1,r2,r3,off);
        }
    }
}
function traverseFKLink(m1,m2){
    m1.xform = matrix_copy(m2.xform);
}
function traverseFKJoint(x1,off){
    m1 = generate_rotation_matrix_X(robot.joints[x1].origin.rpy[0]);
    m2 = generate_rotation_matrix_Y(robot.joints[x1].origin.rpy[1]);
    m3 = generate_rotation_matrix_Z(robot.joints[x1].origin.rpy[2]);
    n = generate_translation_matrix(robot.joints[x1].origin.xyz[0],robot.joints[x1].origin.xyz[1],robot.joints[x1].origin.xyz[2]);
    mat = matrix_multiply(m1,off);
    mat = matrix_multiply(m2,mat);
    mat = matrix_multiply(m3,mat);
    mat = matrix_multiply(n,mat);
    return mat;
}
function vector_multi(a,b){
    var c=[];
    for (var i = 0; i < b.length; i++) {
        c[i] = b[i]*a;
    }
    return c;
}
    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
    //   if (robot.links_geom_imported) {
    //       var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));
