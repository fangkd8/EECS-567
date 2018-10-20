
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
    kineval.buildFKTransforms = 1;
    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }
    traverseFKBase();
    rlinks = new Array();
    //console.log(base.child);
    for (x in robot.joints){
        for (i=0;i<base.child.length;i++){
            if (robot.joints[x].name == base.child[i]){
                robot.joints[x].xform = matrix_multiply(base.xform,traverseFKJoint(x));
                child = robot.joints[x].child;
                traverseFKLink(robot.links[child],robot.joints[x]);
                rlinks.push(robot.links[child]);
            }
        }
    }
    for (i=0;i<rlinks.length;i++){
        while (rlinks[i].child.length!==0){
            ex = rlinks[i].child.length;
            for (j=0;j<ex;j++){
                nam = rlinks[i].child[j];
                parent = rlinks[i].name;
                robot.joints[nam].xform = matrix_multiply(robot.links[parent].xform,traverseFKJoint(nam)); 
                chi = robot.joints[nam].child;
                traverseFKLink(robot.links[chi],robot.joints[nam]);
                rlinks.push(robot.links[chi]);
                rlinks[i].child.shift();                
            }

        }
    }
    // STENCIL: implement kineval.buildFKTransforms();

}
function traverseFKBase(){
    for (x in robot.links){
        if (robot.links[x].name == robot.base){
            base = robot.links[x];
            base.xform = generate_identity();
        }
    }
}
function traverseFKLink(m1,m2){
    m1.xform = matrix_copy(m2.xform);
}
function traverseFKJoint(x1){
    m1 = generate_rotation_matrix_X(robot.joints[x1].origin.rpy[0]);
    m2 = generate_rotation_matrix_Y(robot.joints[x1].origin.rpy[1]);
    m3 = generate_rotation_matrix_Z(robot.joints[x1].origin.rpy[2]);
    n = generate_translation_matrix(robot.joints[x1].origin.xyz);
    mat = matrix_multiply(m2,m1);
    mat = matrix_multiply(m3,mat);
    mat = matrix_multiply(n,mat);
    return mat;
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

