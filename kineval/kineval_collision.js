
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | collision detection

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

// KE: merge collision test into FK
// KE: make FK for a configuration and independent of current robot state

kineval.robotIsCollision = function robot_iscollision() {
    // test whether geometry of current configuration of robot is in collision with planning world 

    // form configuration from base location and joint angles
    var q_robot_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_robot_config.length;
        q_robot_config = q_robot_config.concat(robot.joints[x].angle);
    }

    // test for collision and change base color based on the result
    collision_result = kineval.poseIsCollision(q_robot_config);

    robot.collision = collision_result;
}


kineval.poseIsCollision = function robot_collision_test(q) {
    // perform collision test of robot geometry against planning world 

    // test base origin (not extents) against world boundary extents
    if ((q[0]<robot_boundary[0][0])||(q[0]>robot_boundary[1][0])||(q[2]<robot_boundary[0][2])||(q[2]>robot_boundary[1][2]))
        return robot.base;
    
    var mstack = generate_identity();
    if (robot.links_geom_imported == true) {
        var offset_xform = [];
        offset_xform = matrix_multiply(generate_rotation_matrix_X(-Math.PI/2),generate_rotation_matrix_Z(-Math.PI/2));
        mstack = matrix_multiply(offset_xform, mstack);
    }
    var xrot = generate_rotation_matrix_X(q[3]);
    var yrot = generate_rotation_matrix_Y(q[4]);
    var zrot = generate_rotation_matrix_Z(q[5]);
    var tran = generate_translation_matrix(q[0],q[1],q[2]);
    mstack = matrix_multiply(xrot, mstack);
    mstack = matrix_multiply(yrot, mstack);
    mstack = matrix_multiply(zrot, mstack);
    mstack = matrix_multiply(tran, mstack);
    // traverse robot kinematics to test each body for collision
    // STENCIL: implement forward kinematics for collision detection
    //return robot_collision_forward_kinematics(q);
    return traverse_collision_forward_kinematics_link(base, mstack, q);
}



function traverse_collision_forward_kinematics_link(link,mstack,q) {

    /* test collision FK
    console.log(link);
    */
    if (typeof link.visual !== 'undefined') {
        var local_link_xform = matrix_multiply(mstack,generate_translation_matrix(link.visual.origin.xyz[0],link.visual.origin.xyz[1],link.visual.origin.xyz[2]));
    }
    else {
        var local_link_xform = matrix_multiply(mstack,generate_identity());
    }

    // test collision by transforming obstacles in world to link space
/*
    mstack_inv = matrix_invert_affine(mstack);
*/
    mstack_inv = matrix_inverse(mstack);
    //mstack_inv = numeric.inv(mstack);

    var i;
    var j;

    // test each obstacle against link bbox geometry by transforming obstacle into link frame and testing against axis aligned bounding box
    //for (j=0;j<robot_obstacles.length;j++) { 
    for (j in robot_obstacles) { 

        var obstacle_local = matrix_multiply(mstack_inv,robot_obstacles[j].location);

        // assume link is in collision as default
        var in_collision = true; 

        // if obstacle lies outside the link extents along any dimension, no collision is detected
        if (
            (obstacle_local[0][0]<(link.bbox.min.x-robot_obstacles[j].radius))
            ||
            (obstacle_local[0][0]>(link.bbox.max.x+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[1][0]<(link.bbox.min.y-robot_obstacles[j].radius))
            ||
            (obstacle_local[1][0]>(link.bbox.max.y+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[2][0]<(link.bbox.min.z-robot_obstacles[j].radius)) 
            ||
            (obstacle_local[2][0]>(link.bbox.max.z+robot_obstacles[j].radius))
        )
                in_collision = false;

        // if obstacle lies within link extents along all dimensions, a collision is detected and return true
        if (in_collision)
            return link.name;
    }

    // recurse child joints for collisions, returning true if child returns collision
    if (typeof link.children !== 'undefined') { // return if there are no children
        var local_collision;
        //for (i=0;i<link.children.length;i++) {
        for (i in link.children) {
            local_collision = traverse_collision_forward_kinematics_joint(robot.joints[link.children[i]],mstack,q)
            if (local_collision)
                return local_collision;
        }
    }

    // return false, when no collision detected for this link and children 
    return false;
}
function traverse_collision_forward_kinematics_joint(joint, mstack, q){
 
    var xrot = generate_rotation_matrix_X(joint.origin.rpy[0]);
    var yrot = generate_rotation_matrix_Y(joint.origin.rpy[1]);
    var zrot = generate_rotation_matrix_Z(joint.origin.rpy[2]);
    var tran = generate_translation_matrix(joint.origin.xyz[0],joint.origin.xyz[1],joint.origin.xyz[2]);

    var Rm = matrix_multiply(yrot, xrot);
    Rm = matrix_multiply(zrot, Rm);
    Rm = matrix_multiply(tran, Rm);

    var quart = new_quarternion(q[q_names[joint.name]], joint.axis);
    quart = quaternion_to_rotation_matrix(quart);
    Rm = matrix_multiply(Rm, quart);
    
    var jstack;
    jstack = matrix_multiply(mstack, Rm);

    if (typeof(joint.child)!=='undefined'){
        return traverse_collision_forward_kinematics_link(robot.links[joint.child], jstack, q);
    }
}
function new_quarternion(angle, axis){
    var x,y,z;
    x = axis[0];
    y = axis[1];
    z = axis[2];
    var quart = [];
    quart[0] = Math.cos(angle/2);
    quart[1] = [];
    quart[1][0] = x*Math.sin(0.5*angle);
    quart[1][1] = y*Math.sin(0.5*angle);
    quart[1][2] = z*Math.sin(0.5*angle);

    return quart;
}

