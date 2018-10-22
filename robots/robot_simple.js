//   CREATE ROBOT STRUCTURE

// KE 


//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "simple robot";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,0,0], rpy:[0,0,0]};  

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "body";  

        
// specify and create data objects for the links of the robot

robot.links = {
	"head":{},
	"neck":{},
	"body":{},
	"leftarm_up":{},
	"rightarm_up":{},
	"leftarm_down":{},
	"rightarm_down":{},
	"lefthand":{},
	"righthand":{},
	"leftlap":{},
	"rightlap":{},
	"leftlegd":{},
	"rightlegd":{},
	"leftfoot":{},
	"rightfoot":{}
}
//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]}; 
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0]; 
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z 
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

robot.joints.neck_down = {parent:"body", child:"neck"};
robot.joints.neck_down.origin = {xyz: [0,3.3,0], rpy:[0,0,0]};
robot.joints.neck_down.axis = [0.0,1.0,0.0]; 

robot.joints.neck_up = {parent:"neck", child:"head"};
robot.joints.neck_up.origin = {xyz: [0,0.2,0], rpy:[0,0,0]};
robot.joints.neck_up.axis = [0.0,1.0,0.0];

robot.joints.shoulder_right_pitch = {parent:"body", child:"rightarm_up"};
robot.joints.shoulder_right_pitch.origin = {xyz: [-0.6,3.15,0], rpy:[0,0,0]};
robot.joints.shoulder_right_pitch.axis = [0,1,0]; 

robot.joints.shoulder_left_pitch = {parent:"body", child:"leftarm_up"};
robot.joints.shoulder_left_pitch.origin = {xyz: [0.6,3.15,0], rpy:[0,0,0]};
robot.joints.shoulder_left_pitch.axis = [0,1,0]; 

robot.joints.shoulder_right_yaw = {parent:"body", child:"rightarm_up"};
robot.joints.shoulder_right_yaw.origin = {xyz: [-0.6,3.15,0], rpy:[0,0,0]};
robot.joints.shoulder_right_yaw.axis = [1,0,0]; 

robot.joints.shoulder_left_yaw = {parent:"body", child:"leftarm_up"};
robot.joints.shoulder_left_yaw.origin = {xyz: [0.6,3.15,0], rpy:[0,0,0]};
robot.joints.shoulder_left_yaw.axis = [1,0,0];

robot.joints.elbow_right = {parent:"rightarm_up", child:"rightarm_down"};
robot.joints.elbow_right.origin = {xyz: [-0.65,0,0], rpy:[0,0,0]};
robot.joints.elbow_right.axis = [0,1,0]; 

robot.joints.elbow_left = {parent:"leftarm_up", child:"leftarm_down"};
robot.joints.elbow_left.origin = {xyz: [0.65,0,0], rpy:[0,0,0]};
robot.joints.elbow_left.axis = [0.0,1.0,0.0]; 

robot.joints.whrist_right = {parent:"rightarm_down", child:"righthand"};
robot.joints.whrist_right.origin = {xyz: [-0.52,0,0], rpy:[0,0,0]};
robot.joints.whrist_right.axis = [0,1,0]; 

robot.joints.whrist_left = {parent:"leftarm_down", child:"lefthand"};
robot.joints.whrist_left.origin = {xyz: [0.52,0,0], rpy:[0,0,0]};
robot.joints.whrist_left.axis = [0,1,0]; 

robot.joints.rightlap_up = {parent:"body", child:"rightlap"};
robot.joints.rightlap_up.origin = {xyz: [-0.3,1.75,0], rpy:[0,0,0]};
robot.joints.rightlap_up.axis = [1,0,0];

robot.joints.leftlap_up = {parent:"body", child:"leftlap"};
robot.joints.leftlap_up.origin = {xyz: [0.3,1.75,0], rpy:[0,0,0]};
robot.joints.leftlap_up.axis = [1,0,0];

robot.joints.rightknee = {parent:"rightlap", child:"rightlegd"};
robot.joints.rightknee.origin = {xyz: [0,-0.67,0], rpy:[0,0,0]};
robot.joints.rightknee.axis = [1,0,0];

robot.joints.leftknee = {parent:"leftlap", child:"leftlegd"};
robot.joints.leftknee.origin = {xyz: [0,-0.67,0], rpy:[0,0,0]};
robot.joints.leftknee.axis = [1,0,0];

robot.joints.right_ankle = {parent:"rightlegd", child:"rightfoot"};
robot.joints.right_ankle.origin = {xyz: [0, -0.62, 0], rpy:[0,0,0]};
robot.joints.right_ankle.axis = [0,1,0]; 

robot.joints.left_ankle = {parent:"leftlegd", child:"leftfoot"};
robot.joints.left_ankle.origin = {xyz: [0, -0.62, 0], rpy:[0,0,0]};
robot.joints.left_ankle.axis = [0,1,0];


// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "elbow_left";
robot.endeffector.position = [[0],[0],[0.5],[1]];
//robot.endeffector.left.frame = "forearm_left_yaw";
//robot.endeffector.left.position = [[0],[0],[0.5],[1]];

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

/*  threejs geometry definition template, will be used by THREE.Mesh() to create threejs object
    // create threejs geometry and insert into links_geom data object
    links_geom["link1"] = new THREE.CubeGeometry( 5+2, 2, 2 );

    // example of translating geometry (in object space)
    links_geom["link1"].applyMatrix( new THREE.Matrix4().makeTranslation(5/2, 0, 0) );

    // example of rotating geometry 45 degrees about y-axis (in object space)
    var temp3axis = new THREE.Vector3(0,1,0);
    links_geom["link1"].rotateOnAxis(temp3axis,Math.PI/4);
*/

// define threejs geometries and associate with robot links 
links_geom = {};

links_geom["body"] = new THREE.CubeGeometry( 1, 1.5, 0.6 );
links_geom["body"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 2.5, 0) );

links_geom["neck"] = new THREE.CubeGeometry( 0.2, 0.2, 0.1 );
links_geom["neck"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["head"] = new THREE.SphereGeometry( 0.3, 0.3, 0.2 );
links_geom["head"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.0) ); 

links_geom["leftarm_up"] = new THREE.CubeGeometry( 0.7, 0.2, 0.2 );
links_geom["leftarm_up"].applyMatrix( new THREE.Matrix4().makeTranslation(0.3, 0, 0) );

links_geom["rightarm_up"] = new THREE.CubeGeometry( 0.7, 0.2, 0.2 );
links_geom["rightarm_up"].applyMatrix( new THREE.Matrix4().makeTranslation(-0.3, 0, 0) );

links_geom["leftarm_down"] = new THREE.CubeGeometry( 0.5, 0.2, 0.25 );
links_geom["leftarm_down"].applyMatrix( new THREE.Matrix4().makeTranslation(0.3, 0, 0) );

links_geom["rightarm_down"] = new THREE.CubeGeometry( 0.5, 0.2, 0.25 );
links_geom["rightarm_down"].applyMatrix( new THREE.Matrix4().makeTranslation(-0.3, 0, 0) );

links_geom["lefthand"] = new THREE.CubeGeometry( 0.3, 0.3, 0.1 );
links_geom["lefthand"].applyMatrix( new THREE.Matrix4().makeTranslation(0.2, 0, 0) );

links_geom["righthand"] = new THREE.CubeGeometry( 0.3, 0.3, 0.1 );
links_geom["righthand"].applyMatrix( new THREE.Matrix4().makeTranslation(-0.2, 0, 0) );

links_geom["leftlap"] = new THREE.CubeGeometry( 0.5, 0.65, 0.5 );
links_geom["leftlap"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.35, 0) );

links_geom["rightlap"] = new THREE.CubeGeometry( 0.5, 0.65, 0.5 );
links_geom["rightlap"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.35, 0) );

links_geom["leftlegd"] = new THREE.CubeGeometry( 0.4, 0.6, 0.4 );
links_geom["leftlegd"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.35, 0) );

links_geom["rightlegd"] = new THREE.CubeGeometry( 0.4, 0.6, 0.4 );
links_geom["rightlegd"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.35, 0) );


links_geom["leftfoot"] = new THREE.CubeGeometry( 0.4, 0.2, 0.6 );
links_geom["leftfoot"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.17, 0.12) );

links_geom["rightfoot"] = new THREE.CubeGeometry( 0.4, 0.2, 0.6 );
links_geom["rightfoot"].applyMatrix( new THREE.Matrix4().makeTranslation(0, -0.17, 0.12) );
