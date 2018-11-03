
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 

    var m=[];
    m = kineval.params.dance_sequence_index;
    
    if (typeof(flag) !== 'undefined'){
        i = t;
    }
    else if (typeof(flag) == 'undefined'){
        i = 0;
    }
    kineval.params.setpoint_target = kineval.setpoints[m[i]];
    set = 0;
    kkp = 0;
    var gpa = [];
    gpa = kineval.setpoints[m[i]];
    for (x in robot.joints){
        kkp = kkp+robot.joints[x].angle;
    }
    for(x in gpa){
        set = set + gpa[x];
    }
    errorin = Math.abs(set-kkp);
    if (errorin < 0.002){
        t=i+1;
        flag = true;
    }
    if (typeof(t) !== 'undefined'){
        if (t == m.length){
            t = 0;
        }
    }
    
    // STENCIL: implement FSM to cycle through dance pose setpoints
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control
    for (y in robot.joints){
        P = robot.joints[y].servo.p_gain;
        D = robot.joints[y].servo.d_gain;
        P = 0.15;
        robot.joints[y].error = kineval.params.setpoint_target[y]-robot.joints[y].angle;
        robot.joints[y].control = P*robot.joints[y].error;
    }
    // STENCIL: implement P servo controller over joints
}


