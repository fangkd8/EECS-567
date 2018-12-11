
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    eps_p = 0.7;
    eps_a = 0.7;

    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);

    path = [];
}



function robot_rrt_planner_iterate() {

    var i;
    if ((!kineval.params.RRT_original)&&(!kineval.params.RRT_star)){
        rrt_alg = 1;
    }
    else if (kineval.params.RRT_original){
        rrt_alg = 0;
    }
    else if (kineval.params.RRT_star){
        rrt_alg = 2;  
    }
  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED), 2: rrt-star;

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();
        if (rrt_alg == 0){//RRT
            qrand = random_config(q_goal_config);
            ind = find_nearest_neighbor(T_a, qrand);
            q_new = new_config(T_a.vertices[ind], qrand);

            if (!kineval.poseIsCollision(q_new)){
                tree_add_vertex(T_a, q_new);
                tree_add_edge(T_a, T_a.newest, ind);
            }

            if (finish_search(q_new, q_goal_config)){
                rrt_iterate = false;
                kineval.motion_plan = [];
                var n = find_nearest_neighbor(T_a, q_goal_config);
                q_now = T_a.vertices[n];
                kineval.motion_plan.unshift(T_b.vertices[0]);
                while (!finish_search(q_now.vertex, q_start_config)){
                    kineval.motion_plan.unshift(q_now);
                    q_now = q_now.vertex.parent;
                }
                kineval.motion_plan.unshift(q_now);
                for (var i=0; i<kineval.motion_plan.length; i++){
                    kineval.motion_plan[i].geom.material.color = {r:0,g:1,b:0};
                }

                return "reached";
            }

        }

        if (rrt_alg == 1){//RRT-connect
            qrand = random_config(q_goal_config);
            ind = find_nearest_neighbor(T_a, qrand);
            q_new = new_config(T_a.vertices[ind], qrand);
            if (!kineval.poseIsCollision(q_new)&&T_b.vertices.length>=T_a.vertices.length){
                tree_add_vertex(T_a, q_new);
                tree_add_edge(T_a, T_a.newest, ind);
                ind1 = find_nearest_neighbor(T_b, q_new);
                q_new1 = new_config(T_b.vertices[ind1], q_new);
                if (!kineval.poseIsCollision(q_new1)){
                    tree_add_vertex(T_b, q_new1);
                    tree_add_edge(T_b, T_b.newest, ind1);
                    if (!finish_search(q_new1, q_new)){
                        q_new2 = new_config(T_b.vertices[T_b.newest], q_new);
                        if (!kineval.poseIsCollision(q_new2)){
                            tree_add_vertex(T_b,q_new2);
                            tree_add_edge(T_b,T_b.newest,ind1);
                            q_new1 = q_new2;
                        }
                    }
                    else if (finish_search(q_new1, q_new)){
                        rrt_iterate = false;
                        kineval.motion_plan = [];
                        q_now = T_a.vertices[T_a.newest];
                        kineval.motion_plan.unshift(q_now);
                        while (!finish_search(q_now.vertex, q_start_config)){
                            q_now = q_now.vertex.parent;
                            kineval.motion_plan.unshift(q_now);
                        }
                        kineval.motion_plan.unshift(T_a.vertices[0]);
                        q_now = T_b.vertices[T_b.newest];
                        kineval.motion_plan.push(q_now);
                        while (!finish_search(q_now.vertex, q_goal_config)){
                            q_now = q_now.vertex.parent;
                            kineval.motion_plan.push(q_now);
                        }
                        kineval.motion_plan.push(T_b.vertices[0]);
                        for (var i=0; i<kineval.motion_plan.length; i++){
                            kineval.motion_plan[i].geom.material.color = {r:1,g:0,b:0};
                        }
                        return "reached";
                    }
                }
            }
            else if(T_b.vertices.length<T_a.vertices.length){
                qrand = random_config(q_start_config);
                ind = find_nearest_neighbor(T_b, qrand);
                q_new = new_config(T_b.vertices[ind], qrand);
                if (!kineval.poseIsCollision(q_new)){
                    tree_add_vertex(T_b, q_new);
                    tree_add_edge(T_b, T_b.newest, ind);
                    ind1 = find_nearest_neighbor(T_a, q_new);
                    q_new1 = new_config(T_a.vertices[ind1],q_new);
                    if (!kineval.poseIsCollision(q_new1)){
                        tree_add_vertex(T_a, q_new1);
                        tree_add_edge(T_a, T_a.newest, ind1);
                        if (!finish_search(q_new1, q_new)){
                            q_new2 = new_config(T_a.vertices[T_a.newest], q_new);
                            if(!kineval.poseIsCollision(q_new2)){
                                tree_add_vertex(T_a, q_new2);
                                tree_add_edge(T_a, T_a.newest, ind1);
                                q_new1 = q_new2;
                            }
                        }
                        else if(finish_search(q_new1, q_new)){
                            rrt_iterate = false;
                            kineval.motion_plan = [];
                            q_now = T_a.vertices[T_a.newest];
                            kineval.motion_plan.unshift(q_now);
                            while (!finish_search(q_now.vertex, q_start_config)){
                                q_now = q_now.vertex.parent;
                                kineval.motion_plan.unshift(q_now);
                            }
                            kineval.motion_plan.unshift(T_a.vertices[0]);
                            q_now = T_b.vertices[T_b.newest];
                            kineval.motion_plan.push(q_now);
                            while (!finish_search(q_now.vertex, q_goal_config)){
                                q_now = q_now.vertex.parent;
                                kineval.motion_plan.push(q_now);
                            }
                            kineval.motion_plan.push(T_b.vertices[0]);
                            for (var i=0; i<kineval.motion_plan.length; i++){
                                kineval.motion_plan[i].geom.material.color = {r:1,g:0,b:0};
                            }
                            return "reached";
                        }
                    }
                }
            }
        }

        if (rrt_alg == 2){ //RRT*
            qrand = random_config(q_goal_config);
            ind = find_nearest_neighbor(T_a, qrand);
            q_new = steer(T_a, ind, qrand);
            if (q_new !== 'invalid'){
                list = find_near_list(T_a, q_new);
                [z_min, c_min] = choose_parent(T_a, list, ind, q_new);
                tree_add_vertex(T_a, q_new);
                tree_add_edge(T_a, z_min, T_a.newest);
                T_a.vertices[T_a.newest].cost = c_min;
                new_idx = T_a.newest;
                T_a = rewire(T_a, list, z_min, new_idx);
            }

            if (finish_search(q_new, q_goal_config)){
                rrt_iterate=false;
                kineval.motion_plan = [];
                var n = find_nearest_neighbor(T_a, q_goal_config);
                q_now = T_a.vertices[n];
                kineval.motion_plan.unshift(T_b.vertices[0]);
                while (!finish_search(q_now.vertex, q_start_config)){
                    kineval.motion_plan.unshift(q_now);
                    q_now = q_now.vertex.parent;
                }
                kineval.motion_plan.unshift(q_now);
                for (var i=0; i<kineval.motion_plan.length; i++){
                    kineval.motion_plan[i].geom.material.color = {r:0,g:0,b:1};
                }
                //T_a.vertices[T_a.newest].geom.material.color = {r:0,g:0,b:1};

                return "reached";
            }
        }
    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
    }

}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];
    tree.vertices[0].cost = 0;
    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

function removeTreeEdge(tree, q1_idx, q2_idx){
    for (var i=0; i<tree.vertices[q1_idx].edges.length;i++){
        if (tree.vertices[q1_idx].edges[i] == q2_idx)
            tree.vertices[q1_idx].edges.splice(i,1);
    }

    for (var i=0; i<tree.vertices[q2_idx].edges.length;i++){
        if (tree.vertices[q2_idx].edges[i] == q1_idx)
            tree.vertices[q2_idx].edges.splice(i,1);
    }
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////
function random_config(a){
    var xval = Math.abs(robot_boundary[0][0]-robot_boundary[1][0]);
    var zval = Math.abs(robot_boundary[0][2]-robot_boundary[1][2]);
    var q=[];
    q[0] = Math.random()*(xval+6) + robot_boundary[0][0]-3;
    q[1] = 0;
    q[2] = Math.random()*(zval+6) + robot_boundary[0][2]-3;
    for (var i=3; i<q_start_config.length; i++){
        q[i] = Math.random()*2*Math.PI;
    } 
    q[3] = 0;
    q[5] = 0;
    for (var i=6;i<q_start_config.length; i++){
        if (typeof(robot.joints[q_index[i]].limit)!== 'undefined'){
            if (q[i]>robot.joints[q_index[i]].limit.upper){
                q[i] = robot.joints[q_index[i]].limit.upper;
            }
            else if (q[i]<robot.joints[q_index[i]].limit.lower){
                q[i] = robot.joints[q_index[i]].limit.lower;
            }
        }
    }
    var p = Math.random();
    if (p <= 0.08){
        for (var i=0;i<q_start_config.length;i++){
            q[i] = a[i];
        }
    }
    //make the robot can only rotate along y"up".
    return q;
}

function find_nearest_neighbor(tree, q){
    var idx;
    var max_dis = Infinity;
    for (var i=0; i<tree.vertices.length; i++){
        var a = tree.vertices[i].vertex[0];
        var b = tree.vertices[i].vertex[2];
        var dis = Math.pow(Math.pow(q[0]-a,2)+Math.pow(q[2]-b,2),0.5);
        for (var j=4; j<tree.vertices[i].length;j++){
            dis += q[j]-tree.vertices[i].vertex[j];
        }
        if (dis <= max_dis){
            max_dis = dis;
            idx = i;
        }
    }
    return idx;
}

function new_config(qnear, qrand){
    var delta_x = [];
    for (var i=0; i<qrand.length; i++){
        delta_x[i] = qrand[i] - qnear.vertex[i];
    }
    delta_x = vector_normalize(delta_x);
    var q_new = [];
    for (var i=0; i<qrand.length; i++){
        q_new[i] = qnear.vertex[i] + eps_p*delta_x[i];
    }
    for (var i=6; i<qrand.length; i++){
        if (typeof(robot.joints[q_index[i]].limit)!== 'undefined'){
            if (q_new[i]>robot.joints[q_index[i]].limit.upper){
                q_new[i] = robot.joints[q_index[i]].limit.upper;
            }
            else if (q_new[i]<robot.joints[q_index[i]].limit.lower){
                q_new[i] = robot.joints[q_index[i]].limit.lower;
            }
        }
    }
    q_new.parent = qnear;
    return q_new;
}

function finish_search(A, B){
    //test q_new and q_goal;
    var a = A[0] - B[0];
    var b = A[2] - B[2];
    if (A == 'invalid'){return false;}
    if (Math.sqrt(Math.pow(a,2)+Math.pow(b,2))>eps_p) {
        return false;
    }
    for (i=4; i < q_start_config.length; i++){
        if(i==5) {continue;};
        if(Math.abs(B[i]-A[i])>eps_a){
            return false;
        }
    }
    return true;
}

function rrt_extend(T, q){
    var ind = find_nearest_neighbor(T, q);
    q_new = new_config(T.vertices[ind],q);

    if(!kineval.poseIsCollision(q_new)){
        tree_add_vertex(T, q_new);
        tree_add_edge(T, T.newest, ind);

        return "advanced";
    }
    return "trapped";
}
// For RRT*. 
function steer(T, idx, rand){
    q_near = T.vertices[idx];
    q_new = new_config(q_near, rand);
    if (kineval.poseIsCollision(q_new))
        q_new = 'invalid';
    return q_new;
}

function find_near_list(T, new1){
    var list = [];
    var l = new1.length;
    for (i=0;i<T.vertices.length;i++){
        var t = norm_dis(T.vertices[i].vertex, new1);
        if (t < 2*eps_p + 2*(l-5)*eps_a)
            list.push(i);
    }
    return list;
}

function choose_parent(T, list, idx, z_new){
    var z_min = idx;
    var c_min = T.vertices[idx].cost + norm_dis(T.vertices[idx].vertex, z_new);
    for(var i=0;i<list.length;i++){
        var excited = T.vertices[list[i]].cost + norm_dis(T.vertices[list[i]].vertex, z_new);
        if (excited < c_min){
            z_min = list[i];
            c_min = excited;
        }
    }
    return [z_min, c_min];
}

function reconnect(T, idx, znew){
    var p = find_parent_idx(T, T.vertices[idx].vertex);
    removeTreeEdge(T, p, idx);
    T.vertices[idx].parent = T.vertices[znew];
    tree_add_edge(T, znew, idx);
    return T;
}

function rewire(T, list, idx1, idx2){
    for (var i=0;i<list.length;i++){
        if (list[i] !== idx1){
            var znewcost = T.vertices[idx2].cost;
            var n_n = norm_dis(T.vertices[idx2].vertex, T.vertices[list[i]].vertex);
            var znearcost = T.vertices[list[i]].cost;

            var flag_co = false;

            //for (var j=0; j<Math.floor(n_n/eps_p); j++){
                var test = steer(T, idx2, T.vertices[list[i]].vertex);
                if (test == 'invalid')
                    flag_co = true;
            //}

            if(!flag_co && ((znewcost + n_n) < znearcost))
                T = reconnect(T, list[i], idx2);
        }
    }
    return T;
}

//Assistance. 
function norm_dis(A, B){
    var norm = 0;
    for(var i=0; i<A.length; i++){
        norm += Math.pow(A[i] - B[i], 2);
    }
    norm = Math.sqrt(norm);
    return norm;
}

function range_collision(A, B){
    //justify in RRT*. 
    var a = A[0] - B[0];
    var b = A[2] - B[2];
    if (Math.sqrt(Math.pow(a,2)+Math.pow(b,2))>2*eps_p) {
        return false;
    }
    for (i=4; i < q_start_config.length; i++){
        if(i==5) {continue};
        if(Math.abs(b[i]-a[i])>2*eps_a){
            return false;
        }
    }
    return true;
}

function find_parent_idx(T, q){
    for (var i=0; i<T.vertices.length; i++){
        var a = [];
        a = T.vertices[i].vertex;
        if (norm_dis(a, q.parent.vertex) == 0)
            return i;
    }
}

    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs










