//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////
function quaternion_from_axisangle(r,x){
	if (robot.links_geom_imported == true){
		if (r.joints[x].type == 'revolute'){
			if (r.joints[x].angle > r.joints[x].limit.upper){
				r.joints[x].angle = r.joints[x].limit.upper;
			}
			else if (r.joints[x].angle < r.joints[x].limit.lower){
				r.joints[x].angle = r.joints[x].limit.lower;
			}
		}
	}
	a = r.joints[x].angle;
	b = r.joints[x].axis[0];
	c = r.joints[x].axis[1];
	d = r.joints[x].axis[2];
	cos = Math.cos(a/2);
	sin = Math.sin(a/2);
	var q=[];
	q = [cos, [b*sin, c*sin, d*sin]];
	return q;
}

function quaternion_normalize(q){
	q0 = Math.pow(q[0],2);
	q1 = Math.pow(q[1][0],2);
	q2 = Math.pow(q[1][1],2);
	q3 = Math.pow(q[1][2],2);
	s = sqrt(q0+q1+q2+q3);
	q[0] = q[0]/s;
	for(var i=0;i<q[1].length;i++){
		q[1][i] = q[1][i]/s;
	}
	//q = q/sqrt(q0+q1+q2+q3);
	return q;
}

function quaternion_to_rotation_matrix(t){
    var qmat = [];
    q0 = t[0];
    q1 = t[1][0];
    q2 = t[1][1];
    q3 = t[1][2];
    for(var k=0;k<4;k++){
        qmat[k] = [];
    }
    qmat[0][0] = Math.pow(q0,2)+Math.pow(q1,2)-Math.pow(q2,2)-Math.pow(q3,2);
    qmat[0][1] = 2*(q1*q2 - q0*q3);
    qmat[0][2] = 2*(q0*q2 + q1*q3);
    qmat[0][3] = 0;

    qmat[1][0] = 2*(q1*q2+q0*q3);
    qmat[1][1] = Math.pow(q0,2)-Math.pow(q1,2)+Math.pow(q2,2)-Math.pow(q3,2);
    qmat[1][2] = 2*(q2*q3-q0*q1);
    qmat[1][3] = 0;

    qmat[2][0] = 2*(q1*q3-q0*q2);
    qmat[2][1] = 2*(q2*q3+q0*q1);
    qmat[2][2] = Math.pow(q0,2)-Math.pow(q1,2)-Math.pow(q2,2)+Math.pow(q3,2);
    qmat[2][3] = 0;

    qmat[3] = [0,0,0,1];
    return qmat;
}

function quaternion_multiply(q1,q2){
	var q = [];
	var qm1 = [];
	var qm2 = [];
	q10 = q1[0];
	q20 = q2[0];
	q11 = q1[1][0];
	q21 = q2[1][0];
	q12 = q1[1][1];
	q22 = q2[1][1];
	q13 = q1[1][2];
	q23 = q2[1][2];
	q[0] = q10*q20 - q11*q21 - q12*q22 - q13*q23;
	q[1] = [];
	qd = vector_cross(q1[1],q2[1]);
	for (i=0;i<3;i++){
		qm1[i] = q1[0]*q2[1][i];
		qm2[i] = q2[0]*q1[1][i];
		q[1][i] = qm1[i]+qm2[i]+qd[i];
	}
	return q;
}
    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

