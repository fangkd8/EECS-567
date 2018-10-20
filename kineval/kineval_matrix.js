//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}

function matrix_multiply(m1, m2) {
  var mat = [];
  for (var i = 0; i < m1.length; i++) {
    mat[i] = [];
    for (var j = 0; j < m2[0].length; j++) {
      var sum = 0;
      for (var k = 0; k < m1[0].length; k++) {
        sum += m1[i][k] * m2[k][j];
      }
      mat[i][j] = sum;
    }
  }
  return mat;
}

function matrix_transpose(m1) {
	var mat = [];
	var i,j;
	
	for (i=0;i<m1[0].length;i++){
		mat[i] = [];
		for (j=0;j<m1.length;j++){
			mat[i][j]=m1[j][i];
		}
	}
	return mat;	
}

function matrix_invert_affine(M){
    var i, j, k, dim, temp;
	dim = M.length;
    var m1 = [], m2 = [];
    for(i=0; i<dim; i+=1){
        m1[i]=[];
        m2[i]=[];
        for(j=0; j<dim; j+=1){
            if(i==j){ 
				m1[i][j] = 1; 
			}
            else{ 
				m1[i][j] = 0; 
			}
            m2[i][j] = M[i][j];
        }
    }
    for(i=0; i<dim; i+=1){
        temp = m2[i][i];
        if(temp==0){
            for(k=i+1; k<dim; k+=1){
                if(m2[k][i] != 0){
                    for(j=0; j<dim; j++){
                        temp = m2[i][j];
                        m2[i][j] = m2[k][j];
                        m2[k][j] = temp;
                        temp = m1[i][j];
                        m1[i][j] = m1[k][j];
                        m1[k][j] = temp;
                    }
                    break;
                }
            }
            temp = m2[i][i];
            if(temp==0){
				return
			}
        }
        for(j=0; j<dim; j++){
            m2[i][j] = m2[i][j]/temp; 
            m1[i][j] = m1[i][j]/temp; 
        }

        for(k=0; k<dim; k++){
            if(k==i){
				continue;
			}

            temp = m2[k][i];

            for(j=0; j<dim; j++){
                m2[k][j] -= temp*m2[i][j]; 
                m1[k][j] -= temp*m1[i][j]; 
            }
        }
    }
    return m1;
}

function matrix_pseudoinverse(A){
	var mat=[],trans=[];
	var N,M;
	N = A.length;
	M = A[0].length;
	trans = matrix_transpose(A);
	if (N > M){
		mat = matrix_multiply(trans,A);
		mat = matrix_invert_affine(mat);
		mat = matrix_multiply(mat,trans);
	}
	else if(N < M){
		mat = matrix_multiply(A,trans);
		mat = matrix_invert_affine(mat);
		mat = matrix_multiply(trans,mat);
	}
	return mat;
}

function vector_cross(a,b){
	var c=[];
	c[0]=a[1]*b[2]-a[2]*b[1];
	c[1]=a[2]*b[0]-a[0]*b[2];
	c[2]=a[0]*b[1]-a[1]*b[0];
	return c;
}
function vector_normalize(a){
	var i,sum=0;
	for (i=0;i<a.length;i++){
		sum = sum+Math.pow(a[i],2);
	}
	var j;
	var a1 = [];
	for (j=0;j<a.length;j++){
		a1[j] = 0;
		a1[j] = a[j]/Math.sqrt(sum);
	}
	return a1;
}
function generate_identity(){
	var mat=[];
	mat=[
	[1,0,0,0],
	[0,1,0,0],
	[0,0,1,0],
	[0,0,0,1]
	];
	return mat;
}
function generate_translation_matrix(a){
	var mat=[];
	var mat0=[1,0,0,0];
	var mat1=[0,1,0,0];
	var mat2=[0,0,1,0];
	var mat3=[0,0,0,1];
	mat = [mat0,mat1,mat2,mat3];
	mat[0][3] = a[0];
	mat[1][3] = a[1];
	mat[2][3] = a[2];
	return mat;
}
function generate_rotation_matrix_Y(theta){
	var cos,sin;
	cos = Math.cos(theta);
	sin = Math.sin(theta);
	var mat=[];
	mat = [
	[cos,0,sin,0],
	[0,1,0,0],
	[-sin,0,cos,0],
	[0,0,0,1]
	];
	return mat;
}
function generate_rotation_matrix_Z(theta){
	var cos,sin;
	cos = Math.cos(theta);
	sin = Math.sin(theta);
	var mat=[];
	mat = [
	[cos,-sin,0,0],
	[sin,cos,0,0],
	[0,0,1,0],
	[0,0,0,1]
	];
	return mat;
}
function generate_rotation_matrix_X(theta){
	var cos,sin;
	cos = Math.cos(theta);
	sin = Math.sin(theta);
	var mat=[];
	mat = [
	[1,0,0,0],
	[0,cos,-sin,0],
	[0,sin,cos,0],
	[0,0,0,1]
	];
	return mat;
}
    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
    //   matrix_transpose
    //   matrix_pseudoinverse
    //   matrix_invert_affine
    //   vector_normalize
    //   vector_cross
    //   generate_identity
    //   generate_translation_matrix
    //   generate_rotation_matrix_X
    //   generate_rotation_matrix_Y
    //   generate_rotation_matrix_Z

