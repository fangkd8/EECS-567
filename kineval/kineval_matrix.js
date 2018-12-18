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
    var i,j,k;
    if (m1[0].length > 1){
        for (i=0;i<m1[0].length;i++){
            mat[i] = [];
            for (j=0;j<m1.length;j++){
                mat[i][j]=m1[j][i];
            }
        }
    }
    else if(typeof(m1[0].length) == 'undefined'){
        for (var i0=0;i0<m1.length;i0++){
            mat[i0] = [];
        }
        for (j=0;j<m1.length;j++){
            mat[j][0]=m1[j];
        }
    }
    else if(m1[0].length == 1){
        for (k=0;k<m1.length;k++){
            mat[k] = m1[k][0];
        }
    }
    
    return mat; 
}

function matrix_inverse(a){
    //this way is in LU decomposition of solving the inverse of matrix.
    var low = [];
    var up = matrix_copy(a);
    n = a.length;
    for (var i=0;i<n;i++){
        low[i] = [];
        for (l=0;l<n;l++){
            low[i][l]=0;
        }
    }
    for (var i=0;i<n;i++){
        low[i][i] = 1;
    }

    for (j=0;j<n-1;j++){
        for (i=j+1;i<n;i++){
            mult = up[i][j]/up[j][j];
            low[i][j] = mult;
            for(k=j;k<n;k++){
                up[i][k] = up[i][k]-mult*up[j][k];
            }
        }
    }
    //console.log(low, up);
    //invup is the inverse of U, invlo is the inverse of L.
    var invup = [];
    var invlo = [];
    for (i=0;i<n;i++){
        invup[i]=[];
        invlo[i]=[];
        for(j=0;j<n;j++){
            invup[i][j]=0;
            invlo[i][j]=0;
        }
    }
    
    for (i=0;i<n;i++){
        invup[i][i] = 1/up[i][i];
        for(k=i-1;k>=0;k--){
            s=0;
            for(j=k+1;j<=i;j++){
                s=s+up[k][j]*invup[j][i];
            }
            invup[k][i]=-s/up[k][k];
        }
    }
    for (i=0;i<n;i++){
        invlo[i][i]=1/low[i][i];
        for(k=i+1;k<n;k++){
            for (j=i;j<=k-1;j++){
                invlo[k][i]=invlo[k][i]-low[k][j]*invlo[j][i];
            }
        }
    }
    //console.log(invup, invlo);
    var m1=[];
    m1 = matrix_multiply(invup,invlo);
    return m1;
}

function linear_solve(A,b){
    var mat =[];
    mat = matrix_inverse(A);
    if (typeof(b[0].length)=='undefined'){
        var b0 = [];
        b0 = matrix_transpose(b);
        var sol =[];
        sol = matrix_multiply(mat,b0);
    }
    else if (typeof(b[0].length!=='undefined')){
        sol = matrix_multiply(mat,b);
    }
    return sol;
}

function matrix_invert_affine(m1){
    var mat = [];
    var i,j;
    return mat;
}

function matrix_pseudoinverse(A){
    var mat=[],trans=[];
    var N,M;
    N = A.length;
    M = A[0].length;
    trans = matrix_transpose(A);
    if (N == M){
        mat = numeric.inv(A);
        //mat = matrix_inverse(A);
    }
    else if (M < N){
        mat = matrix_multiply(trans, A);
        mat = matrix_multiply(numeric.inv(mat), trans);
        //mat = matrix_multiply(matrix_inverse(mat), trans);
    }
    else{
        mat = matrix_multiply(A, trans);
        mat = matrix_multiply(trans, numeric.inv(mat));
        //mat = matrix_multiply(trans, matrix_inverse(mat))
    }
    return mat;
}

function vector_cross(a,b){
    if (typeof(a[1] == 'undefined')){
        var c=[];
        c[0]=a[1]*b[2]-a[2]*b[1];
        c[1]=a[2]*b[0]-a[0]*b[2];
        c[2]=a[0]*b[1]-a[1]*b[0];
    }
    else {
        var c = [];
        for (var i=0;i<a[0].length;i++){
           c[i]=[];
        }
        c[0][0] = a[1][0]*b[2][0]-a[2][0]*b[1][0];
        c[1][0] = a[2][0]*b[0][0]-a[0][0]*b[2][0];
        c[2][0] = a[0][0]*b[1][0]-a[1][0]*b[0][0];
    }
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
    var mat = [];
    mat = [[1,0,0,0],
           [0,1,0,0],
           [0,0,1,0],
           [0,0,0,1]];
    return mat;
}

function generate_translation_matrix(a,b,c){
    var mat=[];
    var mat0=[1,0,0,0];
    var mat1=[0,1,0,0];
    var mat2=[0,0,1,0];
    var mat3=[0,0,0,1];
    mat = [mat0,mat1,mat2,mat3];
    mat[0][3] = a;
    mat[1][3] = b;
    mat[2][3] = c;
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
