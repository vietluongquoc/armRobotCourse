// This is the some functions to calculate the robotis kinematic
export default class Ultils{

    constructor(){
        this.name = "kinematic sub functions";
        this.author = "lqviet";
        this.version = "1.230327"
    };

    vectoso3(vec){
        // This function return screw matrix from w vector
        // input:
        //  vec = [1, 2, 3]
        // Output = 
        // [ 0    -3     2
        //    3     0    -1
        //   -2     1     0]
        return math.matrix([[0, -1*vec.get([2]), vec.get([1])],[vec.get([2]),0,-1*vec.get([0])],[-1*vec.get([1]), vec.get([0]),0]])
    };

    vectose3(vec){
        // Get T matrices from w and v
        // Input: [1; 2; 3; 4; 5; 6];
        //Output:
        //    [   0    -3     2     4
        //         3     0    -1     5
        //        -2     1     0     6
        //         0     0     0     1]

        return math.matrix([[0,-1*vec.get([2]),vec.get([1]),vec.get([3])],[vec.get([2]),0,-1*vec.get([0]),vec.get([4])],[-1*vec.get([1]),vec.get([0]),0,vec.get([5])],[0,0,0,1]])
    };

    transTorp(T){       
        //Get w matrices and p vector
        // Input = [[1, 0, 0, 0]; [0, 0, -1, 0]; [0, 1, 0, 3]; [0, 0, 0, 1]];
        // Output:  
        // R = [1     0     0
        //      0     0    -1
        //      0     1     0];   
        //  p =[0,0,3]

        let R = math.matrix([[T.get([0,0]),T.get([0,1]),T.get([0,2])],[T.get([1,0]),T.get([1,1]),T.get([1,2])],[T.get([2,0]),T.get([2,1]),T.get([2,2])]]);
        
        let p = math.matrix([T.get([0,3]),T.get([1,3]),T.get([2,3])])
        
        return {"R": R, "p":p}
    };

    matrixEXP6(se3mat){
        // Get homogeneous transformation matrices T using exponentials formula
        // input:
        // se3mat = [ 0,      0,       0,      0;
        //             0,      0, -1.5708, 2.3562;
        //             0, 1.5708,       0, 2.3562;
        //             0,      0,       0,      0]
        // # Output:
        //     T =[  1.0000     0         0         0
        //             0    0.0000   -1.0000   -0.0000
        //             0    1.0000    0.0000    3.0000
        //             0         0         0    1.0000 ]
        let T, omgmat, Rot, Vcolum;
        let wHAT = math.matrix([se3mat.get([2,1]),se3mat.get([0,2]),se3mat.get([1,0])]);
        let theta = math.norm(wHAT);

        if(theta < 0.0000001){
            T = math.matrix([[1,0,0,se3mat.get([0,3])],[0,1,0,se3mat.get([1,3])],[0,0,1,se3mat.get([2,3])],[0,0,0,1]]);
        }else{
            omgmat = math.multiply(math.subset(se3mat, math.index(math.range(0,3),math.range(0,3))), 1/theta);

            Rot = math.add(math.identity(3), math.multiply(math.sin(theta),omgmat), math.multiply(1-math.cos(theta), math.multiply(omgmat, omgmat)));

            Vcolum = math.multiply(math.add(math.multiply(math.identity(3), theta), math.multiply(1-math.cos(theta),omgmat), math.multiply(theta-math.sin(theta), math.multiply(omgmat, omgmat))) , math.multiply(math.subset(se3mat, math.index(math.range(0,3), 3)), 1/theta));

            T = math.matrix([[Rot.get([0,0]), Rot.get([0,1]),Rot.get([0,2]), Vcolum.get([0,0])],[Rot.get([1,0]), Rot.get([1,1]),Rot.get([1,2]), Vcolum.get([1,0])],[Rot.get([2,0]), Rot.get([2,1]),Rot.get([2,2]), Vcolum.get([2, 0])],[0,0,0,1]]);
        };

        return T
    };

    matrixLog6(Tm){
        // Get velocity vector from the homogeneous transformation matrices
        // input:
        //     T = np.array([[1, 0, 0, 0],[0, 0, -1, 0],[0, 1, 0, 3],[0, 0, 0, 1]])
        // output (inclue the theta):
        //     [S] = [1.57  0.  0.  0.  2.36  2.36]
        
        let so3mat, theta, omg;
        let Randp = this.transTorp(Tm);
        let acosinput = math.multiply(math.trace(Randp.R) - 1,0.5);
        if(acosinput >=1 ){
            so3mat = math.zeros(3,3);
            theta = 0.0001;
        }else if(acosinput <= -1){
            if(math.norm(1+Randp.R.get([2,2])) >= 0.0000001){
                omg = math.multiply(1/math.sqrt(2*(1+Randp.R.get([2,2]))), math.subset(Randp.R, math.index(math.range(0,2), 2)))
            }else if(math.norm(1+Randp.R.get([1,1])) >= 0.0000001){
                omg = math.multiply(1/math.sqrt(2*(1+Randp.R.get([1,1]))), math.subset(Randp.R, math.index(math.range(0,2), 1)))
            }else{
                omg = math.multiply(1/math.sqrt(2*(1+Randp.R.get([0,0]))), math.subset(Randp.R, math.index(math.range(0,2), 0)))
            };
            so3mat = ktools.vectoso3(math.multiply(math.pi,omg));
            theta = math.pi;
        }else{
            theta = math.acos(acosinput);
            so3mat = math.multiply(theta*(1/(2*math.sin(theta))),math.subtract(Randp.R,math.transpose(Randp.R)));
        };
        let Ginverse = math.add(math.identity(3), math.multiply(-0.5,so3mat), math.multiply((1/theta)*(1/theta-1/(2*math.tan(theta/2))),math.multiply(so3mat, so3mat)));
        let vout = math.multiply(Ginverse, Randp.p);
        let result = math.matrix([so3mat.get([2,1]),so3mat.get([0,2]),so3mat.get([1,0]),vout.get([0]), vout.get([1]),vout.get([2])]);

        return result;
    };

    Adjoin(Tm){
        // calculate Adjoin of T
        // Input 
        // T = [[1, 0, 0, 0]; [0, 0, -1, 0]; [0, 1, 0, 3]; [0, 0, 0, 1]];
        // Output
        // [   
        //     1     0     0     0     0     0
        //     0     0    -1     0     0     0
        //     0     1     0     0     0     0
        //     0     0     3     1     0     0
        //     3     0     0     0     0    -1
        //     0     0     0     0     1     0 
        // ]

        let Randp = this.transTorp(Tm);
        let upAdT = math.concat(Randp.R, math.zeros(3, 3), 1);
        let downAdT = math.concat(math.multiply(this.vectoso3(Randp.p),Randp.R), Randp.R, 1);
        return math.concat(upAdT, downAdT, 0)
    };

    ForwardkinematicinSpace(M, Slist, thetalist){
        // Forward kinematic in space coordinate 
        // Input:
        // M = math.matrix([[-1, 0, 0, 0],[0, 1, 0, 6],[0, 0, -1, 2],[0, 0, 0, 1]])
        // Slist = math.matrix([[0,0,1,4,0,0],[0,0,0,0,1,0],[0,0,-1,-6,0,-0.1]])
        // thetalist = math.matrix([math.pi/2, 3,math.pi]);
        // Output:
        // [   -0.0000    1.0000         0   -5.0000
        //     1.0000    0.0000         0    4.0000
        //     0         0             -1.0000    1.6858
        //     0         0             0    1.0000 ] 
        let T = M;
        let nrows = math.squeeze(math.size(thetalist));
        let Stheta;
        let tempSt;
        for(let i = nrows-1; i>-1; --i){
            tempSt = math.multiply(math.squeeze(math.row(Slist, i)),thetalist.get([i]));
            Stheta = this.vectose3(math.squeeze(tempSt));
            T = math.multiply(this.matrixEXP6(Stheta), T);
        };
        return T
    };

    JacobianBody(Blist, theta){
        // Calculate Jacobian matrices in body coordinate
        // input
        // Blist = [   [0; 0; 1;   0; 0.2; 0.2], ...
        //             [1; 0; 0;   2;   0;   3], ...
        //             [0; 1; 0;   0;   2;   1], ...
        //             [1; 0; 0; 0.2; 0.3; 0.4]];
        // thetalist = [0.2; 1.1; 0.1; 1.2];
        // output
        //  Jb =[
        //        -0.0453    0.9950         0    1.0000
        //         0.7436    0.0930    0.3624         0
        //        -0.6671    0.0362   -0.9320         0
        //         2.3259    1.6681    0.5641    0.2000
        //        -1.4432    2.9456    1.4331    0.3000
        //        -2.0664    1.8288   -1.5887    0.4000 
        //     ] 

        let nrows = math.squeeze(math.size(theta));
        let T = math.identity(4);
        let JB = [];
        let Stheta;
        for(let i = nrows-2; i>-1; --i){
            Stheta = this.vectose3(math.multiply(math.squeeze(math.row(Blist, i+1)), -1*theta.get([i+1])));
            T = math.multiply(T, this.matrixEXP6(Stheta));
            JB.unshift(math.multiply(this.Adjoin(T), math.transpose(math.row(Blist, i))));
        };
        JB.push(math.transpose(math.row(Blist, nrows-1)));
        let result = JB[nrows-1];
        for(let i=JB.length-2; i>-1; i--){
            result = math.concat(JB[i], result);
        };
        return result
    };

    screwTrajectory(Tstart, Tend, N){
        // # point to point trajectory
        // generate point to point trajectory from 
        // start homogeneous transformation matrices (Tstart) to
        // end homogeneous transformation matrices (Tend)

        // input:
        //     Tstart, Tend
            
        //     Tstart = [[1 ,0, 0, 1]; [0, 1, 0, 0]; [0, 0, 1, 1]; [0, 0, 0, 1]];
        //     Tend = [[0, 0, 1, 0.1]; [1, 0, 0, 0]; [0, 1, 0, 4.1]; [0, 0, 0, 1]];
        //     N = 4
        // output:
        //     List of T

        const timegap = 0.02;
        const TF = timegap*(N-1);
        let traj = [];

        let Vb = this.matrixLog6(math.multiply(math.inv(Tstart), Tend));

        for(let i=0; i < N; i++){
            let s = 3*math.pow((timegap*(i)/TF),2) - 2*math.pow((timegap*(i)/TF),3);
            let Matrixv = this.matrixEXP6(this.vectose3(math.multiply(s, Vb)));
            traj.push(math.multiply(Tstart, Matrixv));
        };
        return traj
    };

};