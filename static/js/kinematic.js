import ultils from "./ultils.js";

export default class Mankinematic{
    constructor(initial_raw_current){
        // Manipulator robot kinematic 
        // author: lqviet
        // version: 6.23621
        // ### FKman()
        // calculate forward kinematic

        // ### invKman()
        // calculate inverse kinematic

        // a1 is distance between coordinate 1 and 2 follow z direction at zero position
        // a2 is distance between coordinate 2 and 3 follow z direction at zero position
        // L1 is distance between coordinate 2 and 3 follow x direction at zero position
        // L2 is distance between coordinate 3 and 4 follow x direction at zero position
        // L3 is distance between coordinate 4 and 5 follow x direction at zero position

        this.a1 = 0.0595;
        this.a2 = 0.128;
        this.L1 = 0.024;
        this.L2 = 0.124;
        this.L3 = 0.126;
        this.joins = [
            {
                "name": "join1",
                "w1": math.matrix([0, 0, -1]),
                "q1": math.matrix([0, 0,  0]),
            },{
                "name": "join2",
                "w1": math.matrix([0, -1, 0]),
                "q1": math.matrix([0, 0, this.a1]),
            },{
                "name": "join3",
                "w1": math.matrix([0, 1, 0]),
                "q1": math.matrix([this.L1, 0, this.a1 + this.a2]),
            },{
                "name": "join4",
                "w1": math.matrix([0, 1, 0]),
                "q1": math.matrix([this.L1+this.L2, 0, this.a1 + this.a2]),
        }
        ];
        this.calibrate = [math.pi, math.pi, math.pi, math.pi];
        this.Slist;
        this.M = math.matrix([[1,0,0,this.L1+this.L2+this.L3],[0,1,0,0],[0,0,1,this.a1+this.a2],[0,0,0,1]]);
        this.ktools = new ultils();
        this.calSlist();
        this.raw_current = initial_raw_current;
        this.cur_T;
        this.update_cur_T();
        this.endEffector = 1600;
    };

    calSlist(){
        let temSlist = [];
        this.joins.forEach(join=>{
            temSlist.push(math.concat(join.w1, math.multiply(math.cross(join.w1,join.q1),-1), 0));
        });
        this.Slist = math.matrix(temSlist);
    };

    manForwardKi(thetalist){
        // # Calculate forward kinematic of manipulator
        // input: thetalist 
        // [0.06073745796940244, 1.5624187463853234, 1.5194138336161829, 1.5490146177300073]
        // output: homogeneous transformation matrices:    
        // T = [[ 0.06462193  0.06070012  0.996062    0.13286962] 
        //     [-0.00392981  0.99815605 -0.06057278 -0.0080801 ] 
        //     [-0.99790208  0.          0.06474131  0.06709081] 
        //     [ 0.          0.          0.          1.        ]]

        return this.ktools.ForwardkinematicinSpace(this.M, this.Slist, thetalist);
    };

    manInvKi1(phi0, Tsd){
        // # Calculate inverse kinematic of manipulator
        // input: 
        // - initial thetalist0
        // - Target homogeneous transformation matrices
        // output:
        // - array of philist

        let isSolve = false;
        let interate = 20;
        let errVb = 0.001;

        let AdjM = this.ktools.Adjoin(this.M);
        let Blist = math.multiply(math.inv(AdjM), math.transpose(this.Slist));
        let it = 0;
        
        let phi_rad = [phi0];
        let Tsb, Vb, Jinv, temphi, editphi;

        while(it <= interate){
            
            Tsb = this.manForwardKi(phi_rad[it]);
            Vb = this.ktools.matrixLog6(math.multiply(math.inv(Tsb), Tsd));
            
            if(math.norm(Vb) >= errVb){
                Jinv = math.pinv(this.ktools.JacobianBody(math.transpose(Blist), phi_rad[it]));
                temphi = math.add(phi_rad[it],math.multiply(Jinv, Vb));
                editphi = math.map(temphi, (value)=> math.asin(math.sin(value)))
                phi_rad.push(editphi);
            }else{
                isSolve = true;
                break;
            };
            // console.log(it);
            it = it +1;
        };

        let Tfinal = this.manForwardKi(math.matrix(phi_rad[phi_rad.length-1]));
        let Vbfinal = this.ktools.matrixLog6(math.multiply(math.inv(Tfinal), Tsd));
        let Vberror = math.norm(Vbfinal);
        let isSolveFinal;
        if(Vberror < errVb){
            isSolveFinal = true;
        }else{
            isSolveFinal = false;
        };

        return {"isSolve": isSolveFinal, "phifinal":phi_rad[phi_rad.length-1]};
    };

    manInvKi2(phi0, Tsd, N){
        // # Calculate inverse kinematic of manipulator
        // input: 
        // - initial thetalist0
        // - Target homogeneous transformation matrices
        // output:
        // - array of philist
        let isSolve = false;
        let errVb = 0.0001;
        let Tstart = this.manForwardKi(math.matrix(phi0));
        let traj = this.ktools.screwTrajectory(Tstart, Tsd, N);
        let phi_rad = [phi0];
        let temphi, tempsolve;
        for(let i =0; i<traj.length; i++){
            temphi = phi_rad[phi_rad.length-1];
            tempsolve = this.manInvKi1(temphi, traj[i]);
            if(tempsolve.isSolve === true){
                temphi.push(tempsolve.phifinal);
            }else{
                console.log("Error in solving the inv!")
                break;
            };
            phi_rad.push(temphi[temphi.length-1]);
        }
                
        let Tfinal = this.manForwardKi(math.matrix(phi_rad[phi_rad.length-1]));
        let Vbfinal = this.ktools.matrixLog6(math.multiply(math.inv(Tfinal), Tsd));
        let Vberror = math.norm(Vbfinal);

        if(Vberror < errVb){
            isSolve = true;
        }else{
            isSolve = false;
        };
        return {"isSolve": isSolve, "phifinal":phi_rad[phi_rad.length-1]};
    };

    cvRawtoRad(raw_angle){
        // # Convert raw value of dxl motor to radiant
        // rad = raw*pi/180*0.088 - motor calibrate
        // input : [2085, 2040, 2012, 3054, 3477]
        // output: 
        // [0.06073745796940244, 1.5624187463853234, 1.5194138336161829, 1.5490146177300073]
        
        let rad_angle = [];
        raw_angle.forEach((raw, index)=>{
            rad_angle.push(raw*0.088*math.pi/180-this.calibrate[index]);
        });
        return rad_angle;
    };

    cvRadtoRaw(rad_angle){
        // # Convert raw value of dxl motor to radiant
        // raw = (rad + motor calibrate)*180/(pi*0.088)
        // raw also satisfy the condition min and max of join
        // input : [0.06073745796940244, 1.5624187463853234, 1.5194138336161829, 1.5490146177300073]
        // output: 
        // [2085, 2040, 2012, 3054]
        let join_min = [1023, 1023, 1023, 1023];
        let join_max = [3068, 3068, 3068, 3068];
        let raw_angle = [];
        rad_angle.forEach((rad, index)=>{
            let tempraw = math.floor((rad+this.calibrate[index])*(180/(math.pi*0.088)));

            tempraw = tempraw <= join_min[index] ? join_min[i]:tempraw;
            tempraw = tempraw >= join_max[index] ? join_max[i]:tempraw;

            raw_angle.push(tempraw);
        });
        return raw_angle;
    };
    update_cur_T(){
        let cur_rad_angle = math.matrix(this.cvRawtoRad(this.raw_current));
        this.cur_T = this.manForwardKi(cur_rad_angle);
    };
    // Only calculate on client 
    update_cur_angle(n_math_angle){
        // console.log(math.flatten(n_math_angle))
        this.raw_current = math.flatten(n_math_angle);
        this.update_cur_T();
    };
    
    update_endEffector(val){
        this.endEffector = val;
    };
};