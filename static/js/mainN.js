import Arm3D from "./arm3D.js";
import * as THREE from '../../vendor/threejs/three.module.js';
import {STLLoader} from '../../vendor/threejs/STLLoader.js';
import Mankinematic from './kinematic.js';

const commendKeys = ["J1","J2","J3","J4","E"];

// buttons
const position_btns = ["#zplus-btn", "#zneg-btn","#xplus-btn", "#xneg-btn"];
const set_position = ["ZP","ZN","XP","XN"];
const UPjoin_btns = ["#btn-up-join1","#btn-up-join2","#btn-up-join3","#btn-up-join4"];
const DOWNjoin_btns = ["#btn-down-join1","#btn-down-join2","#btn-down-join3","#btn-down-join4"];
// current angle
const join_currents = ["#cur-join1","#cur-join2","#cur-join3","#cur-join4"];
// initial data
var init_data = [2045,2565,2507,2098];
// var init_data = [2045,2045,2045,2045];
// Mathjs and kinematic
var manKi = new Mankinematic(init_data);
// Threejs
const robot_parts = [
    {
        name: 'base',
        coordinate: new THREE.Object3D(),
        path: './static/js/model/DUMMY_DC12.stl',// Must be fixed
        angle: {x:-Math.PI/2, y:0, z:0},
        position: {x: 0,y:0,z:0,},
        colors: { color: '#4CAF50', specular: 0x111111, shininess: 10 },
        axis_rotate: '0',
        calibrate: 0,
    },
    {
        name: 'link1',
        coordinate: new THREE.Object3D(),
        path: './static/js/model/Link_01.stl',
        angle: {x:Math.PI/2, y: Math.PI/2, z:0},   
        // rotation angle y: -MATH.PI/2 = initial point
        position: {x: 0,y:0,z:3,},
        colors: { color: 'blue', specular: 0x111111, shininess: 10 },
        axis_rotate: 'Y',
        calibrate: Math.PI/2,
    },
    {
        name: 'link2',
        coordinate: new THREE.Object3D(),
        path: './static/js/model/Link_02.stl',
        angle: {x:Math.PI/2, y:Math.PI/2, z:-Math.PI/2},  
        // rotation angle y: MATH.PI/2 = initial point
        position: {x: 0,y:0,z:0,},
        colors: { color: 'red', specular: 0x111111, shininess: 10 },
        axis_rotate: 'Y',
        calibrate: Math.PI/2,
    },
    {
        name: 'link3',
        coordinate: new THREE.Object3D(),
        path: './static/js/model/Link_03.stl',
        angle: {x:-Math.PI/2, y:0, z:0},  
        // rotation angle x MATH.PI/2 = initial point
        position: {x:0,y:6.4,z: 1.2,},
        colors: { color: 'yellow', specular: 0x111111, shininess: 10},
        axis_rotate: 'X',
        calibrate: Math.PI/2,
    },
    {
        name: 'link4',
        coordinate: new THREE.Object3D(),
        path: './static/js/model/Link_04.stl',
        angle: {x:0, y:Math.PI, z:0},            
        // rotation angle x 0 = initial point
        position: {x: 0,y:6.2,z:0,},
        colors: { color: 'green', specular: 0x111111, shininess: 10 },
        axis_rotate: 'X',
        calibrate: 0,
    },
];
const gripper = {
    name: 'girpper',
    coordinate: new THREE.Object3D(),
    path: './static/js/model/Gripper_01.stl',// Must be fixed
    angle: {x:-Math.PI/2, y:0, z:0},
    position: {x: -1.625,y:3.3,z:0,},
    colors: { color: 'black', specular: 0x111111, shininess: 10 },
    axis_rotate: '0',
    calibrate: 0,
};
const scaleRatio = 0.05;  
const stlLoader = new STLLoader();
var loadRobots = (manipulator) =>{
    // Join
    robot_parts.forEach((part)=>{
        stlLoader.load(part.path, (root)=>{
            // root.center();
            let mater1 = new THREE.MeshPhongMaterial(part.colors)
            let mesh1 = new THREE.Mesh(root, mater1);
            mesh1.name = part.name;
            mesh1.scale.set(scaleRatio, scaleRatio, scaleRatio);
            let axesHelper = new THREE.AxesHelper(2);
            part.coordinate.add(mesh1);
            part.coordinate.add( axesHelper);
            part.coordinate.rotation.set(part.angle.x,part.angle.y, part.angle.z);
            part.coordinate.position.set(part.position.x, part.position.y, part.position.z);
        });
    });
    // Gripper
    stlLoader.load(gripper.path, (root)=>{
        // root.center();
        let mater1 = new THREE.MeshPhongMaterial(gripper.colors)
        let mesh1 = new THREE.Mesh(root, mater1);
        mesh1.name = gripper.name;
        mesh1.scale.set(scaleRatio, scaleRatio, scaleRatio);
        let axesHelper = new THREE.AxesHelper(2);
        gripper.coordinate.add(mesh1);
        gripper.coordinate.add( axesHelper);
        gripper.coordinate.rotation.set(gripper.angle.x,gripper.angle.y, gripper.angle.z);
        gripper.coordinate.position.set(gripper.position.x, gripper.position.y, gripper.position.z);
    });
    // add coordinate
    for(let i = 1; i < robot_parts.length; ++i){
        robot_parts[i-1].coordinate.add(robot_parts[i].coordinate);
    };
    robot_parts[4].coordinate.add(gripper.coordinate);
    let sceneAxis = new THREE.AxesHelper(11);
    manipulator.scene.add(sceneAxis);
    manipulator.scene.add(robot_parts[0].coordinate);
};
var update_data = (data)=>{
    // console.log(data);
    data.forEach((raw, index)=>{
        let g_rad = raw*0.088*Math.PI/180;
        let axisRot =robot_parts[index+1].axis_rotate;
        let goal_rad = (g_rad-Math.PI) + robot_parts[index+1].calibrate;
        // let goal_rad = targetGoal[i]
        if(axisRot == "X"){
            robot_parts[index+1].coordinate.rotation.x = goal_rad;
        }else if(axisRot == "Y"){
            robot_parts[index+1].coordinate.rotation.y = goal_rad;
        }else if(axisRot == "Z"){
            robot_parts[index+1].coordinate.rotation.z = goal_rad;
        };
    });  
    manKi.raw_current.forEach((raw, index)=>{
        $(join_currents[index]).text(Math.floor(raw*0.088)-90);
    });
    $("#Tmatrices-container").empty();
    manKi.cur_T.forEach((value)=>{
        // console.log(value);
        let temptdiv = $("<div></div>").text(`${Math.floor(1000*value)/1000}`)
        $("#Tmatrices-container").append(temptdiv);
    });
};

const solveINV = (rad_phi0, targetT)=>{
    let solve_goal1 = manKi.manInvKi1(math.matrix(rad_phi0), targetT);
    let solve_goal2 = manKi.manInvKi2(math.matrix(rad_phi0), targetT);
    return (solve_goal1.isSolve)? solve_goal1: solve_goal2;
};


$(document).ready(()=>{
    var canvas = document.getElementById('arm-view');
    var manipulator = new Arm3D(canvas);

    $('#btn-open-tab').click(()=>{
        $('#div-reference-tab').show();
    });

    $('#btn-close-tab').click(()=>{
        $('#div-reference-tab').css("display","none");
    });

    $('#home-btn').click(()=>{
        // reset the setting.
        $('#dis-step-input').val(1);
        $('#rot-step-input').val(10);

        manKi.update_endEffector(1600);
        manKi.update_cur_angle(init_data);
        // console.log(manKi.raw_current);
    });
    $('#setting-btn').click(()=>{
        $('#setting-hide-contains').css("display","block");
    });
    $('#okay-setting-btn').click(()=>{
        $('#setting-hide-contains').css("display","none");
    });
    // Motion controller
    for(let i =0; i<4; i++){
        $(position_btns[i]).click(()=>{
            let targetT = [];  
            let rad_cur = manKi.cvRawtoRad(manKi.raw_current);   
            let stepVal = $('#dis-step-input').val();      
            // rotate the coordinate to XOY => join1 always = 0;
            if(set_position[i]==="ZP" || set_position[i]==="ZN"){
    
                // console.log(stepVal);
                if(set_position[i]==="ZP"){
                    targetT = math.add(manKi.cur_T, math.matrix([[0,0,0,0],[0,0,0,0],[0,0,0,0.01*stepVal],[0,0,0,0]]));
                }else{
                    targetT = math.add(manKi.cur_T, math.matrix([[0,0,0,0],[0,0,0,0],[0,0,0,-0.01*stepVal],[0,0,0,0]]));
                };
                let resultINV = solveINV(rad_cur, targetT);
                if(resultINV.isSolve === true){
                    console.log("It's solved!");
                    manKi.update_cur_angle(manKi.cvRadtoRaw(resultINV.phifinal));          
                    sendData(manKi.raw_current);
                }else{
                    alert("The robot reaches the limitation!");
                    console.log("Error in solving the inv!");
                };

            }else if(set_position[i]==="XP" || set_position[i]==="XN"){
                let rotationAngle = [2045, manKi.raw_current[1],manKi.raw_current[2],manKi.raw_current[3]];
                let rotMatrixT= manKi.manForwardKi(math.matrix(manKi.cvRawtoRad(rotationAngle)));
                if(set_position[i]==="XP"){
                    targetT = math.add(rotMatrixT, math.matrix([[0,0,0,0.01*stepVal],[0,0,0,0],[0,0,0,0],[0,0,0,0]]));
                }else{
                    targetT = math.add(rotMatrixT, math.matrix([[0,0,0,-0.01*stepVal],[0,0,0,0],[0,0,0,0],[0,0,0,0]]));
                };
                let resultINV = solveINV(rotationAngle, targetT);
                if(resultINV.isSolve === true){
                    console.log("It's solved!");
                    let frozenCur = math.flatten(resultINV.phifinal);
                    // console.log(frozenCur.get([1]));
                    let tempfinal = [rad_cur[0], frozenCur.get([1]),frozenCur.get([2]),frozenCur.get([3])];
                    manKi.update_cur_angle(manKi.cvRadtoRaw(tempfinal)); 
                }else{
                    alert("The robot reaches the limitation!");
                    console.log("Error in solving the inv!");
                };            
            }
        });

        $(UPjoin_btns[i]).click(()=>{
            let maxvalue = 3068;
            let step_rot_val = ($('#rot-step-output').val())*12;
            if(manKi.raw_current[i] + step_rot_val <= maxvalue){
                // let temp = [0,0,0,0];
                let tempRaw = [...manKi.raw_current];
                // temp[i] = manKi.raw_current[i] + step_rot_val;
                tempRaw[i] = manKi.raw_current[i] + step_rot_val;      
                manKi.update_cur_angle(tempRaw);

            };
        });
        $(DOWNjoin_btns[i]).click(()=>{
            let minvalue = 1023;
            let step_rot_val = ($('#rot-step-output').val())*12;
            if(manKi.raw_current[i] - step_rot_val >= minvalue){
                // let temp = [0,0,0,0];
                let tempRaw = [...manKi.raw_current];
                // temp[i] = manKi.raw_current[i] - step_rot_val;
                tempRaw[i] = manKi.raw_current[i] - step_rot_val;    
                manKi.update_cur_angle(tempRaw);
            };
        });
    };

    // Gripper controller
    $('#gripper-btn').click(()=>{
        manKi.endEffector = (manKi.endEffector == 1600)?420:1600;
        let buttonText =  (manKi.endEffector == 420)? "ON": "OFF";
        $('#gripper-btn').html(buttonText);
    });

    // main
    manipulator.init();
    loadRobots(manipulator);
    
    function render() {
        setTimeout(()=>{
            update_data(manKi.raw_current);
            manipulator.drawing(canvas);
            requestAnimationFrame(render);
        }, 1000/100)
    };
    requestAnimationFrame(render);
});