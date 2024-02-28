import * as THREE from '../../vendor/threejs/three.module.js';
import {OrbitControls} from '../../vendor/threejs/OrbitControls.js';

export default class Arm3D{
    constructor(canvas){
        this.canvas = canvas
        this.renderer = new THREE.WebGLRenderer({canvas});
        this.scene = new THREE.Scene();
        //  fov = 45;aspect = 2;near = 0.1;far = 100;
        this.camera = new THREE.PerspectiveCamera(45, 2, 0.1, 100);
        // size of grid = 10; divisions = 20;
        this.gridhelper = new THREE.GridHelper(100, 50);
    }

    init(){

        this.camera.position.set(-20, 20, -20);
        this.scene.background = new THREE.Color('white');
        this.renderer.outputEncoding = THREE.sRGBEncoding;
        
        const controls = new OrbitControls(this.camera, this.canvas);
        controls.target.set(0, 5, 0);
        controls.update();

        // Light
        //skyColor = 0x000000;groundColor = 0xffffff; hintensity = 1;
        const hlight = new THREE.HemisphereLight(0x000000, 0xffffff, 1);
        this.scene.add(hlight);

        // color = 0xFFFFFF; dintensity = 0.8;
        const dlight = new THREE.DirectionalLight(0xFFFFFF, 0.8);
        dlight.position.set(0, 100, 0);
        dlight.target.position.set(0, 0, 0);
        
        this.scene.add(dlight);
        this.scene.add(dlight.target);
        // grid helper
        this.scene.add(this.gridhelper);
    };

    drawing(canvas){
        function resizeRendererToDisplaySize(renderer){
            const cv = renderer.domElement;
            const width = cv.clientWidth;
            const height = cv.clientHeight;
            const needResize = canvas.width !== width || canvas.height !== height;
            if(needResize){
                renderer.setSize(width, height, false);
            }
            return needResize;
        };
        if (resizeRendererToDisplaySize(this.renderer)) {
            const cv = this.renderer.domElement;
            this.camera.aspect = cv.clientWidth / cv.clientHeight;
            this.camera.updateProjectionMatrix();
        };
        this.renderer.render(this.scene, this.camera);
    };

}