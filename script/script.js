import { WebGLUtility } from "./webgl.js";
import {WebGLMath} from './math.js';
import {WebGLGeometry} from './geometry.js';
import { WebGLOrbitCamera } from "./camera.js";
import '../lib/tweakpane-3.1.0.min.js';
window.addEventListener('DOMContentLoaded', () => {
  const app = new App();
  app.init();
  app.load()
  .then(() => { 
    app.setupGeometry();
    app.setupLocation();
    app.start();
  });
const pane = new Tweakpane.Pane();
const parameter = {
  culling: true,
  depthTest: true,
  rotatino: false,
  halftone: false,
}
pane.addInput(parameter, 'culling')
.on('change', (v) => {
  app.setCulling(v.value);
});
pane.addInput(parameter, 'depthTest')
.on('change', (v) => {
  app.setDepthTest(v.value);
});
pane.addInput(parameter, 'rotatino')
.on('change', (v) => {
  app.setRotatino(v.value);
});
pane.addInput(parameter, 'halftone')
.on('change', (v) => {
  app.setHalftone(v.value);
});
}, false);

class App{
  constructor(){
    this.canvas = null;
    this.gl = null;
    this.camera = null;
    this.program = null;
    this.attributeLocation = null;
    this.attributeStride = null;
    this.uniformLocation = null; 
    this.torusGeometry = null;
    this.torusVBO = null;
    this.torusIBO = null;
    this.startTime = null;
    this.isRender = false;
    this.isRotation = false;
    this.isHalftone = false;
    this.resize = this.resize.bind(this); 
    this.render = this.render.bind(this);
  }

  setCulling(flag){
    const gl = this.gl;
    if(gl === null) return;
    flag === true ? gl.enable(gl.CULL_FACE) : gl.disable(gl.CULL_FACE);
  }
  setDepthTest(flag){
    const gl = this.gl;
    if(gl === null) return;
    flag === true ? gl.enable(gl.DEPTH_TEST) : gl.disable(gl.DEPTH_TEST);
  }
  setRotatino(flag){
    this.isRotation = flag;
  }
  setHalftone(flag){
    this.isHalftone = flag;
  }

  init(){
    this.canvas = document.getElementById('webgl-canvas');
    this.gl = WebGLUtility.createWebGLContext(this.canvas);
    const cameraOption = {
      distance: 5.0,
      min: 1.0,
      max: 10.0,
      move: 2.0,
    };
    this.camera = new WebGLOrbitCamera(this.canvas, cameraOption);
    this.resize();
    window.addEventListener('resize', this.resize, false);
    this.gl.enable(this.gl.CULL_FACE);
    this.gl.enable(this.gl.DEPTH_TEST);
  }

  resize(){
    this.canvas.width = window.innerWidth;
    this.canvas.height = window.innerHeight;
  }

  load(){
    return new Promise((resolve, reject) => {
      const gl = this.gl;
      if(gl === null){
        const error = new Error('not initialized');
        reject(error);
      } else {
        let vs = null;
        let fs = null;
        WebGLUtility.loadFile('./shader/main.vert')
        .then((vertextShaderSource) => {
          vs = WebGLUtility.createShaderObject(gl, vertextShaderSource, gl.VERTEX_SHADER);
          return WebGLUtility.loadFile('./shader/main.frag')
          .then((fragmentShaderSource) => {
            fs = WebGLUtility.createShaderObject(gl, fragmentShaderSource, gl.FRAGMENT_SHADER);
            this.program = WebGLUtility.createProgramObject(gl, vs, fs);
            resolve();
          })
        })
      }
    });
  }

  setupGeometry(){
    const gl = this.gl;
    const row = 32;
    const column = 32;
    const innerRadius = 0.4;
    const outerRadius = 0.8;
    const color = [1.0, 1.0, 1.0, 1.0];
    this.torusGeometry =  WebGLGeometry.torus(
      row,
      column,
      innerRadius,
      outerRadius,
      color,
    );

    this.torusVBO = [
      WebGLUtility.createVBO(gl, this.torusGeometry.position),
      WebGLUtility.createVBO(gl, this.torusGeometry.normal),
      WebGLUtility.createVBO(gl, this.torusGeometry.color),
    ];
    this.torusIBO = WebGLUtility.createIBO(gl, this.torusGeometry.index);
  }
  setupLocation(){
    const gl = this.gl;
    this.attributeLocation = [
      gl.getAttribLocation(this.program, 'position'),
      gl.getAttribLocation(this.program, 'normal'),
      gl.getAttribLocation(this.program, 'color'),
    ];
    this.attributeStride = [
      3,
      3,
      4,
    ];
    this.uniformLocation = {
      mvpMatrix: gl.getUniformLocation(this.program, 'mvpMatrix'),
      normalMatrix: gl.getUniformLocation(this.program, 'normalMatrix'),
      lightDirection: gl.getUniformLocation(this.program, 'lightDirection'),
      isHalftone: gl.getUniformLocation(this.program, 'isHalftone'),
    }
  }
  setupRendering(){
    const gl = this.gl;
    gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    gl.clearColor(0.3, 0.3, 0.3, 1.0);
    gl.clearDepth(1.0);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  }
  start(){
    this.startTime = Date.now();
    this.isRender = true;
    this.render();
  }

  stop(){

  }

  render(){
    const gl = this.gl;
    const m4 = WebGLMath.Mat4;
    const v3 = WebGLMath.Vec3;

    if(this.isRender === true){
      requestAnimationFrame(this.render);
    }
    const nowTime = (Date.now() - this.startTime) * 0.001;

    this.setupRendering();

    const rotateAxis = v3.create(0.0, 1.0, 0.0);

    const m = this.isRotation === true ? 

      m4.rotate(m4.identity(), nowTime, rotateAxis) :
       
      m4.identity();
  
    const v = this.camera.update();
    const fovy = 45;
    const aspect = window.innerWidth / window.innerHeight;
    const near = 0.1;
    const far = 10.0;
    const p = m4.perspective(fovy, aspect, near, far);
    const vp = m4.multiply(p, v);
    const mvp = m4.multiply(vp, m);
    const normalMatrix = m4.transpose(m4.inverse(m));
    const lightDirection = v3.create(1.0, 1.0, 1.0);
    let isHalftone = this.isHalftone ? 1.0 : 0.0;

    gl.useProgram(this.program);
    gl.uniformMatrix4fv(this.uniformLocation.mvpMatrix, false, mvp);
    gl.uniformMatrix4fv(this.uniformLocation.normalMatrix, false, normalMatrix);
    gl.uniform3fv(this.uniformLocation.lightDirection, lightDirection);
    gl.uniform1f(this.uniformLocation.isHalftone, isHalftone);

    WebGLUtility.enableBuffer(
      gl,
      this.torusVBO,
      this.attributeLocation,
      this.attributeStride,
      this.torusIBO
    );
    gl.drawElements(gl.TRIANGLES, this.torusGeometry.index.length, gl.UNSIGNED_SHORT, 0);
  }
}