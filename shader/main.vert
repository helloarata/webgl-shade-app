attribute vec3 position;
attribute vec3 normal;
attribute vec4 color;

uniform mat4 mvpMatrix; 
uniform mat4 normalMatrix;
uniform vec3 lightDirection;
uniform float isHalftone;

varying vec4  vColor;
varying vec3  vNormal;
varying vec3  vLightDirection;
varying float vIsHalftone;

void main(){
  vColor           = color;
  vec3 n           = (normalMatrix * vec4(normal, 0.0)).xyz;
  vNormal          = n;
  vLightDirection  = lightDirection;
  vIsHalftone      = isHalftone;

  gl_Position = mvpMatrix * vec4(position, 1.0);
}