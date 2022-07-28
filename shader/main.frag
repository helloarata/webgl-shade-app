precision mediump float;

varying vec4  vColor;
varying vec3  vNormal;
varying vec3  vLightDirection;
varying float vIsHalftone;

void main(){
    float d = dot(normalize(vNormal), normalize(vLightDirection));

    float diffuse = clamp(d, 0.0, 1.0);
    // float diffuse = d;

  if(vIsHalftone == 1.0){
    vec2 v  = gl_FragCoord.xy * 1.3;
    float f = (sin(v.x) * 0.5 + 0.5) + (sin(v.y) * 0.5 + 0.5);
    float s;

    if(diffuse > 0.6){
      s = 1.0;
    } else if(diffuse > 0.2){
      s = 0.6;
    } else {
      s = 0.4;
    }
    gl_FragColor = vec4(vColor.rgb * (diffuse + vec3(f)) * s, 1.0);
  } else {
    vec4 color = vec4(vColor.rbg * d, vColor.a);
    gl_FragColor = color;
  }
}