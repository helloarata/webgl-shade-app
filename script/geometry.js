export class WebGLGeometry{
  static plane(width, height, color){
    const w = width / 2;
    const h = height / 2;
    const pos = [
      -w, h, 0.0,
      w, h, 0.0,
      -w, -h, 0.0,
      w, -h, 0.0,
    ];
    const nor = [
      0.0, 0.0, 1.0,
      0.0, 0.0, 1.0,
      0.0, 0.0, 1.0,
      0.0, 0.0, 1.0,
    ];
    const col = [
      color[0], color[1], color[2], color[3],
      color[0], color[1], color[2], color[3],
      color[0], color[1], color[2], color[3],
      color[0], color[1], color[2], color[3],
    ];
    const st = [
      0.0, 0.0,
      1.0, 0.0,
      0.0, 1.0,
      1.0, 1.0,
    ];
    const idx = [
      0, 2, 1,
      1, 2, 3,
    ];
    return {position: pos, normarl: nor, color: col, texCoord: st, index: idx};
  }
  static torus(row, column, irad, orad, color){
    const pos = [];
    const nor = [];
    const col = [];
    const st = [];
    const idx = [];
    for(let i = 0; i <= row; i++){
      const r = Math.PI * 2 / row * i;
      const rr = Math.cos(r);
      const ry = Math.sin(r);
      for(let j = 0; j <= column; j++){
        const tr = Math.PI * 2 / column * j;
        const tx = (rr * irad + orad) * Math.cos(tr);
        const ty = ry * irad;
        const tz = (rr * irad + orad) * Math.sin(tr);
        const rx = rr * Math.cos(tr)
        const rz = rr * Math.sin(tr);
        const rs = 1 / column * j;
        let rt = 1 / row * i + 0.5;
        if(rt > 1.0){rt -= 1.0;}
        rt = 1.0 - rt;
        pos.push(tx, ty, tz);
        nor.push(rx, ry, rz);
        col.push(color[0], color[1], color[2], color[3]);
        st.push(rs, rt);
      }
    }
    for(let i = 0; i < row; i++){
      for(let j = 0; j < column; j++){
        const r = (column + 1) * i + j;
        idx.push(r, r + column + 1, r + 1);
        idx.push(r + column + 1, r + column + 2, r + 1);
      }
    }
    return {position: pos, normal: nor, color: col, texCoord: st, index: idx}
  }
}