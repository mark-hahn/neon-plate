const jscad             = require('@jscad/modeling');
const { sphere }        = jscad.primitives;
const { vectorChar }    = jscad.text;
const { hull }          = jscad.hulls;
const { union }         = jscad.booleans;
const { translate }     = jscad.maths.mat4;

// check if two line segments intersect
// https://algs4.cs.princeton.edu/91primitives
const intersects = (segAB, segCD) => {
  const [[Ax,Ay], [Bx,By]] = segAB;
  const [[Cx,Cy], [Dx,Dy]] = segCD;
  const denominator = (Bx-Ax)*(Dy-Cy) - (By-Ay)*(Dx-Cx);
  if(denominator == 0) return null; // parallel
  const r = ((Ay-Cy)*(Dx-Cx) - (Ax-Cx)*(Dy-Cy)) / denominator;
  const s = ((Ay-Cy)*(Bx-Ax) - (Ax-Cx)*(By-Ay)) / denominator;
  return (r >= 0 && r <= 1 && s >= 0 && s <= 1);
}

// vector math
const dot = (v,w) => {
  const [x,y,z] = v;
  const [X,Y,Z] = w;
  return x*X + y*Y + z*Z;
}
const length = (v) => {
  const[x,y,z] = v;
  return Math.sqrt(x*x + y*y + z*z);
}
const vector = (b,e) => {
  const[x,y,z] = b;
  const[X,Y,Z] = e;
  return [X-x, Y-y, Z-z];
}
const unit = (v) => {
  const [x,y,z] = v;
  const mag = length(v);
  return [x/mag, y/mag, z/mag];
}
const distance = (p0,p1) => {
  return length(vector(p0,p1));
}
const scale = (v,sc) => {
  const [x,y,z] = v;
  return [x * sc, y * sc, z * sc];
}
const add = (v,w) => {
  const [x,y,z] = v;
  const [X,Y,Z] = w;
  return [x+X, y+Y, z+Z];
}

const distPntToLine = (pnt, start, end) => {
  const line_vec       = vector(start, end);
  const pnt_vec        = vector(start, pnt);
  const line_len       = length(line_vec);
  const line_unitvec   = unit(line_vec);
  const pnt_vec_scaled = scale(pnt_vec, 1.0/line_len);
  const dt = dot(line_unitvec, pnt_vec_scaled);
  const t  = Math.max(Math.min(dt,1),0);
  const nearest = add(scale(line_vec, t), start);
  const dist    = distance(nearest, pnt_vec);
  return [dist, nearest];
}

const strParam = "a";

const prevVecs = [];
const hulls    = [];
let   radius   = 1;
let   lastPoint = null;

const showVec = (pfx, vec) => {
  console.log( pfx +
    vec[0][0].toString().padStart(2) + ','    + 
    vec[0][1].toString().padStart(2) + ' -> ' +
    vec[1][0].toString().padStart(2)     + ','    + 
    vec[1][1].toString().padStart(2));
}

const handlePoint = (point) => {
  if(lastPoint) {
    const vec = [lastPoint, point];
    showVec('', vec);
    for(const prevVec of prevVecs) {
      if(intersects(vec, prevVec))
        showVec('  --X: ', prevVec);
      else {
        const dstPntToLin = 
                distPntToLine(point, prevVec[0], prevVec[0]);
        if(dstPntToLin[0] < 2*radius) 
          showVec('  --D:', dstPntToLin);
      }
    }
    prevVecs.push(vec);
    
    // radius += 1; //*= 1.5;
    hulls.push( hull(sphere({radius, center: lastPoint.concat(0)}), 
                     sphere({radius, center: point.concat(0)})) );
  }
  lastPoint = point;
}

const main = () => {
  console.log("---- main ----");
  const spacing = 3;
  let xOffset = 0;
  for(const char of strParam) {
    console.log("---- char:",char);
    const vecChar = vectorChar({xOffset}, char);
    const segs  = vecChar.segments;
    xOffset    += vecChar.width + spacing;
    segs.forEach( seg => {
      lastPoint = null;
      seg.forEach( point => handlePoint(point) );
    });
  };
  console.log("---- end ----");
  return hulls;
};

module.exports = {main};

