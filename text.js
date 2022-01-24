const jscad             = require('@jscad/modeling');
const { sphere }        = jscad.primitives;
const { vectorChar }    = jscad.text;
const { hull }          = jscad.hulls;

const strParam   = "X";
const hullRadius = 1;
// distance in mm each step when backing up
const stepDistMm = 0.1;

// return intersection point if exists
// https://algs4.cs.princeton.edu/91primitives
const intersectionPoint = (segAB, segCD) => {
  const [[Ax,Ay], [Bx,By]] = segAB;
  const [[Cx,Cy], [Dx,Dy]] = segCD;
  const numeratorR  = ((Ay-Cy)*(Dx-Cx) - (Ax-Cx)*(Dy-Cy));
  const numeratorS  = ((Ay-Cy)*(Bx-Ax) - (Ax-Cx)*(By-Ay));
  const denominator = ((Bx-Ax)*(Dy-Cy) - (By-Ay)*(Dx-Cx));
  if(denominator == 0) return null; // parallel
  const r = numeratorR / denominator;
  const s = numeratorS / denominator;
  if(!(r >= 0 && r <= 1 && s >= 0 && s <= 1)) 
    return null;                           // doesn't intersect
  return [Ax + r*(Bx-Ax), Ay + r*(By-Ay)]; // intersection point
}

// vector math
// http://www.fundza.com/vectors/vectors.html
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

// http://www.fundza.com/vectors/point2line/index.html
const distPntToVec = (pnt, vec) => {
  const [start, end]   = vec;
  const line_vec       = vector(start, end);
  const pnt_vec        = vector(start, pnt);
  const line_len       = length(line_vec);
  const line_unitvec   = unit(line_vec);
  const pnt_vec_scaled = scale(pnt_vec, 1.0/line_len);
  const dt = dot(line_unitvec, pnt_vec_scaled);
  const t  = Math.max(Math.min(dt,1),0);
  const nearestPoint = add(scale(line_vec, t), start);
  return distance(nearestPoint, pnt_vec);
}

const showVec = (pfx, vec) => {
  console.log( pfx + ' ' +
    vec[0][0].toString().padStart(2) + ','    + 
    vec[0][1].toString().padStart(2) + ' -> ' +
    vec[1][0].toString().padStart(2) + ','    + 
    vec[1][1].toString().padStart(2));
}

const prevVecs  = [];
const hulls     = [];
let   lastPoint = null;

// return point on vec far enough away from prevVec
const backUpPoint = (prevVec, vec) => {
  const [[Ax, Ay],[Bx, By]] = vec;
  const vecW = Bx-Ax;
  const vecH = By-Ay;
  const vecLen = Math.sqrt((vecW*vecW)+(vecH*vecH));
  for(let distOnVec = 0; distOnVec < vecLen; 
            distOnVec += stepDistMm) {
    const frac       = distOnVec/vecLen;
    const trialPoint = [Ax+frac*vecW, Ay+frac*vecH];
    const dist2prev  = distPntToVec(trialPoint, prevVec);
    if(dist2prev > (2 * hullRadius)) return trialPoint;
  }
  // vec was shorter then stepDistMm
  return null;
}

const chkTooClose = (vec, first) => {
  // don't check last vec unless this is first
  let prevVecsTmp = (first ? prevVecs : prevVecs.slice(0,-1));
  for(const prevVec of prevVecsTmp) {
    showVec('vec', vec);
    showVec('prevVec of prevVecsTmp', prevVec);
    let intPt = (intersectionPoint(vec, prevVec));
    if(intPt) {
      // vec intersects an old vec
      // split vec in two with both vecs far enough away
      let vec1 = null;
      let vec2 = null;
      const backUpPt1 = backUpPoint(prevVec, [vec[0], intPt]);
      if(backUpPt1) vec1 = [vec[0], backUpPt1];
      const backUpPt2 = backUpPoint(prevVec, [vec[1], intPt]);
      if(backUpPt2) vec2 = [backUpPt2, vec[1]];
      return {vec1, vec2};
    }
    const dist2prev = distPntToVec(vec[1], prevVec);
    if(dist2prev < (2 * hullRadius)) {
      // latest point too close to an old vec
      // back up to point on vec far enough away
      let vec1 = null;
      const backUpPt = backUpPoint(prevVec, vec);
      if(backUpPt) vec1 = [vec[0], backUpPt];
      return {vec1, vec2: null};
    }
  }
  // vec not too close to any prev vec
  return null;
}

const addHull = (vec) => {
  showVec(' - hull', vec);
  hulls.push( 
    hull(sphere({hullRadius, center: vec[0].concat(0)}), 
         sphere({hullRadius, center: vec[1].concat(0)})) 
  );
}

const addHole = (tailPoint, headPoint) => {
  showVec(' - hole', [tailPoint, headPoint]);
}

// returns next seg idx
const handlePoint = (point, segIdx, segLast) => {
  if(segIdx == 0) {
    // first point
    lastPoint = point;
    return 1; // next segidx is 1
  }
  // not first point
  let vec = [lastPoint, point];
  const vecs = chkTooClose(vec, (segIdx == 1));
  if(vecs != null) { 
    const {vec1, vec2} = vecs;
    console.log({vec1, vec2});
    showVec(' .. close', vec1);
    // point was too close
    // truncated vec1 is now last in segment
    addHull(vec1);
    addHole(vec1[0], vec1[1]);
    prevVecs.push(vec1);
    lastPoint = null;
    if(vec2) {
      showVec('++ split', vec2);
      // had intersetion
      // vec was split into vec1 and vec2
      // handle vec2 as first in new segment
      lastPoint = vec2[0];
      handlePoint(vec2[1], 1, segLast);
      if(!segLast) return 2;  // next segidx is 2
    }
    // start new segment
    lastPoint = null;
    return 0; // next segidx is 0
  }
  // point not too close
  showVec(' ', vec);
  if(segIdx = 1) 
    addHole(point, lastPoint); // add first hole
  addHull(vec);
  if(segLast) addHole(lastPoint, point);
  prevVecs.push(vec);
  lastPoint = point;
  return segIdx + 1; // next segidx
}

const main = () => {
  console.log("---- main ----");
  const spacing = 3;
  let xOffset = 0;
  for(const char of strParam) {
    console.log("\n==== char:",char);
    const vecChar = vectorChar({xOffset}, char);
    const segs  = vecChar.segments;
    xOffset    += vecChar.width + spacing;
    segs.forEach( seg => {
      console.log("--- seg ---");
      let segIdx = 0;
      seg.forEach( point => {
        segIdx = handlePoint(point, segIdx, segIdx == seg.length-1);
      });
    });
  };
  console.log("\n---- end ----");
  return hulls;
};

module.exports = {main};

