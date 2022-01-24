const jscad             = require('@jscad/modeling');
const { sphere }        = jscad.primitives;
const { vectorChar }    = jscad.text;
const { hull }          = jscad.hulls;

const strParam   = "W";
const hullRadius = 1;
// distance in mm each step when backing up
const stepDistMm = 0.1;

const pntEq = (A,B) => A[0] == B[0] && A[1] == B[1];

// return intersection point if exists
// but only if point isn't one of CD end points
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
  const iPnt = [Ax + r*(Bx-Ax), Ay + r*(By-Ay)]; // intersection point
  if(pntEq(iPnt,segCD[0]) || pntEq(iPnt,segCD[1])) 
    return null;
  return iPnt;
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

// const showPnt3D = (pfx, pnt) => {
//   console.log( pfx + ' ' +
//     pnt[0].toFixed(2).padStart(2) + ','    + 
//     pnt[1].toFixed(2).padStart(2) + ','    +
//     pnt[2].toFixed(2).padStart(2))
// }

// http://www.fundza.com/vectors/point2line/index.html
const distPntToVec = (pnt, vec) => {
  pnt = [pnt[0], 0, pnt[1]];
  let [start, end] = vec;
  start = [start[0], 0, start[1]];
  end   = [end[0],   0, end[1]];
  const line_vec       = vector(start, end);
  const pnt_vec        = vector(start, pnt);
  const line_len       = length(line_vec);
  const line_unitvec   = unit(line_vec);
  const pnt_vec_scaled = scale(pnt_vec, 1.0/line_len);
  const dt      = dot(line_unitvec, pnt_vec_scaled);
  const t       = Math.max(Math.min(dt,1),0);
  let   nearest = scale(line_vec, t);
  // showPnt3D('nearest', nearest);
  // showPnt3D('pnt_vec', pnt_vec);
  const dist    = distance(nearest, pnt_vec);
  // console.log('dist', dist);
  // nearest = add(nearest, start);
  return dist;
}

const showVec = (pfx, vec) => {
  console.log( pfx + ' ' +
    vec[0][0].toFixed(2).padStart(2) + ','    + 
    vec[0][1].toFixed(2).padStart(2) + ' -> ' +
    vec[1][0].toFixed(2).padStart(2) + ','    + 
    vec[1][1].toFixed(2).padStart(2));
}

const prevVecs  = [];
const hulls     = [];
let   lastPoint = null;

// return point on vec far enough away from prevVec
const backUpPoint = (prevVec, vec, chkHead) => {
  // vec is A -> B and scan is A -> B
  const [[Ax, Ay],[Bx, By]] = vec;
  showVec('enter backUpPoint, prevVec', prevVec);
  showVec('enter backUpPoint, vec', vec);

  const vecW = Bx-Ax;
  const vecH = By-Ay;
  const vecLen = Math.sqrt((vecW*vecW)+(vecH*vecH));

  for(let distOnVec = 0; distOnVec < vecLen; 
            distOnVec += stepDistMm) {      
    const frac       = distOnVec/vecLen;
    const trialPoint = (chkHead ?
             [Bx-frac*vecW, By-frac*vecH] :
             [Ax+frac*vecW, Ay+frac*vecH]);
    // console.log('before distPntToVec');
    const dist2prev  = distPntToVec(trialPoint, prevVec);
    // console.log('distPntToVec result (frac, chkHead)', frac, chkHead);
    // console.log('debug vec, trialPoint, dist2prev:', 
    //                         trialPoint, dist2prev);
    // console.log('after debug');

    // console.log('after distPntToVec');
    // console.log({vecW, vecH, vecLen:vecLen.toFixed(2)});
    // console.log({frac:frac.toFixed(3), 
    //              dist2prev:dist2prev.toFixed(2)});
    // console.log('trialPoint', 
    //              trialPoint[0].toFixed(2), trialPoint[1].toFixed(2));
    if(dist2prev > (2 * hullRadius)) return trialPoint;
  }
  // vec was shorter then stepDistMm
  // console.log('vec was shorter then stepDistMm');
  return null;
}

const chkTooClose = (vec, first) => {
  let prevVecsTmp = (first ? prevVecs : prevVecs.slice(0,-1));
  // checking against all previous vecs (slow way)
  for(const prevVec of prevVecsTmp) {
    // ------ check vecs intersection  ------
    let intPt = (intersectionPoint(vec, prevVec));
    if(intPt) {
      // vec intersects an old vec
      // and neither end point of vec is int point
      // split vec in two with both vecs far enough away
      console.log('intersected, at', intPt[0], intPt[1]);
      showVec('      prevVec', prevVec);
      showVec('      vec', vec);

      let vec1 = null;
      const backUpPt1 = backUpPoint(prevVec, [intPt, vec[0]]);
      if(backUpPt1) vec1 = [vec[0], backUpPt1];
      console.log('intersected, backUpPt1', backUpPt1);

      let vec2 = null;
      const backUpPt2 = backUpPoint(prevVec, [intPt, vec[1]]);
      if(backUpPt2) vec2 = [backUpPt2, vec[1]];
      console.log('intersected, backUpPt2', backUpPt2);

      return {vec1, vec2};
    }
    // ------ check for either vec end point too close ------
    // tail point of vec only checked on first vector
    if(first) {
      console.log('starting tail dist chk');
      const dist2prev = distPntToVec(vec[0], prevVec);
      if(dist2prev < (2 * hullRadius)) {
        // tail end point too close to an old vec
        // back up to point on vec far enough away
        let vec1 = null;
        const backUpPt = backUpPoint(prevVec, vec, false);
        console.log('tail dist chk, backUpPoint result', backUpPt);
        if(backUpPt) vec1 = [backUpPt, vec[1]];
        return {vec1, vec2: null};
      }
    }
    console.log('starting head dist chk');
    const dist2prev = distPntToVec(vec[1], prevVec);
    console.log('did head dist chk, dist2prev:', dist2prev);
    if(dist2prev < (2 * hullRadius)) {
      // head end point too close to an old vec
      // back up to point on vec far enough away
      let vec1 = null;
      const backUpPt = backUpPoint(prevVec, vec, true);
      console.log('head dist chk, backUpPoint result', backUpPt);
      if(backUpPt) vec1 = [vec[0], backUpPt];
      showVec('head dist chk, vec1', vec1);
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
  console.log('handlePoint vec, segIdx', vec[0], vec[1], segIdx);

  const vecs = chkTooClose(vec, (segIdx == 1));
  if(vecs != null) { 
    const {vec1, vec2} = vecs;
    if(vec1) showVec('chkTooClose result vec1', vec1);
    if(vec2) showVec('chkTooClose result vec2', vec2);

    showVec('> close', vec1);
    // point was too close
    // truncated vec1 is now last in segment
    addHull(vec1);
    addHole(vec1[0], vec1[1]);
    prevVecs.push(vec1);
    lastPoint = null;
    if(vec2) {
      showVec('> split', vec2);
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

