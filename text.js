const jscad                = require('@jscad/modeling');
const {subtract}           = jscad.booleans;
const {sphere, cuboid}     = jscad.primitives;
const {vectorChar}         = jscad.text;
const {hull}               = jscad.hulls;
const {measureBoundingBox} = jscad.measurements;
const {translate}          = jscad.transforms;
const transformScale       = jscad.transforms.scale;

const strParam   = "Wyatt";

const hullRadius = 1.5;
const textYofs   = 2.5; // fraction of hullRadius
const padSides   = 10;
const spacing    = 0;
const baseline   = 0.3; // fraction of plateH
const plateW     = 160;
const plateH     = 76.5;
const plateDepth = 5;
const holeDepth  = plateDepth;
const stepDist   = 0.1; // step size when backing up

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
  if((pntEq(iPnt,segAB[0]) || pntEq(iPnt,segAB[1])) ||
     (pntEq(iPnt,segCD[0]) || pntEq(iPnt,segCD[1]))) 
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
  const dist    = distance(nearest, pnt_vec);
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
  showVec('                   vec    ', vec);
  const vecW = Bx-Ax;
  const vecH = By-Ay;
  const vecLen = Math.sqrt((vecW*vecW)+(vecH*vecH));
  // walk vec, each stepDist
  for(let distOnVec = 0; distOnVec < vecLen; 
            distOnVec += stepDist) {      
    const frac       = distOnVec/vecLen;
    const trialPoint = (chkHead ?
             [Bx-frac*vecW, By-frac*vecH] :
             [Ax+frac*vecW, Ay+frac*vecH]);
    const dist2prev  = distPntToVec(trialPoint, prevVec);
    // if(!chkHead) {
    //   console.log('distPntToVec result (frac, chkHead)', frac, chkHead);
    //   console.log('debug vec, trialPoint, dist2prev:', 
    //                           trialPoint, dist2prev);
    // }
    // console.log('after debug');

    // console.log('after distPntToVec');
    // console.log({vecW, vecH, vecLen:vecLen.toFixed(2)});
    // console.log({frac:frac.toFixed(3), 
    //              dist2prev:dist2prev.toFixed(2)});
    // console.log('trialPoint', 
    //              trialPoint[0].toFixed(2), trialPoint[1].toFixed(2));
    if(dist2prev > (2 * hullRadius)) return trialPoint;
  }
  // vec was shorter then stepDist
  return null;
}

const chkTooClose = (vec, first) => {
  let prevVecsTmp = (first ? prevVecs : prevVecs.slice(0,-1));
  // checking against all previous vecs (slow way)
  for(const prevVec of prevVecsTmp) {
    // ------ check ends touching  ------
    if( pntEq(prevVec[0], vec[0]) ||
        pntEq(prevVec[0], vec[1]) ||
        pntEq(prevVec[1], vec[0]) ||
        pntEq(prevVec[1], vec[1])) {
      // vec head or tail touching prevVec head
      // return vec unchanged
      return {vec1:vec, vec2:null};
    }
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
      console.log('             backUpPt2', backUpPt2);

      return {vec1, vec2};
    }
    // ------ check for either vec end point too close ------
    // tail point of vec only checked on first vector
    if(first) {
      // console.log('starting tail dist chk');
      const dist2prev = distPntToVec(vec[0], prevVec);
      if(dist2prev < (2 * hullRadius)) {
        // tail end point too close to an old vec
        // back up to point on vec far enough away
        let vec1 = null;
        const backUpPt = backUpPoint(prevVec, vec, false);
        // console.log('tail dist chk, backUpPoint result', backUpPt);
        if(backUpPt) {
          // console.log('vec tail too close to prev vec');
          return {vec1:[backUpPt, vec[1]], vec2: null};
        }
        else {
          console.log('both ends of vec too close');
          return {vec1:null, vec2: null};  // skip vec
        }
      }
    }
    // console.log('starting head dist chk');
    const dist2prev = distPntToVec(vec[1], prevVec);
    // console.log('did head dist chk, dist2prev:', dist2prev);
    if(dist2prev < (2 * hullRadius)) {
      // head end point too close to an old vec
      // back up to point on vec far enough away
      let vec1 = null;
      const backUpPt = backUpPoint(prevVec, vec, true);
      // console.log('head dist chk, backUpPoint result', backUpPt);
      if(backUpPt) {
        vec1 = [vec[0], backUpPt];
        showVec('head dist chk, vec1', vec1);
        return {vec1, vec2: null};
      }
      else {
        // console.log('head dist chk, both ends too close');
        return {vec1: null, vec2: null};
      }
    }
  }
  // vec not too close to any prev vec
  return null;
}

const addHull = (vec, z0 = 0, z1 = 0) => {
  if(z1 == 0) showVec(' - hull', vec);
  hulls.push(hull(sphere({hullRadius, center: vec[0].concat(z0)}),
                  sphere({hullRadius, center: vec[1].concat(z1)})));
}

const addHole = (tailPoint, headPoint) => {
  showVec(' - hole', [tailPoint, headPoint]);
  const w = headPoint[0] - tailPoint[0];
  const h = headPoint[1] - tailPoint[1];
  const len = Math.sqrt(w*w+h*h);
  const scale = holeDepth / len;
  const x = headPoint[0] + scale*w;
  const y = headPoint[1] + scale*h;
  addHull([headPoint,[x,y]], 0, -holeDepth);
}

// returns next seg idx
const handlePoint = (point, segIdx, segLast) => {
  if(segIdx == 0) {
    // first point
    console.log('first point of segment', point[0], point[1]);
    // console.log('only setting lastPoint');
    lastPoint = point;
    return 1; // next segidx is 1
  }
  // not first point
  let vec = [lastPoint, point];
  console.log('handlePoint vec, segIdx, segLast',
                vec[0], vec[1], segIdx, segLast);

  const vecs = chkTooClose(vec, (segIdx == 1));
  if(vecs != null) { 
    const {vec1, vec2} = vecs;
    if(vec1) showVec('chkTooClose result vec1', vec1);
    if(vec2) showVec('                   vec2', vec2);
    if(vec1 == null) {
      // both ends too close, skipping vec
      lastPoint = null;
      return 0; // next segidx is 0
    }
    showVec('> close', vec1);
    // point was too close
    if(segIdx == 1) addHole(point, vec1[0]); // add first hole
    addHull(vec1);
    // addHole(vec1[0], vec1[1]);
    prevVecs.push(vec1);
    if(vec2) {
      showVec('> split', vec2);
      // had intersetion
      // vec was split into vec1 and vec2
      // handle vec2 as first in new segment
      lastPoint = vec2[0];
      handlePoint(vec2[1], 1, segLast);
      if(!segLast) return 2;  // next segidx is 2
    }
    if(segLast) addHole(vec1[0], vec1[1]); // add last hole
    lastPoint = vec1[1];
    return segIdx + 1; // next segidx
  }
  // point not too close
  showVec(' ', vec);
  if(segIdx == 1) addHole(point, lastPoint); // add first hole
  addHull(vec);
  if(segLast) addHole(lastPoint, point); // add last hole
  prevVecs.push(vec);
  lastPoint = point;
  return segIdx + 1; // next segidx
}

const main = () => {
  console.log("---- main ----");
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
  // return hulls;
  let plate = cuboid({
      size:   [plateW, plateH, plateDepth]}
  );
  xOffset -= spacing;
  const textScale = (plateW - padSides*2) / xOffset;
  const sizedHulls = [];
  const xOfs = padSides - plateW/2;
  const yOfs = plateH*baseline - plateH/2;
  const zOfs = plateDepth - textYofs*hullRadius;
  hulls.forEach( hull => {
    const scaledHull = 
          transformScale([textScale,textScale,textScale], hull);
    const xlatedHull = translate([xOfs, yOfs, zOfs],
                                  scaledHull);
    plate = subtract(plate, xlatedHull);
    // sizedHulls.push(xlatedHull);
  });
  // const objects = sizedHulls.concat(plate);
  return plate;
};

module.exports = {main};

