const jscad             = require('@jscad/modeling');
const { sphere }        = jscad.primitives;
const { vectorChar }    = jscad.text;
const { hull }          = jscad.hulls;

const strParam   = "Wyatt";
const hullRadius = 1;
// distance in mm each step when backing up
const stepDistMm = 0.1;

// return intersection point if exists
// https://algs4.cs.princeton.edu/91primitives
const intersectionPoint = (segAB, segCD) => {
  const [[Ax,Ay], [Bx,By]] = segAB;
  const [[Cx,Cy], [Dx,Dy]] = segCD;
  const numeratorR  = ((Ay-Cy)*(Dx-Cx) - (Ax-Cx)*(Dy-Cy));
  const numeratorS  = ((Ay-Cy)*(Bx-Ax) - (Ax-Cx)*(Dy-Cy));
  const denominator = ((Bx-Ax)*(Dy-Cy) - (By-Ay)*(By-Ay));
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
  console.log( pfx +
    vec[0][0].toString().padStart(2) + ','    + 
    vec[0][1].toString().padStart(2) + ' -> ' +
    vec[1][0].toString().padStart(2) + ','    + 
    vec[1][1].toString().padStart(2));
}

const prevVecs  = [];
const hulls     = [];
let   lastPoint = null;

const addHull = (vec) => {
  showVec(' - hull :', vec);
  hulls.push( 
    hull(sphere({hullRadius, center: vec[0].concat(0)}), 
         sphere({hullRadius, center: vec[1].concat(0)})) 
  );
}

const addHole = (tailPoint, headPoint) => {
  showVec(' - hole :', [tailPoint, headPoint]);
}

// return point on vec far enough away from prevVec
const backUpPoint = (prevVec, vec) => {
  const [[Ax, Ay],[Bx, By]] = vec;
  const vecW = Bx-Ax;
  const vecH = By-Ay;
  const vecLen = Math.sqrt((vecW*vecW)+(vecH*vecH));
  for(const distOnVec = 0; distOnVec < vecLen; 
            distOnVec += stepDistMm) {
    const frac       = distOnVec/vecLen;
    const trialPoint = [Ax+frac*vecW, Ay+frac*vecH];
    const dist2prev  = distPntToVec(trialPoint, prevVec);
    if(dist2prev > (2 * hullRadius)) return trialPoint;
  }
  // vec was shorter then stepDistMm
  return null;
}

const chkPoint = (vec) => {
  for(const prevVec of prevVecs.slice(0,-1)) {
    let intPt = (intersectionPoint(vec, prevVec));
    if(intPt) {
      showVec('  --X: ', prevVec, intPt);
      // vec intersects a previous vector
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
      showVec('  --D:', dstPntToLin);
      // latest point too close to old vec
      // back up to point on vec far enough away
      let vec1 = null;
      const backUpPt = backUpPoint(prevVec, vec);
      if(backUpPt) vec1 = [vec[0], backUpPt];
      return {vec1, vec2: null};
    }
  }
  // point not too close to any prev vec
  return null;
}

// returns false if point deleted
const handlePoint = (point, segIdx, segLast) => {
  if(lastPoint == null) { // first point
    lastPoint = chkPoint([null, point]);
  }
  else {                   // not first point
    let vec = [lastPoint, point];
    // if vec too close newPoint is fractional point on vec
    const newPoint = chkPoint(vec);
    if(newPoint != null) { 
      // point too close, 


      // entire vector too close, remove it
      if(prevVecs.length == 0) {
        // vec is first in seg, delete seg
        lastPoint = null;
        return false;
      }
      // remove last vector in history
      prevVecs  = prevVecs.slice(0,-1);
      lastPoint = point;
      vec = prevVecs.slice(-1);  // replace current vec with last
      point = vec[0];
    } 
    else {
      // point not too close
      point = newPoint;
      if(segIdx = 1) addHole(point, lastPoint); // add first hole
      showVec(' ', vec);
      prevVecs.push(vec);
      addHull(vec);
      if(segLast) addHole(lastPoint, point);
      lastPoint = point;
    }
  }
  return true;
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
      let pointIdx = 0;
      lastPoint = null;
      seg.forEach( point => {
        if(pointIdx == 0
        if(!handlePoint(point, pointIdx, pointIdx == seg.length-1)) {
          // split segment
          pointIdx = 0;
          return;
        }
        pointIdx++;
      });
    });
  };
  console.log("\n---- end ----");
  return hulls;
};

module.exports = {main};

