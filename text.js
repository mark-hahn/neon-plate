const jscad             = require('@jscad/modeling');
const {union, subtract} = jscad.booleans;
const {sphere, cube, cuboid, cylinderElliptic, cylinder}  
       = jscad.primitives;
const {vectorChar}      = jscad.text;
const {hull, hullChain} = jscad.hulls;
const {translate, translateZ} = jscad.transforms;

const debug = false;

// ------ default params --------- 
let strParam    = "Clive";
let fontsizeAdj = 1.1;
let vertOfs     = -7;
let genHulls    = true;
let genHoles    = true;
let genPlate    = true;

const radius     = 0.75 + 0.2; // 0.2 is for expansion
const stepDist   = 0.1;        // step size when backing up
const segments   = 16;         // sphere segments
const bkupDist   = 3.0;        // dist to back up, frac times radius
const holeTop    = 1.5*radius; // only affects top sphere
const holeBot    = 2.0*radius; // only affects bottom sphere

const plateW      = 182.3;
const plateH      = 84.4;
const plateDepth  = 2.4;
const filetRadius = 2;
const textZofs    = 0.75;  // fraction of diameter below the surface
const padSides    = 20;
const padTopBot   = 15;
// const baseline   = 0.3;   // fraction of plateH

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
  // console.log(typeof vec, typeof vec?.[0], typeof vec?.[1]);
  console.log( pfx + ' ' +
    ((typeof vec?.[0] === 'undefined' || 
      typeof vec?.[1] === 'undefined') ? 'null' : (
      vec[0][0].toFixed(1).padStart(4) + ','    + 
      vec[0][1].toFixed(1).padStart(4) + ' -> ' +
      vec[1][0].toFixed(1).padStart(4) + ','    + 
      vec[1][1].toFixed(1).padStart(4)))
  );
}

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
    const trialPoint = 
      (chkHead ? [Bx-frac*vecW, By-frac*vecH]
               : [Ax+frac*vecW, Ay+frac*vecH]);
    const dist2prev  = distPntToVec(trialPoint, prevVec);
    if(dist2prev > bkupDist * radius) return trialPoint;
  }
  // vec was shorter then stepDist
  return null;
}

const prevVecs  = [];

const chkTooClose = (vec, first) => {
  let prevVecsTmp = (first ? prevVecs : prevVecs.slice(0,-1));
  // checking against all previous vecs (slow way)
  for(const prevVec of prevVecsTmp) {

    // ------ check ends touching  ------
    if((pntEq(prevVec[0], vec[0]) ||
        pntEq(prevVec[1], vec[0]))) {
      // vec tail is touching prevVec head or tail
      showVec('tail touches prev', vec);
      const backUpPnt = backUpPoint(prevVec, vec, false);
      if(!backUpPnt) return {
        headClose:false, tailClose:true, vec1:null, vec2:null};
      return {
        headClose:false, tailClose:true, 
        vec1:[backUpPnt, vec[1]], vec2:null};
    }
    if( pntEq(prevVec[0], vec[1]) ||
        pntEq(prevVec[1], vec[1])) {
      // vec head is touching prevVec head or tail
      showVec('head touches prev', vec);
      const backUpPnt = backUpPoint(prevVec, vec, true);
      if(!backUpPnt) return {
        headClose:true, tailClose:false, vec1:null, vec2:null};
      return {
        headClose:true, tailClose:false, 
        vec1:[vec[0], backUpPnt], vec2:null};
    }

    // ------ check vecs intersection  ------
    const intPt = (intersectionPoint(vec, prevVec));
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

      return {headClose:true, tailClose:true, vec1, vec2};
    }

    // ------ check for either vec end point too close ------
    // tail point of vec only checked on first vector
    if(first) {
      // console.log('starting tail dist chk');
      const dist2prev = distPntToVec(vec[0], prevVec);
      if(dist2prev < (2 * radius)) {
        // tail end point too close to an old vec
        // back up to point on vec far enough away
        let vec1 = null;
        const backUpPt = backUpPoint(prevVec, vec, false);
        // console.log('tail dist chk, backUpPoint result', 
                        // backUpPt);
        if(backUpPt) {
          // console.log('vec tail too close to prev vec');
          return {
            headClose:false, tailClose:true, 
            vec1:[backUpPt, vec[1]], vec2: null};
        }
        else {
          console.log('both ends of vec too close');
          return {  // skip vec
            headClose:true, tailClose:true, 
            vec1:null, vec2: null};
        }
      }
    }
    // console.log('starting head dist chk');
    const dist2prev = distPntToVec(vec[1], prevVec);
    // console.log('did head dist chk, dist2prev:', dist2prev);
    if(dist2prev < (2 * radius)) {
      // head end point too close to an old vec
      // back up to point on vec far enough away
      let vec1 = null;
      const backUpPt = backUpPoint(prevVec, vec, true);
      // console.log('head dist chk, backUpPoint result', backUpPt);
      if(backUpPt) {
        vec1 = [vec[0], backUpPt];
        showVec('head dist chk, vec1', vec1);
        return {headClose:true, tailClose:false, vec1, vec2: null};
      }
      else {
        // console.log('head dist chk, both ends too close');
        return {headClose:true, tailClose:true,  vec1: null, vec2: null};
      }
    }
  }
  // vec not too close to any prev vec
  return {headClose:false, tailClose:false, 
          vec1: null, vec2: null};
}

const ptEq = (A,B) => (A[0] == B[0] && A[1] == B[1]);

const hullChains = [];
let   spherePts  = [];

const addToHullChains = () => {
  if(spherePts.length) {
    const spheres = spherePts.map((pt) =>
      sphere({radius, segments, center: pt.concat(0)}));
    hullChains.push(hullChain(...spheres));
  }
}

const addHull = (vec) => {
  showVec(' - hull', vec);
  if(genHulls) {
    if(spherePts.length && ptEq(spherePts.at(-1), vec[0]))
      spherePts.push(vec[1]);
    else {
      addToHullChains();
      spherePts = [vec[0], vec[1]];
    }
  }
}

holes = [];

const addHole = (tailPoint, headPoint) => {
  showVec(' - hole', [tailPoint, headPoint]);
  if(genHoles) {
    const w       = headPoint[0] - tailPoint[0];
    const h       = headPoint[1] - tailPoint[1];
    const len     = Math.sqrt(w*w + h*h);
    const scale   = plateDepth/len;
    const holeLen = plateDepth*1.414;
    const x       = headPoint[0] + scale*w;
    const y       = headPoint[1] + scale*h;
    holes.push( hull(
      sphere({radius:holeTop, segments, 
              center:headPoint.concat(0)}),
      sphere({radius:holeBot, segments, 
              center: [x,y,-holeLen]})));
  }
}

let lastPoint = null;

// returns next seg idx
const handlePoint = (point, segIdx, segLast) => {
  if(segIdx == 0) {
    // first point
    // console.log('first point of segment', point[0], point[1]);
    // console.log('only setting lastPoint');
    lastPoint = point;
    return 1; // next segidx is 1
  }
  // not first point
  let vec = [lastPoint, point];
  console.log('\nhandlePoint segIdx, segLast:', segIdx, segLast);
  showVec(    '                       vec:', vec);

  const {headClose, tailClose, vec1, vec2} = 
         chkTooClose(vec, (segIdx == 1));
  if(headClose || tailClose) { 
    // ---- something was too close
    showVec('too close result, vec1', vec1);
    showVec('                  vec2', vec2);

    if(vec1 == null) {
      // ---- both ends too close, skipping vec -----
      lastPoint = null;
      return 0; // next segidx is 0
    }

    // ---- an end was too close
    if(segIdx == 1 || tailClose) 
      addHole(point, vec1[0]); // add first hole
    addHull(vec1);
    prevVecs.push(vec1);
    if(vec2) {
      // had intersetion
      // vec was split into vec1 and vec2
      // handle vec2 as first in new segment
      lastPoint = vec2[0];
      handlePoint(vec2[1], 1, segLast);
      if(!segLast) return 2;  // next segidx is 2
    }
    if(segLast || headClose) 
      addHole(vec1[0], vec1[1]); // add last hole
    lastPoint = vec[1];
    return segIdx + 1; // next segidx
  }

  // ---- point was not too close
  showVec('  -- not too close', vec);
  if(segIdx == 1) addHole(point, lastPoint); // add first hole
  addHull(vec);
  if(segLast) addHole(lastPoint, point); // add last hole
  prevVecs.push(vec);
  lastPoint = point;
  return segIdx + 1; // next segidx
}

const getParameterDefinitions = () => {
  return [
    { name: 'strParam', type: 'text', initial: strParam, 
      size: 10, maxLength: 6, caption: 'Display Text:', 
      placeholder: 'Enter 3 to 6 characters' 
    },
    { name: 'fontsizeAdj', type: 'number', initial: fontsizeAdj, min: 0.25, max: 4.0, 
      step: 0.1, caption: 'Font Size:' 
    },
    { name: 'vertOfs', type: 'int', 
      initial: vertOfs, min: -plateH, max: plateH, 
      step: 1, caption: 'Vertical Offset:' 
    },
    { name: 'show', type: 'choice', caption: 'Show:', 
      values: [0, 1, 2], initial: 0,
      captions: ['Plate With Cutouts', 'Channels and Holes', 'Only Channels']
    },
  ];
}

// flat-head M3 bolt
const fhBolt_M3 = (length) => {
  const plasticOfs  = 0.3
  const topRadius   = 6/2  + plasticOfs;
  const shaftRadius = 3/2  + plasticOfs;
  const topH        = 1.86 + plasticOfs;
  const zOfs        = -topH/2;
  const cylH        = length - topH + 1; // 1mm excess
  const cylZofs     = -cylH/2 - topH;
  const top = translateZ(zOfs, cylinderElliptic(
                {startRadius:[shaftRadius, shaftRadius], 
                  endRadius:[topRadius,topRadius],
                  height:topH}));
  const shaft = translateZ(cylZofs, cylinder(
                  {height:cylH, radius:shaftRadius}));
  return union(top, shaft);
};

const main = (params) => {
  strParam    = params.strParam;
  vertOfs     = params.vertOfs;
  fontsizeAdj = params.fontsizeAdj;
  switch(params.show) {
    case 0: genHulls = true; genHoles = true; 
            genPlate = true; break;
    case 1: genHulls = true; genHoles = true; 
            genPlate = false; break;
    case 2: genHulls = true; genHoles = false; 
            genPlate = false; break;
  }
  console.log("---- main ----");

  let strWidth  = 0;
  let strHeight = 0;
  for(const char of strParam) {
    const {width, height} = vectorChar(char);
    strWidth  += width;
    strHeight = Math.max(strHeight, height);
  };

  if (debug) console.log({strWidth, strHeight});
  
  const scaleW    = (plateW - padSides*2)  / strWidth;
  const scaleH    = (plateH - padTopBot*2) / strHeight;
  let   textScale = Math.min(scaleW, scaleH) * fontsizeAdj;

  strWidth       *= textScale;
  strHeight      *= textScale;
  let xOfs        = (plateW - strWidth)/2  - plateW/2;
  let yOfs        = (plateH - strHeight)/2 - plateH/2 + vertOfs;

  if (debug) {
    textScale = 1;
    xOfs      = 0;
    yOfs      = 0
  }

  strWidth  = 0;
  for(const char of strParam) {
    console.log("\n======== CHAR:  " + char + '  ========');
    const {width, segments:segs} = 
           vectorChar({xOffset:strWidth}, char);
    strWidth  += width;
    segs.forEach( seg => {
      console.log("\n--- seg ---");
      let segIdx = 0;
      seg.forEach( point => {
        point[0] *= textScale;
        point[1] *= textScale;
        segIdx = handlePoint(point, segIdx, segIdx == seg.length-1);
      });
    });
  };
  console.log("\n---- end ----");

  addToHullChains(); // add remaining spheres to hullchains
  const allHulls = hullChains.concat(holes);
  const zOfs     =  plateDepth/2 - textZofs*radius;
  const hullsOfs = translate([xOfs, yOfs, zOfs], allHulls);

  if(!genPlate) return hullsOfs;
  
  const plate = cuboid({size: [plateW, plateH, plateDepth]});

  const notchedPlate = subtract(plate, 
      translate( [-plateW/2 + filetRadius/2, plateH/2 - filetRadius/2, 0],
        cuboid({size: [filetRadius, filetRadius, plateDepth]})),     
      translate( [ plateW/2 - filetRadius/2, plateH/2 - filetRadius/2, 0],
        cuboid({size: [filetRadius, filetRadius, plateDepth]})));      

  const roundedPlate = union(notchedPlate,    
    cylinder({
      center:[-plateW/2 + filetRadius, plateH/2 - filetRadius, 0],
      height:plateDepth, radius:filetRadius}), 
    cylinder({
      center:[ plateW/2 - filetRadius, plateH/2-filetRadius, 0],
      height:plateDepth, radius:filetRadius}));;

  const boltOfs = 5.25;

  const boltUL = translate(
      [-plateW/2 + boltOfs,  plateH/2 - boltOfs, plateDepth/2],
      fhBolt_M3(plateDepth + 1));
  const boltUR = translate(
      [ plateW/2 - boltOfs,  plateH/2 - boltOfs, plateDepth/2],
      fhBolt_M3(plateDepth + 1));
  const boltBL = translate(
      [-plateW/2 + boltOfs, -plateH/2 + boltOfs, plateDepth/2],
      fhBolt_M3(plateDepth + 1));
  const boltBR = translate(
      [ plateW/2 - boltOfs, -plateH/2 + boltOfs, plateDepth/2],
      fhBolt_M3(plateDepth + 1));

  const plateOut = subtract(roundedPlate, hullsOfs, 
                            boltUL, boltUR, boltBL,boltBR);

  return plateOut;
};

module.exports = {main, getParameterDefinitions};

