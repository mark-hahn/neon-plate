const jscad                   = require('@jscad/modeling');
const {union, subtract}       = jscad.booleans;
const {sphere, cube, cuboid, cylinderElliptic, cylinder}  
                              = jscad.primitives;
const {vectorChar}            = jscad.text;
const {hull, hullChain}       = jscad.hulls;
const {translate, translateZ} = jscad.transforms;

const debug      = false;
const debugScale = false;

const MAX_ANGLE = 90

// ------ default params --------- 
let fontIdx     = 0;
let text        = 'Cam';
let fontsizeAdj = 1.1;
let vertOfs     = -7;
let genHulls    = true;
let genHoles    = true;
let genPlate    = true;

// channel mm, 0.2 is expansion
const radius = (debugScale? .2 : 0.75 + 0.2); 

const stepDist   = 0.1;        // mm step size when backing up
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

const ptEq         = (A,B) => A && B && A[0] == B[0] && A[1] == B[1];
const vecEq        = (A,B) => A && B && ptEq(A[0],B[0]) && ptEq(A[1],B[1]);
const vecRevEq     = (A,B) => A && B && ptEq(A[0],B[1]) && ptEq(A[1],B[0]);
const ptTouchesEnd = (P,V) => P && V && ptEq(P, V[0]) || ptEq(P, V[1]);

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
    return null;   // doesn't intersect
  const iPnt = [Ax + r*(Bx-Ax), Ay + r*(By-Ay)]; // intersection point
  if(ptTouchesEnd(iPnt,segAB) || ptTouchesEnd(iPnt,segCD)) 
    return null;  // ends of vectors at same point
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
//get the distance between a point and a line
const distPntToVec = (pnt, vec) => {
  pnt                  = [pnt[0], 0, pnt[1]];
  let [start, end]     = vec;
  start                = [start[0], 0, start[1]];
  end                  = [end[0],   0, end[1]];
  const line_vec       = vector(start, end);
  const pnt_vec        = vector(start, pnt);
  const line_len       = length(line_vec);
  const line_unitvec   = unit(line_vec);
  const pnt_vec_scaled = scale(pnt_vec, 1.0/line_len);
  const dt             = dot(line_unitvec, pnt_vec_scaled);
  const t              = Math.max(Math.min(dt,1),0);
  let   nearest        = scale(line_vec, t);
  const dist           = distance(nearest, pnt_vec);
  return dist;
}

const showVec = (pfx, vec) => {
  // console.log('vec[0][0]', vec[0][0]);
  // console.log('vec[0][1]', vec[0][1]);
  // console.log('vec[1][0]', vec[1][0]);
  // console.log('vec[1][1]', vec[1][1]);
  console.log( pfx + ' ' +
    ((typeof vec?.[0]?.[0] === 'undefined' || 
      typeof vec?.[0]?.[1] === 'undefined' ||
      typeof vec?.[1]?.[0] === 'undefined' ||
      typeof vec?.[1]?.[1] === 'undefined') ? 'null' : (
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
  if(debug)  showVec('enter backUpPoint, prevVec', prevVec);
  if(debug)  showVec('                   vec    ', vec);
  const vecW   = Bx-Ax;
  const vecH   = By-Ay;
  const vecLen = Math.sqrt((vecW*vecW)+(vecH*vecH));
  // walk vec, each stepDist
  for(let distOnVec = 0; distOnVec < vecLen; 
                         distOnVec += stepDist) {      
    const frac = distOnVec/vecLen;
    const trialPoint = 
      (chkHead ? [Bx-frac*vecW, By-frac*vecH]
               : [Ax+frac*vecW, Ay+frac*vecH]);
    const dist2prev  = distPntToVec(trialPoint, prevVec);
    if(dist2prev > bkupDist * radius) return trialPoint;
  }
  // vec was shorter then stepDist
  return null;
}


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
    let scale   = plateDepth/len;
    let holeLen = plateDepth*1.414;
    if(debugScale) {
      scale   = .1;
      holeLen = .5;
    }
    const x       = headPoint[0] + scale*w;
    const y       = headPoint[1] + scale*h;
    holes.push( hull(
      sphere({radius:holeTop, segments, 
              center:headPoint.concat(0)}),
      sphere({radius:holeBot, segments, 
              center: [x,y,-holeLen]})));
  }
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
// check if vertex angle is greater than 90 degrees
// if so then add a hole to each vec at vertex
const chkSharpBend = (lastVec, vec) => {
  if(debug) showVec('entering chkSharpBend, lastVec:',lastVec);
  if(debug) showVec('                          vec:',vec);
  const [p1, p2] = lastVec;
  const  p3      = vec[1];
  // translate p1 and p3 to origin
  let [x1,y1] = [p2[0]-p1[0], p2[1]-p1[1]];
  let [x2,y2] = [p3[0]-p2[0], p3[1]-p2[1]];
  if((x1 == 0) && (x2 == 0)) {
    // each vector pointing directly up or down
    if(debug) showVec('both vectors up or down:',lastVec);
    if(debug) showVec('                    vec:',vec);
    if((y1 > 0) != (y2 > 0)) {
      // bend is 180 degrees, add holes to both vecs at p2
      addHole(p1,p2);
      addHole(p3,p2);
    }
    return;
  }
  if((x1 == 0) || (x2 == 0)) {
    if(debug) showVec('a vec points up or down:',lastVec);
    if(debug) showVec('                    vec:',vec);
    // switch x and y which rotates both 90 degrees
    [x1,y1,x2,y2] = [y1,x1,y2,x2];
    // now check again
    if((x1 == 0) || (x2 == 0)) {
      if(debug) showVec('90 degrees apart:',lastVec);
      if(debug) showVec('             vec:',vec);
      if(90 > MAX_ANGLE) { // (yes, 2 constants)
        addHole(p1,p2);
        addHole(p3,p2);
      }
      return;
    }
  }
  if(debug) console.log('before angle calc', {x1,y1,x2,y2});
  const rad2deg = 360 / (2*Math.PI);
  let   angle1  = Math.atan(y1/x1)* rad2deg
  let   angle2  = Math.atan(y2/x2)* rad2deg;
  if(x1 < 0) angle1 += 180;
  if(x2 < 0) angle2 += 180;
  if(debug) console.log('angle calc result', {angle1,angle2});
  // if(angle1 < 0) angle1 += 360;
  // if(angle2 < 0) angle2 += 360;
  const angle = Math.abs(angle2 - angle1);
  if(debug) console.log('angle', {angle});
  if(angle > MAX_ANGLE) {
    // bend is too sharp, add holes to both at p2
    addHole(p1,p2);
    addHole(p3,p2);
  }
  return;
}

let lastVec = null;
const prevVecs = [];

const chkTooClose = (vec, first) => {

  // let prevVecsTmp = (first ? prevVecs : prevVecs.slice(0,-1));
  // checking against all previous vecs (slow way)
  for(const prevVec of prevVecs) {
    if(debug) showVec('- checking prev vec', prevVec);

    // check exact match either direction with prev vec
    if(debug)  showVec('check exact match, old vec', prevVec);
    if(debug)  showVec('                   new vec', vec);
    if(vecEq(prevVec, vec) || vecRevEq(prevVec, vec)) {
      if(debug) showVec('vec exactly matches prevVec', prevVec);
      return {  // skip vec
        headClose:true, tailClose:true, vec1:null, vec2: null};
    }

    // ------ check ends touching  ------
    // check if vec is extending last, vec tail == lastVec.head
    if(debug) showVec('checking vec extension, lastVec', lastVec);
    if(debug) showVec('                            vec', vec);
    const extendingLastVec = lastVec && (ptEq(vec[0], lastVec[1]));
    if(extendingLastVec) {
      if(debug) console.log('vec is extending last');
      chkSharpBend(lastVec, vec);
    }
    else {
      if(debug) console.log('vec is not extending last');
      if(ptTouchesEnd(vec[0],prevVec)) {
        // vec tail is touching prevVec head or tail
        if(debug)  showVec('tail touches prev', prevVec);
        const backUpPnt = backUpPoint(prevVec, vec, false);
        if(!backUpPnt) return {
          headClose:false, tailClose:true, vec1:null, vec2:null};
        return {
          headClose:false, tailClose:true, 
          vec1:[backUpPnt, vec[1]], vec2:null};
      }
    }
    if(ptTouchesEnd(vec[1],prevVec)) {
      // vec head is touching prevVec head or tail
      if(debug)  showVec('head touching prev', prevVec);
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
      if(debug) console.log('intersected, at', intPt[0], intPt[1]);
      if(debug)  showVec('      prevVec', prevVec);
      if(debug)  showVec('      vec', vec);

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
    // tail point of vec only checked on ...
    //   first vector and not extendingLastVec
    if(first && !extendingLastVec) {
      if(debug) showVec('lastVec:', lastVec);
      if (debug) console.log('starting tail dist chk');
      const dist2prev = distPntToVec(vec[0], prevVec);
      if(dist2prev < (2 * radius)) {
        // tail end point too close to an old vec
        // back up to point on vec far enough away
        let vec1 = null;
        const backUpPt = backUpPoint(prevVec, vec, false);
        console.log('tail too close, backUpPoint result', 
                                backUpPt);
        if(backUpPt) {
          console.log('vec tail too close to prev vec');
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
    if(debug) showVec('lastVec:', lastVec);
    if(vecEq(prevVec, lastVec)) {
      if (debug) console.log(
        'prevVec == lastVec, skipping head dist check to last vec');
    }
    else {
      if (debug) console.log('starting head dist chk');
      const dist2prev = distPntToVec(vec[1], prevVec);
      if (debug) console.log('head dist is', dist2prev);
      if(dist2prev < (2 * radius)) {
        // head end point too close to an old vec
        // back up to point on vec far enough away
        let vec1 = null;
        const backUpPt = backUpPoint(prevVec, vec, true);
        showVec('head too close to old vec (head, backUpPoint):', 
                                        [vec[1], backUpPt]);
        if(backUpPt) {
          vec1 = [vec[0], backUpPt];
          if(debug)  showVec('head too close to old vec, vec1', vec1);
          return {headClose:true, tailClose:false, vec1, vec2: null};
        }
        else {
          if (debug) showVec('both ends too close, vec:', vec);
          return {headClose:true, tailClose:true,  vec1: null, vec2: null};
        }
      }
    }
  }
  // vec not too close to any prev vec
  return {headClose:false, tailClose:false, vec1: null, vec2: null};
}

let lastPoint = null;

// returns next seg idx
const handlePoint = (point, ptIdx, lastPtInSeg) => {
  if(ptIdx == 0) {
    // first point of segment
    if (debug) console.log('first point of segment', 
                  point[0].toFixed(1), point[1].toFixed(1));
    lastPoint = point;
    lastVec   = null;
    if(debug) showVec('setting lastVec', lastVec);
    return 1; // next ptIdx is 1
  }
  // not first point
  let vec = [lastPoint, point];
  console.log('\n-- handlePoint', {ptIdx, lastPtInSeg});
  showVec(      '-- vec:', vec);

  const {headClose, tailClose, vec1, vec2} = 
                       chkTooClose(vec, (ptIdx == 1));

  if(headClose || tailClose) { 
    // ---- something was too close
    if(debug) console.log('====== something was too close ======');
    if(debug) showVec('too close result, vec1', vec1);
    if(debug) showVec('                  vec2', vec2);

    if(vec1 == null) {
      // ---- both ends too close, skipping vec -----
      if(debug) showVec('both ends too close, skipping vec', vec);
      lastPoint = null;
      if(debug) showVec('checking lastVec', lastVec);
      if(lastVec) {
        if(debug) showVec('add last hole to lastVec', lastVec);
        addHole(lastVec[0], lastVec[1]);
      }
      return 0; // next ptIdx is 0
    }

    // ---- an end was too close
    if(ptIdx == 1 || tailClose) 
      addHole(point, vec1[0]); // add first hole
    addHull(vec1);
    if(debug) showVec('adding to prevVecs vec1', vec1);
    prevVecs.push(vec1);
    lastVec = vec1;
    if(debug) showVec('setting lastVec:', lastVec);
    if(vec2) {
      // had intersection
      // vec was split into vec1 and vec2
      addHole(vec1[0], vec1[1]);

      // handle vec2 as first in new segment
      if(debug) console.log('before recursive call to handlePoint');
      lastPoint = vec2[0];
      // next ptIdx is 1, lastPtInSeg is still correct
      handlePoint(vec2[1], 1, lastPtInSeg);
      if(debug) console.log('after recursive call to handlePoint');

      lastVec = vec2;
      if(debug) showVec('(intersection) lastVec:', lastVec);
      if(!lastPtInSeg) return 2;  // next ptIdx is 2
    }
    if(debug) console.log('checking last hole', {lastPtInSeg, headClose});
    if(lastPtInSeg || headClose) 
      addHole(vec1[0], vec1[1]); // add last hole
    lastPoint = vec[1];
    return ptIdx + 1; // next ptIdx
  }

  // ---- point was not too close
  if(debug) showVec('  -- not too close', vec);
  if(ptIdx == 1) addHole(point, lastPoint); // add first hole
  addHull(vec);
  if(lastPtInSeg) addHole(lastPoint, point); // add last hole
  if(debug)  showVec('adding to prevVecs vec', vec);
  prevVecs.push(vec);
  lastPoint = point;
  lastVec   = vec;
  if(debug) showVec('lastVec = vec', lastVec);
  return ptIdx + 1; // next ptIdx
}

const getParameterDefinitions = () => {
  console.log('getParameterDefinitions', Object.keys(fonts));
  return [
    { name: 'fontIdx', type: 'choice', caption: 'Font:', 
      values: Object.keys(fonts).map((key,idx) => idx),
      captions: Object.keys(fonts),
      initial:  fontIdx, 
    },
    { name: 'text', type: 'text', initial: text, 
      size: 10, maxLength: 7, caption: 'Display Text:', 
      placeholder: 'Enter 3 to 7 characters' 
    },
    { name: 'fontsizeAdj', type: 'number', 
      initial: fontsizeAdj, min: 0.25, max: 4.0, 
      step: 0.1, caption: 'Font Size:' 
    },
    { name: 'vertOfs', type: 'int', 
      initial: vertOfs, min: -plateH, max: plateH, 
      step: 1, caption: 'Vertical Offset:' 
    },
    { name: 'show', type: 'choice', caption: 'Show:', 
      values: [0, 1, 2], initial: 1,
      captions: ['Plate With Cutouts', 
                 'Channels and Holes', 
                 'Only Channels']
    },
  ];
}

const main = (params) => {
  const {fontIdx, text, vertOfs, fontsizeAdj} = params;
  const font = fonts[Object.keys(fonts)[fontIdx]];

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
  for(const char of text) {
    const {width, height} = vectorChar({font}, char);
    strWidth  += width;
    strHeight = Math.max(strHeight, height);
  };

  if (debug) console.log({strWidth, strHeight});
  
  const scaleW    = (plateW - padSides*2)  / strWidth;
  const scaleH    = (plateH - padTopBot*2) / strHeight;
  let   textScale = Math.min(scaleW, scaleH) * fontsizeAdj;

  strWidth  *= textScale;
  strHeight *= textScale;
  let xOfs   = (plateW - strWidth)/2  - plateW/2;
  let yOfs   = (plateH - strHeight)/2 - plateH/2 + vertOfs;

  if (debugScale) {
    textScale = 1;
    xOfs      = 0;
    yOfs      = 0
  }

  let xOffset = 0;
  for(const char of text) {
    const charRes = vectorChar({font, xOffset}, char);
    let {width, segments} = charRes;
    console.log("\n======== CHAR: " + char + 
                 ', segment count:', segments.length);
    xOffset += width;
    segments.forEach( seg => {
      console.log("\n--- seg ---, point count: ", seg.length);
      let ptIdx  = 0;
      let segIdx = 0;
      seg.every( point => {
        point[0] *= textScale;
        point[1] *= textScale;
        ptIdx = handlePoint(point, ptIdx, ++segIdx == seg.length);
        return (segIdx != null);
      });
    });
  };
  console.log("\n---- end ----, hole count:", holes.length);

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
  const boltUM = translate(
      [                  0,  plateH/2 - boltOfs, plateDepth/2],
      fhBolt_M3(plateDepth + 1));
  const boltUR = translate(
      [ plateW/2 - boltOfs,  plateH/2 - boltOfs, plateDepth/2],
      fhBolt_M3(plateDepth + 1));
  const boltBL = translate(
      [-plateW/2 + boltOfs, -plateH/2 + boltOfs, plateDepth/2],
      fhBolt_M3(plateDepth + 1));
  const boltBM = translate(
      [                  0, -plateH/2 + boltOfs, plateDepth/2],
      fhBolt_M3(plateDepth + 1));
  const boltBR = translate(
      [ plateW/2 - boltOfs, -plateH/2 + boltOfs, plateDepth/2],
      fhBolt_M3(plateDepth + 1));

  const plateOut = subtract(roundedPlate, hullsOfs, 
                            boltUL, boltUM, boltUR, boltBL, boltBM, boltBR);

  return plateOut;
};
module.exports = {main, getParameterDefinitions};

const fonts = {

camBamStick9Font:{height:50,
67: [62,53,9,46,9,41,9,38,10,33,12,29,14,26,16,22,19,18,23,16,26,14,31,12,35,11,38,11,41,10,47,10,52,11,56,13,61,15,66,17,69,21,73,25,77,28,79,33,82,37,84,40,85,43,85,50,85,53,85,],

97: [69,43,10,41,9,40,9,39,8,38,8,34,8,30,8,27,8,24,10,20,11,18,13,16,15,13,18,12,20,11,23,9,27,9,30,9,34,9,38,9,41,11,44,12,48,13,50,16,52,18,55,20,56,24,58,27,59,30,60,34,60,38,60,40,59,44,58,47,57,49,55,52,53,53,52,54,51,55,49,57,46,58,43,58,41,58,37,58,9,],

109: [89,
12,9,12,43,12,46,12,48,13,50,14,52,14,53,15,54,16,55,18,56,19,57,21,58,23,59,25,59,27,59,32,59,34,59,36,58,38,56,40,54,44,49,45,9,
,
45,47,45,48,46,51,49,54,52,57,53,58,54,59,55,59,56,59,62,59,64,59,66,59,68,58,70,57,72,56,73,55,75,53,76,52,77,50,77,48,78,46,78,43,78,9,
]},

EMSBird:{height:500,
66:[630,101,712,271,699,397,658,472,605,491,539,457,454,369,391,287,362,202,350,126,346,299,356,447,350,529,328,586,268,589,186,526,117,432,63,274,34.6,88.2,47.2,126,47.2,126,25.2,123,205,126,419,161,592,192,743,205,750,],
67:[706,595,554,602,558,617,586,608,652,545,702,460,715,328,702,217,639,135,554,75.6,416,63,293,91.4,154,161,85.1,255,34.6,362,22,479,40.9,580,88.2,627,120,662,164,662,180,],
87:[879,129,636,154,702,198,728,151,646,104,504,85,359,72.5,236,75.6,148,104,78.8,151,56.7,198,63,239,91.4,306,158,359,236,419,346,450,416,491,554,517,630,539,677,517,677,491,649,495,624,491,570,469,432,460,299,472,167,507,78.8,570,53.6,639,88.2,721,192,781,299,832,438,873,558,895,646,885,724,],
97:[343,306,397,230,413,173,406,97.6,309,63,239,31.5,158,15.8,94.5,18.9,59.9,34.6,44.1,81.9,72.4,145,135,205,211,246,265,277,334,293,372,306,406,318,413,312,381,268,302,236,195,217,117,211,69.3,217,50.4,233,47.2,268,59.9,302,88.2,340,129,],
101:[258,41,255,117,243,195,246,233,265,252,287,255,331,243,375,220,397,183,400,142,378,101,309,59.9,214,41,151,34.6,94.5,47.2,44.1,81.9,31.5,123,44.1,173,81.9,217,142,],
105:[170,101,318,139,387,148,403,170,419,97.6,265,72.5,183,44.1,101,28.4,50.4,34.6,44.1,75.6,53.6,142,94.5,,239,598,186,548,180,520,],
109:[501,104,328,139,403,158,416,88.2,246,12.6,40.9,81.9,145,154,255,208,334,252,378,309,425,346,432,331,391,274,287,230,164,211,75.6,208,44.1,230,56.7,271,158,331,261,387,331,447,378,491,413,507,422,542,428,507,375,454,249,400,132,381,56.7,378,18.9,381,3.15,406,-6.3,444,18.9,],
111:[265,239,391,192,413,126,365,88.2,287,56.7,208,37.8,151,34.6,78.8,44.1,44.1,72.5,25.2,117,31.5,173,85.1,220,148,239,211,252,265,249,312,252,375,230,432,214,441,224,378,265,328,299,328,],
116:[239,173,472,202,548,236,570,151,346,88.2,132,75.6,69.3,81.9,40.9,113,50.4,142,63,211,120,,78.8,406,315,413,],
119:[517,25.2,378,63,406,81.9,403,97.6,337,104,249,107,202,88.2,129,63,63,56.7,47.2,75.6,31.5,94.5,126,173,261,230,356,255,387,274,406,302,378,328,274,343,192,353,110,346,50.4,384,110,438,202,491,306,510,356,526,397,526,435,520,441,513,419,],
121:[334,56.7,362,75.6,387,117,290,120,205,120,88.2,110,0,,315,403,359,419,343,381,293,284,230,195,164,101,59.9,-37.8,-15.8,-151,-56.7,-214,-72.4,-268,-69.3,-302,],},

EMSDecorousScript:{height:500,
66:[791,432,6.3,469,-28.4,536,-41,614,-12.6,677,28.4,728,78.8,756,139,772,189,772,246,747,293,718,315,662,337,709,337,791,372,847,422,891,485,904,545,879,598,828,633,747,633,668,624,570,592,476,542,413,501,337,438,261,353,208,280,167,192,139,117,132,53.6,135,-3.15,180,-31.5,233,-37.8,315,-9.45,413,88.2,491,211,586,378,655,491,702,554,753,595,806,617,],
67:[469,639,205,614,167,567,107,510,56.7,454,9.45,387,-22.1,309,-37.8,220,-22.1,161,56.7,161,176,214,306,280,416,362,507,460,583,536,624,602,633,668,621,690,580,674,507,643,466,589,425,],
87:[876,192,444,236,510,299,586,350,624,413,636,444,614,450,570,432,513,350,362,261,211,205,97.7,192,47.2,195,9.45,220,-25.2,265,-41,324,-18.9,397,37.8,472,113,548,195,756,576,513,135,495,75.6,501,28.4,517,-18.9,576,-34.6,633,-18.9,706,22.1,797,113,882,227,954,343,1014.3,460,1052.1,570,1055.2,630,1042.7,668,],
97:[595,517,277,542,350,542,422,507,457,416,454,302,384,227,309,180,217,142,151,120,56.7,126,6.3,176,-31.5,258,-6.3,359,78.8,450,180,542,318,614,444,438,129,419,56.7,425,6.3,466,-31.5,529,-22.1,608,22.1,702,126,769,205,],
101:[428,195,205,287,233,384,258,450,290,504,337,526,403,507,454,428,469,324,416,233,309,176,198,145,97.7,154,22.1,205,-25.2,287,-28.4,387,6.3,485,81.9,539,139,589,205,],
105:[284,369,595,350,586,353,558,378,561,391,583,369,595,,299,450,142,154,113,91.4,101,50.4,110,9.45,139,-22.1,205,-25.2,274,9.45,359,81.9,400,132,450,205,],
109:[866,302,454,50.4,-18.9,183,224,258,296,353,403,444,460,501,466,536,447,551,403,542,340,523,287,353,-18.9,517,258,592,337,668,428,724,460,781,472,838,454,850,403,850,350,775,220,721,120,693,53.6,699,3.15,747,-31.5,816,-22.1,907,37.8,973,107,1045.8,205,],
111:[476,211,-25.2,280,-9.45,378,50.4,472,161,529,280,551,375,532,438,485,466,419,466,337,422,268,362,208,290,170,208,145,135,132,59.9,148,3.15,173,-18.9,211,-25.2,],
116:[306,356,558,306,454,457,454,164,454,306,454,142,161,113,81.9,110,9.45,173,-34.6,246,-18.9,328,28.4,416,126,472,205,],
119:[690,302,450,117,107,104,59.9,104,15.8,123,-15.8,158,-31.5,205,-22.1,271,9.45,321,56.7,365,101,419,164,580,450,387,88.2,378,37.8,387,-3.15,416,-25.2,454,-28.4,504,-9.45,558,25.2,627,91.4,687,180,728,258,756,346,769,397,759,450,],
121:[536,299,450,142,154,110,69.3,107,12.6,129,-15.8,170,-31.5,230,-15.8,293,25.2,378,107,479,224,523,284,614,450,387,34.6,321,-81.9,268,-161,217,-211,158,-236,91.4,-243,34.6,-236,0,-198,-3.15,-151,18.9,-132,44.1,-151,34.6,-167,],},

EMSHerculean:{height:500,
66:[397,104,12.6,104,662,135,662,176,646,208,624,233,592,239,564,243,526,227,488,198,454,170,432,139,425,104,425,139,425,183,416,220,400,255,378,284,346,309,312,321,274,331,230,324,186,312,132,287,94.5,252,59.9,208,31.5,167,15.8,126,12.6,104,12.6,],
67:[665,586,53.6,498,12.6,425,3.15,312,15.8,227,59.9,158,123,101,208,75.6,290,75.6,365,97.6,466,135,532,186,583,243,627,318,662,400,674,469,668,532,649,589,621,],

//Width was 958
87:[900,59.9,662,334,25.2,617,662,,343,662,621,22.1,904,665,],

97:[551,466,211,463,255,425,318,365,375,287,400,205,391,139,353,91.4,296,66.1,236,69.3,167,94.5,97.6,139,47.2,211,9.45,280,3.15,372,31.5,425,85.1,457,139,469,186,469,394,469,12.6,],
101:[482,132,63,413,337,365,381,306,400,236,397,173,375,113,328,78.8,268,69.3,198,78.8,139,104,94.5,148,47.2,189,18.9,227,3.15,274,-3.15,340,12.6,394,44.1,422,72.4,],
105:[198,101,12.6,101,391,,91.4,513,94.5,536,117,536,117,513,91.4,513,],
109:[715,88.2,15.8,88.2,394,88.2,315,129,359,180,394,230,406,277,394,321,359,343,324,356,290,356,12.6,356,296,375,340,416,381,485,403,542,397,589,362,617,309,617,15.8,],
111:[532,66.1,208,85,284,132,350,192,387,249,400,312,397,369,372,425,328,460,258,463,176,447,110,400,53.6,334,9.45,274,0,211,9.45,151,40.9,101,91.4,69.3,158,66.1,208,],
116:[258,208,6.3,170,12.6,132,34.6,101,66.1,81.9,107,78.8,151,78.8,387,205,387,78.8,387,78.8,551,],
119:[655,63,394,239,18.9,416,394,,246,397,422,18.9,592,391,],
121:[501,78.8,394,78.8,161,88.2,117,120,59.9,176,15.8,236,0,293,3.15,353,40.9,394,88.2,416,135,416,394,416,-6.3,397,-66.1,362,-113,315,-148,261,-167,208,-167,148,-142,113,-117,94.5,-94.5,],},

EMSOsmotron:{height:500,
66:[756,129,595,639,595,706,529,706,372,668,315,170,315,135,350,135,284,170,315,668,315,724,261,724,59.9,668,3.15,126,3.15,129,595,],
67:[718,750,602,186,602,126,542,126,63,186,3.15,753,3.15,],
87:[1067.8,117,621,353,-15.8,589,627,844,-25.2,1080.4,630,],
97:[611,101,476,545,476,598,425,598,6.3,173,6.3,123,56.7,123,236,592,236,],
101:[617,624,3.15,176,3.15,126,56.7,126,422,180,479,545,479,598,425,598,243,123,243,],
105:[145,126,-22.1,123,-15.8,123,495,,107,633,107,674,148,674,148,633,107,633,],
109:[885,123,-15.8,123,472,460,472,507,428,507,-25.2,507,428,551,472,835,472,885,422,885,-22.1,],
111:[633,145,413,145,63,198,6.3,564,6.3,614,56.7,614,435,570,482,208,482,145,413,],
116:[346,126,680,126,472,387,472,126,472,126,50.4,161,12.6,387,12.6,],
119:[973,120,491,324,3.15,542,495,778,9.45,970,501,],
121:[595,186,-211,539,-211,595,-154,595,501,595,40.9,558,6.3,158,6.3,104,59.9,104,498,],},

EMSPepita:{height:500,
66:[529,56.7,148,31.5,217,15.8,321,25.2,428,47.2,504,78.8,558,123,602,167,611,211,602,239,548,236,485,208,403,202,343,217,293,258,280,328,271,410,246,447,198,435,120,372,59.9,277,12.6,208,3.15,158,12.6,120,37.8,],
67:[501,387,583,324,611,217,602,123,532,63,450,31.5,353,22,243,41,139,97.6,50.4,195,3.15,318,12.6,410,69.3,],
87:[939,22,513,25.2,362,44.1,224,88.2,113,139,28.4,189,3.15,227,25.2,369,406,397,447,425,463,472,460,491,410,548,81.9,564,50.4,595,31.5,624,40.9,649,85.1,734,340,794,498,850,608,],
97:[454,306,271,271,324,173,337,63,268,3.15,158,0,66.1,69.3,-9.45,205,37.8,261,81.9,337,151,340,69.3,365,12.6,394,3.15,422,22.1,454,78.8,],
101:[318,6.3,75.6,123,104,208,189,217,268,192,321,101,343,-6.3,271,-28.4,158,-12.6,91.4,72.5,12.6,186,3.15,299,44.1,,302,47.2,321,78.8,],
105:[230,18.9,343,-9.45,192,0,72.4,72.5,3.15,173,18.9,205,34.6,230,78.8,,63,586,94.5,536,],
109:[1089.9,1089.9,78.8,1071.0,47.2,1004.8,3.15,964,3.15,923,37.8,917,167,907,236,841,277,806,249,750,189,674,59.9,608,6.3,551,12.6,529,66.1,529,192,517,274,463,312,419,299,378,255,255,59.9,195,37.8,148,53.6,135,113,129,296,104,318,88.2,265,50.4,167,25.2,126,-3.15,75.6,],
111:[400,41,277,3.15,205,15.8,97.7,59.9,28.4,189,0,309,63,372,176,356,271,277,334,183,337,145,315,],
116:[375,145,677,126,186,154,75.6,220,3.15,299,15.8,356,44.1,375,78.8,,15.8,343,158,340,296,362,],
119:[951,154,309,145,126,173,44.1,249,3.15,340,44.1,422,142,517,312,523,183,558,81.9,608,66.1,706,107,803,192,869,280,885,353,891,236,929,186,],
121:[520,22,302,15.8,198,31.5,94.5,132,31.5,252,85.1,315,180,337,246,337,324,425,-78.8,432,-202,422,-321,369,-413,296,-413,252,-353,271,-239,359,-107,444,-18.9,472,9.45,501,44.1,520,78.8,],},

EMSQwandry:{height:500,
66:[810,350,614,198,362,117,167,69.3,22,63,-88.2,94.5,-139,,246,3.15,362,-37.8,532,-9.45,630,69.3,674,161,643,236,554,293,476,306,403,321,324,318,324,306,510,362,690,463,753,539,765,630,718,687,567,721,394,718,224,677,-9.45,602,],
67:[784,781,658,671,702,479,687,299,598,161,476,56.7,337,22,202,28.4,94.5,104,-9.45,220,-63,365,-53.5,510,-6.3,598,44.1,680,120,,696,460,712,520,756,614,797,684,841,734,],
87:[1134.0,580,693,378,564,173,353,47.2,173,-6.3,37.8,0,-50.4,34.6,-97.6,113,-97.6,217,-41,331,66.1,545,302,712,466,699,261,699,113,731,22,788,-22,854,-12.6,914,66.1,976,205,992,441,973,517,951,548,926,576,895,583,],
97:[447,400,359,359,406,277,419,186,372,72.5,255,3.15,139,-15.8,53.6,9.45,3.15,66.1,3.15,110,44.1,328,302,331,296,284,148,284,37.8,299,-9.45,343,-15.8,],
101:[372,101,202,180,224,255,290,277,337,277,384,249,410,205,416,132,378,47.2,274,-15.8,161,-28.4,85.1,-22,34.6,18.9,-6.3,97.6,-9.45,167,31.5,227,91.4,274,148,312,202,],
105:[268,126,400,63,265,28.4,135,15.8,12.6,37.8,-22,78.8,-12.6,129,25.2,,158,520,233,586,208,520,158,520,],
109:[693,63,450,75.6,450,117,435,132,400,120,331,15.8,25.2,164,277,239,381,309,410,346,378,350,299,277,25.2,362,120,532,315,573,328,539,208,523,72.4,542,6.3,570,3.15,],
111:[384,129,258,75.6,293,44.1,337,47.2,419,107,469,180,472,227,441,261,378,274,293,261,189,208,75.6,142,12.6,107,3.15,41,6.3,-6.3,59.9,-9.45,145,18.9,233,56.7,280,110,324,154,353,205,372,280,372,328,362,],
116:[287,63,-18.9,28.4,-28.4,-3.15,-15.8,-12.6,37.8,9.45,120,158,454,170,520,170,576,151,592,117,586,,41,403,132,378,217,384,280,400,334,422,],
119:[699,170,378,53.5,205,-3.15,88.2,-9.45,34.6,18.9,0,66.1,9.45,107,47.2,208,173,315,324,337,328,334,230,343,129,365,56.7,397,12.6,441,3.15,504,40.9,570,145,592,243,592,312,580,362,548,410,],
121:[466,69.3,450,107,444,123,419,123,381,75.6,252,-9.45,15.8,75.6,135,176,255,255,334,306,397,331,416,381,378,356,170,249,-123,151,-318,31.5,-482,-85.1,-602,-145,-652,-236,-658,-284,-633,-302,-558,-236,-413,-113,-258,28.4,-123,132,-41,198,15.8,],},

EMSReadability:{height:500,
66:[602,129,28.4,129,646,312,646,365,643,419,621,463,580,476,532,476,476,454,425,406,391,365,372,309,362,135,362,312,362,365,353,428,337,479,302,507,271,517,227,517,164,491,104,447,66.1,369,34.6,312,28.4,135,28.4,],
67:[589,520,97.6,454,47.2,365,15.8,277,28.4,195,72.4,132,148,101,227,91.4,346,101,457,142,548,205,617,277,652,359,658,432,643,482,611,498,592,],
87:[791,66.1,655,214,18.9,227,22.1,391,652,403,652,570,18.9,583,18.9,724,655,],
97:[520,117,428,183,466,239,482,296,488,340,472,378,438,403,387,406,318,406,107,406,22.1,403,107,362,78.8,318,47.2,268,25.2,211,15.8,164,31.5,126,56.7,97.6,97.6,97.6,158,120,205,170,246,220,265,280,284,337,293,372,296,400,296,],
101:[504,88.2,265,435,265,435,321,419,394,372,450,324,482,258,485,192,466,135,419,113,372,88.2,312,81.9,249,97.6,173,126,104,183,47.2,236,22.1,302,15.8,359,31.5,413,59.9,],
105:[252,126,15.8,126,479,,97.6,671,97.6,621,148,621,148,671,97.6,671,],
109:[835,123,15.8,123,479,123,375,167,413,208,447,246,472,293,485,334,479,362,466,394,435,416,381,422,315,422,15.8,422,365,435,391,501,450,551,476,592,488,636,476,665,460,699,419,715,337,715,18.9,],
111:[561,287,18.9,290,18.9,353,34.6,413,75.6,450,139,466,208,469,274,460,334,438,387,406,428,362,466,315,482,271,485,220,476,173,447,132,400,110,356,91.4,290,91.4,239,97.6,180,113,126,151,75.6,195,40.9,243,22.1,287,18.9,],
116:[334,299,37.8,265,22.1,220,22.1,180,34.6,145,75.6,139,145,139,469,47.2,469,290,469,139,469,139,611,],
119:[702,59.9,482,198,9.45,211,9.45,343,482,362,482,495,9.45,507,9.45,643,482,],
121:[460,53.5,479,243,3.15,,406,476,211,-78.8,189,-126,158,-161,120,-183,53.5,-183,],},

EMSReadabilityItalic:{height:500,
66:[602,148,28.4,258,646,441,646,495,643,545,621,583,580,586,532,573,476,545,425,491,391,447,372,391,362,214,362,394,362,444,353,504,337,548,302,570,271,573,227,561,164,526,104,472,66.1,391,34.6,331,28.4,158,28.4,],
67:[589,551,97.6,476,47.2,384,15.8,296,28.4,224,72.4,173,148,154,227,167,346,195,457,252,548,328,617,406,652,491,658,564,643,605,611,617,592,],
87:[791,198,655,233,18.9,246,22.1,523,652,536,652,589,18.9,602,18.9,857,655,],
97:[520,208,428,280,466,340,482,400,488,441,472,472,438,485,387,476,318,438,107,425,22.1,438,107,391,78.8,340,47.2,287,25.2,230,15.8,183,31.5,151,56.7,129,97.6,139,158,173,205,230,246,284,265,346,284,403,293,441,296,469,296,],
101:[504,151,265,498,265,507,321,504,394,469,450,425,482,359,485,290,466,227,419,192,372,158,312,142,249,142,173,158,104,205,47.2,255,22.1,321,15.8,378,31.5,441,59.9,],
105:[252,145,15.8,227,479,,233,671,224,621,274,621,284,671,233,671,],
109:[835,142,15.8,224,479,205,375,255,413,302,447,346,472,394,485,435,479,460,466,488,435,498,381,491,315,438,15.8,501,365,520,391,595,450,652,476,693,488,734,476,762,460,788,419,788,337,731,18.9,],
111:[561,306,18.9,309,18.9,375,34.6,441,75.6,488,139,520,208,532,274,532,334,523,387,501,428,460,466,416,482,375,485,321,476,268,447,220,400,189,356,158,290,151,239,145,180,151,126,180,75.6,217,40.9,261,22.1,306,18.9,],
116:[334,321,37.8,284,22.1,239,22.1,198,34.6,173,75.6,180,145,239,469,145,469,387,469,239,469,265,611,],
119:[702,164,482,214,9.45,227,9.45,444,482,463,482,513,9.45,526,9.45,743,482,],
121:[460,154,479,258,3.15,,507,476,214,-78.8,183,-126,142,-161,101,-183,34.6,-183,],},

EMSSpaceRocks:{height:500,
97:[512,0.00,0.00,0.00,372.36,186.18,558.55,372.36,372.36,372.36,0.00,,0.00,186.18,372.36,186.18,],
66:[512,0.00,0.00,0.00,558.55,186.18,558.55,372.36,465.45,186.18,279.27,372.36,93.09,186.18,0.00,0.00,0.00,],
67:[512,372.36,0.00,0.00,0.00,0.00,558.55,372.36,558.55,],
101:[512,372.36,0.00,0.00,0.00,0.00,558.55,372.36,558.55,,0.00,279.27,279.27,279.27,],
105:[512,0.00,0.00,372.36,0.00,,186.18,0.00,186.18,558.55,,0.00,558.55,372.36,558.55,],
109:[512,0.00,0.00,0.00,558.55,186.18,372.36,372.36,558.55,372.36,0.00,],
111:[512,0.00,0.00,0.00,558.55,372.36,558.55,372.36,0.00,0.00,0.00,],
116:[512,0.00,558.55,372.36,558.55,,186.18,558.55,186.18,0.00,],
87:[512,0.00,558.55,93.09,0.00,186.18,186.18,279.27,0.00,372.36,558.55,],
119:[512,0.00,558.55,93.09,0.00,186.18,186.18,279.27,0.00,372.36,558.55,],
121:[512,0.00,558.55,186.18,279.27,372.36,558.55,,186.18,279.27,186.18,0.00,],},
}