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
let fontIdx     =    0;
let text        = 'Bowie';
let fontsizeAdj =    1;
let vertOfs     = -5.5;
let horizOfs    = -1.5;
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
  // if(debug)  showVec('enter backUpPoint, prevVec', prevVec);
  // if(debug)  showVec('                   vec    ', vec);
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

const addHull = (vec, dbg = '') => {
  showVec(' - add hull, '+dbg, vec);
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

const addHole = (tailPoint, headPoint, dbg='') => {
  showVec(' - add hole, '+dbg, [tailPoint, headPoint]);
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
  // if(debug) showVec('entering chkSharpBend, lastVec:',lastVec);
  // if(debug) showVec('                          vec:',vec);
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
      addHole(p1,p2, 'vectors up or down');
      addHole(p3,p2, '                  ');
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
        addHole(p1,p2, '> MAX_ANGLE');
        addHole(p3,p2, '           ');
      }
      return;
    }
  }
  // if(debug) console.log('before angle calc', {x1,y1,x2,y2});
  const rad2deg = 360 / (2*Math.PI);
  let angle1  = Math.atan(y1/x1)* rad2deg
  let angle2  = Math.atan(y2/x2)* rad2deg;
  if(x1 < 0) angle1 += 180;
  if(x2 < 0) angle2 += 180;
  let minAngle = Math.min();
  for(let i=-1; i <= +1; i++) {
    for(let j=-1; j <= +1; j++) {
      const a1  = angle1 + i*360;
      const a2  = angle2 + j*360;
      const dif = Math.abs(a2 - a1);
      minAngle  = Math.min(dif,minAngle);
    }
  }
  let angle = minAngle;
  if(debug) console.log('angle', {angle});
  if(angle > MAX_ANGLE) {
    if(debug) console.log(
      `bend too sharp, adding 2 holes at ${p2[0].toFixed(1)},${p2[1].toFixed(1)}`);
    addHole(p1,p2, 'bend too sharp');
    addHole(p3,p2, '              ');
  }
  return;
}

let lastVec = null;
let prevVecs = [];

const chkTooClose = (vec, first) => {

  // ------ check ends touching  ------
  // check if vec is extending last, vec tail == lastVec.head
  // if(debug) showVec('checking vec extension, lastVec', lastVec);
  // if(debug) showVec('                            vec', vec);
  const extendingLastVec = lastVec && (ptEq(vec[0], lastVec[1]));
  if(extendingLastVec) {
    if(debug) console.log('vec is extending last');
    chkSharpBend(lastVec, vec);
  }

  // let prevVecsTmp = (first ? prevVecs : prevVecs.slice(0,-1));
  // checking against all previous vecs (slow way)
  for(const prevVec of prevVecs) {
    // if(debug) showVec('- checking prev vec', prevVec);

    // // check exact match either direction with prev vec
    // // if(debug)  showVec('check exact match, old vec', prevVec);
    // // if(debug)  showVec('                   new vec', vec);
    // if(vecEq(prevVec, vec) || vecRevEq(prevVec, vec)) {
    //   if(debug) showVec('vec exactly matches prevVec', prevVec);
    //   return {  // skip vec
    //     headClose:true, tailClose:true, vec1:null, vec2: null};
    // }

    // // if(debug) console.log('vec is not extending last');
    // if(ptTouchesEnd(vec[0],prevVec)) {
    //   // vec tail is touching prevVec head or tail
    //   if(debug)  showVec('tail touches prev', prevVec);
    //   const backUpPnt = backUpPoint(prevVec, vec, false);
    //   if(!backUpPnt) return {
    //     headClose:false, tailClose:true, vec1:null, vec2:null};
    //   return {
    //     headClose:false, tailClose:true, 
    //     vec1:[backUpPnt, vec[1]], vec2:null};
    // }

    // if(ptTouchesEnd(vec[1],prevVec)) {
    //   // vec head is touching prevVec head or tail
    //   if(debug)  showVec('head touching prev', prevVec);
    //   const backUpPnt = backUpPoint(prevVec, vec, true);
    //   if(!backUpPnt) return {
    //     headClose:true, tailClose:false, vec1:null, vec2:null};
    //   return {
    //     headClose:true, tailClose:false, 
    //     vec1:[vec[0], backUpPnt], vec2:null};
    // }

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

      // starting tail dist chk'
      const dist2prev = distPntToVec(vec[0], prevVec);
      if(dist2prev < (2 * radius)) {
        // tail end point too close to an prev vec
        // back up to point on vec far enough away
        let vec1 = null;
        const backUpPt = backUpPoint(prevVec, vec, false);
        console.log('tail too close to prev vec');
        if(backUpPt)
          console.log(`backUpPoint result ${backUpPt[0].toFixed(1)},` +
                                         `${backUpPt[0].toFixed(1)}`);
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
    if(debug) showVec('lastVec:', lastVec);

    if(vecEq(prevVec, lastVec)) {
      // if (debug) console.log(
        // 'prevVec == lastVec, skipping head dist check to last vec');
    }
    else {
      // starting head dist chk
      const dist2prev = distPntToVec(vec[1], prevVec);
      // if (debug) console.log('head dist is', dist2prev);
      if(dist2prev < (2 * radius)) {
        // head end point too close to an old vec
        // back up to point on vec far enough away
        let vec1 = null;
        const backUpPt = backUpPoint(prevVec, vec, true);
        console.log('head too close to old vec');
        if(backUpPt) {
          vec1 = [vec[0], backUpPt];
          if(debug)  showVec('new vec', vec1);
          return {headClose:true, tailClose:false, vec1, vec2: null};
        }
        else {
          if (debug) console.log('both ends too close');
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
  if(ptIdx > 0 && ptEq(point, lastPoint)) {
    showVec("ERROR: handlePoint, point == lastPoint", [point, lastPoint]);
    return 1;
  }
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
    if(vec1 == null) {
      if(debug) showVec('both ends too close, skipping vec', vec);
      lastPoint = null;
      if(debug) showVec('checking lastVec', lastVec);
      if(lastVec)
        addHole(lastVec[0], lastVec[1], 'last hole in lastVec');
      return 0; // next ptIdx is 0
    }

    // ---- an end was too close
    if(ptIdx == 1 || tailClose) 
      addHole(point, vec1[0], 'first hole'); // add first hole
    addHull(vec1, 'end too close');
    if(debug) showVec('adding to prevVecs vec1', vec1);
    prevVecs.push(vec1);
    lastVec = vec1;
    if(debug) showVec('setting lastVec:', lastVec);

    if(vec2) {
      // had intersection
      // vec was split into vec1 and vec2
      addHole(vec1[0], vec1[1], 'vec was split');

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
      addHole(vec1[0], vec1[1], 'last hole'); // add last hole
    lastPoint = vec[1];
    return ptIdx + 1; // next ptIdx
  }

  // ---- point was not too close
  // if(debug) showVec('  -- not too close', vec);
  if(ptIdx == 1) addHole(point, lastPoint, 'first hole'); // add first hole
  addHull(vec);
  if(lastPtInSeg) addHole(lastPoint, point, 'last hole'); // add last hole
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
      step: .2, caption: 'Font Size:' 
    },
    { name: 'vertOfs', type: 'number', 
      initial: vertOfs, min: -plateH, max: plateH, 
      step: -10, caption: 'Vertical Offset:' 
    },
    { name: 'horizOfs', type: 'number', 
      initial: horizOfs, min: -plateW, max: plateW, 
      step: .25, caption: 'Horizontal Offset:' 
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
  const {fontIdx, text, 
         vertOfs, horizOfs, fontsizeAdj} = params;
  const fontKey  = Object.keys(fonts)[fontIdx];
  const font     = fonts[fontKey];
  const fontName = Object.keys(fonts)[fontIdx];
  console.log(`using font ${fontName}`);

  const patchChars = fontPatch?.[fontName];
  if(patchChars) {
    console.log(`---- found fontPatch[${fontName}]`);
    for(const ascii in font ) {
      if(!ascii || ascii === 'height') continue;
      const charStr   = String.fromCharCode(ascii);
      const fontPath  = font[ascii];
      const patchList = patchChars[ascii];
      if(patchList?.length) {
        // console.log(patchList.length, {patchList});
        for (const patch of patchList) {
          [fromArr, toArr] = patch;
          for(let i=0; i <= (fontPath.length - fromArr.length); i++) {
            let j; for(j = 0; j < fromArr.length; j++)
              if(fromArr[j] != fontPath[i+j]) break;
            if(j == fromArr.length) {
              fontPath.splice(i, fromArr.length, ...toArr);
              console.log(`patched fontPath for char ${charStr}`, fontPath);
              break;
            }
          }
        }
      }
    }
  }
  switch(params.show) {
    case 0: genHulls = true; genHoles = true; 
            genPlate = true; break;
    case 1: genHulls = true; genHoles = true; 
            genPlate = false; break;
    case 2: genHulls = true; genHoles = false; 
            genPlate = false; break;
  }
  console.log("\n\n---- main ----\n");

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
  let xOfs   = (plateW - strWidth)/2  - plateW/2 + horizOfs;
  let yOfs   = (plateH - strHeight)/2 - plateH/2 + vertOfs;

  if (debugScale) {
    textScale = 1;
    xOfs      = 0;
    yOfs      = 0
  }

  let xOffset = 0;
  for(const char of text) {
    prevVecs = [];
    const charRes = vectorChar({font, xOffset}, char);
    console.log({charRes});
    let {width, segments} = charRes;

    console.log({font, width, segments});

    if(segments.length == 0) {
      console.log(`\n\n---- Error: character ${char} not in font ----\n\n`);
      continue;
    }
    console.log("\n======== CHAR: " + char + 
                ', segment count:', segments.length);
    xOffset += width;
    segments.forEach( (seg) => {
      let ptIdx  = 0;
      seg.every( (point, segIdx) => {
        point[0] *= textScale;
        point[1] *= textScale;
        do {
          if(ptIdx == 0)
            console.log("\n--- seg ---, points remaining: ", 
                         seg.length - segIdx);
          ptIdx = handlePoint(point, ptIdx, segIdx == seg.length-1);
          // console.log('handlePoint returned:', {ptIdx});
        } while (ptIdx === 0);
        return (ptIdx !== null);
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

const fontPatch = {  

EMSHerculean:{
// W
  87:[[[ 958, 
  59.9,662, 334,25.2, 617,662, , 343,662, 621,22.1, 904,665,  
  ],[    958-40,  
  59.9,662, 334-40,25.2, , 
  334,25.2, 617,662, , 
  343,662, 621-40,22.1, ,
  621,22.1, 904,665, 
]]],

// y
  121:[ [ [416,-6.3,],[, 416,-6.3+95,] ] ],

// a
  97:[[[  
  466,211, 463,255, 425,318, 365,375, 287,400, 205,391, 139,353, 91.4,296, 66.1,236, 69.3,167, 94.5,97.6, 139,47.2, 211,9.45, 280,3.15, 372,31.5, 425,85.1, 457,139, 469,186, 
],[  
  /* 466,211, */ 463,255, 425,318, 365,375, 287,400, 205,391, 139,353, 91.4,296,  66.1,236,  69.3,167, 94.5,97.6, 139,47.2, 211,9.45, 280,3.15, 372,31.5, 425,85.1, 457,139, /*469,186*/ , ]]],

// t
  116:[[[  
  208,6.3, 170,12.6, 132,34.6, 101,66.1, 81.9,107, 78.8,151, 78.8,387, 205,387, 78.8,387, 78.8,551,  
],[  
  208,6.3, 170,12.6, 132,34.6, 101,66.1, 81.9,107, 78.8,151, 
  78.8,387, /*205,387, ,78.8,387, */
  78.8,551, , 205,387, 78.8,387, 
]]],
},
camBamStick9Font: {
/* C */ 67: [[[
 50,85, 43,85, 40,85, 37,84, 33,82, 28,79, 25,77, 21,73, 17,69, 15,66, 13,61, 11,56, 10,52, 10,47, 11,41, 11,38, 12,35, 14,31, 16,26, 18,23, 22,19, 26,16, 29,14, 33,12, 38,10, 41,9, 46,9, 53,9
],[]]],

/* a */ 97: [[[ 43,10, 41,9, 40,9, 39,8, 38,8, 34,8, 30,8, 27,8, 24,10, 20,11, 18,13, 16,15, 13,18, 12,20, 11,23, 9,27, 9,30, 9,34, 9,38, 9,41, 11,44, 12,48, 13,50, 16,52, 18,55, 20,56, 24,58, 27,59, 30,60, 34,60, 38,60, 40,59, 44,58, 47,57, 49,55, 52,53, 53,52, 54,51, 55,49, 57,46, 58,43, 58,41, 58,37, 58,9, 58,37, 58,41, 58,43, 57,46, 55,49, 54,51, 53,52, 52,53, 49,55, 47,57, 44,58, 40,59, 38,60,
],[58,9, 58,37, 58,41, 58,43, 57,46, 55,49, 54,51, 53,52, 52,53, 49,55, 47,57, 44,58, 40,59, 38,60, ]]],

/* m */ 109: [[[ 45,40, 45,45, 45,47, 45,48, 46,49, 46,51, 48,52, 49,54, 52,57, 53,58, 54,59, 55,59, 56,59, 62,59, 64,59, 66,59, 68,58, 70,57, 72,56, 73,55, 75,53, 76,52, 77,50, 77,48, 78,46, 78,43, 78,9, 78,43, 78,46, 77,48, 77,50, 76,52, 75,53, 73,55, 72,56, 70,57, 68,58, 66,59, 64,59, 62,59, 58,59, 56,59, 54,59, 53,58, 52,58, 51,56, 49,54, 47,52, 46,51, 46,50, 45,49, 45,47, 45,40, 45,9, 45,40, 45,45, 44,47, 44,48, 44,49, 43,51, 42,52, 40,54, 38,57, 37,58, 36,58, 35,59, 34,59, 33,59, 27,59, 25,59, 23,59, 21,58, 19,57, 18,56, 16,55, 15,54, 14,53, 14,52, 13,50, 12,48, 12,46, 12,43, 12,9
],[ , 45+1,48+1, 49,54, 52,57, 53,58, 54,59, 55,59, 56,59, 62,59, 64,59, 66,59, 68,58, 70,57, 72,56, 73,55, 75,53, 76,52, 77,50, 77,48, 78,46, 78,43, 78,9]]],

/* e */ 101: [[[
59,26, 57,21, 57,19, 56,18, 55,17, 54,16, 50,12, 46,10, 45,9, 43,8, 41,8, 35,8, 31,8, 28,8, 24,10, 21,11, 19,13, 16,15, 14,18, 13,20, 11,23, 10,27, 9,29, 9,34, 9,38, 10,41, 11,44, 13,48, 14,50, 16,52, 19,55, 21,56, 24,58, 28,59, 31,59, 35,59, 38,59, 40,59, 42,59, 43,58, 48,56, 51,54, 52,53, 53,53, 54,52, 54,51, 56,47, 26,31, 56,47, 55,50, 54,52, 53,52, 53,53, 52,54, 48,56, 45,58, 43,59, 41,59, 40,59, 35,59, 31,59, 28,59, 24,58, 21,56, 19,55, 16,52, 14,50, 13,48, 11,44, 10,41, 9,38, 9,34, 9,29, 10,27, 11,23, 13,20, 
],[ 26,31, 56,47, 55,50, 54,52, 53,52, 53,53, 52,54, 48,56, 45,58, 43,59, 41,59, 40,59, 35,59, 31,59, 28,59, 24,58, 21,56, 19,55, 16,52, 14,50, 13,48, 11,44, 10,41, 9,38, 9,34, 9,29, 10,27, 11,23, 13,20, 
]]],

/* r */ 114: [[[
11,9, 11,33, 11,38, 11,41, 12,44, 13,48, 14,50, 16,52, 18,54, 19,55, 20,56, 22,57, 25,58, 26,59, 29,59, 26,59, 25,58, 22,57, 20,56, 19,56, 18,55, 16,52, 14,50, 13,48, 12,44, 11,41, 11,38, 11,33, 11,9,
/* segment */, 11,9, 11,58, 11,9
],[ 
13,48, 14,50, 16,52, 18,54, 19,55, 20,56, 22,57, 25,58, 26,59, 
/* segment */, 11,58, 11,9
]]],

/* o */ 111:  [[[
9,34, 9,38, 10,41, 11,44, 13,48, 14,50, 16,52, 19,55, 21,56, 24,58, 28,59, 30,59, 34,59, 38,59, 41,59, 44,58, 48,56, 50,55, 52,52, 55,50, 56,48, 58,44, 59,41, 59,38, 60,34, 59,29, 59,27, 58,23, 56,20, 55,18, 52,15, 50,13, 48,11, 44,10, 41,8, 38,8, 34,8, 30,8, 28,8, 24,10, 21,11, 19,13, 16,15, 14,18, 13,20, 11,23, 10,27, 9,29, 9,34,
/* segment */, 9,34, 9,29, 10,27, 11,23, 13,20, 14,18, 16,15, 19,13, 21,11, 24,10, 28,8, 30,8, 34,8, 38,8, 41,8, 44,10, 48,11, 50,13, 52,15, 55,18, 56,20, 58,23, 59,27, 59,29, 60,34, 59,38, 59,41, 58,44, 56,48, 55,50, 52,52, 50,55, 48,56, 44,58, 41,59, 38,59, 34,59, 30,59, 28,59, 24,58, 21,56, 19,55, 16,52, 14,50, 13,48, 11,44, 10,41, 9,38, 9,34
],[ 
9,34-2, 9,29, 10,27, 11,23, 13,20, 14,18, 16,15, 19,13, 21,11, 24,10, 28,8, 30,8, 34,8, 38,8, 41,8, 44,10, 48,11, 50,13, 52,15, 55,18, 56,20, 58,23, 59,27, 59,29, 60,34, 59,38, 59,41, 58,44, 56,48, 55,50, 52,52, 50,55, 48,56, 44,58, 41,59, 38,59, 34,59, 30,59, 28,59, 24,58, 21,56, 19,55, 16,52, 14,50, 13,48, 11,44, 10,41, 9,38, 9,34
]]]
},

"SLFWhiteLinen-Regular": {

/* b */ 98: [[[
23,30, 23.00,130.00, 29.97,150.70, 36.88,171.20, 43.73,191.50, 50.52,211.60, 57.25,231.50, 63.92,251.20, 70.53,270.70, 77.08,290.00, 83.57,309.10, 90.00,328.00, 96,344, 173.00,565.00, 195.78,606.23, 219.32,643.12, 243.62,675.67, 268.68,703.88, 294.50,727.75, 321.08,747.28, 348.42,762.47, 376.52,773.32, 405.38,779.83, 435.00,782.00, 483.00,782.00, 490.03,777.45, 496.32,772.60, 501.87,767.45, 506.68,762.00, 510.75,756.25, 514.08,750.20, 516.67,743.85, 518.52,737.20, 519.63,730.25, 520.00,723.00, , 44,179, 65.00,230.00, 78.71,242.92, 92.24,254.48, 105.59,264.68, 118.76,273.52, 131.75,281.00, 144.56,287.12, 157.19,291.88, 169.64,295.28, 181.91,297.32, 194.00,298.00, 235.00,298.00, 237.66,291.59, 240.04,285.16, 242.14,278.71, 243.96,272.24, 245.50,265.75, 246.76,259.24, 247.74,252.71, 248.44,246.16, 248.86,239.59, 249.00,233.00, 249.00,147.00, 230.02,118.88, 211.08,93.72, 192.18,71.52, 173.32,52.28, 154.50,36.00, 135.72,22.68, 116.98,12.32, 98.28,4.92, 79.62,0.48, 61.00,-1.00, 42.00,-1.00, 38.39,0.39, 35.16,2.16, 32.31,4.31, 29.84,6.84, 27.75,9.75, 26.04,13.04, 24.71,16.71, 23.76,20.76, 23.19,25.19, 23.00,30.00
],[ 

23,30, 23.00,130.00, 29.97,150.70, 36.88,171.20, 43.73,191.50, 50.52,211.60, 57.25,231.50, 63.92,251.20, 70.53,270.70, 77.08,290.00, 83.57,309.10, 90.00,328.00, 96,344, 173.00,565.00, 195.78,606.23, 219.32,643.12, 243.62,675.67, 268.68,703.88, 294.50,727.75, 321.08,747.28, 348.42,762.47, 376.52,773.32, 405.38,779.83, 435.00,782.00, 483.00,782.00, 490.03,777.45, 496.32,772.60, 501.87,767.45, 506.68,762.00, 510.75,756.25, 514.08,750.20, 516.67,743.85, 518.52,737.20, 519.63,730.25, 520.00,723.00, , 44,179, 65.00,230.00, 78.71,242.92, 92.24,254.48, 105.59,264.68, 118.76,273.52, 131.75,281.00, 144.56,287.12, 157.19,291.88, 169.64,295.28, 181.91,297.32, 194.00,298.00, 235.00,298.00, 237.66,291.59, 240.04,285.16, 242.14,278.71, 243.96,272.24, 245.50,265.75, 246.76,259.24, 247.74,252.71, 248.44,246.16, 248.86,239.59, 249.00,233.00, 249.00,147.00, 230.02,118.88, 211.08,93.72, 192.18,71.52, 173.32,52.28, 154.50,36.00, 135.72,22.68, 116.98,12.32, 98.28,4.92, 79.62,0.48, 61.00,-1.00, 42.00,-1.00, 38.39,0.39, 35.16,2.16, 32.31,4.31, 29.84,6.84, 27.75,9.75, 26.04,13.04, 24.71,16.71, 23.76,20.76, 23.19,25.19, 23.00,30.00

]]]
}
}

// /* e */ 101:[236, 205,64, 175.00,33.00, 167.75,26.35, 160.40,20.40, 152.95,15.15, 145.40,10.60, 137.75,6.75, 130.00,3.60, 122.15,1.15, 114.20,-0.60, 106.15,-1.65, 98.00,-2.00, 47.00,-2.00, 43.77,8.64, 40.88,18.96, 38.33,28.96, 36.12,38.64, 34.25,48.00, 32.72,57.04, 31.53,65.76, 30.68,74.16, 30.17,82.24, 30.00,90.00, 30.00,156.00, 40.54,182.79, 51.76,206.76, 63.66,227.91, 76.24,246.24, 89.50,261.75, 103.44,274.44, 118.06,284.31, 133.36,291.36, 149.34,295.59, 166.00,297.00, 190.00,297.00, 194.37,294.63, 198.28,291.92, 201.73,288.87, 204.72,285.48, 207.25,281.75, 209.32,277.68, 210.93,273.27, 212.08,268.52, 212.77,263.43, 213.00,258.00, 213.00,192.00, 188.59,186.79, 165.76,181.96, 144.51,177.51, 124.84,173.44, 106.75,169.75, 90.24,166.44, 75.31,163.51, 61.96,160.96, 50.19,158.79, 40.00,157.00, ],

// /* i */ 105:[168, 139,68, 125.00,49.00, 115.61,39.50, 106.64,31.00, 98.09,23.50, 89.96,17.00, 82.25,11.50, 74.96,7.00, 68.09,3.50, 61.64,1.00, 55.61,-0.50, 50.00,-1.00, 34.00,-1.00, 33.05,2.60, 32.20,6.20, 31.45,9.80, 30.80,13.40, 30.25,17.00, 29.80,20.60, 29.45,24.20, 29.20,27.80, 29.05,31.40, 29.00,35.00, 29.00,100.00, 42.28,127.56, 54.92,153.44, 66.92,177.64, 78.28,200.16, 89.00,221.00, 99.08,240.16, 108.52,257.64, 117.32,273.44, 125.48,287.56, 133.00,300.00, , 169,430, 160.00,430.00, 158.86,428.77, 157.84,427.48, 156.94,426.13, 156.16,424.72, 155.50,423.25, 154.96,421.72, 154.54,420.13, 154.24,418.48, 154.06,416.77, 154.00,415.00, 154.00,406.00, 155.23,404.86, 156.52,403.84, 157.87,402.94, 159.28,402.16, 160.75,401.50, 162.28,400.96, 163.87,400.54, 165.52,400.24, 167.23,400.06, 169.00,400.00, 177.00,400.00, 178.14,401.23, 179.16,402.52, 180.06,403.87, 180.84,405.28, 181.50,406.75, 182.04,408.28, 182.46,409.87, 182.76,411.52, 182.94,413.23, 183.00,415.00, 183.00,424.00, 181.78,425.14, 180.52,426.16, 179.22,427.06, 177.88,427.84, 176.50,428.50, 175.08,429.04, 173.62,429.46, 172.12,429.76, 170.58,429.94, 169.00,430.00, ],

// /* o */ 111:[231, 170,290, 208.00,290.00, 210.85,282.27, 213.40,274.68, 215.65,267.23, 217.60,259.92, 219.25,252.75, 220.60,245.72, 221.65,238.83, 222.40,232.08, 222.85,225.47, 223.00,219.00, 223.00,150.00, 209.84,121.50, 196.36,96.00, 182.56,73.50, 168.44,54.00, 154.00,37.50, 139.24,24.00, 124.16,13.50, 108.76,6.00, 93.04,1.50, 77.00,0.00, 35.00,0.00, 33.48,9.06, 32.12,17.84, 30.92,26.34, 29.88,34.56, 29.00,42.50, 28.28,50.16, 27.72,57.54, 27.32,64.64, 27.08,71.46, 27.00,78.00, 27.00,150.00, 38.57,176.60, 50.71,200.40, 63.42,221.40, 76.70,239.60, 90.56,255.00, 104.99,267.60, 119.99,277.40, 135.56,284.40, 151.70,288.60, 168.41,290.00, ],

// /* w */ 119:[420, 331,260, 336.00,274.00, 338.81,275.71, 341.64,277.24, 344.49,278.59, 347.36,279.76, 350.25,280.75, 353.16,281.56, 356.09,282.19, 359.04,282.64, 362.01,282.91, 365.00,283.00, 394.00,283.00, 396.09,276.30, 397.96,269.80, 399.61,263.50, 401.04,257.40, 402.25,251.50, 403.24,245.80, 404.01,240.30, 404.56,235.00, 404.89,229.90, 405.00,225.00, 405.00,144.00, 397.49,129.09, 389.36,114.76, 380.61,101.01, 371.24,87.84, 361.25,75.25, 350.64,63.24, 339.41,51.81, 327.56,40.96, 315.09,30.69, 302.00,21.00, 289.00,12.00, 283.90,9.34, 279.00,6.96, 274.30,4.86, 269.80,3.04, 265.50,1.50, 261.40,0.24, 257.50,-0.74, 253.80,-1.44, 250.30,-1.86, 247.00,-2.00, 210.00,-2.00, 209.24,4.56, 208.56,11.04, 207.96,17.44, 207.44,23.76, 207.00,30.00, 206.64,36.16, 206.36,42.24, 206.16,48.24, 206.04,54.16, 206.00,60.00, 206.00,128.00, 208.34,142.69, 210.96,157.16, 213.86,171.41, 217.04,185.44, 220.50,199.25, 224.24,212.84, 228.26,226.21, 232.56,239.36, 237.14,252.29, 242.00,265.00, 192.00,172.00, 179.65,154.97, 167.40,137.88, 155.25,120.73, 143.20,103.52, 131.25,86.25, 119.40,68.92, 107.65,51.53, 96.00,34.08, 84.45,16.57, 73.00,-1.00, 94.00,66.00, 96.09,80.19, 97.96,94.36, 99.61,108.51, 101.04,122.64, 102.25,136.75, 103.24,150.84, 104.01,164.91, 104.56,178.96, 104.89,192.99, 105.00,207.00, 105.00,232.00, 103.71,242.45, 101.84,251.80, 99.39,260.05, 96.36,267.20, 92.75,273.25, 88.56,278.20, 83.79,282.05, 78.44,284.80, 72.51,286.45, 66.00,287.00, 46.00,287.00, 41.15,283.22, 36.60,279.48, 32.35,275.78, 28.40,272.12, 24.75,268.50, 21.40,264.92, 18.35,261.38, 15.60,257.88, 13.15,254.42, 11.00,251.00]
// }
// }



//=== Fonts injected by jscad-font-gen ===
const fonts = {
"Fontain152":{height:329.58984375,

/* B */ 66:[368.16, 74.29,-22.5, 74.29,704.29, 74.29,704.29, 110.12,701.44, 147.06,691.48, 183.25,675.09, 216.82,652.99, 245.92,625.87, 268.67,594.44, 283.22,559.39, 287.71,521.43, 287.71,521.43, 277.14,471.12, 252.04,431.27, 217.32,400.73, 177.90,378.35, 138.69,362.96, 104.60,353.40, 80.54,348.51, 71.43,347.14, 71.43,347.14, 114.83,344.63, 160.64,335.59, 206.29,320.34, 249.25,299.22, 286.94,272.58, 316.81,240.74, 336.30,204.06, 342.86,162.86, 342.86,162.86, 330.41,112.04, 299.23,71.83, 255.60,41.04, 205.80,18.49, 156.13,3.00, 112.87,-6.60, 82.31,-11.49, 70.72,-12.86, ],

/* b */ 98:[374.51, 75,261, 75.00,261.00, 78.34,271.64, 87.97,299.67, 103.35,339.22, 123.92,384.45, 149.13,429.49, 178.41,468.50, 211.22,495.62, 247.00,505.00, 247.00,505.00, 289.84,482.69, 317.61,427.31, 330.83,350.23, 330.06,262.85, 315.85,176.55, 288.74,102.72, 249.27,52.74, 198.00,38.00, 198.00,38.00, 172.34,44.42, 148.84,56.19, 127.90,71.39, 109.89,88.12, 95.21,104.45, 84.24,118.48, 77.37,128.30, 75.00,132.00, , 75,696, 75.00,696.00, 75.08,669.57, 75.28,599.14, 75.53,497.98, 75.75,379.38, 75.88,256.60, 75.84,142.92, 75.57,51.63, 75.00,-4.00, ],

/* e */ 101:[289.06, 63.49,344.11, 63.49,344.11, 105.43,344.66, 140.37,348.22, 168.81,354.50, 191.24,363.20, 208.17,374.02, 220.10,386.67, 227.53,400.84, 230.96,416.23, 230.96,416.23, 231.33,432.50, 229.24,448.95, 224.68,464.75, 217.62,479.08, 208.03,491.13, 195.90,500.06, 181.19,505.07, 163.90,505.33, 163.90,505.33, 127.58,482.43, 94.76,429.88, 68.28,357.57, 50.99,275.38, 45.73,193.21, 55.34,120.95, 82.67,68.49, 130.55,45.71, 130.55,45.71, 163.12,48.83, 191.19,62.29, 214.78,83.99, 233.88,111.87, 248.52,143.86, 258.70,177.88, 264.43,211.85, 265.73,243.70, ],

/* i */ 105:[134.77, 72,455, 72.00,455.00, 72.00,397.63, 72.00,340.25, 72.00,282.88, 72.00,225.50, 72.00,168.13, 72.00,110.75, 72.00,53.38, 72.00,-4.00, , 74.92,594.84, 74.92,594.84, 62.25,592.61, 53.40,586.72, 48.25,578.39, 46.63,568.85, 48.39,559.30, 53.40,550.97, 61.50,545.08, 72.54,542.85, 72.54,542.85, 83.76,545.05, 91.91,550.87, 96.93,559.11, 98.77,568.56, 97.37,578.05, 92.69,586.37, 84.66,592.34, 73.25,594.75, ],

/* o */ 111:[355.47, 192.74,504.74, 192.74,504.74, 130.54,486.18, 85.24,434.86, 57.02,361.60, 46.06,277.17, 52.53,192.37, 76.62,118.00, 118.50,64.85, 178.34,43.71, 178.34,43.71, 236.20,62.40, 279.05,114.07, 306.29,187.78, 317.35,272.59, 311.63,357.56, 288.56,431.75, 247.54,484.21, 188.00,504.00, ],

/* w */ 119:[545.41, 65,496, 65.00,496.00, 64.85,423.38, 65.26,345.75, 67.55,267.59, 73.02,193.34, 83.00,127.50, 98.78,74.51, 121.67,38.86, 153.00,25.00, 153.00,25.00, 186.56,34.74, 214.14,64.83, 236.30,112.10, 253.63,173.38, 266.68,245.48, 276.05,325.23, 282.29,409.47, 286.00,495.00, 286.00,495.00, 285.64,411.56, 286.14,329.03, 288.69,250.52, 294.53,179.09, 304.86,117.84, 320.91,69.85, 343.88,38.21, 375.00,26.00, 375.00,26.00, 408.48,36.55, 436.38,68.63, 459.02,118.32, 476.75,181.75, 489.88,255.02, 498.75,334.25, 503.68,415.54, 505.00,495.00, ],},
};

//=== End of injected fonts ===
