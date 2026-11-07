/* 3D pick-and-place workcell used as the project picker.
   A 6-axis industrial arm (J1..J6, spherical wrist, parallel-jaw gripper) with analytic inverse
   kinematics, rendered with a small dependency-free WebGL renderer (Blinn-Phong, planar shadows).
   window.initRobotCell3D(options) -> api, or null when WebGL is unavailable. Same options/api as
   the 2D cell in robot-cell.js, which the page uses as the fallback. */
(function () {
  'use strict';

  var reduce = matchMedia('(prefers-reduced-motion:reduce)').matches;
  var PI = Math.PI, cos = Math.cos, sin = Math.sin;

  // ---- arm and scene dimensions (1 unit is about 10 cm) -------------------------------------
  var H1 = 1.15, L2 = 2.5, L3 = 2.3, J4L = 0.45, L3E = L3 + J4L, LT = 1.3;
  var SAFE = 2.7, HOVER = 1.75, GRASP = 0.54, HANDLE_Z = -0.5, TH = 0.18, TOTE = 1.15;
  var RING = 3.9, PHIS = [-75, -50, -25, 0, 25, 50, 75];
  var STAGE = { x: 3.5, z: -1.6, h: 0.5 };
  var HOME = { psi: -PI / 2, rho: 2.6, y: 3.0 };
  var SPEED = { psi: 2.7, rho: 7.5, y: 6.5, g: 6 };

  var clamp = function (v, a, b) { return Math.min(b, Math.max(a, v)); };
  var sleep = function (ms) { return new Promise(function (r) { setTimeout(r, reduce ? 0 : ms); }); };

  // ---- matrices (column-major) --------------------------------------------------------------
  function I() { return new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]); }
  function mul(a, b) {
    var o = new Float32Array(16);
    for (var c = 0; c < 4; c++) for (var r = 0; r < 4; r++) {
      o[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] + a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
    }
    return o;
  }
  function T(x, y, z) { var m = I(); m[12] = x; m[13] = y; m[14] = z; return m; }
  function Sc(x, y, z) { var m = I(); m[0] = x; m[5] = y; m[10] = z; return m; }
  function RX(a) { var c = cos(a), s = sin(a); return new Float32Array([1, 0, 0, 0, 0, c, s, 0, 0, -s, c, 0, 0, 0, 0, 1]); }
  function RY(a) { var c = cos(a), s = sin(a); return new Float32Array([c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, 0, 0, 0, 1]); }
  function RZ(a) { var c = cos(a), s = sin(a); return new Float32Array([c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]); }
  function chain() { var m = arguments[0]; for (var i = 1; i < arguments.length; i++) m = mul(m, arguments[i]); return m; }
  function persp(fov, asp, n, f) {
    var t = 1 / Math.tan(fov / 2), m = new Float32Array(16);
    m[0] = t / asp; m[5] = t; m[10] = (f + n) / (n - f); m[11] = -1; m[14] = 2 * f * n / (n - f); return m;
  }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
  function norm(a) { var l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; }
  function lookAt(e, t, up) {
    var f = norm(sub(t, e)), r = norm(cross(f, up)), u = cross(r, f), m = I();
    m[0] = r[0]; m[4] = r[1]; m[8] = r[2]; m[1] = u[0]; m[5] = u[1]; m[9] = u[2];
    m[2] = -f[0]; m[6] = -f[1]; m[10] = -f[2];
    m[12] = -dot(r, e); m[13] = -dot(u, e); m[14] = dot(f, e); return m;
  }
  function normalMat(M) { // inverse transpose of the upper 3x3
    var a = M[0], b = M[1], c = M[2], d = M[4], e = M[5], f = M[6], g = M[8], h = M[9], i = M[10];
    var A = e * i - f * h, B = f * g - d * i, C = d * h - e * g, det = a * A + b * B + c * C || 1, k = 1 / det;
    return new Float32Array([A * k, B * k, C * k,
      (c * h - b * i) * k, (a * i - c * g) * k, (b * g - a * h) * k,
      (b * f - c * e) * k, (c * d - a * f) * k, (a * e - b * d) * k]);
  }
  function alignY(p0, p1) { // frame whose Y axis runs p0 -> p1
    var y = norm(sub(p1, p0)), h = Math.abs(y[2]) > 0.9 ? [1, 0, 0] : [0, 0, 1], x = norm(cross(h, y)), z = cross(x, y), m = I();
    m[0] = x[0]; m[1] = x[1]; m[2] = x[2]; m[4] = y[0]; m[5] = y[1]; m[6] = y[2]; m[8] = z[0]; m[9] = z[1]; m[10] = z[2];
    m[12] = p0[0]; m[13] = p0[1]; m[14] = p0[2]; return m;
  }

  // ---- geometry ------------------------------------------------------------------------------
  function mesh(gl, pos, idx) {
    var vb = gl.createBuffer(), ib = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vb); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(pos), gl.STATIC_DRAW);
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ib); gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint16Array(idx), gl.STATIC_DRAW);
    return { vb: vb, ib: ib, n: idx.length };
  }
  function boxGeo() {
    var P = [], I2 = [], f = [
      [[0, 0, 1], [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], [[0, 0, -1], [1, -1, -1], [-1, -1, -1], [-1, 1, -1], [1, 1, -1]],
      [[1, 0, 0], [1, -1, 1], [1, -1, -1], [1, 1, -1], [1, 1, 1]], [[-1, 0, 0], [-1, -1, -1], [-1, -1, 1], [-1, 1, 1], [-1, 1, -1]],
      [[0, 1, 0], [-1, 1, 1], [1, 1, 1], [1, 1, -1], [-1, 1, -1]], [[0, -1, 0], [-1, -1, -1], [1, -1, -1], [1, -1, 1], [-1, -1, 1]]];
    f.forEach(function (q, k) {
      for (var i = 1; i < 5; i++) P.push(q[i][0] * 0.5, q[i][1] * 0.5, q[i][2] * 0.5, q[0][0], q[0][1], q[0][2]);
      I2.push(k * 4, k * 4 + 1, k * 4 + 2, k * 4, k * 4 + 2, k * 4 + 3);
    });
    return [P, I2];
  }
  function cylGeo(n, tr) { // base at y=0 (radius 1), top at y=1 (radius tr)
    var P = [], I2 = [], k, s = Math.hypot(1, 1 - tr);
    for (k = 0; k <= n; k++) {
      var a = k / n * PI * 2, c = cos(a), sn = sin(a);
      P.push(c, 0, sn, c / s, (1 - tr) / s, sn / s, c * tr, 1, sn * tr, c / s, (1 - tr) / s, sn / s);
    }
    for (k = 0; k < n; k++) I2.push(k * 2, k * 2 + 1, k * 2 + 3, k * 2, k * 2 + 3, k * 2 + 2);
    var base = P.length / 6;
    P.push(0, 0, 0, 0, -1, 0); for (k = 0; k <= n; k++) { var b = k / n * PI * 2; P.push(cos(b), 0, sin(b), 0, -1, 0); }
    for (k = 0; k < n; k++) I2.push(base, base + 2 + k, base + 1 + k);
    var top = P.length / 6;
    P.push(0, 1, 0, 0, 1, 0); for (k = 0; k <= n; k++) { var d = k / n * PI * 2; P.push(cos(d) * tr, 1, sin(d) * tr, 0, 1, 0); }
    for (k = 0; k < n; k++) I2.push(top, top + 1 + k, top + 2 + k);
    return [P, I2];
  }
  function sphGeo(la, lo) {
    var P = [], I2 = [], i, j;
    for (i = 0; i <= la; i++) for (j = 0; j <= lo; j++) {
      var th = i / la * PI, ph = j / lo * PI * 2, x = sin(th) * cos(ph), y = cos(th), z = sin(th) * sin(ph);
      P.push(x, y, z, x, y, z);
    }
    for (i = 0; i < la; i++) for (j = 0; j < lo; j++) {
      var a = i * (lo + 1) + j, b = a + lo + 1; I2.push(a, b, a + 1, a + 1, b, b + 1);
    }
    return [P, I2];
  }
  function quadGeo() { return [[-1, 0, -1, 0, 1, 0, 1, 0, -1, 0, 1, 0, 1, 0, 1, 0, 1, 0, -1, 0, 1, 0, 1, 0], [0, 1, 2, 0, 2, 3]]; }

  // ---- shaders -------------------------------------------------------------------------------
  var VS = 'attribute vec3 aP;attribute vec3 aN;uniform mat4 uVP;uniform mat4 uM;uniform mat3 uNM;varying vec3 vN;varying vec3 vW;varying vec3 vL;' +
    'void main(){vec4 w=uM*vec4(aP,1.);vW=w.xyz;vN=uNM*aN;vL=aP;gl_Position=uVP*w;}';
  var FS = 'precision mediump float;varying vec3 vN;varying vec3 vW;varying vec3 vL;uniform vec3 uCol;uniform vec3 uCol2;uniform float uA;uniform float uMode;uniform float uSpec;uniform float uShin;uniform vec3 uEye;uniform vec3 uLight;' +
    'void main(){' +
    'if(uMode>0.5&&uMode<1.5){gl_FragColor=vec4(uCol,uA);return;}' +
    'if(uMode>1.5&&uMode<2.5){float l=length(vL.xz);gl_FragColor=vec4(uCol,uA*(1.-smoothstep(.15,1.,l)));return;}' +
    'if(uMode>2.5&&uMode<3.5){float l=length(vL.xz);float r=smoothstep(.8,.86,l)-smoothstep(.92,.98,l);gl_FragColor=vec4(uCol,uA*r);return;}' +
    'if(uMode>3.5&&uMode<4.5){gl_FragColor=vec4(uCol,uA*(1.-vL.y*.85));return;}' +
    'if(uMode>4.5){gl_FragColor=vec4(uCol,1.);return;}' +
    'vec3 N=normalize(vN);vec3 Ld=normalize(uLight);vec3 V=normalize(uEye-vW);float df=max(dot(N,Ld),0.);float hemi=.5+.5*N.y;vec3 H=normalize(Ld+V);' +
    'float sp=pow(max(dot(N,H),0.),uShin)*uSpec;float rim=pow(1.-max(dot(N,V),0.),3.)*.12;' +
    'vec3 c=uCol*(.44+.17*hemi+.5*df)+vec3(sp)+vec3(rim);' +
    'if(uMode<-.5){vec2 g=abs(fract(vW.xz)-.5);float ln=smoothstep(.465,.5,max(g.x,g.y));c=mix(c,uCol2,ln*.55);}' +
    'gl_FragColor=vec4(c,1.);}';

  // ---- palette -------------------------------------------------------------------------------
  function parseCol(s) {
    s = (s || '').trim();
    var m = /^#([0-9a-f]{3}|[0-9a-f]{6})$/i.exec(s);
    if (m) { var h = m[1]; if (h.length === 3) h = h.replace(/./g, '$&$&'); return [parseInt(h.slice(0, 2), 16) / 255, parseInt(h.slice(2, 4), 16) / 255, parseInt(h.slice(4, 6), 16) / 255]; }
    m = /rgba?\(([^)]+)\)/.exec(s);
    if (m) { var p = m[1].split(',').map(parseFloat); return [p[0] / 255, p[1] / 255, p[2] / 255]; }
    return [0.5, 0.5, 0.5];
  }
  function mixc(a, b, t) { return [a[0] * (1 - t) + b[0] * t, a[1] * (1 - t) + b[1] * t, a[2] * (1 - t) + b[2] * t]; }
  function readPalette() {
    var cs = getComputedStyle(document.documentElement), g = function (n) { return parseCol(cs.getPropertyValue(n)); };
    var ink = g('--ink'), card = g('--card'), soft = g('--soft'), ok = g('--ok'), wip = g('--wip'), p = {};
    p.bg = soft; p.ink = ink; p.ok = ok; p.wip = wip;
    p.table = card; p.slab = mixc(card, ink, 0.18); p.grid = mixc(card, ink, 0.16);
    p.tote = mixc(card, ink, 0.1); p.tote2 = mixc(card, ink, 0.04);
    p.dark = mixc(ink, card, 0.08); p.mid = mixc(ink, card, 0.5); p.light = mixc(ink, card, 0.8);
    p.body = mixc(ink, card, 0.1); p.cap = mixc(card, ink, 0.03); p.steel = mixc(ink, card, 0.62);
    p.teal = [0.16, 0.62, 0.56]; p.pedestal = mixc(card, ink, 0.22); p.shadow = [0, 0, 0];
    p.dim = ink[0] > 0.5; // dark theme
    return p;
  }

  // ---- part models: local to the tote top, handle on the rear edge ---------------------------
  var lB = function (cx, cy, cz, sx, sy, sz, c, rot) { var m = T(cx, cy, cz); if (rot) m = mul(m, rot); return { p: 'box', M: mul(m, Sc(sx, sy, sz)), c: c }; };
  var lC = function (cx, cy, cz, r, h, c, ax, kind) {
    var m = T(cx, cy, cz);
    if (ax === 'z') m = mul(m, RX(PI / 2)); else if (ax === 'zn') m = mul(m, RX(-PI / 2)); else if (ax === 'x') m = mul(m, RZ(-PI / 2));
    return { p: kind || 'cyl', M: chain(m, Sc(r, h, r), T(0, -0.5, 0)), c: c };
  };
  var lD = function (cx, cy, cz, r, h, c, a) { return { p: 'cyl', M: chain(T(cx, cy, cz), RZ(a - PI / 2), Sc(r, h, r), T(0, -0.5, 0)), c: c }; };
  var lS = function (cx, cy, cz, r, c) { return { p: 'sph', M: mul(T(cx, cy, cz), Sc(r, r, r)), c: c }; };
  var A = PI / 180;
  var MODELS = {
    rover: function () { return [lB(0, 0.17, 0, 0.5, 0.1, 0.72, 'mid'), lC(-0.3, 0.14, -0.12, 0.14, 0.07, 'dark', 'x'), lC(0.3, 0.14, -0.12, 0.14, 0.07, 'dark', 'x'),
      lS(0, 0.07, 0.28, 0.07, 'light'), lB(0, 0.26, -0.05, 0.2, 0.04, 0.28, 'dark'), lB(0, 0.29, 0.17, 0.22, 0.08, 0.14, 'light'),
      lC(-0.12, 0.22, 0.37, 0.05, 0.05, 'light', 'z'), lC(0.12, 0.22, 0.37, 0.05, 0.05, 'light', 'z')]; },
    myo: function () { return [lB(-0.46, 0.14, 0, 0.14, 0.24, 0.24, 'dark'), lD(-0.25, 0.14, 0, 0.09, 0.34, 'mid', 0), lS(-0.07, 0.14, 0, 0.105, 'dark'),
      lD(0.06, 0.18, 0, 0.085, 0.26, 'mid', 18 * A), lS(0.185, 0.22, 0, 0.1, 'dark'), lD(0.26, 0.275, 0, 0.075, 0.2, 'mid', 42 * A), lS(0.35, 0.34, 0, 0.075, 'light'),
      lD(-0.06, 0.04, 0, 0.02, 0.5, 'light', 6 * A)]; },
    rsx: function () { return [lB(0, 0.03, 0, 0.56, 0.06, 0.46, 'mid'), lB(0, 0.16, 0, 0.26, 0.2, 0.26, 'dark'), lC(0, 0.26, 0, 0.1, 0.04, 'light'),
      lB(-0.17, 0.43, 0, 0.04, 0.3, 0.2, 'mid'), lB(0.17, 0.43, 0, 0.04, 0.3, 0.2, 'mid'), lB(-0.27, 0.43, 0, 0.12, 0.14, 0.16, 'dark'),
      lB(0, 0.43, -0.02, 0.28, 0.17, 0.2, 'light'), lC(0, 0.43, 0.17, 0.075, 0.1, 'dark', 'z'), lC(0, 0.43, 0.225, 0.045, 0.02, 'steel', 'z')]; },
    igvc: function () { return [lB(0, 0.2, 0, 0.56, 0.12, 0.8, 'mid'), lC(-0.34, 0.14, -0.28, 0.14, 0.09, 'dark', 'x'), lC(0.34, 0.14, -0.28, 0.14, 0.09, 'dark', 'x'),
      lC(-0.34, 0.14, 0.28, 0.14, 0.09, 'dark', 'x'), lC(0.34, 0.14, 0.28, 0.14, 0.09, 'dark', 'x'), lB(0, 0.28, 0.05, 0.4, 0.04, 0.5, 'dark'),
      lC(0, 0.5, -0.22, 0.03, 0.46, 'light'), lC(0, 0.76, -0.22, 0.1, 0.05, 'dark'), lB(0, 0.24, 0.42, 0.3, 0.06, 0.04, 'light')]; },
    cvt: function () { return [lB(0, 0.03, 0, 0.9, 0.06, 0.3, 'mid'), lC(-0.26, 0.4, -0.085, 0.28, 0.17, 'mid', 'z', 'frus'), lC(-0.26, 0.4, 0.085, 0.28, 0.17, 'mid', 'zn', 'frus'),
      lC(0.3, 0.4, -0.085, 0.16, 0.17, 'mid', 'z', 'frus'), lC(0.3, 0.4, 0.085, 0.16, 0.17, 'mid', 'zn', 'frus'),
      lC(-0.26, 0.4, 0, 0.035, 0.42, 'light', 'z'), lC(0.3, 0.4, 0, 0.035, 0.42, 'light', 'z'),
      lB(0.02, 0.57, 0, 0.58, 0.02, 0.1, 'dark', RZ(-10 * A)), lB(0.02, 0.23, 0, 0.58, 0.02, 0.1, 'dark', RZ(10 * A))]; },
    tower: function () { return [lC(0, 0.07, 0, 0.34, 0.12, 'mid'), lC(0, 0.12, 0, 0.2, 0.74, 'light', 'y', 'hex'), lC(0, 0.86, 0, 0.14, 0.04, 'dark'),
      lC(-0.1, 0.3, 0.2, 0.065, 0.13, 'teal', 'z'), lC(0.1, 0.3, 0.2, 0.065, 0.13, 'teal', 'z'), lC(-0.1, 0.5, 0.2, 0.065, 0.13, 'teal', 'z'),
      lC(0.1, 0.5, 0.2, 0.065, 0.13, 'teal', 'z'), lC(-0.1, 0.7, 0.2, 0.065, 0.13, 'teal', 'z'), lC(0.1, 0.7, 0.2, 0.065, 0.13, 'teal', 'z'),
      lC(0.36, 0.4, 0.04, 0.025, 0.74, 'steel')]; },
    piston: function () { return [lB(0, 0.04, 0, 0.8, 0.08, 0.5, 'mid'), lB(-0.35, 0.36, 0, 0.08, 0.56, 0.3, 'dark'), lC(-0.1, 0.42, 0, 0.12, 0.36, 'light', 'x'),
      lC(-0.2, 0.42, 0, 0.133, 0.03, 'dark', 'x'), lC(-0.14, 0.42, 0, 0.133, 0.03, 'dark', 'x'), lC(0.18, 0.42, 0, 0.025, 0.26, 'dark', 'x'),
      lC(0.37, 0.42, 0, 0.15, 0.04, 'mid', 'z'), lC(0.37, 0.42, 0.04, 0.03, 0.05, 'dark', 'z')]; }
  };
  var HANDLE = [lB(0, 0.185, HANDLE_Z, 0.1, 0.37, 0.1, 'dark'), lB(0, 0.44, HANDLE_Z, 0.5, 0.14, 0.22, 'dark')];

  window.initRobotCell3D = function (o) {
    // --- WebGL context (bail out before touching the DOM if unavailable) ----------------------
    var canvas = document.createElement('canvas');
    var gl = null;
    try { gl = canvas.getContext('webgl', { antialias: true, stencil: true, alpha: false, powerPreference: 'low-power' }) || canvas.getContext('experimental-webgl', { stencil: true }); } catch (e) { gl = null; }
    if (!gl) return null;
    function sh(type, src) { var s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s); if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s)); return s; }
    var prog;
    try {
      prog = gl.createProgram(); gl.attachShader(prog, sh(gl.VERTEX_SHADER, VS)); gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, FS)); gl.linkProgram(prog);
      if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error('link');
    } catch (e) { return null; }
    gl.useProgram(prog);
    var U = {}; ['uVP', 'uM', 'uNM', 'uCol', 'uCol2', 'uA', 'uMode', 'uSpec', 'uShin', 'uEye', 'uLight'].forEach(function (n) { U[n] = gl.getUniformLocation(prog, n); });
    var aP = gl.getAttribLocation(prog, 'aP'), aN = gl.getAttribLocation(prog, 'aN');
    gl.enableVertexAttribArray(aP); gl.enableVertexAttribArray(aN);
    var G = {
      box: mesh(gl, boxGeo()[0], boxGeo()[1]), cyl: mesh(gl, cylGeo(32, 1)[0], cylGeo(32, 1)[1]), hex: mesh(gl, cylGeo(6, 1)[0], cylGeo(6, 1)[1]),
      frus: mesh(gl, cylGeo(32, 0.45)[0], cylGeo(32, 0.45)[1]), cone: mesh(gl, cylGeo(32, 0.35)[0], cylGeo(32, 0.35)[1]),
      sph: mesh(gl, sphGeo(14, 20)[0], sphGeo(14, 20)[1]), quad: mesh(gl, quadGeo()[0], quadGeo()[1])
    };

    // --- DOM ---------------------------------------------------------------------------------
    o.mount.innerHTML = '';
    o.mount.classList.add('r3');
    canvas.className = 'rc3-canvas';
    canvas.setAttribute('role', 'img');
    canvas.setAttribute('aria-label', 'Interactive 3D robot arm workcell. Use the buttons below to choose a project. Drag to look around.');
    var labels = document.createElement('div'); labels.className = 'rc3-labels'; labels.setAttribute('aria-hidden', 'true');
    o.mount.appendChild(canvas); o.mount.appendChild(labels);

    var items = o.items.map(function (it, i) {
      var phi = PHIS[i] * A;
      return { id: it.id, name: it.name, i: i, hx: RING * sin(phi), hz: RING * cos(phi), x: RING * sin(phi), y: 0, z: RING * cos(phi), model: MODELS[it.id]() };
    });
    var byId = {}; items.forEach(function (it) { byId[it.id] = it; });
    items.forEach(function (it) {
      var d = document.createElement('span'); d.className = 'rc3-lbl'; d.textContent = it.name; labels.appendChild(d); it.lbl = d;
    });
    var stageLbl = document.createElement('span'); stageLbl.className = 'rc3-lbl rc3-stage'; stageLbl.textContent = 'Inspection stage'; labels.appendChild(stageLbl);

    o.chips.innerHTML = items.map(function (it) {
      return '<button type="button" class="chip" data-id="' + it.id + '" aria-pressed="false"><span class="mono">' + ('0' + (it.i + 1)) + '</span>' + it.name + '</button>';
    }).join('');

    // --- state --------------------------------------------------------------------------------
    var pal = readPalette();
    var cur = { psi: HOME.psi, rho: HOME.rho, y: HOME.y, g: 0.5 }, tgt = { psi: HOME.psi, rho: HOME.rho, y: HOME.y, g: 0.5 };
    var held = null, onStage = null, want = null, busy = false, quiet = false, touched = false, hotId = null;
    var state = 'IDLE', waiters = [], raf = 0, last = 0, teleT = 0, dirty = true, joints = {}, moving = false;
    var cam = { yaw: 0, pitch: 24 * A, dist: 14, sway: !reduce, t0: performance.now() };
    var size = { w: 1, h: 1, dpr: 1 };
    var tip = { x: 0, y: 0, z: 0 }, view = { eye: [0, 5, 10], f: [0, 0, -1], r: [1, 0, 0], u: [0, 1, 0], tv: 0.2867, th: 0.5, VP: I() };
    var LIGHT = norm([-0.5, 1.0, 0.45]), FOV = 32 * A, TARGET = [0, 1.3, 0.8];

    // --- kinematics ----------------------------------------------------------------------------
    function ik(psi, rho, y) {
      var h = y + LT - H1, d = Math.hypot(rho, h);
      d = clamp(d, Math.abs(L2 - L3E) + 0.01, L2 + L3E - 0.01);
      var q3 = PI - Math.acos(clamp((L2 * L2 + L3E * L3E - d * d) / (2 * L2 * L3E), -1, 1));
      var al = Math.acos(clamp((L2 * L2 + d * d - L3E * L3E) / (2 * L2 * d), -1, 1));
      var q2 = Math.atan2(rho, h) - al;
      return { j1: psi, j2: q2, j3: q3, j4: 0, j5: PI - q2 - q3, j6: psi };
    }
    var pol = function (P) { return { psi: Math.atan2(-P.z, P.x), rho: Math.hypot(P.x, P.z), y: P.y }; };

    // --- drawing list ---------------------------------------------------------------------------
    var list = [];
    var col = function (c) { return typeof c === 'string' ? pal[c] : c; };
    function add(prim, M, c, spec, shin, noShadow) { list.push({ p: prim, M: M, c: col(c), s: spec == null ? 0.12 : spec, sh: shin || 20, ns: noShadow }); }
    function addParts(parts, base) { for (var i = 0; i < parts.length; i++) add(parts[i].p, mul(base, parts[i].M), parts[i].c, 0.1, 18); }
    var cylY = function (F, y0, r, h, c, spec) { add('cyl', chain(F, T(0, y0, 0), Sc(r, h, r)), c, spec == null ? 0.25 : spec, 30); };
    var cylZ = function (F, r, len, c) { add('cyl', chain(F, RX(PI / 2), Sc(r, len, r), T(0, -0.5, 0)), c, 0.4, 40); };

    function itemMatrix(it) { return T(it.x, it.y, it.z); }
    function buildScene() {
      list.length = 0;
      // table, base mount
      add('box', mul(T(0, -0.15, 0.9), Sc(11.6, 0.3, 8.8)), 'slab', 0.05, 8, true);
      add('box', mul(T(0, 0.0015, 0.9), Sc(11.6, 0.003, 8.8)), 'table', 0.02, 6, true); list[1].mode = -1;
      // stage pedestal + camera post
      add('cyl', chain(T(STAGE.x, 0, STAGE.z), Sc(1.05, STAGE.h, 1.05)), 'pedestal', 0.2, 24);
      add('cyl', chain(T(STAGE.x, STAGE.h, STAGE.z), Sc(1.0, 0.04, 1.0)), 'cap', 0.3, 30);
      var head = [STAGE.x + 0.9, 2.5, STAGE.z - 1.5];
      add('cyl', chain(T(head[0], 0, head[2]), Sc(0.09, 2.4, 0.09)), 'mid', 0.2, 20);
      add('box', chain(T(head[0], 2.5, head[2]), RY(-0.5), Sc(0.5, 0.3, 0.4)), 'dark', 0.3, 30);
      // base
      add('cyl', chain(T(0, 0, 0), Sc(1.15, 0.1, 1.15)), 'mid', 0.3, 30);
      add('cyl', chain(T(0, 0.1, 0), Sc(0.95, 0.3, 0.95)), 'cap', 0.45, 40);
      // items
      items.forEach(function (it) {
        var M = itemMatrix(it);
        add('box', chain(M, T(0, TH / 2, 0), Sc(TOTE, TH, TOTE)), 'tote', 0.08, 16);
        add('box', chain(M, T(0, TH + 0.004, 0), Sc(TOTE - 0.16, 0.008, TOTE - 0.16)), 'tote2', 0.05, 10);
        addParts(it.model, mul(M, T(0, TH, 0)));
        addParts(HANDLE, mul(M, T(0, TH, 0)));
      });
      // arm
      var q = joints, B = T(0, 0, 0);
      var Aa = chain(B, RY(q.j1));
      cylY(Aa, 0.4, 0.72, 0.5, 'body');
      cylY(Aa, 0.38, 0.76, 0.06, 'cap', 0.5);
      var S2 = chain(Aa, T(0, H1, 0));
      add('box', chain(Aa, T(0, 0.9, 0), Sc(0.95, 0.5, 0.95)), 'body', 0.25, 30);
      var S2r = chain(S2, RZ(-q.j2));
      cylZ(S2r, 0.46, 1.05, 'cap');
      cylY(S2r, 0.0, 0.3, L2, 'body');
      var E = chain(S2r, T(0, L2, 0), RZ(-q.j3));
      cylZ(E, 0.37, 0.9, 'cap');
      cylY(E, 0, 0.25, L3, 'body');
      var Wf = chain(E, T(0, L3, 0));
      cylY(Wf, 0, 0.21, J4L, 'cap', 0.4);
      var Wr = chain(Wf, RY(q.j4), T(0, J4L, 0));
      cylZ(Wr, 0.25, 0.58, 'cap');
      var Wp = chain(Wr, RZ(-q.j5));
      cylY(Wp, 0, 0.17, 0.38, 'body');
      var Wt = chain(Wp, T(0, 0.38, 0), RY(q.j6));
      cylY(Wt, 0, 0.23, 0.1, 'cap', 0.5);
      add('cyl', chain(Wt, T(0, 0.095, 0), Sc(0.235, 0.025, 0.235)), state === 'IDLE' ? 'ok' : 'wip', 0, 1, true); list[list.length - 1].mode = 5;
      add('box', chain(Wt, T(0, 0.27, 0), Sc(0.62, 0.34, 0.34)), 'body', 0.3, 30);
      add('box', chain(Wt, T(0, 0.5, 0), Sc(0.9, 0.1, 0.3)), 'dark', 0.3, 30);
      var gap = 0.25 + 0.17 * cur.g;
      [-1, 1].forEach(function (s) {
        add('box', chain(Wt, T(s * (gap + 0.05), 0.78, 0), Sc(0.1, 0.5, 0.3)), 'steel', 0.4, 40);
        add('box', chain(Wt, T(s * (gap - 0.01), 1.02, 0), Sc(0.05, 0.1, 0.3)), 'dark', 0.2, 20);
        add('box', chain(Wt, T(s * (gap + 0.05), 0.6, 0), Sc(0.14, 0.2, 0.32)), 'dark', 0.3, 30);
      });
    }

    function fkTip() {
      var q = joints;
      var M = chain(RY(q.j1), T(0, H1, 0), RZ(-q.j2), T(0, L2, 0), RZ(-q.j3), T(0, L3, 0), RY(q.j4), T(0, J4L, 0), RZ(-q.j5), T(0, LT, 0), RY(q.j6));
      return { x: M[12], y: M[13], z: M[14], M: M };
    }

    // --- rendering ------------------------------------------------------------------------------
    function setCam() {
      var yaw = cam.yaw + (cam.sway ? 0.075 * sin((performance.now() - cam.t0) / 1000 * 0.45) * Math.exp(-(performance.now() - cam.t0) / 14000) : 0);
      var a = size.w / size.h, tv = Math.tan(FOV / 2), th = tv * a;
      var D = Math.max(13.2, 6.0 / th);
      cam.dist = D;
      var cp = cos(cam.pitch), e = [TARGET[0] + D * sin(yaw) * cp, TARGET[1] + D * sin(cam.pitch), TARGET[2] + D * cos(yaw) * cp];
      var f = norm(sub(TARGET, e)), r = norm(cross(f, [0, 1, 0])), u = cross(r, f);
      view = { eye: e, f: f, r: r, u: u, tv: tv, th: th, VP: mul(persp(FOV, a, 0.5, 80), lookAt(e, TARGET, [0, 1, 0])) };
    }
    function drawItem(d, mode, alpha, c2) {
      gl.uniformMatrix4fv(U.uM, false, d.M);
      gl.uniformMatrix3fv(U.uNM, false, normalMat(d.M));
      var P = G[d.p];
      gl.bindBuffer(gl.ARRAY_BUFFER, P.vb); gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, P.ib);
      gl.vertexAttribPointer(aP, 3, gl.FLOAT, false, 24, 0); gl.vertexAttribPointer(aN, 3, gl.FLOAT, false, 24, 12);
      gl.drawElements(gl.TRIANGLES, P.n, gl.UNSIGNED_SHORT, 0);
    }
    function render() {
      dirty = false; setCam(); buildScene();
      gl.viewport(0, 0, canvas.width, canvas.height);
      gl.clearColor(pal.bg[0], pal.bg[1], pal.bg[2], 1); gl.clearStencil(0); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT | gl.STENCIL_BUFFER_BIT);
      gl.enable(gl.DEPTH_TEST); gl.depthFunc(gl.LEQUAL); gl.disable(gl.BLEND); gl.disable(gl.STENCIL_TEST);
      gl.uniformMatrix4fv(U.uVP, false, view.VP); gl.uniform3fv(U.uEye, view.eye); gl.uniform3fv(U.uLight, LIGHT);
      var i, d;
      for (i = 0; i < list.length; i++) {
        d = list[i];
        gl.uniform1f(U.uMode, d.mode || 0); gl.uniform3fv(U.uCol, d.c); gl.uniform3fv(U.uCol2, pal.grid); gl.uniform1f(U.uSpec, d.s); gl.uniform1f(U.uShin, d.sh); gl.uniform1f(U.uA, 1);
        drawItem(d);
      }
      // contact blobs and hover/selection rings
      gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA); gl.depthMask(false);
      gl.uniform3fv(U.uCol, pal.shadow);
      gl.uniform1f(U.uMode, 2);
      var blob = function (x, z, s, a, y0) { gl.uniform1f(U.uA, a); drawItem({ p: 'quad', M: chain(T(x, y0 || 0.006, z), Sc(s, 1, s)) }); };
      blob(0, 0, 1.9, 0.35); blob(STAGE.x, STAGE.z, 1.7, 0.3);
      items.forEach(function (it) { if (it.y < 0.01 && it !== held) blob(it.x, it.z, 1.15, pal.dim ? 0.55 : 0.3); });
      // planar shadows, drawn once per pixel through the stencil
      gl.enable(gl.STENCIL_TEST); gl.stencilFunc(gl.EQUAL, 0, 255); gl.stencilOp(gl.KEEP, gl.KEEP, gl.INCR);
      gl.uniform1f(U.uMode, 1); gl.uniform1f(U.uA, pal.dim ? 0.5 : 0.22); gl.uniform3fv(U.uCol, pal.shadow);
      var SM = new Float32Array([1, 0, 0, 0, -LIGHT[0] / LIGHT[1], 0, -LIGHT[2] / LIGHT[1], 0, 0, 0, 1, 0, 0, 0.012, 0, 1]);
      for (i = 2; i < list.length; i++) { d = list[i]; if (d.ns || d.mode) continue; drawItem({ p: d.p, M: mul(SM, d.M) }); }
      gl.disable(gl.STENCIL_TEST);
      // rings
      gl.uniform1f(U.uMode, 3);
      var ringCol = pal.ink;
      var ring = function (x, z, y0, a) { gl.uniform3fv(U.uCol, ringCol); gl.uniform1f(U.uA, a); drawItem({ p: 'quad', M: chain(T(x, y0, z), Sc(0.82, 1, 0.82)) }); };
      items.forEach(function (it) {
        if (it.id === hotId && it !== held) ring(it.x, it.z, it.y + 0.012, 0.9);
        else if (it.id === (want || onStage) && onStage === it.id && it !== held) ring(it.x, it.z, it.y + 0.012, 0.55);
      });
      // scanner beam
      gl.uniform1f(U.uMode, 4); gl.uniform3fv(U.uCol, pal.ink); gl.uniform1f(U.uA, 0.11);
      var head = [STAGE.x + 0.9, 2.45, STAGE.z - 1.5], tc = [STAGE.x, STAGE.h + 0.5, STAGE.z + 0.2];
      var len = Math.hypot(head[0] - tc[0], head[1] - tc[1], head[2] - tc[2]);
      drawItem({ p: 'cone', M: mul(alignY(tc, head), Sc(0.62, len, 0.62)) });
      gl.depthMask(true); gl.disable(gl.BLEND);
      placeLabels();
    }

    function project(p) {
      var v = sub(p, view.eye), z = dot(v, view.f);
      return { x: (dot(v, view.r) / (z * view.th) + 1) / 2 * size.cssW, y: (1 - dot(v, view.u) / (z * view.tv)) / 2 * size.cssH, z: z };
    }
    function placeLabels() {
      var narrow = size.cssW < 640;
      items.forEach(function (it) {
        var p = project([it.x, it.y + 0.1, it.z + TOTE / 2 + 0.28]);
        var show = !narrow || it.id === hotId || it.id === onStage;
        it.lbl.style.transform = 'translate(' + p.x.toFixed(1) + 'px,' + p.y.toFixed(1) + 'px) translate(-50%,-50%)';
        it.lbl.style.opacity = show ? '' : '0';
        it.lbl.classList.toggle('on', it.id === (want || onStage) && it.id === onStage);
      });
      var s = project([STAGE.x + 0.2, 0.02, STAGE.z - 1.3]);
      stageLbl.style.transform = 'translate(' + s.x.toFixed(1) + 'px,' + s.y.toFixed(1) + 'px) translate(-50%,-50%)';
      stageLbl.style.opacity = narrow ? '0' : '';
    }

    function resize() {
      var r = canvas.getBoundingClientRect(), dpr = Math.min(window.devicePixelRatio || 1, 2);
      var w = Math.max(1, Math.round(r.width)), h = Math.max(1, Math.round(r.height));
      if (w === size.cssW && h === size.cssH && dpr === size.dpr) return;
      size = { w: w, h: h, cssW: w, cssH: h, dpr: dpr }; canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
      size.w = w; size.h = h; dirty = true; kick();
    }
    size.cssW = 1; size.cssH = 1;

    // --- motion ----------------------------------------------------------------------------------
    function frame(t) {
      raf = 0;
      var dt = Math.min(0.05, (t - last) / 1000 || 0.016); last = t; moving = false;
      ['psi', 'rho', 'y', 'g'].forEach(function (k) {
        var e = tgt[k] - cur[k]; if (e === 0) return;
        var v = clamp(e * (k === 'g' ? 12 : 5.5), -SPEED[k], SPEED[k]), st = v * dt, eps = k === 'g' ? 0.004 : k === 'psi' ? 0.0015 : 0.006;
        if (Math.abs(e) <= Math.max(Math.abs(st), eps)) cur[k] = tgt[k]; else { cur[k] += st; moving = true; }
      });
      joints = ik(cur.psi, cur.rho, cur.y);
      var tx = cur.rho * cos(cur.psi), tz = -cur.rho * sin(cur.psi);
      tip.x = tx; tip.y = cur.y; tip.z = tz;
      if (held) { held.x = tx; held.y = cur.y - GRASP; held.z = tz - HANDLE_Z; }
      var swaying = cam.sway && performance.now() - cam.t0 < 42000;
      if (moving || dirty || swaying) render();
      writeTele(t);
      o.root.dataset.state = moving ? 'moving' : 'idle';
      if (!moving) { var w = waiters; waiters = []; w.forEach(function (r) { r(); }); }
      if (moving || waiters.length || swaying) raf = requestAnimationFrame(frame);
      else if (dirty) render();
    }
    function kick() { if (!raf) { last = performance.now(); raf = requestAnimationFrame(frame); } }
    function writeTele(now, force) {
      if (!o.tele || (!force && now - teleT < 110)) return;
      teleT = now;
      var f = function (v) { var s = (v * 180 / PI).toFixed(1); while (s.length < 6) s = ' ' + s; return s; };
      var q = joints; if (q.j1 === undefined) return;
      o.tele.querySelector('.tv').textContent = (state + '         ').slice(0, 9) + ' J1' + f(q.j1) + ' J2' + f(q.j2) + ' J3' + f(q.j3) + ' J4' + f(q.j4) + ' J5' + f(q.j5) + ' J6' + f(q.j6) +
        '   GRIP ' + (cur.g > 0.75 ? 'open  ' : cur.g < 0.25 ? 'closed' : 'part  ');
    }
    function mvPol(psi, rho, y, g) {
      tgt.psi = psi; tgt.rho = rho; tgt.y = y; if (g !== undefined) tgt.g = g;
      if (reduce) { cur.psi = tgt.psi; cur.rho = tgt.rho; cur.y = tgt.y; cur.g = tgt.g; syncNow(); return Promise.resolve(); }
      return new Promise(function (r) { waiters.push(r); kick(); });
    }
    function syncNow() {
      joints = ik(cur.psi, cur.rho, cur.y);
      var tx = cur.rho * cos(cur.psi), tz = -cur.rho * sin(cur.psi); tip.x = tx; tip.y = cur.y; tip.z = tz;
      if (held) { held.x = tx; held.y = cur.y - GRASP; held.z = tz - HANDLE_Z; }
      render(); writeTele(0, true);
    }
    function mv(P, g) { var p = pol(P); return mvPol(p.psi, p.rho, P.y, g); }
    async function travel(P, g) {
      var p = pol(P), far = Math.abs(p.psi - cur.psi) * Math.max(cur.rho, 1) > 0.2 || Math.abs(p.rho - cur.rho) > 0.2;
      if (far) {
        if (cur.y < SAFE - 0.05) await mvPol(cur.psi, cur.rho, SAFE, g);
        await mvPol(p.psi, p.rho, SAFE, g);
      }
      await mvPol(p.psi, p.rho, P.y, g);
    }
    async function grip(open) {
      tgt.g = open;
      if (reduce) { cur.g = open; syncNow(); return; }
      await new Promise(function (r) { waiters.push(r); kick(); }); await sleep(110);
    }
    var gpos = function (it) { return { x: it.x, y: it.y + GRASP, z: it.z + HANDLE_Z }; };
    var slotPos = function (it) { return { x: it.hx, y: 0, z: it.hz }; };
    var stagePos = { x: STAGE.x, y: STAGE.h, z: STAGE.z };

    async function grab(it) {
      var P = gpos(it);
      state = 'APPROACH'; await travel({ x: P.x, y: P.y, z: P.z }, 1);
      state = 'PICK'; await grip(0); held = it;
      state = 'LIFT'; await mv({ x: P.x, y: SAFE, z: P.z }, 0);
    }
    async function put(it, pos) {
      var P = gpos(pos);
      state = 'CARRY'; await mv({ x: P.x, y: SAFE, z: P.z }, 0);
      state = 'PLACE'; await mv(P, 0);
      var h = held; held = null; h.x = pos.x; h.y = pos.y; h.z = pos.z; dirty = true;
      await grip(1);
      state = 'RETREAT'; await mv({ x: P.x, y: SAFE, z: P.z }, 1);
    }

    function markChips() {
      var id = want || o.initial;
      o.chips.querySelectorAll('.chip').forEach(function (b) { b.setAttribute('aria-pressed', b.dataset.id === id); });
      dirty = true; kick();
    }
    async function run() {
      busy = true; o.onBusy && o.onBusy(true, quiet);
      while (onStage !== want) {
        var target = want;
        if (onStage) {
          var out = byId[onStage];
          await grab(out); await put(out, slotPos(out)); onStage = null;
          if (want !== target) continue;
        }
        var it = byId[target], home = { x: it.x, y: it.y, z: it.z };
        await grab(it);
        if (want !== target) { await put(it, home); continue; }
        await put(it, stagePos); onStage = target;
        if (want === target) {
          o.live && (o.live.textContent = it.name + ' is on the inspection stage. Details are below.');
          o.onPlace && o.onPlace(target, quiet);
        }
      }
      state = 'IDLE'; await mvPol(HOME.psi, HOME.rho, HOME.y, 0.5);
      busy = false; o.onBusy && o.onBusy(false, quiet); dirty = true; kick();
    }
    function select(id, q) {
      if (!byId[id]) return;
      if (id === want && (busy || onStage === id)) return;
      touched = touched || !q;
      var wasQuiet = quiet; quiet = !!q; want = id; markChips();
      if (busy) { if (wasQuiet && !quiet) o.onBusy && o.onBusy(true, false); } else run();
    }

    // --- hover, picking, orbit ----------------------------------------------------------------------
    function setHot(id) { if (hotId !== id) { hotId = id; dirty = true; kick(); } }
    function trackItem(it) {
      if (busy || reduce || !it) return;
      state = 'TRACK'; setHot(it.id);
      var P = gpos(it); var p = pol({ x: P.x, y: HOVER, z: P.z });
      mvPol(p.psi, p.rho, HOVER + (it.y > 0.1 ? 0.5 : 0), 1);
    }
    var relTimer = 0;
    function release() {
      clearTimeout(relTimer);
      if (busy || reduce) { setHot(null); return; }
      state = 'IDLE'; setHot(null); mvPol(HOME.psi, HOME.rho, HOME.y, 0.5);
    }
    function pickAt(cx, cy) {
      var r = canvas.getBoundingClientRect(), nx = (cx - r.left) / r.width * 2 - 1, ny = 1 - (cy - r.top) / r.height * 2;
      var d = norm([view.f[0] + nx * view.th * view.r[0] + ny * view.tv * view.u[0], view.f[1] + nx * view.th * view.r[1] + ny * view.tv * view.u[1], view.f[2] + nx * view.th * view.r[2] + ny * view.tv * view.u[2]]);
      var best = null, bt = 1e9;
      items.forEach(function (it) {
        if (it === held) return;
        var c = [it.x, it.y + 0.55, it.z - 0.1], v = sub(c, view.eye), t = dot(v, d);
        if (t < 0) return;
        var dist = Math.hypot(v[0] - d[0] * t, v[1] - d[1] * t, v[2] - d[2] * t);
        if (dist < 0.85 && t < bt) { bt = t; best = it; }
      });
      return best;
    }
    var drag = null;
    canvas.addEventListener('pointerdown', function (e) {
      if (e.button > 0) return;
      drag = { x: e.clientX, y: e.clientY, moved: false, id: e.pointerId, type: e.pointerType };
      cam.sway = false;
    });
    canvas.addEventListener('pointermove', function (e) {
      if (drag && drag.id === e.pointerId) {
        var dx = e.clientX - drag.x, dy = e.clientY - drag.y;
        if (!drag.moved && Math.hypot(dx, dy) > 5) { drag.moved = true; try { canvas.setPointerCapture(e.pointerId); } catch (x) { } }
        if (drag.moved) {
          cam.yaw = clamp(cam.yaw - dx * 0.006, -0.9, 0.9); cam.pitch = clamp(cam.pitch + dy * 0.004, 0.2, 0.95);
          drag.x = e.clientX; drag.y = e.clientY; dirty = true; kick(); return;
        }
      }
      if (e.pointerType !== 'mouse' || busy || reduce) return;
      var it = pickAt(e.clientX, e.clientY);
      if (it) { clearTimeout(relTimer); trackItem(it); } else { clearTimeout(relTimer); relTimer = setTimeout(release, 450); }
    });
    var endDrag = function (e) {
      if (!drag || drag.id !== e.pointerId) return;
      var d = drag; drag = null;
      if (!d.moved && e.type === 'pointerup') { var it = pickAt(e.clientX, e.clientY); if (it) select(it.id); }
    };
    canvas.addEventListener('pointerup', endDrag); canvas.addEventListener('pointercancel', endDrag);
    canvas.addEventListener('pointerleave', function () { if (!drag) { clearTimeout(relTimer); release(); } });
    canvas.style.cursor = 'grab';

    o.chips.addEventListener('click', function (e) { var b = e.target.closest('.chip'); if (b) select(b.dataset.id); });
    o.chips.querySelectorAll('.chip').forEach(function (b) {
      var on = function () { trackItem(byId[b.dataset.id]); };
      b.addEventListener('mouseenter', on); b.addEventListener('focus', on);
      b.addEventListener('mouseleave', release); b.addEventListener('blur', release);
    });

    // --- lifecycle ---------------------------------------------------------------------------------
    if (window.ResizeObserver) new ResizeObserver(resize).observe(canvas); else addEventListener('resize', resize);
    new MutationObserver(function () { pal = readPalette(); dirty = true; kick(); }).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
    canvas.addEventListener('webglcontextlost', function (e) { e.preventDefault(); });
    resize(); markChips(); syncNow(); kick();

    return {
      select: select, items: items, is3d: true,
      get touched() { return touched; }, get busy() { return busy; }, get onStage() { return onStage; },
      get pose() { return { psi: cur.psi, rho: cur.rho, y: cur.y, g: cur.g }; },
      get joints() { return joints; }, get tip() { return { x: tip.x, y: tip.y, z: tip.z }; },
      fk: fkTip, ik: ik, view: function () { return view; }, project: project, canvas: canvas
    };
  };
})();
