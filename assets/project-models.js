/* Parametric 3D reconstructions of the portfolio projects.
   Every model is a function build(state) -> list of parts {p: primitive, M: 4x4 matrix, c: colour name}.
   The same part lists drive the pedestals in the front-page workcell (robot-cell-3d.js) and the
   interactive viewers inside each project card (createPartsViewer, below).
   Units are millimetres, +y is up, the model sits on y = 0. Dependency-free WebGL.
   window.PM = { models, FIXED, geo, cellParts, createPartsViewer, supported } */
(function () {
  'use strict';
  var PI = Math.PI, cos = Math.cos, sin = Math.sin, A = PI / 180, hyp = Math.hypot;
  var reduce = matchMedia('(prefers-reduced-motion:reduce)').matches;

  // ---- matrices (column-major) -------------------------------------------------------------------
  function I() { return new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]); }
  function mul(a, b) {
    var o = new Float32Array(16);
    for (var c = 0; c < 4; c++) for (var r = 0; r < 4; r++) o[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] + a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
    return o;
  }
  function T(x, y, z) { var m = I(); m[12] = x; m[13] = y; m[14] = z; return m; }
  function Sc(x, y, z) { var m = I(); m[0] = x; m[5] = y; m[10] = z; return m; }
  function RX(a) { var c = cos(a), s = sin(a); return new Float32Array([1, 0, 0, 0, 0, c, s, 0, 0, -s, c, 0, 0, 0, 0, 1]); }
  function RY(a) { var c = cos(a), s = sin(a); return new Float32Array([c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, 0, 0, 0, 1]); }
  function RZ(a) { var c = cos(a), s = sin(a); return new Float32Array([c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]); }
  function chain() { var m = arguments[0]; for (var i = 1; i < arguments.length; i++) m = mul(m, arguments[i]); return m; }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function norm(a) { var l = hyp(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; }
  var clamp = function (v, a, b) { return Math.min(b, Math.max(a, v)); };

  // ---- part builder -------------------------------------------------------------------------------
  function B() {
    var L = [], cur = I(), st = [];
    var o = {
      L: L,
      put: function (p, M, c) { L.push({ p: p, M: mul(cur, M), c: c }); },
      box: function (x, y, z, sx, sy, sz, c, rot) { var m = T(x, y, z); if (rot) m = mul(m, rot); o.put('box', mul(m, Sc(sx, sy, sz)), c); },
      cyl: function (x, y, z, r, h, c, ax, kind) { // centred on (x,y,z); axis 'y' (default), 'x' or 'z'
        var m = T(x, y, z); if (ax === 'x') m = mul(m, RZ(-PI / 2)); else if (ax === 'z') m = mul(m, RX(PI / 2));
        o.put(kind || 'cyl', chain(m, Sc(r, h, r), T(0, -0.5, 0)), c);
      },
      sph: function (x, y, z, r, c) { o.put('sph', mul(T(x, y, z), Sc(r, r, r)), c); },
      rod: function (p0, p1, r, c) { // cylinder from p0 to p1
        var d = sub(p1, p0), len = hyp(d[0], d[1], d[2]); if (len < 1e-6 || r < 0.02) return;
        var y = [d[0] / len, d[1] / len, d[2] / len], h = Math.abs(y[1]) > 0.95 ? [1, 0, 0] : [0, 1, 0], x = norm(cross(h, y)), z = cross(x, y), m = I();
        m[0] = x[0]; m[1] = x[1]; m[2] = x[2]; m[4] = y[0]; m[5] = y[1]; m[6] = y[2]; m[8] = z[0]; m[9] = z[1]; m[10] = z[2];
        m[12] = p0[0]; m[13] = p0[1]; m[14] = p0[2];
        o.put('cyl', mul(m, Sc(r, len, r)), c);
      },
      grp: function (M, fn) { st.push(cur); cur = mul(cur, M); fn(); cur = st.pop(); },
      gear: function (x, y, z, r, w, c, ax, n, spin) { // spur gear drawn as a hub plus n teeth
        var q = ax === 'x' ? RZ(-PI / 2) : ax === 'z' ? RX(PI / 2) : I();
        o.grp(chain(T(x, y, z), q, RY(spin || 0)), function () {
          o.cyl(0, 0, 0, r, w, c, 'y');
          var tw = 2 * PI * r / n * 0.5, th = r * 0.13 + 1, k, a, rr = r + th / 2 - 0.6;
          for (k = 0; k < n; k++) { a = k / n * 2 * PI; o.box(rr * cos(a), 0, -rr * sin(a), th, w, tw, c, RY(a)); }
        });
      },
      ring: function (y, r, seg, rr, c) { // thin hoop made of rods
        for (var k = 0; k < seg; k++) {
          var a0 = k / seg * 2 * PI, a1 = (k + 1) / seg * 2 * PI;
          o.rod([r * cos(a0), y, r * sin(a0)], [r * cos(a1), y, r * sin(a1)], rr, c);
        }
      }
    };
    return o;
  }

  // ---- colours (names resolve against the page palette first, then these fixed hues) -------------------
  var FIXED = {
    red: [0.8, 0.17, 0.15], blue: [0.17, 0.45, 0.8], green: [0.34, 0.72, 0.22], alu: [0.8, 0.82, 0.85], rubber: [0.07, 0.07, 0.08],
    pcb: [0.07, 0.37, 0.28], gold: [0.8, 0.63, 0.24], white: [0.95, 0.95, 0.95], orange: [0.95, 0.52, 0.12], black: [0.05, 0.05, 0.06],
    teal: [0.16, 0.62, 0.56], yellow: [0.95, 0.8, 0.15], steelf: [0.62, 0.64, 0.67]
  };
  var SPEC = { alu: [0.55, 50], steel: [0.5, 48], gold: [0.5, 40], steelf: [0.5, 48], rubber: [0.04, 8], white: [0.2, 24] };

  // ================================================================================================
  //  MODELS
  // ================================================================================================
  var M = {};

  // ---- Rover-1: 2WD ESP32 rover. Plate 150 x 110 x 4 mm, 2 x N20 gearmotors with encoders ----------------
  M.rover = {
    name: 'Rover-1', R: 115, cy: 30, cam: { yaw: -38 * A, pitch: 24 * A, dist: 1.0 },
    rest: { e: 0, drive: 0, w: 0 },
    cap: '3D reconstruction of the Rover-1 design: 150 x 110 x 4 mm printed chassis plate, two N20 encoder motors, a ball caster and the ESP32 electronics stack.',
    states: [{ label: 'Assembled', tgt: { e: 0, drive: 1 } }, { label: 'Exploded', tgt: { e: 1, drive: 0 } }],
    step: function (s, dt) { s.w = (s.w || 0) + dt * (s.drive || 0) * 3.2; },
    tags: [{ p: [18, 46, 2], t: 'ESP32' }, { p: [-30, 21, 44], t: 'N20 motor + encoder' }, { p: [58, 31, -40], t: 'Printed chassis plate' }, { p: [62, 6, 0], t: 'Ball caster' }],
    build: function (s) {
      var b = B(), e = s.e || 0, w = s.w || 0, X = -30, AY = 21, Y0 = 30, k, wire = Math.max(0, 1 - e * 4);
      b.box(0, Y0, 0, 150, 4, 110, 'light');
      b.box(30, Y0 + 2.05, 38, 26, 0.2, 7, 'dark'); b.box(30, Y0 + 2.05, -38, 26, 0.2, 7, 'dark');
      [-1, 1].forEach(function (sd) {
        var zw = sd * (66.5 + e * 48), zm = sd * e * 26;
        b.grp(T(X, AY, zw), function () {
          b.cyl(0, 0, 0, 21, 19, 'rubber', 'z'); b.cyl(0, 0, 0, 14, 19.6, 'light', 'z'); b.cyl(0, 0, sd * 9.9, 3.5, 1, 'steel', 'z');
          for (k = 0; k < 3; k++) { var a = w * sd + k * 2 * PI / 3; b.box(8.5 * cos(a), 8.5 * sin(a), sd * 9.8, 10, 3, 1.4, 'dark', RZ(a)); }
        });
        b.box(X, AY, zm + sd * 27.5, 12, 10, 15, 'steelf'); b.box(X, AY, zm + sd * 39.5, 12, 10, 9, 'gold');
        b.rod([X, AY, zm + sd * 44], [X, AY, zw], 1.5, 'steel');
        b.box(X, AY, zm + sd * 19.8, 12, 10, 1.6, 'pcb'); b.cyl(X, AY, zm + sd * 18.4, 4, 1.4, 'black', 'z');
        b.box(X, 27, zm + sd * 30, 16, 2, 24, 'mid'); b.box(X, AY - 7, zm + sd * 30, 16, 4, 24, 'mid');
        b.box(X - 7.5, 20, zm + sd * 30, 2, 16, 24, 'mid'); b.box(X + 7.5, 20, zm + sd * 30, 2, 16, 24, 'mid');
      });
      b.grp(T(0, -e * 30, 0), function () { b.sph(62, 8, 0, 8, 'steelf'); b.cyl(62, 20, 0, 10, 12, 'dark'); b.box(62, 27, 0, 24, 2, 24, 'mid'); });
      b.grp(T(18, Y0 + 2 + e * 70, 2), function () { // ESP32 dev board on standoffs
        [[-21, -11], [21, -11], [-21, 11], [21, 11]].forEach(function (q) { b.cyl(q[0], 3.5, q[1], 2.4, 7, 'gold'); });
        b.box(0, 7.8, 0, 51, 1.6, 28, 'pcb'); b.box(12, 10.2, 0, 18, 3.2, 25, 'steelf'); b.box(-24, 9.6, 0, 8, 3, 6, 'steelf');
        b.box(-4, 9.6, 11.8, 36, 2.6, 2.4, 'black'); b.box(-4, 9.6, -11.8, 36, 2.6, 2.4, 'black');
        b.box(-17, 9.1, 8, 3.5, 1.5, 3.5, 'white'); b.box(-17, 9.1, -8, 3.5, 1.5, 3.5, 'white');
      });
      b.grp(T(-38, Y0 + 2 + e * 40, 9), function () { // motor driver
        b.cyl(-7, 2.5, -7, 2, 5, 'gold'); b.cyl(7, 2.5, 7, 2, 5, 'gold');
        b.box(0, 5.8, 0, 20, 1.6, 20, 'red'); b.box(0, 7.4, 0, 8, 1.6, 5, 'black'); b.box(0, 7, 8.4, 18, 2.4, 2.2, 'black');
      });
      b.grp(T(-6, Y0 + 2 + e * 18, -34), function () { // 2 x 18650 holder
        b.box(0, 9.5, 0, 77, 19, 38, 'black'); b.cyl(0, 12.5, 9.5, 9, 72, 'steelf', 'x'); b.cyl(0, 12.5, -9.5, 9, 72, 'steelf', 'x');
        b.box(0, 19.3, 0, 77, 0.8, 4, 'dark');
      });
      if (wire > 0.05) {
        b.rod([-40, 37, 16], [-40, 33, 38], 0.8 * wire, 'orange'); b.rod([-36, 37, 16], [-35, 33, 40], 0.8 * wire, 'white');
        b.rod([-40, 37, 2], [-36, 33, -26], 0.8 * wire, 'red'); b.rod([-26, 37, 2], [-14, 40, -26], 0.8 * wire, 'black');
        b.rod([-40, 37, 6], [-30, 33, -38], 0.8 * wire, 'yellow');
      }
      return b.L;
    }
  };

  // ---- Myo-hand: underactuated tendon hand. Index finger 45/25/23 mm, 74 mm palm ------------------------
  M.myo = {
    name: 'Myo-hand', R: 104, cy: 88, cam: { yaw: -30 * A, pitch: 16 * A, dist: 1.05 },
    rest: { curl: 1, lift: 1, lod: 1 }, cellRot: 'side', cellH: 0.8,
    cap: 'Concept model of the Myo-hand: one tendon pulled by a single actuator closes all five fingers around the object. Finger segments are scaled from my own index finger (45 / 25 / 23 mm) on a 74 mm palm.',
    states: [{ label: 'Open hand', tgt: { curl: 0, lift: 0 } }, { label: 'Grasp cube', tgt: { curl: 1, lift: 1 } }],
    tags: [{ p: [0, 130, 10], t: 'One actuator' }, { p: [-38, 56, 8], t: 'Thumb' }, { p: [40, 62, 36], t: 'Tendon-driven fingers' }, { p: [0, 14, 29], t: '57 mm cube' }],
    build: function (s) {
      var b = B(), c = clamp(s.curl || 0, 0, 1), lift = clamp(s.lift || 0, 0, 1), YB = 91;
      b.box(0, YB + 9, 0, 80, 18, 74, 'steelf'); b.box(0, YB + 18.2, 0, 76, 0.6, 70, 'mid');
      b.box(0, 122, 0, 40, 26, 20, 'dark'); b.box(0, 122, 10.2, 22, 10, 0.6, 'light');
      b.cyl(14, 135, 0, 9, 3, 'light', 'y'); b.cyl(14, 138, 0, 2.2, 3, 'steel', 'y'); b.box(14, 136.2, 0, 34, 2, 6, 'light', RY(0.4));
      b.cyl(0, 155, 0, 32, 40, 'steelf', 'y'); b.cyl(0, 176, 0, 33, 6, 'mid', 'y');
      // fingers: x base, z lane, segment lengths (proximal/middle/distal), grasp joint angles (deg), mirror sign
      var fingers = [
        { x: 35, z: -27, L: [36, 20, 20], a: [4, 30, 48], sg: 1 }, { x: 35, z: -9, L: [44, 24, 22], a: [4, 30, 48], sg: 1 },
        { x: 35, z: 9, L: [45, 25, 23], a: [4, 30, 48], sg: 1 }, { x: 35, z: 27, L: [42, 23, 21], a: [4, 30, 48], sg: 1 },
        { x: -35, z: 0, L: [34, 25, 20], a: [4, 16, 40], sg: -1 }
      ];
      fingers.forEach(function (f) {
        var ph = 0, x = f.x, y = YB, i, a, d, n, J = [[x, y]], N = [];
        for (i = 0; i < 3; i++) {
          ph += f.a[i] * c * A; a = PI - f.sg * ph; d = [-f.sg * sin(ph), -cos(ph)]; n = [f.sg * cos(a), f.sg * sin(a)]; N.push(n);
          (function (x0, y0, a0, Li, wd) {
            b.grp(chain(T(x0, y0, f.z), RZ(a0)), function () {
              b.box(0, Li / 2, 0, 13, Li - 3, wd, 'light'); b.box(f.sg * 7.3, Li / 2, 0, 1.6, Li - 8, 11, 'rubber');
            });
          })(x, y, a, f.L[i], 15);
          x += f.L[i] * d[0]; y += f.L[i] * d[1]; J.push([x, y]);
        }
        for (i = 0; i < 4; i++) b.cyl(J[i][0], J[i][1], f.z, 6.6, 16, 'steelf', 'z');
        b.sph(J[3][0], J[3][1], f.z, 6.8, 'rubber');
        for (i = 0; i < 3; i++) b.rod([J[i][0] + N[i][0] * 8.4, J[i][1] + N[i][1] * 8.4, f.z], [J[i + 1][0] + N[i][0] * 8.4, J[i + 1][1] + N[i][1] * 8.4, f.z], 1.1, 'orange');
      });
      // the object: Rubik's cube, 57 mm
      var cy = 28.5 + 34 * lift;
      b.grp(T(0, cy, 0), function () {
        b.box(0, 0, 0, 57, 57, 57, 'black');
        if (!s.lod) {
          var cols = ['red', 'orange', 'white', 'yellow', 'blue', 'green'], f, i, j, u, v;
          for (f = 0; f < 6; f++) for (i = -1; i <= 1; i++) for (j = -1; j <= 1; j++) {
            u = i * 18.5; v = j * 18.5;
            var pos = [[28.7, u, v], [-28.7, u, v], [u, 28.7, v], [u, -28.7, v], [u, v, 28.7], [u, v, -28.7]][f], sz = f < 2 ? [0.8, 16, 16] : f < 4 ? [16, 0.8, 16] : [16, 16, 0.8];
            b.box(pos[0], pos[1], pos[2], sz[0], sz[1], sz[2], cols[f]);
          }
        } else { b.box(0, 29, 0, 55, 0.6, 55, 'orange'); b.box(29, 0, 0, 0.6, 55, 55, 'blue'); b.box(0, 0, 29, 55, 55, 0.6, 'red'); }
      });
      return b.L;
    }
  };

  // ---- RSX: 2 DOF pan-tilt camera mount, two MG995 servos ----------------------------------------------
  M.rsx = {
    name: 'RSX camera mount', R: 122, cy: 66, cx: -26, cam: { yaw: -40 * A, pitch: 22 * A, dist: 1.0 },
    rest: { pa: 0, ta: 0, tb: 14 * A, ph: 0 },
    cap: 'Concept model of the 2 DOF camera mount: a pan servo under a tilt servo, both MG995, carrying a small webcam, bolted to a 42 x 42 mm arm link.',
    states: [{ label: 'Pan axis', tgt: { pa: 58 * A, ta: 0, tb: 10 * A } }, { label: 'Tilt axis', tgt: { pa: 0, ta: 30 * A, tb: 8 * A } }],
    step: function (s, dt) { s.ph = (s.ph || 0) + dt * 1.5; },
    tags: [{ p: [-10, 60, 10], t: 'Pan servo (MG995)' }, { p: [0, 100, 20], t: 'Tilt servo (MG995)' }, { p: [66, 112, 0], t: 'Webcam' }, { p: [-80, 21, 21], t: '42 mm arm link' }],
    build: null
  };
  // Tilt servo lies on its side so its output shaft points along z.
  (function () {
    M.rsx.build = function (s) {
      var b = B(), pan = (s.pa || 0) * sin(s.ph || 0), tilt = (s.tb || 0) + (s.ta || 0) * sin(s.ph || 0), YP = 46;
      function servoFlat(cx, cy, cz) { // pan servo: height along y
        b.box(cx, cy, cz, 40.7, 42.9, 19.7, 'dark'); b.box(cx, cy + 4, cz + 9.95, 22, 16, 0.5, 'light');
        b.box(cx + 24, cy + 6, cz, 7.5, 2.5, 19.7, 'dark'); b.box(cx - 24, cy + 6, cz, 7.5, 2.5, 19.7, 'dark');
        [[26, 6], [26, -6], [-26, 6], [-26, -6]].forEach(function (q) { b.cyl(cx + q[0], cy + 8.3, cz + q[1], 1.6, 3, 'steel', 'y'); });
      }
      function servoSide(cx, cy, cz) { // tilt servo: height along z, shaft toward +z
        b.box(cx, cy, cz, 40.7, 19.7, 42.9, 'dark'); b.box(cx, cy + 9.95, cz + 4, 22, 0.5, 16, 'light');
        b.box(cx + 24, cy, cz + 6, 7.5, 19.7, 2.5, 'dark'); b.box(cx - 24, cy, cz + 6, 7.5, 19.7, 2.5, 'dark');
        [[26, 6], [26, -6], [-26, 6], [-26, -6]].forEach(function (q) { b.cyl(cx + q[0], cy + q[1], cz + 6, 1.6, 3, 'steel', 'z'); });
      }
      b.box(-60, 21, 0, 180, 42, 42, 'alu'); b.box(-60, 21, 21.2, 176, 36, 0.4, 'steelf');
      b.box(0, 44, 0, 70, 4, 60, 'mid');
      [[-28, -22], [28, -22], [-28, 22], [28, 22]].forEach(function (q) { b.cyl(q[0], 46, q[1], 3.2, 2.4, 'black', 'y'); });
      servoFlat(-10, YP + 21.45, 0); b.cyl(0, YP + 42.9 + 2, 0, 3.4, 4, 'steel', 'y');
      b.rod([-30, YP + 10, 0], [-48, 44, 12], 0.9, 'orange'); b.rod([-30, YP + 10, 1], [-48, 44, 14], 0.9, 'red'); b.rod([-30, YP + 10, -1], [-48, 44, 16], 0.9, 'black');
      b.grp(chain(T(0, YP + 45.4, 0), RY(pan)), function () {
        b.cyl(0, 1.5, 0, 10.5, 2, 'light', 'y'); b.box(0, 1.5, 0, 62, 3, 40, 'mid');
        var TY = 3 + 9.85; servoSide(-10, TY, 0);
        b.box(-10, 3 + 1, -26, 44, 3, 10, 'mid');
        b.grp(chain(T(0, TY, 21.45 + 3), RZ(tilt)), function () {
          b.cyl(0, 0, 0, 10.5, 2, 'light', 'z'); b.cyl(0, 0, 2, 3.4, 3, 'steel', 'z');
          b.box(24, 0, 2.5, 54, 24, 3, 'mid'); b.box(49.5, 0, -10, 3, 36, 56, 'mid');
          b.box(62, 0, -10, 21, 34, 44, 'dark'); b.cyl(74.5, 0, -10, 9.5, 5, 'black', 'x'); b.cyl(77.5, 0, -10, 11, 1.6, 'steelf', 'x');
          b.box(62, 17.6, -10, 8, 1.2, 14, 'light');
        });
      });
      return b.L;
    };
    M.rsx.tags = [{ p: [-28, 52, 10], t: 'Pan servo (MG995)' }, { p: [-10, 106, 20], t: 'Tilt servo (MG995)' }, { p: [66, 100, -4], t: 'Webcam' }, { p: [-95, 21, 21], t: '42 mm arm link' }];
  })();

  // ---- UTRA Autonomous Rover Team (IGVC): rover concept and the dead axle assembly ------------------------
  M.igvc = {
    name: 'UTRA rover', R: 690, cy: 290, cam: { yaw: -42 * A, pitch: 20 * A, dist: 1.0 },
    rest: { v: 0, e: 0, drive: 0, w: 0, lod: 1 }, cellState: { v: 0 },
    cap: 'Concept layout of the IGVC rover (T-slot frame, four wheels on dead axles, sensor mast) and the dead axle assembly: a fixed shaft, two bearings, a hub and a drive sprocket. Chassis design is the next step.',
    states: [{ label: 'Rover concept', tgt: { v: 0, e: 0, drive: 1, R: 690, cy: 290, cyaw: 0, cpitch: 0 } }, { label: 'Dead axle', tgt: { v: 1, e: 0, drive: 1, R: 185, cy: 90, cyaw: 2.1, cpitch: -0.05 } }, { label: 'Axle exploded', tgt: { v: 1, e: 1, drive: 0, R: 245, cy: 90, cyaw: 2.1, cpitch: -0.05 } }],
    step: function (s, dt) { s.w = (s.w || 0) + dt * (s.drive || 0) * 2.0; },
    tagsFor: function (s) {
      return (s.v || 0) > 0.5
        ? [{ p: [0, 110, 0], t: 'Fixed (dead) shaft' }, { p: [0, 110, -44], t: 'Hub on 2 bearings' }, { p: [0, 150, 26], t: 'Drive sprocket' }, { p: [0, 95, 105], t: 'Shaft mount' }]
        : [{ p: [400, 600, 0], t: 'LiDAR' }, { p: [-120, 247, 0], t: 'Electronics bay' }, { p: [320, 150, 410], t: 'Wheel on dead axle' }, { p: [0, 410, 300], t: 'T-slot frame' }];
    },
    build: function (s) {
      var b = B(), w = s.w || 0, e = s.e || 0, k, sx, sz;
      if ((s.v || 0) > 0.5) {
        var Y = 110, z = e;
        b.box(0, 30, 0, 40, 40, 340, 'alu'); b.box(0, 30, 0, 14, 41, 340, 'mid');
        b.rod([0, Y, -150 - e * 40], [0, Y, 150 + e * 40], 12.7, 'steel');
        [-1, 1].forEach(function (sd) {
          b.box(0, 95, sd * (105 + z * 40), 44, 90, 30, 'alu'); b.cyl(0, Y, sd * (120 + z * 40), 23, 4, 'steel', 'z');
          b.cyl(0, 52, sd * (105 + z * 40), 5, 24, 'dark', 'y');
          b.cyl(0, Y, sd * (70 + z * 100), 22, 14, 'dark', 'z'); b.cyl(0, Y + 22, sd * (70 + z * 100), 3.4, 8, 'black', 'y');
          b.cyl(0, Y, sd * (42 + z * 62), 24, 12, 'steelf', 'z'); b.cyl(0, Y, sd * (42 + z * 62), 17, 12.4, 'black', 'z');
        });
        b.cyl(0, Y, 0, 30, 68, 'alu', 'z');
        b.cyl(0, Y, -26 - z * 28, 62, 8, 'alu', 'z');
        for (k = 0; k < 6; k++) { var a = w + k * PI / 3; b.cyl(48 * cos(a), Y + 48 * sin(a), -31 - z * 28, 4.6, 6, 'dark', 'z'); }
        b.gear(0, Y, 26 + z * 30, 46, 5, 'steelf', 'z', 28, w);
        for (k = 0; k < 6; k++) { var a2 = w + k * PI / 3 + 0.5; b.cyl(32 * cos(a2), Y + 32 * sin(a2), 30 + z * 30, 3.6, 4, 'dark', 'z'); }
      } else {
        var Yw = 150, ls = s.lod ? 12 : 12;
        [-1, 1].forEach(function (a1) { b.box(0, Yw, a1 * 300, 900, 40, 40, 'alu'); b.box(0, 410, a1 * 300, 900, 40, 40, 'alu'); b.box(a1 * 430, 410, 0, 40, 40, 560, 'alu'); });
        [-420, 0, 420].forEach(function (x) { b.box(x, Yw, 0, 40, 40, 560, 'alu'); });
        [-1, 1].forEach(function (a1) { [-1, 1].forEach(function (a2) { b.box(a1 * 430, 280, a2 * 300, 40, 220, 40, 'alu'); }); });
        b.box(0, 173, 0, 880, 6, 560, 'mid');
        b.box(-130, 247, 0, 380, 140, 300, 'light'); b.box(-130, 322, 0, 392, 10, 312, 'mid'); b.box(60.5, 247, 0, 1, 100, 240, 'dark');
        b.cyl(400, 495, 0, 9, 170, 'dark', 'y'); b.cyl(400, 580, 0, 48, 38, 'dark', 'y'); b.cyl(400, 604, 0, 34, 10, 'steelf', 'y');
        b.cyl(-420, 481, 0, 8, 142, 'dark', 'y'); b.cyl(-420, 560, 0, 38, 16, 'white', 'y');
        b.box(450, 360, 0, 50, 40, 60, 'dark'); b.cyl(480, 360, 0, 14, 18, 'black', 'x');
        b.cyl(-440, 436, 230, 30, 12, 'gold', 'y'); b.cyl(-440, 454, 230, 22, 24, 'red', 'y');
        [-1, 1].forEach(function (a1) {
          [-1, 1].forEach(function (a2) {
            b.rod([a1 * 320, Yw, a2 * 320], [a1 * 320, Yw, a2 * 372], 12.7, 'steel'); b.box(a1 * 320, Yw, a2 * 328, 60, 60, 16, 'dark');
            b.grp(T(a1 * 320, Yw, a2 * 410), function () {
              b.cyl(0, 0, 0, 150, 90, 'rubber', 'z'); b.cyl(0, 0, a2 * 46, 70, 4, 'alu', 'z'); b.cyl(0, 0, a2 * 48, 16, 6, 'steelf', 'z');
              for (k = 0; k < 5; k++) { var a3 = w * a1 + k * 2 * PI / 5; b.box(100 * cos(a3), 100 * sin(a3), a2 * 46, 60, 14, 4, 'dark', RZ(a3)); }
            });
          });
        });
      }
      return b.L;
    }
  };
  M.igvc.tags = null;

  // ---- CVT drivetrain teaching model: candidate 3, cone pair with a friction wheel (1.5x scale) -----------
  M.cvt = {
    name: 'CVT teaching model', R: 290, cy: 95, cam: { yaw: -18 * A, pitch: 52 * A, dist: 1.0 },
    rest: { wx: 0, drive: 0, ain: 0, aB: 0, ap: 0, aw: 0, aff: 0, ph: 0, lod: 1 },
    cap: 'Candidate 3 of the MIE243 teaching model: two opposed cones, a friction wheel on a tilted lead screw, a miter pair, an open differential, band brakes and a pivoting front axle. Dimensions follow the team documentation at 1.5x scale.',
    states: [{ label: 'Low gear', tgt: { wx: -62.25, drive: 1 } }, { label: 'High gear', tgt: { wx: 62.25, drive: 1 } }],
    step: function (s, dt) {
      var rA = 22.5 + 0.2 * ((s.wx || 0) + 75), rB = 75 - rA, wi = 2.4 * (s.drive || 0), wB = -wi * rA / rB;
      s.ain = (s.ain || 0) + dt * wi; s.aB = (s.aB || 0) + dt * wB; s.ap = (s.ap || 0) + dt * wB; s.aw = (s.aw || 0) - dt * wB * 17 / 50;
      s.aff = (s.aff || 0) + dt * wi * rA / 12; s.ph = (s.ph || 0) + dt;
    },
    tags: [{ p: [40, 196, -62], t: 'Drive cone' }, { p: [0, 150, 0], t: 'Friction wheel' }, { p: [-110, 226, -22], t: 'Ratio knob' }, { p: [125, 40, 0], t: 'Differential' }, { p: [-160, 60, 60], t: 'Pivot front axle' }],
    build: function (s) {
      var b = B(), ya = 150, wx = s.wx || 0, ain = s.ain || 0, aB = s.aB || 0, ap = s.ap || 0, aw = s.aw || 0, lod = s.lod ? 0.5 : 1, k;
      var alpha = Math.atan(0.2), zc = 0.2 * wx;
      b.box(0, 4, 0, 440, 8, 340, 'light'); b.box(0, 4, 0, 440.4, 1, 340.4, 'mid');
      // cones: radius 22.5 -> 52.5 over 150 mm, opposed, 99 mm between the shafts
      b.grp(chain(T(75, ya, -49.5), RZ(PI / 2), RY(ain)), function () { b.put('frus', Sc(52.5, 150, 52.5), 'mid'); });
      b.grp(chain(T(-75, ya, 49.5), RZ(-PI / 2), RY(aB)), function () { b.put('frus', Sc(52.5, 150, 52.5), 'alu'); });
      [[-49.5, ain, 1, 75], [49.5, aB, -1, -75]].forEach(function (q) { // stripes so the rotation reads
        for (k = 0; k < 3; k++) {
          var th = q[1] + k * 2 * PI / 3, rm = 37.5;
          b.grp(chain(T(0, ya, q[0]), RX(th)), function () { b.box(0, rm + 0.6, 0, 150, 1.4, 5, 'dark', chain(RZ(q[2] > 0 ? -alpha : alpha))); });
        }
      });
      b.rod([-125, ya, -49.5], [102, ya, -49.5], 6, 'steel'); b.rod([-102, ya, 49.5], [118, ya, 49.5], 6, 'steel');
      [-1, 1].forEach(function (sd) {
        b.box(sd * 100, 117, 115, 10, 217, 12, 'light'); b.box(sd * 100, 117, -115, 10, 217, 12, 'light'); b.box(sd * 100, 224, 0, 10, 12, 242, 'light');
        b.box(sd * 102, 70, -49.5, 14, 124, 14, 'mid'); b.box(sd * 102, 70, 49.5, 14, 124, 14, 'mid'); b.box(sd * 102, ya, -49.5, 14, 36, 30, 'mid');
        b.box(sd * 102, ya, 49.5, 14, 36, 30, 'alu'); b.rod([sd * 102, ya, 64.5], [sd * 102, ya, 104], 3.2, 'steel'); b.cyl(sd * 102, ya, 108, 12, 10, 'orange', 'z');
      });
      // friction wheel on its carriage, riding on the tilted lead screw
      b.grp(chain(T(wx, ya, zc), RY(-alpha)), function () {
        b.grp(RX(s.aff || 0), function () { b.cyl(0, 0, 0, 12, 12, 'rubber', 'x'); b.cyl(0, 0, 0, 7, 12.8, 'light', 'x'); b.box(0, 11.4, 0, 12.4, 1.2, 3, 'orange'); });
      });
      b.rod([wx, ya, zc], [wx, ya + 62, zc], 4, 'dark');
      b.grp(chain(T(wx, ya + 62, zc), RY(-alpha)), function () { b.box(0, 0, 0, 24, 14, 24, 'dark'); });
      b.rod([-108, ya + 62, -21.6], [108, ya + 62, 21.6], 4.6, 'steelf');
      b.grp(chain(T(-110, ya + 62, -22), RY(-alpha), RX(wx * 0.35)), function () { b.cyl(0, 0, 0, 15, 14, 'orange', 'x'); b.box(-7.5, 14.5, 0, 1.6, 4, 3, 'dark'); });
      // crank on the input shaft
      b.grp(chain(T(-125, ya, -49.5), RX(ain)), function () { b.box(-4, 30, 0, 8, 60, 12, 'dark'); b.cyl(-18, 56, 0, 7, 30, 'orange', 'x'); });
      // miter pair, pinion and carrier gear
      b.put('cone', chain(T(110, ya, 49.5), RZ(-PI / 2), Sc(15, 15, 15)), 'alu'); b.put('cone', chain(T(125, ya, 34.5), RX(PI / 2), Sc(15, 15, 15)), 'alu');
      b.rod([125, ya, 34.5], [125, ya, -20], 4, 'steel');
      b.gear(125, ya, 0, 18, 12, 'alu', 'z', Math.round(17 * lod + (lod < 1 ? 1 : 0)), ap);
      b.gear(125, 79, 0, 53, 10, 'alu', 'z', Math.round(50 * lod), aw);
      b.cyl(125, 79, 0, 24, 40, 'dark', 'z'); b.cyl(125, 79, 0, 12, 48, 'steelf', 'z');
      // rear axle: differential, band brakes, stands, wheels
      b.rod([125, 79, -178], [125, 79, 178], 5, 'steel');
      [-1, 1].forEach(function (sd) {
        b.cyl(125, 79, sd * 85, 22, 16, 'steelf', 'z'); b.cyl(125, 79, sd * 85, 23.6, 8, 'rubber', 'z'); b.box(125, 79 + 36, sd * 85, 6, 36, 8, 'orange');
        b.box(125, 43.5, sd * 115, 28, 71, 10, 'mid'); b.box(125, 79, sd * 115, 28, 28, 10, 'alu');
        b.grp(chain(T(125, 79, sd * 165), RZ(-aw * (sd > 0 ? 1 : 1))), function () {
          b.cyl(0, 0, 0, 60, 28, 'rubber', 'z'); b.cyl(0, 0, 0, 40, 28.6, 'light', 'z');
          for (k = 0; k < 3; k++) { var a = k * 2 * PI / 3; b.box(24 * cos(a), 24 * sin(a), sd * 14.4, 22, 5, 1.4, 'dark', RZ(a)); }
        });
      });
      // pivoting front axle with rigid stops
      b.cyl(-160, 8, 0, 7, 71, 'steel', 'y'); b.box(-160, 12, 0, 40, 8, 40, 'dark');
      b.grp(chain(T(-160, 79, 0), RY(sin(s.ph || 0) * 0.07 * (s.drive || 0))), function () {
        b.box(0, 0, 0, 18, 18, 330, 'mid'); b.cyl(0, 0, 0, 12, 14, 'steelf', 'y');
        [-1, 1].forEach(function (sd) {
          b.grp(T(0, 0, sd * 165), function () { b.cyl(0, 0, 0, 60, 28, 'rubber', 'z'); b.cyl(0, 0, 0, 40, 28.6, 'light', 'z'); b.cyl(0, 0, sd * 14.6, 6, 2, 'steelf', 'z'); });
        });
      });
      b.box(-160, 79, 24, 24, 24, 6, 'dark'); b.box(-160, 79, -24, 24, 24, 6, 'dark');
      return b.L;
    }
  };

  // ---- Hydroponic tower redesign (APS112): hex tower, angled net-cup ports, cage, reservoir --------------
  M.tower = {
    name: 'Hydroponic tower', R: 365, cy: 290, cam: { yaw: -35 * A, pitch: 14 * A, dist: 1.02 },
    rest: { e: 0, lod: 1 }, cellH: 1.35,
    cap: 'Concept model of the redesigned tower, rebuilt from the concept renders: hexagonal body, six angled net-cup ports per row, a support cage, a central spray head and the reservoir. The net cup itself is the real STL.',
    states: [{ label: 'Assembled', tgt: { e: 0 } }, { label: 'Exploded', tgt: { e: 1 } }],
    tags: [{ p: [30, 346, 40], t: 'Angled net-cup port' }, { p: [90, 190, 70], t: 'Support cage' }, { p: [0, 566, 0], t: 'Spray head' }, { p: [75, 50, 80], t: 'Reservoir' }],
    build: function (s) {
      var b = B(), e = s.e || 0, lod = s.lod ? 1 : 0, rows = 8, r, f, seg = lod ? 10 : 18, R = 45, ap = R * cos(30 * A);
      b.cyl(0, 55, 0, 105, 110, 'green', 'y'); b.cyl(0, 112, 0, 107, 4, 'green', 'y');
      b.put('hexc', chain(T(0, 150 - e * 0, 0), RX(PI), Sc(R, 38, R)), 'red');
      b.put('hex', chain(T(0, 150, 0), Sc(R, 370, R)), 'red');
      b.put('hex', chain(T(0, 519, 0), Sc(R + 1.5, 3, R + 1.5)), 'red');
      for (r = 0; r < rows; r++) {
        var y = 185 + r * 44;
        for (f = 0; f < 6; f++) {
          var ph = (30 + 60 * f) * A, th = 38 * A, dx = cos(ph) * cos(th), dy = sin(th), dz = sin(ph) * cos(th), P = [ap * cos(ph), y, ap * sin(ph)];
          b.rod([P[0] - dx * 8, P[1] - dy * 8, P[2] - dz * 8], [P[0] + dx * 18, P[1] + dy * 18, P[2] + dz * 18], 11, 'red');
          var o = e * 55;
          b.rod([P[0] + dx * (-2 + o), P[1] + dy * (-2 + o), P[2] + dz * (-2 + o)], [P[0] + dx * (22 + o), P[1] + dy * (22 + o), P[2] + dz * (22 + o)], 8.2, 'teal');
        }
      }
      var cageUp = e * 70;
      [45, 135, 225, 315].forEach(function (d) { var a = d * A; b.box(95 * cos(a), 300 + cageUp, 95 * sin(a), 6, 440, 6, 'blue'); });
      [190, 320, 450].forEach(function (y) { b.ring(y + cageUp, 95, seg, 2.4, 'blue'); });
      var top = e * 80;
      [90, 210, 330].forEach(function (d) {
        var a = d * A, x = 36 * cos(a), z = 36 * sin(a);
        b.cyl(x, 565 + top, z, 4, 90, 'red', 'y');
        b.cyl(x, 570 + top, z, 12, 38, 'red', 'y'); b.cyl(x, 551 + top, z, 18, 2.4, 'red', 'y'); b.cyl(x, 589 + top, z, 18, 2.4, 'red', 'y');
      });
      b.cyl(0, 520 + top * 0.5, 0, 4, 44, 'green', 'y'); b.sph(0, 548 + top * 0.5, 0, 14, 'green');
      return b.L;
    }
  };

  // ---- Pneumatic piston (George Brown College machining): grooved piston, rod, crank, flywheel ---------------
  M.piston = {
    name: 'Pneumatic piston', R: 100, cy: 62, cam: { yaw: -40 * A, pitch: 18 * A, dist: 1.0 },
    rest: { e: 0, run: 0, th: 0.5, lod: 1 },
    cap: 'Reconstruction of the machined assembly from the photos: grooved aluminum piston, connecting rod, crank disc, steel flywheel, upright and base plate. Dimensions are approximate.',
    states: [{ label: 'Running', tgt: { e: 0, run: 1 } }, { label: 'Exploded', tgt: { e: 1, run: 0 } }],
    step: function (s, dt) { s.th = (s.th || 0) + dt * 5.2 * (s.run || 0); },
    tags: [{ p: [0, 96, 28], t: 'Grooved piston (lathe)' }, { p: [0, 36, 28], t: 'Crank disc' }, { p: [0, 36, -36], t: 'Flywheel' }, { p: [30, 6, 30], t: 'Base plate (mill)' }],
    build: function (s) {
      var b = B(), e = s.e || 0, th = s.th || 0, CY = 36, rho = 10, Lr = 40, k;
      b.box(0, 3 - e * 20, 0, 84, 6, 78, 'steelf');
      b.box(0, 56, 0, 38, 100, 18, 'steelf');
      b.rod([0, CY, -32 - e * 30], [0, CY, 26 + e * 34], 5, 'steel');
      b.grp(chain(T(0, CY, -31 - e * 52), RZ(-th)), function () { b.cyl(0, 0, 0, 27, 26, 'steelf', 'z'); b.cyl(0, 0, 13.2, 9, 0.8, 'dark', 'z'); b.box(19, 0, 13.4, 14, 3, 0.8, 'dark'); });
      b.grp(chain(T(0, CY, 12 + e * 16), RZ(-th)), function () { b.cyl(0, 0, 0, 19, 6, 'alu', 'z'); b.cyl(0, 0, 3.4, 4, 1, 'steel', 'z'); b.box(8, 0, 3.4, 12, 2.4, 0.6, 'dark'); });
      var px = rho * cos(th), py = CY + rho * sin(th), pz = 20 + e * 34, yp = py + Math.sqrt(Math.max(1, Lr * Lr - px * px)) + e * 18;
      b.grp(chain(T(px, py, 0), I()), function () { });
      b.box(px, py, 18 + e * 25, 12, 12, 10, 'alu'); b.cyl(px, py, 24 + e * 25, 4.2, 4, 'black', 'z');
      b.rod([px, py, 22 + e * 34], [0, yp, 22 + e * 34], 3, 'steel');
      // piston: three grooves between four lands, then the milled top block with its set screw
      var z0 = 22 + e * 34, y0 = yp + 2 + e * 14;
      for (k = 0; k < 4; k++) b.cyl(0, y0 + k * 7, z0, 12, 4, 'alu', 'y');
      for (k = 0; k < 3; k++) b.cyl(0, y0 + 4 + k * 7, z0, 9.6, 3, 'alu', 'y');
      b.box(0, y0 + 29, z0, 24, 14, 24, 'alu'); b.cyl(0, y0 + 29, z0 + 12.2, 5, 4, 'black', 'z'); b.cyl(0, y0 + 29, z0 + 14.4, 2.4, 1, 'dark', 'z');
      return b.L;
    }
  };

  // ================================================================================================
  //  bounds + cell (pedestal) models
  // ================================================================================================
  function bounds(parts) {
    var mn = [1e9, 1e9, 1e9], mx = [-1e9, -1e9, -1e9];
    parts.forEach(function (p) {
      var lo = p.p === 'box' ? -0.5 : -1, hi = p.p === 'box' ? 0.5 : 1, y0 = (p.p === 'box' || p.p === 'sph') ? lo : 0, y1 = p.p === 'box' ? 0.5 : p.p === 'sph' ? 1 : 1, m = p.M;
      for (var i = 0; i < 8; i++) {
        var x = i & 1 ? hi : lo, y = i & 2 ? y1 : y0, z = i & 4 ? hi : lo;
        var X = m[0] * x + m[4] * y + m[8] * z + m[12], Y = m[1] * x + m[5] * y + m[9] * z + m[13], Z = m[2] * x + m[6] * y + m[10] * z + m[14];
        if (X < mn[0]) mn[0] = X; if (Y < mn[1]) mn[1] = Y; if (Z < mn[2]) mn[2] = Z; if (X > mx[0]) mx[0] = X; if (Y > mx[1]) mx[1] = Y; if (Z > mx[2]) mx[2] = Z;
      }
    });
    return { mn: mn, mx: mx };
  }
  function cellParts(id) {
    var m = M[id]; if (!m) return null;
    var s = {}, k; for (k in m.rest) s[k] = m.rest[k]; if (m.cellState) for (k in m.cellState) s[k] = m.cellState[k];
    s.lod = 1; var L = m.build(s);
    if (m.cellRot === 'side') L = L.map(function (p) { return { p: p.p, M: mul(mul(RY(-PI / 2 + 0.5), RZ(PI / 2)), p.M), c: p.c }; });
    var bb = bounds(L), dx = bb.mx[0] - bb.mn[0], dy = bb.mx[1] - bb.mn[1], dz = bb.mx[2] - bb.mn[2];
    var sc = Math.min(0.96 / Math.max(dx, dz), (m.cellH || 0.9) / dy), cx = (bb.mx[0] + bb.mn[0]) / 2, cz = (bb.mx[2] + bb.mn[2]) / 2;
    var pre = chain(Sc(sc, sc, sc), T(-cx, -bb.mn[1], -cz));
    return L.map(function (p) { return { p: p.p, M: mul(pre, p.M), c: p.c }; });
  }

  // ================================================================================================
  //  geometry factories (shared with the workcell renderer)
  // ================================================================================================
  var geo = {
    box: function () {
      var P = [], I2 = [], f = [
        [[0, 0, 1], [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], [[0, 0, -1], [1, -1, -1], [-1, -1, -1], [-1, 1, -1], [1, 1, -1]],
        [[1, 0, 0], [1, -1, 1], [1, -1, -1], [1, 1, -1], [1, 1, 1]], [[-1, 0, 0], [-1, -1, -1], [-1, -1, 1], [-1, 1, 1], [-1, 1, -1]],
        [[0, 1, 0], [-1, 1, 1], [1, 1, 1], [1, 1, -1], [-1, 1, -1]], [[0, -1, 0], [-1, -1, -1], [1, -1, -1], [1, -1, 1], [-1, -1, 1]]];
      f.forEach(function (q, k) { for (var i = 1; i < 5; i++) P.push(q[i][0] * 0.5, q[i][1] * 0.5, q[i][2] * 0.5, q[0][0], q[0][1], q[0][2]); I2.push(k * 4, k * 4 + 1, k * 4 + 2, k * 4, k * 4 + 2, k * 4 + 3); });
      return [P, I2];
    },
    cyl: function (n, tr) {
      var P = [], I2 = [], k, s = hyp(1, 1 - tr);
      for (k = 0; k <= n; k++) { var a = k / n * PI * 2, c = cos(a), sn = sin(a); P.push(c, 0, sn, c / s, (1 - tr) / s, sn / s, c * tr, 1, sn * tr, c / s, (1 - tr) / s, sn / s); }
      for (k = 0; k < n; k++) I2.push(k * 2, k * 2 + 1, k * 2 + 3, k * 2, k * 2 + 3, k * 2 + 2);
      var base = P.length / 6; P.push(0, 0, 0, 0, -1, 0); for (k = 0; k <= n; k++) { var b = k / n * PI * 2; P.push(cos(b), 0, sin(b), 0, -1, 0); }
      for (k = 0; k < n; k++) I2.push(base, base + 2 + k, base + 1 + k);
      var top = P.length / 6; P.push(0, 1, 0, 0, 1, 0); for (k = 0; k <= n; k++) { var d = k / n * PI * 2; P.push(cos(d) * tr, 1, sin(d) * tr, 0, 1, 0); }
      for (k = 0; k < n; k++) I2.push(top, top + 1 + k, top + 2 + k);
      return [P, I2];
    },
    prism: function (n, tr) { // flat-shaded n-gon prism, optionally tapered
      var P = [], I2 = [], k, s = hyp(1, 1 - tr);
      for (k = 0; k < n; k++) {
        var a0 = k / n * 2 * PI, a1 = (k + 1) / n * 2 * PI, am = (a0 + a1) / 2, nx = cos(am) / s, nz = sin(am) / s, ny = (1 - tr) / s, b0 = P.length / 6;
        P.push(cos(a0), 0, sin(a0), nx, ny, nz, cos(a1), 0, sin(a1), nx, ny, nz, cos(a1) * tr, 1, sin(a1) * tr, nx, ny, nz, cos(a0) * tr, 1, sin(a0) * tr, nx, ny, nz);
        I2.push(b0, b0 + 3, b0 + 2, b0, b0 + 2, b0 + 1);
      }
      var base = P.length / 6; P.push(0, 0, 0, 0, -1, 0); for (k = 0; k <= n; k++) P.push(cos(k / n * 2 * PI), 0, sin(k / n * 2 * PI), 0, -1, 0);
      for (k = 0; k < n; k++) I2.push(base, base + 2 + k, base + 1 + k);
      var top = P.length / 6; P.push(0, 1, 0, 0, 1, 0); for (k = 0; k <= n; k++) P.push(cos(k / n * 2 * PI) * tr, 1, sin(k / n * 2 * PI) * tr, 0, 1, 0);
      for (k = 0; k < n; k++) I2.push(top, top + 1 + k, top + 2 + k);
      return [P, I2];
    },
    sph: function (la, lo) {
      var P = [], I2 = [], i, j;
      for (i = 0; i <= la; i++) for (j = 0; j <= lo; j++) { var th = i / la * PI, ph = j / lo * PI * 2, x = sin(th) * cos(ph), y = cos(th), z = sin(th) * sin(ph); P.push(x, y, z, x, y, z); }
      for (i = 0; i < la; i++) for (j = 0; j < lo; j++) { var a = i * (lo + 1) + j, c = a + lo + 1; I2.push(a, c, a + 1, a + 1, c, c + 1); }
      return [P, I2];
    },
    quad: function () { return [[-1, 0, -1, 0, 1, 0, 1, 0, -1, 0, 1, 0, 1, 0, 1, 0, 1, 0, -1, 0, 1, 0, 1, 0], [0, 1, 2, 0, 2, 3]]; }
  };
  var CONE_TR = 22.5 / 52.5;
  function primSet() { return { box: geo.box(), cyl: geo.cyl(32, 1), hex: geo.prism(6, 1), hexc: geo.prism(6, 0.3), frus: geo.cyl(40, CONE_TR), cone: geo.cyl(32, 0.35), sph: geo.sph(14, 20), quad: geo.quad() }; }

  // ================================================================================================
  //  card viewer: orbitable, theme aware, animated 3D view of one model
  // ================================================================================================
  var VS = 'attribute vec3 aP;attribute vec3 aN;uniform mat4 uVP;uniform mat4 uM;uniform mat3 uNM;varying vec3 vN;varying vec3 vW;varying vec3 vL;' +
    'void main(){vec4 w=uM*vec4(aP,1.);vW=w.xyz;vN=uNM*aN;vL=aP;gl_Position=uVP*w;}';
  var FS = 'precision mediump float;varying vec3 vN;varying vec3 vW;varying vec3 vL;uniform vec3 uCol;uniform float uA;uniform float uMode;uniform float uSpec;uniform float uShin;uniform vec3 uEye;uniform vec3 uLight;' +
    'void main(){' +
    'if(uMode>1.5&&uMode<2.5){float l=length(vL.xz);gl_FragColor=vec4(uCol,uA*(1.-smoothstep(.15,1.,l)));return;}' +
    'if(uMode>2.5){float l=length(vL.xz);float r=smoothstep(.9,.93,l)-smoothstep(.97,1.,l);gl_FragColor=vec4(uCol,uA*r);return;}' +
    'vec3 N=normalize(vN);vec3 Ld=normalize(uLight);vec3 V=normalize(uEye-vW);float df=max(dot(N,Ld),0.);float hemi=.5+.5*N.y;vec3 H=normalize(Ld+V);' +
    'float sp=pow(max(dot(N,H),0.),uShin)*uSpec;float rim=pow(1.-max(dot(N,V),0.),3.)*.1;' +
    'vec3 c=uCol*(.42+.2*hemi+.52*df)+vec3(sp)+vec3(rim);gl_FragColor=vec4(c,1.);}';
  function normalMat(m) {
    var a = m[0], b = m[1], c = m[2], d = m[4], e = m[5], f = m[6], g = m[8], h = m[9], i = m[10];
    var A1 = e * i - f * h, B1 = f * g - d * i, C1 = d * h - e * g, det = a * A1 + b * B1 + c * C1 || 1, k = 1 / det;
    return new Float32Array([A1 * k, B1 * k, C1 * k, (c * h - b * i) * k, (a * i - c * g) * k, (b * g - a * h) * k, (b * f - c * e) * k, (c * d - a * f) * k, (a * e - b * d) * k]);
  }
  function lookAt(e, t, up) {
    var f = norm(sub(t, e)), r = norm(cross(f, up)), u = cross(r, f), m = I();
    m[0] = r[0]; m[4] = r[1]; m[8] = r[2]; m[1] = u[0]; m[5] = u[1]; m[9] = u[2]; m[2] = -f[0]; m[6] = -f[1]; m[10] = -f[2];
    m[12] = -dot(r, e); m[13] = -dot(u, e); m[14] = dot(f, e); return m;
  }
  function persp(fov, asp, n, f) { var t = 1 / Math.tan(fov / 2), m = new Float32Array(16); m[0] = t / asp; m[5] = t; m[10] = (f + n) / (n - f); m[11] = -1; m[14] = 2 * f * n / (n - f); return m; }
  function parseCol(s) {
    s = (s || '').trim(); var m = /^#([0-9a-f]{3}|[0-9a-f]{6})$/i.exec(s);
    if (m) { var h = m[1]; if (h.length === 3) h = h.replace(/./g, '$&$&'); return [parseInt(h.slice(0, 2), 16) / 255, parseInt(h.slice(2, 4), 16) / 255, parseInt(h.slice(4, 6), 16) / 255]; }
    m = /rgba?\(([^)]+)\)/.exec(s); if (m) { var p = m[1].split(',').map(parseFloat); return [p[0] / 255, p[1] / 255, p[2] / 255]; }
    return [0.5, 0.5, 0.5];
  }
  var mixc = function (a, b, t) { return [a[0] * (1 - t) + b[0] * t, a[1] * (1 - t) + b[1] * t, a[2] * (1 - t) + b[2] * t]; };
  function readPalette() {
    var cs = getComputedStyle(document.documentElement), g = function (n) { return parseCol(cs.getPropertyValue(n)); }, ink = g('--ink'), card = g('--card'), p = {};
    p.bg = g('--soft'); p.ink = ink; p.dark = mixc(ink, card, 0.08); p.mid = mixc(ink, card, 0.5); p.light = mixc(ink, card, 0.8); p.steel = mixc(ink, card, 0.62);
    p.body = mixc(ink, card, 0.1); p.dim = ink[0] > 0.5; return p;
  }
  var LIGHT = norm([0.45, 0.95, 0.55]);

  var _ok = null;
  function supported() {
    if (_ok !== null) return _ok;
    try { var c = document.createElement('canvas'); _ok = !!(c.getContext('webgl') || c.getContext('experimental-webgl')); } catch (e) { _ok = false; }
    return _ok;
  }

  function createPartsViewer(host, id, o) {
    o = o || {};
    var model = M[id]; if (!model || !supported()) return null;
    var canvas = document.createElement('canvas'), gl = null;
    try { gl = canvas.getContext('webgl', { antialias: true, alpha: false, powerPreference: 'low-power' }) || canvas.getContext('experimental-webgl'); } catch (e) { gl = null; }
    if (!gl) return null;
    function sh(type, src) { var s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s); if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s)); return s; }
    var prog;
    try { prog = gl.createProgram(); gl.attachShader(prog, sh(gl.VERTEX_SHADER, VS)); gl.attachShader(prog, sh(gl.FRAGMENT_SHADER, FS)); gl.linkProgram(prog); if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error('link'); } catch (e) { return null; }
    gl.useProgram(prog);
    var U = {}; ['uVP', 'uM', 'uNM', 'uCol', 'uA', 'uMode', 'uSpec', 'uShin', 'uEye', 'uLight'].forEach(function (n) { U[n] = gl.getUniformLocation(prog, n); });
    var aP = gl.getAttribLocation(prog, 'aP'), aN = gl.getAttribLocation(prog, 'aN'); gl.enableVertexAttribArray(aP); gl.enableVertexAttribArray(aN);
    var G = {}, ps = primSet();
    Object.keys(ps).forEach(function (k) {
      var vb = gl.createBuffer(), ib = gl.createBuffer();
      gl.bindBuffer(gl.ARRAY_BUFFER, vb); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(ps[k][0]), gl.STATIC_DRAW);
      gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ib); gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint16Array(ps[k][1]), gl.STATIC_DRAW);
      G[k] = { vb: vb, ib: ib, n: ps[k][1].length };
    });

    canvas.className = 'pv-canvas'; canvas.tabIndex = 0; canvas.setAttribute('role', 'img');
    canvas.setAttribute('aria-label', 'Interactive 3D model of ' + model.name + '. Drag to rotate; use the arrow keys when focused.');
    var tagsEl = document.createElement('div'); tagsEl.className = 'pv-tags'; tagsEl.setAttribute('aria-hidden', 'true');
    var hint = document.createElement('div'); hint.className = 'pv-hint'; hint.textContent = 'Drag to rotate'; hint.setAttribute('aria-hidden', 'true');
    host.appendChild(canvas); host.appendChild(tagsEl); host.appendChild(hint);

    var pal = readPalette(), cur = {}, tgt = {}, k, view = null, W = 1, H = 1, dpr = 1;
    for (k in model.rest) { cur[k] = model.rest[k]; tgt[k] = model.rest[k]; }
    cur.R = model.R; cur.cy = model.cy; cur.cx = model.cx || 0; cur.cyaw = 0; cur.cpitch = 0; tgt.cyaw = 0; tgt.cpitch = 0; tgt.R = model.R; tgt.cy = model.cy; tgt.cx = cur.cx; cur.t = 0; cur.lod = 0; tgt.lod = 0;
    var yaw = model.cam.yaw, pitch = model.cam.pitch, userT = -1e9, drag = null, zoom = 1, visible = true, alive = true, raf = 0, last = 0, auto = !reduce, ptr = null;
    var tagEls = [];
    function colOf(c) { return pal[c] || FIXED[c] || pal.mid; }

    function setState(i, snap) {
      var st = model.states[clamp(i, 0, model.states.length - 1)];
      for (k in st.tgt) tgt[k] = st.tgt[k];
      if (snap || reduce || !visible) { for (k in st.tgt) cur[k] = st.tgt[k]; }
      kick();
    }
    function resize() {
      var r = host.getBoundingClientRect(); dpr = Math.min(2, window.devicePixelRatio || 1);
      W = Math.max(2, Math.round(r.width * dpr)); H = Math.max(2, Math.round(r.height * dpr));
      if (canvas.width !== W || canvas.height !== H) { canvas.width = W; canvas.height = H; }
      kick();
    }
    function camera() {
      var asp = W / H, fov = 34 * A, R = cur.R, dist = R / Math.sin(fov / 2) * 1.04 * (model.cam.dist || 1) * Math.max(1, 0.95 / asp) * zoom;
      var yw = yaw + (cur.cyaw || 0), pt = clamp(pitch + (cur.cpitch || 0), 0.03, 1.4), cp = cos(pt), tg = [cur.cx || 0, cur.cy, 0], e = [tg[0] + dist * sin(yw) * cp, tg[1] + dist * sin(pt), tg[2] + dist * cos(yw) * cp];
      var P = persp(fov, asp, dist * 0.05, dist * 6);
      view = { eye: e, VP: mul(P, lookAt(e, tg, [0, 1, 0])), dist: dist };
    }
    function drawPart(p) {
      var Mx = p.M, pr = G[p.p]; if (!pr) return;
      gl.uniformMatrix4fv(U.uM, false, Mx); gl.uniformMatrix3fv(U.uNM, false, normalMat(Mx));
      gl.uniform3fv(U.uCol, colOf(p.c)); var sp = SPEC[p.c] || [0.14, 22]; gl.uniform1f(U.uSpec, sp[0]); gl.uniform1f(U.uShin, sp[1]); gl.uniform1f(U.uMode, 0); gl.uniform1f(U.uA, 1);
      gl.bindBuffer(gl.ARRAY_BUFFER, pr.vb); gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, pr.ib);
      gl.vertexAttribPointer(aP, 3, gl.FLOAT, false, 24, 0); gl.vertexAttribPointer(aN, 3, gl.FLOAT, false, 24, 12);
      gl.drawElements(gl.TRIANGLES, pr.n, gl.UNSIGNED_SHORT, 0);
    }
    function render() {
      camera(); var parts = model.build(cur);
      gl.viewport(0, 0, W, H); gl.clearColor(pal.bg[0], pal.bg[1], pal.bg[2], 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      gl.enable(gl.DEPTH_TEST); gl.depthFunc(gl.LEQUAL); gl.disable(gl.BLEND);
      gl.uniformMatrix4fv(U.uVP, false, view.VP); gl.uniform3fv(U.uEye, view.eye); gl.uniform3fv(U.uLight, LIGHT);
      for (var i = 0; i < parts.length; i++) drawPart(parts[i]);
      gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA); gl.depthMask(false);
      var pr = G.quad; gl.bindBuffer(gl.ARRAY_BUFFER, pr.vb); gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, pr.ib);
      gl.vertexAttribPointer(aP, 3, gl.FLOAT, false, 24, 0); gl.vertexAttribPointer(aN, 3, gl.FLOAT, false, 24, 12);
      var rr = cur.R * 1.0, sd = [0, 0, 0];
      gl.uniform3fv(U.uCol, sd); gl.uniform1f(U.uMode, 2); gl.uniform1f(U.uA, pal.dim ? 0.45 : 0.2);
      var Ms = chain(T(cur.cx || 0, 0.3, 0), Sc(rr, 1, rr)); gl.uniformMatrix4fv(U.uM, false, Ms); gl.uniformMatrix3fv(U.uNM, false, normalMat(Ms)); gl.drawElements(gl.TRIANGLES, pr.n, gl.UNSIGNED_SHORT, 0);
      gl.uniform3fv(U.uCol, pal.ink); gl.uniform1f(U.uMode, 3); gl.uniform1f(U.uA, 0.22);
      Ms = chain(T(cur.cx || 0, 0.2, 0), Sc(cur.R * 0.92, 1, cur.R * 0.92)); gl.uniformMatrix4fv(U.uM, false, Ms); gl.drawElements(gl.TRIANGLES, pr.n, gl.UNSIGNED_SHORT, 0);
      gl.depthMask(true); gl.disable(gl.BLEND);
      placeTags();
    }
    function placeTags() {
      var list = model.tagsFor ? model.tagsFor(cur) : model.tags;
      if (!list || host.clientWidth < 430) { tagsEl.style.display = 'none'; return; }
      tagsEl.style.display = '';
      if (tagEls.length !== list.length || tagEls._src !== list) {
        tagsEl.innerHTML = ''; tagEls = list.map(function (t) { var d = document.createElement('span'); d.className = 'pv-tag'; d.textContent = t.t; tagsEl.appendChild(d); return d; }); tagEls._src = list;
      }
      var cw = host.clientWidth, ch = host.clientHeight;
      list.forEach(function (t, i) {
        var p = t.p, v = view.VP, x = v[0] * p[0] + v[4] * p[1] + v[8] * p[2] + v[12], y = v[1] * p[0] + v[5] * p[1] + v[9] * p[2] + v[13], w = v[3] * p[0] + v[7] * p[1] + v[11] * p[2] + v[15];
        var el = tagEls[i], on = w > 0 && (!t.when || t.when(cur)) && (cur.e || 0) < 0.35;
        if (!on) { el.style.opacity = 0; return; }
        var px = (x / w * 0.5 + 0.5) * cw, py = (1 - (y / w * 0.5 + 0.5)) * ch;
        el.style.opacity = 1; el.style.transform = 'translate(' + Math.round(px) + 'px,' + Math.round(py) + 'px) translate(-50%,-130%)';
      });
    }
    function frame(t) {
      raf = 0; if (!alive) return;
      var dt = Math.min(0.05, last ? (t - last) / 1000 : 0.016); last = t;
      var busy = false, kk = 1 - Math.exp(-dt * 5.5);
      for (k in tgt) { if (k === 't' || typeof tgt[k] !== 'number') continue; var d = tgt[k] - cur[k]; if (Math.abs(d) > 1e-4) { cur[k] += d * kk; busy = true; } else cur[k] = tgt[k]; }
      if (!reduce) { cur.t += dt; model.step && model.step(cur, dt); }
      if (auto && !drag && performance.now() - userT > 5000) yaw += dt * 0.16;
      render();
      if (visible && !document.hidden && (!reduce || busy)) raf = requestAnimationFrame(frame); else last = 0;
    }
    function kick() { if (!raf && alive && visible) raf = requestAnimationFrame(frame); }

    // pointer orbit (horizontal drag rotates; vertical swipe on touch still scrolls the page)
    canvas.style.touchAction = 'pan-y';
    canvas.addEventListener('pointerdown', function (e) { drag = { x: e.clientX, y: e.clientY, yaw: yaw, pitch: pitch, id: e.pointerId, moved: false }; userT = performance.now(); try { canvas.setPointerCapture(e.pointerId); } catch (er) { } hint.style.opacity = 0; });
    canvas.addEventListener('pointermove', function (e) {
      if (!drag || e.pointerId !== drag.id) return; var dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      if (!drag.moved && Math.abs(dx) < 3 && Math.abs(dy) < 3) return; drag.moved = true;
      yaw = drag.yaw - dx * 0.008; pitch = clamp(drag.pitch + dy * 0.006, 0.05, 1.35); userT = performance.now(); kick();
      if (e.pointerType === 'touch' && Math.abs(dx) > Math.abs(dy)) e.preventDefault();
    });
    function end(e) { if (drag && e.pointerId === drag.id) { drag = null; userT = performance.now(); } }
    canvas.addEventListener('pointerup', end); canvas.addEventListener('pointercancel', end);
    canvas.addEventListener('dblclick', function () { yaw = model.cam.yaw; pitch = model.cam.pitch; zoom = 1; userT = performance.now(); kick(); });
    canvas.addEventListener('wheel', function (e) { if (!e.ctrlKey && !e.metaKey) return; e.preventDefault(); zoom = clamp(zoom * (1 + e.deltaY * 0.0025), 0.45, 1.6); userT = performance.now(); kick(); }, { passive: false });
    canvas.addEventListener('keydown', function (e) {
      var m = { ArrowLeft: [-0.12, 0], ArrowRight: [0.12, 0], ArrowUp: [0, -0.08], ArrowDown: [0, 0.08] }[e.key]; if (!m) return;
      e.preventDefault(); yaw += m[0]; pitch = clamp(pitch + m[1], 0.05, 1.35); userT = performance.now(); kick();
    });
    canvas.addEventListener('webglcontextlost', function (e) { e.preventDefault(); if (!alive) return; alive = false; if (o.onFail) o.onFail(); });

    var ro = window.ResizeObserver ? new ResizeObserver(resize) : null; if (ro) ro.observe(host); else window.addEventListener('resize', resize);
    var io = window.IntersectionObserver ? new IntersectionObserver(function (es) { visible = es[0].isIntersecting; if (visible) kick(); }, { threshold: 0.01 }) : null; if (io) io.observe(host);
    var mo = new MutationObserver(function () { pal = readPalette(); kick(); }); mo.observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme', 'class'] });
    var vis = function () { if (!document.hidden) kick(); }; document.addEventListener('visibilitychange', vis);

    resize(); setState(o.state || 0, true); setTimeout(function () { hint.style.opacity = 0; }, 5500);
    var api = {
      model: model, setState: setState,
      destroy: function () { alive = false; if (raf) cancelAnimationFrame(raf); try { var lc = gl.getExtension('WEBGL_lose_context'); if (lc) lc.loseContext(); } catch (er) { } if (ro) ro.disconnect(); if (io) io.disconnect(); mo.disconnect(); document.removeEventListener('visibilitychange', vis); window.removeEventListener('resize', resize); if (canvas.parentNode) canvas.parentNode.removeChild(canvas); tagsEl.remove(); hint.remove(); },
      canvas: canvas, state: cur, get yaw() { return yaw; }, get pitch() { return pitch; }
    };
    return api;
  }

  window.PM = { models: M, FIXED: FIXED, geo: geo, cellParts: cellParts, createPartsViewer: createPartsViewer, supported: supported, bounds: bounds, build: function (id, s) { return M[id].build(s || M[id].rest); } };
})();
