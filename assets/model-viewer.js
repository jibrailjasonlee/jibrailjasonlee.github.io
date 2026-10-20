/* Small dependency-free WebGL viewer for binary STL files.
   Usage: createModelViewer(hostElement, 'model.stl', {yaw, pitch, dist}) -> Promise<api>
   hostElement must contain a <canvas>. api: start(), stop(), reset(), destroy(). */
(function () {
  'use strict';

  function parseSTL(buf) {
    var dv = new DataView(buf), n = dv.getUint32(80, true);
    if (84 + n * 50 !== buf.byteLength) throw new Error('Only binary STL files are supported');
    var raw = new Float32Array(n * 9), fn = new Float32Array(n * 3), area = new Float32Array(n), i, k, o;
    for (i = 0; i < n; i++) {
      o = 84 + i * 50;
      for (k = 0; k < 9; k++) raw[i * 9 + k] = dv.getFloat32(o + 12 + k * 4, true);
    }
    // centre on the bounding box and scale the bounding sphere to radius 1
    var mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
    for (i = 0; i < raw.length; i += 3) for (k = 0; k < 3; k++) { mn[k] = Math.min(mn[k], raw[i + k]); mx[k] = Math.max(mx[k], raw[i + k]); }
    var c = [(mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, (mn[2] + mx[2]) / 2], r = 0;
    for (i = 0; i < raw.length; i += 3) r = Math.max(r, Math.hypot(raw[i] - c[0], raw[i + 1] - c[1], raw[i + 2] - c[2]));
    var key = [];
    for (i = 0; i < raw.length; i += 3) {
      key.push(Math.round(raw[i] * 1000) + ',' + Math.round(raw[i + 1] * 1000) + ',' + Math.round(raw[i + 2] * 1000));
      raw[i] = (raw[i] - c[0]) / r; raw[i + 1] = (raw[i + 1] - c[1]) / r; raw[i + 2] = (raw[i + 2] - c[2]) / r;
    }
    // face normals: from the winding, oriented to agree with the normal stored in the file
    for (i = 0; i < n; i++) {
      var p = i * 9, ux = raw[p + 3] - raw[p], uy = raw[p + 4] - raw[p + 1], uz = raw[p + 5] - raw[p + 2],
          vx = raw[p + 6] - raw[p], vy = raw[p + 7] - raw[p + 1], vz = raw[p + 8] - raw[p + 2],
          nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx, len = Math.hypot(nx, ny, nz) || 1;
      o = 84 + i * 50;
      var sx = dv.getFloat32(o, true), sy = dv.getFloat32(o + 4, true), sz = dv.getFloat32(o + 8, true);
      var flip = (nx * sx + ny * sy + nz * sz) < 0 ? -1 : 1;
      fn[i * 3] = flip * nx / len; fn[i * 3 + 1] = flip * ny / len; fn[i * 3 + 2] = flip * nz / len; area[i] = len;
    }
    // smooth normals across shallow edges, keep hard edges (crease angle about 40 degrees)
    var byVertex = {}, cosCrease = Math.cos(40 * Math.PI / 180);
    key.forEach(function (kk, idx) { (byVertex[kk] = byVertex[kk] || []).push((idx / 3) | 0); });
    var nor = new Float32Array(n * 9);
    for (i = 0; i < n; i++) for (k = 0; k < 3; k++) {
      var list = byVertex[key[i * 3 + k]], sx2 = 0, sy2 = 0, sz2 = 0, j, f;
      for (j = 0; j < list.length; j++) {
        f = list[j];
        if (fn[f * 3] * fn[i * 3] + fn[f * 3 + 1] * fn[i * 3 + 1] + fn[f * 3 + 2] * fn[i * 3 + 2] >= cosCrease) {
          sx2 += fn[f * 3] * area[f]; sy2 += fn[f * 3 + 1] * area[f]; sz2 += fn[f * 3 + 2] * area[f];
        }
      }
      var l2 = Math.hypot(sx2, sy2, sz2) || 1;
      nor[i * 9 + k * 3] = sx2 / l2; nor[i * 9 + k * 3 + 1] = sy2 / l2; nor[i * 9 + k * 3 + 2] = sz2 / l2;
    }
    var data = new Float32Array(n * 18);
    for (i = 0; i < n * 3; i++) {
      data.set(raw.subarray(i * 3, i * 3 + 3), i * 6);
      data.set(nor.subarray(i * 3, i * 3 + 3), i * 6 + 3);
    }
    return { data: data, count: n * 3 };
  }

  function mul(a, b) {
    var o = new Float32Array(16);
    for (var c = 0; c < 4; c++) for (var r = 0; r < 4; r++) {
      var s = 0; for (var k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k]; o[c * 4 + r] = s;
    }
    return o;
  }
  function rotX(a) { var c = Math.cos(a), s = Math.sin(a); return new Float32Array([1, 0, 0, 0, 0, c, s, 0, 0, -s, c, 0, 0, 0, 0, 1]); }
  function rotY(a) { var c = Math.cos(a), s = Math.sin(a); return new Float32Array([c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, 0, 0, 0, 1]); }
  function persp(fovy, asp, n, f) {
    var t = 1 / Math.tan(fovy / 2);
    return new Float32Array([t / asp, 0, 0, 0, 0, t, 0, 0, 0, 0, (f + n) / (n - f), -1, 0, 0, 2 * f * n / (n - f), 0]);
  }

  var VS = 'attribute vec3 aPos;attribute vec3 aNor;uniform mat4 uMVP;uniform mat3 uRot;varying vec3 vN;' +
    'void main(){vN=uRot*aNor;gl_Position=uMVP*vec4(aPos,1.0);}';
  var FS = 'precision mediump float;varying vec3 vN;uniform vec3 uColor;' +
    'void main(){vec3 n=normalize(vN);vec3 L1=normalize(vec3(-0.5,0.7,0.6));vec3 L2=normalize(vec3(0.7,-0.2,0.5));' +
    'float d=max(dot(n,L1),0.0)*0.72+max(dot(n,L2),0.0)*0.28;float hemi=0.5+0.5*n.y;' +
    'vec3 col=uColor*(0.26+0.2*hemi+d);vec3 H=normalize(L1+vec3(0.0,0.0,1.0));' +
    'col+=vec3(pow(max(dot(n,H),0.0),40.0)*0.22);gl_FragColor=vec4(col,1.0);}';

  function shader(gl, type, src) {
    var s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
    return s;
  }

  window.createModelViewer = function (host, url, opts) {
    opts = opts || {};
    var canvas = host.querySelector('canvas');
    var gl = canvas.getContext('webgl', { antialias: true, alpha: true, premultipliedAlpha: true });
    if (!gl) return Promise.reject(new Error('WebGL is not available'));
    var def = { yaw: opts.yaw != null ? opts.yaw : 0.6, pitch: opts.pitch != null ? opts.pitch : 0.5, dist: opts.dist || 3.3 };
    var yaw = def.yaw, pitch = def.pitch, dist = def.dist;
    var running = false, raf = 0, auto = !matchMedia('(prefers-reduced-motion:reduce)').matches, last = 0, destroyed = false;
    var pointers = {}, pinch = 0;

    return fetch(url).then(function (r) {
      if (!r.ok) throw new Error('Could not load ' + url);
      return r.arrayBuffer();
    }).then(function (buf) {
      var mesh = parseSTL(buf);
      var prog = gl.createProgram();
      gl.attachShader(prog, shader(gl, gl.VERTEX_SHADER, VS));
      gl.attachShader(prog, shader(gl, gl.FRAGMENT_SHADER, FS));
      gl.linkProgram(prog);
      if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
      gl.useProgram(prog);
      var vb = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, vb); gl.bufferData(gl.ARRAY_BUFFER, mesh.data, gl.STATIC_DRAW);
      var aPos = gl.getAttribLocation(prog, 'aPos'), aNor = gl.getAttribLocation(prog, 'aNor');
      gl.enableVertexAttribArray(aPos); gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 24, 0);
      gl.enableVertexAttribArray(aNor); gl.vertexAttribPointer(aNor, 3, gl.FLOAT, false, 24, 12);
      var uMVP = gl.getUniformLocation(prog, 'uMVP'), uRot = gl.getUniformLocation(prog, 'uRot'), uColor = gl.getUniformLocation(prog, 'uColor');
      gl.uniform3f(uColor, 0.1, 0.6, 0.52);
      gl.enable(gl.DEPTH_TEST);

      function resize() {
        var dpr = Math.min(window.devicePixelRatio || 1, 2), w = Math.max(1, Math.round(canvas.clientWidth * dpr)), h = Math.max(1, Math.round(canvas.clientHeight * dpr));
        if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
      }
      function draw() {
        resize();
        gl.viewport(0, 0, canvas.width, canvas.height);
        gl.clearColor(0, 0, 0, 0); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
        var rot = mul(rotX(pitch), rotY(yaw));
        var asp = canvas.width / canvas.height, fit = asp < 0.95 ? 0.95 / asp : 1; // keep the whole model in view on narrow screens
        var view = mul(new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, -dist * fit, 1]), rot);
        gl.uniformMatrix4fv(uMVP, false, mul(persp(35 * Math.PI / 180, asp, 0.1, 40), view));
        gl.uniformMatrix3fv(uRot, false, new Float32Array([rot[0], rot[1], rot[2], rot[4], rot[5], rot[6], rot[8], rot[9], rot[10]]));
        gl.drawArrays(gl.TRIANGLES, 0, mesh.count);
        if (!host.dataset.ready) { host.dataset.ready = '1'; host.dispatchEvent(new CustomEvent('model-ready')); }
      }
      function frame(t) {
        raf = 0; if (!running || destroyed) return;
        if (auto) { yaw += Math.min(0.05, (t - (last || t)) / 1000) * 0.45; last = t; }
        draw();
        if (auto) raf = requestAnimationFrame(frame);
      }
      function invalidate() { if (running && !raf && !destroyed) raf = requestAnimationFrame(frame); }
      function userTook() { auto = false; }
      function clampPitch() { pitch = Math.max(-1.45, Math.min(1.45, pitch)); }
      function clampDist() { dist = Math.max(1.6, Math.min(8, dist)); }

      canvas.addEventListener('pointerdown', function (e) {
        canvas.setPointerCapture(e.pointerId); pointers[e.pointerId] = { x: e.clientX, y: e.clientY }; userTook();
        canvas.style.cursor = 'grabbing';
        var ids = Object.keys(pointers); if (ids.length === 2) pinch = Math.hypot(pointers[ids[0]].x - pointers[ids[1]].x, pointers[ids[0]].y - pointers[ids[1]].y);
      });
      canvas.addEventListener('pointermove', function (e) {
        var p = pointers[e.pointerId]; if (!p) return;
        var ids = Object.keys(pointers);
        if (ids.length === 1) { yaw += (e.clientX - p.x) * 0.01; pitch += (e.clientY - p.y) * 0.01; clampPitch(); }
        p.x = e.clientX; p.y = e.clientY;
        if (ids.length === 2) {
          var d = Math.hypot(pointers[ids[0]].x - pointers[ids[1]].x, pointers[ids[0]].y - pointers[ids[1]].y);
          if (pinch) { dist /= d / pinch; clampDist(); } pinch = d;
        }
        invalidate();
      });
      function up(e) { delete pointers[e.pointerId]; pinch = 0; if (!Object.keys(pointers).length) canvas.style.cursor = 'grab'; }
      canvas.addEventListener('pointerup', up); canvas.addEventListener('pointercancel', up);
      canvas.addEventListener('wheel', function (e) { e.preventDefault(); userTook(); dist *= Math.exp(e.deltaY * 0.0012); clampDist(); invalidate(); }, { passive: false });
      canvas.addEventListener('keydown', function (e) {
        var used = true;
        if (e.key === 'ArrowLeft') yaw -= 0.15; else if (e.key === 'ArrowRight') yaw += 0.15;
        else if (e.key === 'ArrowUp') pitch -= 0.15; else if (e.key === 'ArrowDown') pitch += 0.15;
        else if (e.key === '+' || e.key === '=') dist /= 1.15; else if (e.key === '-' || e.key === '_') dist *= 1.15;
        else if (e.key === 'Home') { yaw = def.yaw; pitch = def.pitch; dist = def.dist; }
        else used = false;
        if (used) { e.preventDefault(); e.stopPropagation(); userTook(); clampPitch(); clampDist(); invalidate(); }
      });
      var ro = typeof ResizeObserver !== 'undefined' ? new ResizeObserver(invalidate) : null; if (ro) ro.observe(host);

      return {
        start: function () { running = true; last = 0; invalidate(); },
        stop: function () { running = false; if (raf) { cancelAnimationFrame(raf); raf = 0; } },
        reset: function () { yaw = def.yaw; pitch = def.pitch; dist = def.dist; userTook(); invalidate(); },
        destroy: function () {
          destroyed = true; running = false; if (raf) cancelAnimationFrame(raf); if (ro) ro.disconnect();
          var ext = gl.getExtension('WEBGL_lose_context'); if (ext) ext.loseContext();
        }
      };
    });
  };
})();
