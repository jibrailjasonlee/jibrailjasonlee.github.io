/* Pick-and-place workcell used as the project picker.
   A ceiling-free, rail-mounted 2-link arm with inverse kinematics, a parallel gripper,
   and a small sequencer that fetches a part to the inspection stage.
   No dependencies. window.initRobotCell(options) -> { select, touched, items } */
(function () {
  'use strict';

  var W = 1000, H = 270, VBY = 90, TABLE = 330, BW = 76, BH = 64;
  var SLOT0 = 130, PITCH = 108, STAGE_X = 890, STAGE_TOP = 300;
  var L = 125, NECK = 40, OFF = 110, SH_Y = 270;
  var SAFE = 176, HOVER = 222, HOME_X = 520;
  var SPEED = { x: 1100, y: 800, c: 700, g: 6 };
  var reduce = matchMedia('(prefers-reduced-motion:reduce)').matches;

  // Icons are drawn in block coordinates (76 x 64), strokes inherit the ink colour.
  var ICONS = {
    rover: '<rect class="f" x="18" y="15" width="40" height="22" rx="3"/><rect class="d" x="24" y="9" width="12" height="6" rx="1"/><rect class="d" x="24" y="37" width="12" height="6" rx="1"/><rect class="d" x="24" y="20" width="16" height="12" rx="1"/><circle class="d" cx="50" cy="26" r="3"/>',
    myo: '<rect class="f" x="8" y="20" width="28" height="13" rx="6.5"/><rect class="f" x="36" y="21" width="18" height="11" rx="5.5"/><rect class="f" x="54" y="22" width="14" height="9" rx="4.5"/><circle class="d" cx="36" cy="26.5" r="2"/><circle class="d" cx="54" cy="26.5" r="2"/><path class="m" d="M64 29H12"/>',
    rsx: '<rect class="f" x="20" y="36" width="34" height="6" rx="1"/><rect class="f" x="28" y="27" width="18" height="9"/><rect class="f" x="24" y="20" width="26" height="7"/><rect class="f" x="40" y="8" width="22" height="12" rx="2"/><circle class="d" cx="62" cy="14" r="3.5"/>',
    igvc: '<rect class="f" x="14" y="22" width="48" height="12" rx="3"/><circle class="f" cx="24" cy="38" r="5"/><circle class="f" cx="52" cy="38" r="5"/><path class="d" d="M38 22V13"/><circle class="f" cx="38" cy="10" r="3.5"/>',
    cvt: '<circle class="f" cx="24" cy="25" r="13"/><circle class="f" cx="54" cy="25" r="7"/><path class="d" d="M24 12L54 18M24 38L54 32"/><circle class="d" cx="24" cy="25" r="2"/><circle class="d" cx="54" cy="25" r="2"/>',
    tower: '<rect class="f" x="30" y="6" width="16" height="32" rx="3"/><path class="d" d="M30 13H22L20 18M30 22H22L20 27M30 31H22L20 36M46 13H54L56 18M46 22H54L56 27M46 31H54L56 36"/><path class="f" d="M22 38H54L56 44H20Z"/>',
    piston: '<rect class="f" x="20" y="8" width="36" height="16" rx="2"/><path class="d" d="M28 8V24M34 8V24"/><path class="d" d="M38 24V36"/><circle class="f" cx="38" cy="41" r="5"/>'
  };

  var slotX = function (i) { return SLOT0 + i * PITCH - BW / 2; };
  var sleep = function (ms) { return new Promise(function (r) { setTimeout(r, reduce ? 0 : ms); }); };
  var clamp = function (v, a, b) { return Math.min(b, Math.max(a, v)); };

  function build(items) {
    var s = '<svg viewBox="0 ' + VBY + ' ' + W + ' ' + H + '" aria-hidden="true" focusable="false">';
    s += '<defs><pattern id="rc-grid" width="25" height="25" patternUnits="userSpaceOnUse"><path class="rc-gl" d="M25 0H0V25"/></pattern></defs>';
    s += '<rect y="' + VBY + '" width="' + W + '" height="' + H + '" fill="url(#rc-grid)"/>';
    s += '<text class="t" x="24" y="122">WORKCELL 01  /  PICK AND PLACE</text>';
    s += '<path class="m" d="M' + STAGE_X + ' 120L' + (STAGE_X - 54) + ' 232M' + STAGE_X + ' 120L' + (STAGE_X + 54) + ' 232"/><rect class="f" x="' + (STAGE_X - 24) + '" y="104" width="48" height="16" rx="3"/><text class="t" x="' + STAGE_X + '" y="100" text-anchor="middle">scanner</text>';
    s += '<rect class="f" x="20" y="' + TABLE + '" width="960" height="14" rx="2"/>';
    s += '<rect class="f" x="' + (STAGE_X - 60) + '" y="' + STAGE_TOP + '" width="120" height="' + (TABLE - STAGE_TOP) + '"/>';
    s += '<text class="t" x="24" y="357">parts tray</text><text class="t" x="' + STAGE_X + '" y="357" text-anchor="middle">inspection stage</text>';
    s += '<g id="rc-base"><path class="f" id="rc-ped"/><rect class="rc-car" id="rc-car" y="' + (TABLE + 3) + '" width="64" height="8" rx="2"/></g>';
    items.forEach(function (it) {
      s += '<g class="rc-item" data-id="' + it.id + '" transform="translate(' + it.x + ' ' + it.y + ')">' +
        '<rect class="blk" width="' + BW + '" height="' + BH + '" rx="6"/>' +
        '<rect class="blk" x="26" y="-14" width="24" height="14" rx="3"/><circle class="d" cx="38" cy="-7" r="2.5"/>' +
        ICONS[it.id] +
        '<text class="tb" x="38" y="58" text-anchor="middle" style="font-size:10.5px">' + it.name + '</text></g>';
    });
    s += '<path class="rc-lo" id="rc-u1"/><path class="rc-lo" id="rc-f1"/><path class="rc-li" id="rc-u2"/><path class="rc-li" id="rc-f2"/>';
    s += '<circle class="rc-hub" id="rc-hs" r="9"/><circle class="rc-hub" id="rc-he" r="8"/><circle class="rc-hub" id="rc-hw" r="6"/>';
    s += '<g id="rc-g"><rect class="rc-gr" x="-11" y="-38" width="22" height="13" rx="3"/><rect class="rc-gr" id="rc-rail" y="-26" height="5" rx="2"/>' +
      '<rect class="rc-gr" id="rc-fl" y="-22" width="6" height="22" rx="1.5"/><rect class="rc-gr" id="rc-fr" y="-22" width="6" height="22" rx="1.5"/>' +
      '<rect class="rc-gr" id="rc-tl" y="-5" width="5" height="5"/><rect class="rc-gr" id="rc-tr" y="-5" width="5" height="5"/></g>';
    return s + '</svg>';
  }

  window.initRobotCell = function (o) {
    var items = o.items.map(function (it, i) {
      return { id: it.id, name: it.name, i: i, x: slotX(i), y: TABLE - BH };
    });
    var byId = {};
    items.forEach(function (it) { byId[it.id] = it; });

    o.mount.innerHTML = build(items);
    var svg = o.mount.querySelector('svg');
    var q = function (id) { return svg.getElementById ? svg.getElementById(id) : svg.querySelector('#' + id); };
    var E = {
      ped: q('rc-ped'), car: q('rc-car'), u1: q('rc-u1'), u2: q('rc-u2'), f1: q('rc-f1'), f2: q('rc-f2'),
      hs: q('rc-hs'), he: q('rc-he'), hw: q('rc-hw'), g: q('rc-g'), fl: q('rc-fl'), fr: q('rc-fr'), tl: q('rc-tl'), tr: q('rc-tr'), rail: q('rc-rail')
    };
    items.forEach(function (it) { it.el = svg.querySelector('.rc-item[data-id="' + it.id + '"]'); });

    o.chips.innerHTML = items.map(function (it) {
      return '<button type="button" class="chip" data-id="' + it.id + '" aria-pressed="false"><span class="mono">' +
        ('0' + (it.i + 1)) + '</span>' + it.name + '</button>';
    }).join('');

    var cur = { x: HOME_X, y: SAFE, c: HOME_X - OFF, g: 0.5 };
    var tgt = { x: HOME_X, y: SAFE, c: HOME_X - OFF, g: 0.5 };
    var held = null, onStage = null, want = null, busy = false, quiet = false, touched = false;
    var state = 'IDLE', waiters = [], raf = 0, last = 0, teleT = 0, telemetry = { j1: 0, j2: 0 };

    function setItem(it, x, y) {
      it.x = x; it.y = y;
      it.el.setAttribute('transform', 'translate(' + x.toFixed(1) + ' ' + y.toFixed(1) + ')');
    }

    function draw() {
      var x = cur.x, y = cur.y;
      var xc = clamp(cur.c, Math.max(40, x - 160), x - 50);
      var sx = xc, sy = SH_Y, dx = x - sx, dy = (y - NECK) - sy;
      var d = Math.min(Math.hypot(dx, dy), 2 * L - 0.5);
      var a = Math.acos(d / (2 * L)), phi = Math.atan2(dy, dx);
      var t1 = phi - a, t2 = phi + a;
      var ex = sx + L * Math.cos(t1), ey = sy + L * Math.sin(t1);
      var fx = ex + L * Math.cos(t2), fy = ey + L * Math.sin(t2);
      var P = function (n) { return n.toFixed(1); };
      E.ped.setAttribute('d', 'M' + P(xc - 22) + ' ' + TABLE + 'L' + P(xc - 11) + ' ' + (SH_Y + 8) + 'H' + P(xc + 11) + 'L' + P(xc + 22) + ' ' + TABLE + 'Z'); E.car.setAttribute('x', P(xc - 32));
      var up = 'M' + P(sx) + ' ' + P(sy) + 'L' + P(ex) + ' ' + P(ey), fo = 'M' + P(ex) + ' ' + P(ey) + 'L' + P(fx) + ' ' + P(fy);
      E.u1.setAttribute('d', up); E.u2.setAttribute('d', up); E.f1.setAttribute('d', fo); E.f2.setAttribute('d', fo);
      E.hs.setAttribute('cx', P(sx)); E.hs.setAttribute('cy', P(sy));
      E.he.setAttribute('cx', P(ex)); E.he.setAttribute('cy', P(ey));
      E.hw.setAttribute('cx', P(fx)); E.hw.setAttribute('cy', P(fy));
      E.g.setAttribute('transform', 'translate(' + P(x) + ' ' + P(y) + ')');
      var gap = 12 + 14 * cur.g;
      E.fl.setAttribute('x', P(-(gap + 6))); E.fr.setAttribute('x', P(gap)); E.tl.setAttribute('x', P(-gap)); E.tr.setAttribute('x', P(gap - 5));
      E.rail.setAttribute('x', P(-(gap + 6))); E.rail.setAttribute('width', P(2 * (gap + 6)));
      if (held) setItem(held, x - BW / 2, y + 2);
      telemetry.j1 = -t1 * 180 / Math.PI; telemetry.j2 = (t2 - t1) * 180 / Math.PI;
      if (o.scroller && o.scroller.scrollWidth > o.scroller.clientWidth + 2 && (busy || held)) {
        o.scroller.scrollLeft = x * (svg.clientWidth / W) - o.scroller.clientWidth / 2;
      }
    }

    function writeTele(now) {
      if (!o.tele || (now !== 1e9 && now - teleT < 110)) return;
      teleT = now === 1e9 ? 0 : now;
      var f = function (v, w) { var s = v.toFixed(1); while (s.length < w) s = ' ' + s; return s; };
      o.tele.querySelector('.tv').textContent = (state + '          ').slice(0, 9) +
        ' X' + f(cur.x, 6) + '  Z' + f(TABLE - cur.y, 6) + '  J1' + f(telemetry.j1, 7) + '°  J2' + f(telemetry.j2, 7) +
        '°  GRIP ' + (cur.g > 0.75 ? 'open  ' : cur.g < 0.25 ? 'closed' : 'part  ');
    }

    function frame(t) {
      raf = 0;
      var dt = Math.min(0.05, (t - last) / 1000 || 0.016); last = t;
      var moving = false;
      ['x', 'y', 'c', 'g'].forEach(function (k) {
        var e = tgt[k] - cur[k];
        if (e === 0) return;
        var v = clamp(e * (k === 'g' ? 12 : 7), -SPEED[k], SPEED[k]), st = v * dt;
        if (Math.abs(e) <= Math.max(Math.abs(st), k === 'g' ? 0.004 : 0.35)) cur[k] = tgt[k];
        else { cur[k] += st; moving = true; }
      });
      draw(); writeTele(t);
      o.root.dataset.state = moving ? 'moving' : 'idle';
      if (!moving) { var w = waiters; waiters = []; w.forEach(function (r) { r(); }); }
      if (moving || waiters.length) raf = requestAnimationFrame(frame);
      else writeTele(1e9);
    }
    function kick() { if (!raf) { last = performance.now(); raf = requestAnimationFrame(frame); } }

    function mv(x, y, g) {
      tgt.x = x; tgt.y = y; tgt.c = clamp(x - OFF, 40, 960);
      if (g !== undefined) tgt.g = g;
      if (reduce) { cur.x = tgt.x; cur.y = tgt.y; cur.c = tgt.c; cur.g = tgt.g; draw(); writeTele(1e9); return Promise.resolve(); }
      return new Promise(function (r) { waiters.push(r); kick(); });
    }
    async function travel(x, y, g) {
      if (Math.abs(x - cur.x) > 2) {
        if (cur.y > SAFE + 2) await mv(cur.x, SAFE, g);
        await mv(x, SAFE, g);
      }
      await mv(x, y, g);
    }
    async function grip(open) { tgt.g = open; if (reduce) { cur.g = open; draw(); return; } await new Promise(function (r) { waiters.push(r); kick(); }); await sleep(110); }

    var stagePos = { x: STAGE_X - BW / 2, y: STAGE_TOP - BH };

    async function grab(it) {
      var cx = it.x + BW / 2, ty = it.y - 2;
      state = 'APPROACH'; await travel(cx, SAFE, 1);
      state = 'PICK'; await mv(cx, ty, 1);
      await grip(0); held = it;
      state = 'LIFT'; await mv(cx, SAFE, 0);
    }
    async function put(it, pos) {
      var cx = pos.x + BW / 2, ty = pos.y - 2;
      state = 'CARRY'; await mv(cx, SAFE, 0);
      state = 'PLACE'; await mv(cx, ty, 0);
      var h = held; held = null; setItem(h, pos.x, pos.y);
      await grip(1);
      state = 'RETREAT'; await mv(cx, SAFE, 1);
    }

    function markChips() {
      var id = want || o.initial;
      o.chips.querySelectorAll('.chip').forEach(function (b) { b.setAttribute('aria-pressed', b.dataset.id === id); });
      items.forEach(function (it) { it.el.classList.toggle('on', it.id === (want || onStage)); });
    }

    async function run() {
      busy = true; o.onBusy && o.onBusy(true, quiet);
      while (onStage !== want) {
        var target = want;
        if (onStage) {
          var out = byId[onStage];
          await grab(out); await put(out, { x: slotX(out.i), y: TABLE - BH }); onStage = null;
          if (want !== target) continue;
        }
        var it = byId[target], home = { x: it.x, y: it.y };
        await grab(it);
        if (want !== target) { await put(it, home); continue; }
        await put(it, stagePos); onStage = target;
        if (want === target) {
          o.live && (o.live.textContent = it.name + ' is on the inspection stage. Details are below.');
          o.onPlace && o.onPlace(target, quiet);
        }
      }
      state = 'IDLE'; await mv(HOME_X, SAFE, 0.5);
      busy = false; o.onBusy && o.onBusy(false, quiet);
    }

    function select(id, q) {
      if (!byId[id]) return;
      if (id === want && (busy || onStage === id)) return;
      touched = touched || !q;
      var wasQuiet = quiet; quiet = !!q; want = id; markChips();
      if (busy) { if (wasQuiet && !quiet) o.onBusy && o.onBusy(true, false); }
      else run();
    }

    // Idle behaviour: the arm follows the pointer along the tray.
    function hot(id) { items.forEach(function (it) { it.el.classList.toggle('hot', it.id === id); }); }
    function trackItem(it) {
      if (busy || reduce || !it) return;
      state = 'TRACK'; hot(it.id);
      mv(onStage === it.id ? STAGE_X : it.x + BW / 2, HOVER, 1);
    }
    function release() { if (busy || reduce) return; state = 'IDLE'; hot(null); mv(HOME_X, SAFE, 0.5); }

    svg.addEventListener('pointermove', function (e) {
      if (e.pointerType !== 'mouse' || busy || reduce) return;
      var r = svg.getBoundingClientRect(), x = (e.clientX - r.left) / r.width * W, best = null, bd = 1e9;
      items.forEach(function (it) {
        var d = Math.abs((onStage === it.id ? STAGE_X : it.x + BW / 2) - x);
        if (d < bd) { bd = d; best = it; }
      });
      if (Math.abs(STAGE_X - x) < bd && onStage) best = byId[onStage];
      trackItem(best);
    });
    svg.addEventListener('pointerleave', release);
    svg.addEventListener('click', function (e) {
      var g = e.target.closest && e.target.closest('.rc-item');
      if (g) select(g.dataset.id);
    });
    o.chips.addEventListener('click', function (e) {
      var b = e.target.closest('.chip'); if (b) select(b.dataset.id);
    });
    o.chips.querySelectorAll('.chip').forEach(function (b) {
      var on = function () { trackItem(byId[b.dataset.id]); };
      b.addEventListener('mouseenter', on); b.addEventListener('focus', on);
      b.addEventListener('mouseleave', release); b.addEventListener('blur', release);
    });

    markChips(); draw(); writeTele(1e9);
    return {
      select: select, items: items,
      get touched() { return touched; }, get busy() { return busy; }, get onStage() { return onStage; },
      get pose() { return { x: cur.x, y: cur.y, g: cur.g, c: cur.c }; }
    };
  };
})();
