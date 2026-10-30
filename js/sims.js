/* Interactive simulations for the Adaptive WPT site.
 * Sim 1: charging bay - the transmitter pad moves on an x-y-lift stage to find the car's receiver,
 *        then tunes (C1, f) with the mode-centred search.
 * Sim 2: resonance lab - oscilloscope, live field lines and the power-vs-frequency curve.
 * Sim 3: results explorer - the recorded 1,485-pose runs.
 * All numbers come from physics.js (port of the project's Python models).
 */
(function () {
  'use strict';
  const W = window.WPT.init(window.WPT_DATA);
  const D = W.data, LIM = D.limits, PR = LIM.P_RATED;
  const $ = (s, r) => (r || document).querySelector(s);
  const $$ = (s, r) => [...(r || document).querySelectorAll(s)];
  const fmt = (v, d) => (v == null || !isFinite(v)) ? '–' : v.toFixed(d === undefined ? 1 : d);
  const clamp = (v, a, b) => Math.min(Math.max(v, a), b);
  const TAU = Math.PI * 2;
  const reduceMotion = window.matchMedia && matchMedia('(prefers-reduced-motion: reduce)').matches;

  // ------------------------------------------------------------------ theme tokens (read from CSS)
  let C = {};
  function readTheme() {
    const s = getComputedStyle($('.viz-root') || document.body), g = n => s.getPropertyValue(n).trim();
    C = { surface: g('--surface-1'), ink: g('--text-primary'), ink2: g('--text-secondary'), muted: g('--text-muted'),
          grid: g('--grid'), s1: g('--series-1'), s2: g('--series-2'), s3: g('--series-3'), bad: g('--critical'),
          good: g('--good'), ground: g('--ground'), ground2: g('--ground-2'), pit: g('--pit'), copper: g('--copper'),
          padTx: g('--pad-tx'), padRx: g('--pad-rx'), car: g('--car'), fpos: g('--field-pos'), fneg: g('--field-neg'),
          accent: g('--accent'), seq: [100, 200, 300, 400, 500, 600, 700].map(k => g('--seq-' + k)) };
  }
  readTheme();
  if (window.matchMedia) matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => setTimeout(readTheme, 50));

  // ------------------------------------------------------------------ canvas helpers
  function fit(cv, aspect, minH, maxH) {
    const w = cv.clientWidth || 300, h = clamp(Math.round(w * aspect), minH || 0, maxH || 1e9);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
      cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); cv.style.height = h + 'px';
    }
    const ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { ctx, w, h };
  }
  function seqColor(t) {                       // sequential ramp, t in [0, 1]
    const s = C.seq, x = clamp(t, 0, 1) * (s.length - 1), i = Math.min(Math.floor(x), s.length - 2), f = x - i;
    return mix(s[i], s[i + 1], f);
  }
  function hex2rgb(h) { h = h.replace('#', ''); if (h.length === 3) h = h.split('').map(c => c + c).join('');
    const n = parseInt(h, 16); return [n >> 16 & 255, n >> 8 & 255, n & 255]; }
  function mix(a, b, f) { const A = hex2rgb(a), B = hex2rgb(b);
    return 'rgb(' + A.map((v, i) => Math.round(v + (B[i] - v) * f)).join(',') + ')'; }
  function rgba(h, a) { const c = hex2rgb(h); return `rgba(${c[0]},${c[1]},${c[2]},${a})`; }
  function roundRect(ctx, x, y, w, h, r) { ctx.beginPath(); ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r); ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath(); }

  // ------------------------------------------------------------------ pad geometry (for drawing and field lines)
  const GEO = {
    circular: { tx: { kind: 'circ', rin: .12, rout: .25, N: 24, th: .026 }, rx: { kind: 'circ', rin: .07, rout: .16, N: 18, th: .02 } },
    dd: { tx: { kind: 'dd', a: .2, b: .4, N: 11, band: .066, th: .026 }, rx: { kind: 'dd', a: .2, b: .4, N: 6, band: .036, th: .02 } }
  };
  const halfW = g => g.kind === 'circ' ? g.rout : g.a;
  function filaments(g) {                      // cross-section along x: [u, sign, turns]
    const out = [];
    if (g.kind === 'circ') {
      for (let k = 0; k < 4; k++) { const r = g.rin + (k + .5) * (g.rout - g.rin) / 4; out.push([r, 1, g.N / 4], [-r, -1, g.N / 4]); }
    } else {
      for (let k = 0; k < 3; k++) { const o = (k + .5) * g.band / 3;
        out.push([-g.a + o, 1, g.N / 3], [-o, -1, g.N / 3], [o, -1, g.N / 3], [g.a - o, 1, g.N / 3]); }
    }
    return out;
  }

  // ------------------------------------------------------------------ field lines: contours of A_z from line currents
  const fieldCache = {};
  function fieldDelta(pad) {                   // contour spacing: a tx current of 20 A gives ~7 lines per sign
    if (fieldCache[pad]) return fieldCache[pad];
    const f = filaments(GEO[pad].tx); let mx = 0;
    for (let x = -.45; x <= .45; x += .01) { let a = 0;
      for (const [u, sg, n] of f) a -= sg * n * 20 * Math.log(Math.hypot(x - u, .07));
      mx = Math.max(mx, Math.abs(a)); }
    return (fieldCache[pad] = mx / 9);
  }
  // sources: [{x, z, I}] (I already includes turns); draws into world box using toPx
  function drawField(ctx, box, toPx, srcs, delta, alpha) {
    const nx = Math.max(60, Math.min(150, Math.round(box.pw / 5))), nz = Math.max(24, Math.round(nx * box.ph / box.pw));
    const A = new Float32Array(nx * nz), dx = (box.x1 - box.x0) / (nx - 1), dz = (box.z1 - box.z0) / (nz - 1);
    for (let j = 0; j < nz; j++) { const z = box.z0 + j * dz;
      for (let i = 0; i < nx; i++) { const x = box.x0 + i * dx; let a = 0;
        for (let s = 0; s < srcs.length; s++) { const S = srcs[s], rx = x - S.x, rz = z - S.z;
          a -= S.I * 0.5 * Math.log(rx * rx + rz * rz + 1.2e-4); }
        A[j * nx + i] = a; } }
    const KMAX = 16, paths = [new Path2D(), new Path2D()];
    let any = false;
    for (let j = 0; j < nz - 1; j++) for (let i = 0; i < nx - 1; i++) {
      const a = A[j * nx + i], b = A[j * nx + i + 1], c = A[(j + 1) * nx + i + 1], d = A[(j + 1) * nx + i];
      const mn = Math.min(a, b, c, d), mx = Math.max(a, b, c, d);
      let k0 = Math.ceil(mn / delta), k1 = Math.floor(mx / delta);
      if (k1 < k0) continue;
      k0 = Math.max(k0, -KMAX); k1 = Math.min(k1, KMAX);
      const X = box.x0 + i * dx, Z = box.z0 + j * dz;
      for (let k = k0; k <= k1; k++) { if (k === 0) continue;
        const L = k * delta, pts = [];
        if ((a - L) * (b - L) < 0) pts.push([X + dx * (L - a) / (b - a), Z]);
        if ((b - L) * (c - L) < 0) pts.push([X + dx, Z + dz * (L - b) / (c - b)]);
        if ((d - L) * (c - L) < 0) pts.push([X + dx * (L - d) / (c - d), Z + dz]);
        if ((a - L) * (d - L) < 0) pts.push([X, Z + dz * (L - a) / (d - a)]);
        if (pts.length >= 2) { const p = paths[k > 0 ? 0 : 1];
          for (let q = 0; q + 1 < pts.length; q += 2) { const P0 = toPx(pts[q][0], pts[q][1]), P1 = toPx(pts[q + 1][0], pts[q + 1][1]);
            p.moveTo(P0[0], P0[1]); p.lineTo(P1[0], P1[1]); any = true; } }
      } }
    if (!any) return;
    ctx.save(); ctx.lineCap = 'round';
    [[paths[0], C.fpos], [paths[1], C.fneg]].forEach(([p, col]) => {
      ctx.strokeStyle = rgba(col, .18 * alpha); ctx.lineWidth = 4; ctx.stroke(p);
      ctx.strokeStyle = rgba(col, .9 * alpha); ctx.lineWidth = 1.3; ctx.stroke(p); });
    ctx.restore();
  }
  function halo(ctx, t, x, y) { ctx.save(); ctx.strokeStyle = C.surface; ctx.lineWidth = 4; ctx.lineJoin = 'round'; ctx.strokeText(t, x, y); ctx.restore(); ctx.fillText(t, x, y); }
  // instantaneous current from an rms phasor (re, im) at visual phase theta
  const inst = (re, im, th) => Math.SQRT2 * (re * Math.cos(th) - im * Math.sin(th));

  // ------------------------------------------------------------------ side view renderer (shared by sims 1 and 2)
  // cfg: pad, txX, lift, rxX, clear (rx bottom height at lift 0 = gap + lift), tilt (deg), i1, i2 ({re, im, on}), theta,
  //      stage (bool), car (bool), view {x0, x1, z0, z1}
  function drawSide(ctx, w, h, cfg) {
    const v = cfg.view, sx = w / (v.x1 - v.x0), sz = h / (v.z1 - v.z0), s = Math.min(sx, sz);
    const toPx = (x, z) => [(x - v.x0) * sx, h - (z - v.z0) * sz];
    const gtx = GEO[cfg.pad].tx, grx = GEO[cfg.pad].rx, tilt = cfg.tilt * Math.PI / 180;
    ctx.clearRect(0, 0, w, h);
    // ground
    const [, gy] = toPx(0, 0);
    ctx.fillStyle = C.ground; ctx.fillRect(0, gy, w, h - gy);
    if (cfg.stage) {
      const [px0, py0] = toPx(-.47, 0), [px1, py1] = toPx(.47, -.115);
      ctx.fillStyle = C.pit; ctx.fillRect(px0, py0, px1 - px0, py1 - py0);
      ctx.strokeStyle = C.ground2; ctx.lineWidth = 2; ctx.strokeRect(px0, py0, px1 - px0, py1 - py0);
      // rail
      ctx.fillStyle = C.ground2; const [rx0, ry] = toPx(-.46, -.108), [rx1] = toPx(.46, 0); ctx.fillRect(rx0, ry - 3, rx1 - rx0, 3);
    }
    ctx.strokeStyle = C.ground2; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(w, gy); ctx.stroke();
    // field lines
    const txTop = cfg.lift, txMid = txTop - .008, rxBot = cfg.clear, rxMid = rxBot + .008;
    if (cfg.i1.on || cfg.i2.on) {
      const srcs = [], I1 = inst(cfg.i1.re, cfg.i1.im, cfg.theta), I2 = inst(cfg.i2.re, cfg.i2.im, cfg.theta);
      for (const [u, sg, n] of filaments(gtx)) srcs.push({ x: cfg.txX + u, z: txMid, I: sg * n * I1 });
      for (const [u, sg, n] of filaments(grx)) srcs.push({ x: cfg.rxX + u * Math.cos(tilt), z: rxMid + u * Math.sin(tilt), I: sg * n * I2 });
      const box = { x0: v.x0, x1: v.x1, z0: cfg.stage ? -.02 : 0, z1: v.z1 };
      const [bx0, by1] = toPx(box.x0, box.z0), [bx1, by0] = toPx(box.x1, box.z1);
      box.pw = bx1 - bx0; box.ph = by1 - by0;
      ctx.save(); ctx.beginPath(); ctx.rect(0, 0, w, cfg.stage ? toPx(0, -.02)[1] : gy); ctx.clip();
      drawField(ctx, box, toPx, srcs, fieldDelta(cfg.pad), cfg.fieldAlpha || 1);
      ctx.restore();
    }
    // scissor lift
    if (cfg.stage) {
      const hw = halfW(gtx) * .55, [a0, b0] = toPx(cfg.txX - hw, -.108), [a1, b1] = toPx(cfg.txX + hw, txTop - gtx.th);
      ctx.strokeStyle = C.ground2; ctx.lineWidth = 3; ctx.beginPath();
      ctx.moveTo(a0, b0); ctx.lineTo(a1, b1); ctx.moveTo(a1, b0); ctx.lineTo(a0, b1); ctx.stroke();
      ctx.fillStyle = C.ground2; ctx.fillRect(a0 - 6, b0 - 4, a1 - a0 + 12, 6);
    }
    // transmitter pad
    const drawPad = (g, cx, zb, ang, fill, I) => {
      const hw = halfW(g) + .01;
      ctx.save(); const [px, pz] = toPx(cx, zb); ctx.translate(px, pz); ctx.rotate(-ang);
      roundRect(ctx, -hw * sx, -g.th * sz, 2 * hw * sx, g.th * sz, 3); ctx.fillStyle = fill; ctx.fill();
      // copper cross-sections
      for (const [u, sg] of filaments(g)) {
        const cur = I * sg, r = clamp(2 + .1 * s * .02, 2, 4);
        ctx.beginPath(); ctx.arc(u * sx, -g.th * sz / 2, r, 0, TAU);
        ctx.fillStyle = cur > 0 ? mix(C.copper, C.fpos, clamp(Math.abs(cur) / 30, 0, .8)) : mix(C.copper, C.fneg, clamp(Math.abs(cur) / 30, 0, .8));
        ctx.fill(); }
      ctx.restore();
    };
    drawPad(gtx, cfg.txX, txTop - gtx.th, 0, C.padTx, cfg.i1.on ? inst(cfg.i1.re, cfg.i1.im, cfg.theta) : 0);
    // car underbody + receiver
    if (cfg.car) {
      ctx.save(); const [cx, cz] = toPx(cfg.rxX, rxBot); ctx.translate(cx, cz); ctx.rotate(-tilt);
      ctx.fillStyle = rgba(C.car, .85); ctx.fillRect(-w * 1.5, -(grx.th + .028) * sz - h, w * 3, h);
      ctx.strokeStyle = C.ink2; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(-w * 1.5, -(grx.th + .028) * sz); ctx.lineTo(w * 1.5, -(grx.th + .028) * sz); ctx.stroke();
      ctx.fillStyle = C.ground2; ctx.fillRect(-(halfW(grx) + .02) * sx, -(grx.th + .028) * sz, 2 * (halfW(grx) + .02) * sx, .028 * sz - 1);
      ctx.restore();
    }
    drawPad(grx, cfg.rxX, rxBot, tilt, C.padRx, cfg.i2.on ? inst(cfg.i2.re, cfg.i2.im, cfg.theta) : 0);
    // gap marker
    const gx = Math.min(cfg.rxX, cfg.txX) - Math.max(halfW(gtx), halfW(grx)) - .05;
    const mx = gx > v.x0 + .04 ? gx : Math.max(cfg.rxX, cfg.txX) + Math.max(halfW(gtx), halfW(grx)) + .05;
    const [ax, ay0] = toPx(mx, txTop), [, ay1] = toPx(mx, rxBot);
    ctx.strokeStyle = C.ink2; ctx.fillStyle = C.ink2; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(ax, ay0); ctx.lineTo(ax, ay1); ctx.stroke();
    [[ay0, -1], [ay1, 1]].forEach(([yy, d]) => { ctx.beginPath(); ctx.moveTo(ax, yy); ctx.lineTo(ax - 4, yy + 6 * d); ctx.lineTo(ax + 4, yy + 6 * d); ctx.closePath(); ctx.fill(); });
    ctx.font = '600 12px system-ui, sans-serif'; ctx.textAlign = mx > cfg.txX ? 'left' : 'right'; ctx.textBaseline = 'middle';
    halo(ctx, `gap ${Math.round((cfg.clear - cfg.lift) * 1000)} mm`, ax + (mx > cfg.txX ? 7 : -7), (ay0 + ay1) / 2);
    return toPx;
  }

  // ------------------------------------------------------------------ top view pad drawing
  function drawPadTop(ctx, g, cx, cy, s, style) {
    ctx.save(); ctx.translate(cx, cy);
    if (g.kind === 'circ') {
      ctx.beginPath(); ctx.arc(0, 0, (g.rout + .012) * s, 0, TAU); ctx.fillStyle = style.fill; ctx.fill();
      if (style.dash) { ctx.setLineDash([5, 4]); ctx.strokeStyle = style.edge; ctx.lineWidth = 1.5; ctx.stroke(); ctx.setLineDash([]); }
      const n = Math.min(g.N, 10);
      ctx.strokeStyle = style.coil; ctx.lineWidth = 1.2;
      for (let k = 0; k < n; k++) { const r = (g.rin + (k + .5) * (g.rout - g.rin) / n) * s; ctx.beginPath(); ctx.arc(0, 0, r, 0, TAU); ctx.stroke(); }
    } else {
      const a = g.a * s, b = g.b * s;
      roundRect(ctx, -a - 6, -b / 2 - 6, 2 * a + 12, b + 12, 6); ctx.fillStyle = style.fill; ctx.fill();
      if (style.dash) { ctx.setLineDash([5, 4]); ctx.strokeStyle = style.edge; ctx.lineWidth = 1.5; ctx.stroke(); ctx.setLineDash([]); }
      ctx.strokeStyle = style.coil; ctx.lineWidth = 1.2;
      const n = Math.min(g.N, 6);
      for (const side of [-1, 1]) for (let k = 0; k < n; k++) {
        const o = (k + .5) * g.band / n * s, x0 = side < 0 ? -a + o : o, x1 = side < 0 ? -o : a - o;
        roundRect(ctx, Math.min(x0, x1), -b / 2 + o, Math.abs(x1 - x0), b - 2 * o, 4); ctx.stroke(); }
    }
    ctx.restore();
  }

  // ------------------------------------------------------------------ small chart helpers
  function axes(ctx, box, xr, yr, xt, yt, xl, yl) {
    const X = x => box.x + (x - xr[0]) / (xr[1] - xr[0]) * box.w, Y = y => box.y + box.h - (y - yr[0]) / (yr[1] - yr[0]) * box.h;
    ctx.font = '12px system-ui, sans-serif'; ctx.strokeStyle = C.grid; ctx.fillStyle = C.ink2; ctx.lineWidth = 1;
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (const t of yt) { ctx.beginPath(); ctx.moveTo(box.x, Y(t)); ctx.lineTo(box.x + box.w, Y(t)); ctx.stroke(); ctx.fillText(t, box.x - 6, Y(t)); }
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (const t of xt) ctx.fillText(t, X(t), box.y + box.h + 5);
    ctx.fillText(xl, box.x + box.w / 2, box.y + box.h + 21);
    ctx.save(); ctx.translate(box.x - 36, box.y + box.h / 2); ctx.rotate(-Math.PI / 2); ctx.textBaseline = 'middle'; ctx.fillText(yl, 0, 0); ctx.restore();
    return { X, Y };
  }
  function tooltip(el, html, x, y, host) {
    if (!html) { el.hidden = true; return; }
    el.innerHTML = html; el.hidden = false;
    const hw = host.clientWidth, tw = el.offsetWidth;
    el.style.left = clamp(x + 12, 4, hw - tw - 4) + 'px'; el.style.top = Math.max(4, y - el.offsetHeight - 10) + 'px';
  }

  // ------------------------------------------------------------------ animation loop (only the visible sim runs)
  const loops = {};
  let active = 'bay', last = performance.now(), visible = true;
  function frame(t) {
    const dt = Math.min((t - last) / 1000, .1); last = t;
    if (visible && loops[active]) loops[active](dt, t / 1000);
    requestAnimationFrame(frame);
  }
  const sec = $('#simulations');
  if ('IntersectionObserver' in window) new IntersectionObserver(e => { visible = e[0].isIntersecting; }).observe(sec);
  $$('.simtab').forEach(b => b.addEventListener('click', () => {
    $$('.simtab').forEach(x => x.setAttribute('aria-selected', x === b ? 'true' : 'false'));
    $$('.simpanel').forEach(p => { p.hidden = p.id !== 'sim-' + b.dataset.sim; });
    active = b.dataset.sim; try { localStorage.setItem('wpt-sim-tab', active); } catch (e) {}
  }));
  try { const t = localStorage.getItem('wpt-sim-tab'); if (t && $(`.simtab[data-sim="${t}"]`)) $(`.simtab[data-sim="${t}"]`).click(); } catch (e) {}

  // segmented controls
  function seg(el, cb) { $$('button', el).forEach(b => b.addEventListener('click', () => {
    $$('button', el).forEach(x => x.setAttribute('aria-pressed', x === b ? 'true' : 'false')); cb(b.dataset.v); })); }
  function setSeg(el, v) { $$('button', el).forEach(x => x.setAttribute('aria-pressed', x.dataset.v === v ? 'true' : 'false')); }
  const bankLabel = (pad, b) => `#${b + 1} · ${fmt(D.pads[pad].design.C1_bank[b] * 1e9, 2)} nF`;

  // ====================================================================================
  // SIM 1 - CHARGING BAY
  // ====================================================================================
  const BAY = { pad: 'circular', car: { x: .13, y: -.07 }, clear: { circular: .175, dd: .125 }, tilt: 0, lift: true,
    tx: { x: 0, y: 0, lift: 0 }, set: null, phase: 'idle', plan: null, pi: 0, pt: 0, dots: [], probes: [], tune: null,
    speed: 10, simClock: 0, seed: 11, changed: false, theta: 0, result: null };
  const TRAVEL = .2;
  const bayEl = $('#sim-bay');
  const cvTop = $('#bay-top', bayEl), cvSide = $('#bay-side', bayEl), cvTune = $('#bay-tune', bayEl);

  function bayRel(tx) { tx = tx || BAY.tx; return { x: BAY.car.x - tx.x, y: BAY.car.y - tx.y, gap: BAY.clear[BAY.pad] - tx.lift, tilt: BAY.tilt }; }
  function bayM(tx) { const r = bayRel(tx); return W.interpM(BAY.pad, r.x, r.y, r.gap, r.tilt); }
  // operate the link at a (C1, f, s) setting with the controller's protective derating
  function operate(pad, M, C1, f, s) {
    const p = W.truthP(pad), RL = D.pads[pad].RL, r1 = W.solve(p, C1, f, M, RL, 1), g = LIM.GUARD;
    const sMax = Math.min(1, g * LIM.I1_MAX / Math.max(r1.I1, 1e-9), g * LIM.VC1_MAX / Math.max(r1.VC1, 1e-9),
                          Math.sqrt(LIM.P_OVER * PR / Math.max(r1.PL, 1e-9)));
    const se = Math.min(s, sMax), r = W.solve(p, C1, f, M, RL, se);
    return Object.assign(r, { s: se, derated: se < s - 1e-6, zvs: r.phi >= 0, effOk: r.eta >= LIM.ETA_MIN });
  }
  function fixedSet(pad) { return { C1: D.pads[pad].design.C1_nom, f: LIM.F0, s: 1, b: null, label: 'fixed tuning' }; }

  function bayReset(keepCar) {
    BAY.tx = { x: 0, y: 0, lift: 0 }; BAY.set = fixedSet(BAY.pad); BAY.phase = 'idle'; BAY.plan = null; BAY.dots = [];
    BAY.probes = []; BAY.tune = null; BAY.simClock = 0; BAY.changed = false; BAY.result = null;
    if (!keepCar) {}
    bayUI();
  }

  function buildPlan() {
    const pad = BAY.pad, R = W.rng(BAY.seed++), ev = [], meas = new Map();
    let cur = { ...BAY.tx }, tReal = 0;
    const lifts = BAY.lift ? [0, .025, .05, .075, .1].filter(l => BAY.clear[pad] - l >= .05 - 1e-9) : [0];
    const moveTo = (x, y, lift, phase) => {
      const d = Math.max(Math.abs(x - cur.x) / LIM.ACT_XY, Math.abs(y - cur.y) / LIM.ACT_XY, Math.abs(lift - cur.lift) / LIM.ACT_Z) + LIM.ACT_SETTLE;
      ev.push({ type: 'move', from: { ...cur }, to: { x, y, lift }, real: d, phase }); cur = { x, y, lift }; tReal += d;
    };
    const measure = (x, y, lift, phase) => {
      const key = [x, y, lift].map(v => v.toFixed(3)).join();
      if (meas.has(key)) return meas.get(key);
      moveTo(x, y, lift, phase);
      const M = bayM(cur), Me = W.estimateM(pad, Math.abs(M), R), pr = W.predict(pad, Me);
      const rec = { x, y, lift, Me, M, score: pr.score, predP: pr.P, eta: pr.eta };
      meas.set(key, rec); ev.push({ type: 'meas', rec, real: 3 * LIM.T_PROBE, phase }); tReal += 3 * LIM.T_PROBE;
      return rec;
    };
    const better = (a, b) => !b || a.score > b.score + 1e-9 || (Math.abs(a.score - b.score) < 1e-9 && a.Me > b.Me) ||
                             (!isFinite(b.score) && !isFinite(a.score) && a.Me > b.Me);
    const span = (c, h, st) => { const o = []; for (let v = -h; v <= h + 1e-9; v += st) { const q = +(c + v).toFixed(3); if (Math.abs(q) <= TRAVEL + 1e-9) o.push(q); } return [...new Set(o)]; };
    const scan = (xs, ys, lift, phase) => { let best = null;
      ys.forEach((y, j) => (j % 2 ? [...xs].reverse() : xs).forEach(x => { const r = measure(x, y, lift, phase); if (better(r, best)) best = r; }));
      return best; };
    // 1 locate (coarse), 2 refine, 3 fine
    let best = scan(span(0, .2, .1), span(0, .2, .1), cur.lift, 'locate');
    best = scan(span(best.x, .05, .025), span(best.y, .05, .025), best.lift, 'refine');
    best = scan(span(best.x, .01, .01), span(best.y, .01, .01), best.lift, 'refine');
    // 4 lift
    for (const l of lifts) { const r = measure(best.x, best.y, l, 'lift'); if (better(r, best)) best = r; }
    moveTo(best.x, best.y, best.lift, 'lift');
    // 5 tune (C1, f) at the chosen position
    const M = bayM(cur), Me = W.estimateM(pad, Math.abs(M), R), ms = W.modeSearch(pad, M, Me, R);
    const tTune = ms.nProbes * LIM.T_PROBE; tReal += tTune;
    ev.push({ type: 'tune', ms, Me, M, real: tTune, phase: 'tune' });
    ev.push({ type: 'apply', ms, real: .5, phase: 'charge' });
    return { ev, best, ms, Me, M, tReal, nMeas: meas.size, lifts };
  }
  function visDur(e) {
    if (e.type === 'move') return reduceMotion ? 0.02 : clamp(e.real / BAY.speed, .04, 1.6);
    if (e.type === 'meas') return reduceMotion ? 0.01 : clamp(.35 / Math.sqrt(BAY.speed), .03, .25);
    if (e.type === 'tune') return reduceMotion ? 0.05 : clamp(e.ms.probes.length * .028 * 5 / BAY.speed, 1.2, 8);
    return .8;
  }
  function bayStart() {
    if (BAY.plan && BAY.pi < BAY.plan.ev.length) return;
    BAY.dots = []; BAY.probes = []; BAY.tune = null; BAY.simClock = 0; BAY.changed = false; BAY.result = null;
    BAY.set = null;
    BAY.plan = buildPlan(); BAY.pi = 0; BAY.pt = 0; BAY.phase = 'locate';
    bayUI();
  }
  function bayStep(dt) {
    if (!BAY.plan || BAY.pi >= BAY.plan.ev.length) return;
    let rem = dt;
    while (rem > 0 && BAY.pi < BAY.plan.ev.length) {
      const e = BAY.plan.ev[BAY.pi], vd = visDur(e), left = vd - BAY.pt, use = Math.min(rem, left);
      BAY.pt += use; rem -= use; const f = clamp(BAY.pt / vd, 0, 1);
      BAY.phase = e.phase;
      if (e.type === 'move') { const ease = f < .5 ? 2 * f * f : 1 - Math.pow(-2 * f + 2, 2) / 2;
        BAY.tx = { x: e.from.x + (e.to.x - e.from.x) * ease, y: e.from.y + (e.to.y - e.from.y) * ease, lift: e.from.lift + (e.to.lift - e.from.lift) * ease }; }
      if (e.type === 'tune') { const n = Math.floor(f * e.ms.probes.length); BAY.tune = e; BAY.probes = e.ms.probes.slice(0, n); }
      if (f >= 1) {
        BAY.simClock += e.real;
        if (e.type === 'meas') BAY.dots.push(e.rec);
        if (e.type === 'move') BAY.tx = { ...e.to };
        if (e.type === 'tune') BAY.probes = e.ms.probes.slice();
        if (e.type === 'apply') { const ms = e.ms;
          BAY.set = ms.source === 'none' ? null : { C1: ms.C1, f: ms.f, s: ms.s, b: ms.b, label: ms.source };
          BAY.result = { P: ms.P, eta: ms.eta, nProbes: ms.nProbes, nMeas: BAY.plan.nMeas, time: BAY.plan.tReal, source: ms.source,
                         Me: BAY.plan.Me, M: BAY.plan.M, k: W.kOf(BAY.pad, BAY.plan.M) };
          BAY.phase = 'charge'; }
        BAY.pi++; BAY.pt = 0;
      }
    }
    if (BAY.pi >= BAY.plan.ev.length) BAY.phase = 'charge';
  }

  // ---- drawing
  function bayDrawTop() {
    const { ctx, w, h } = fit(cvTop, .62, 220, 520), vx = 1.5, s = w / vx, cx = w / 2, cy = h / 2;
    const P = (x, y) => [cx + x * s, cy - y * s];
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = C.ground; ctx.fillRect(0, 0, w, h);
    // stage pit and rails
    const g = GEO[BAY.pad], gtx = g.tx, hw = halfW(gtx) + .02, pitH = TRAVEL + hw + .02;
    const [p0x, p0y] = P(-pitH, pitH); ctx.fillStyle = C.pit; ctx.fillRect(p0x, p0y, 2 * pitH * s, 2 * pitH * s);
    ctx.strokeStyle = C.ground2; ctx.lineWidth = 2; ctx.strokeRect(p0x, p0y, 2 * pitH * s, 2 * pitH * s);
    ctx.fillStyle = C.ground2;
    [-1, 1].forEach(sg => { const [rx, ry] = P(-pitH + .01, sg * (pitH - .015)); ctx.fillRect(rx, ry - 2, (2 * pitH - .02) * s, 4); });
    const [gx0, gy0] = P(BAY.tx.x, pitH - .015), [, gy1] = P(BAY.tx.x, -pitH + .015);
    ctx.fillRect(gx0 - 5, gy0, 10, gy1 - gy0);                       // gantry beam at tx.x
    // scan dots (|M| estimate)
    const mMax = BAY.pad === 'circular' ? 36e-6 : 14e-6;
    for (const d of BAY.dots) { const [x, y] = P(d.x, d.y);
      ctx.beginPath(); ctx.arc(x, y, 5.5, 0, TAU); ctx.fillStyle = seqColor(.08 + .92 * d.Me / mMax); ctx.fill();
      ctx.lineWidth = 2; ctx.strokeStyle = C.pit; ctx.stroke(); }
    // transmitter pad
    const [tx, ty] = P(BAY.tx.x, BAY.tx.y);
    drawPadTop(ctx, gtx, tx, ty, s, { fill: C.padTx, coil: C.copper });
    // car (seen through) + receiver
    ctx.fillStyle = rgba(C.car, .24); ctx.fillRect(0, 0, w, h);
    const [rx, ry] = P(BAY.car.x, BAY.car.y);
    drawPadTop(ctx, g.rx, rx, ry, s, { fill: rgba(C.padRx, .32), coil: rgba(C.copper, .95), edge: C.ink, dash: true });
    // centres + offset line
    ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(tx, ty); ctx.lineTo(rx, ry); ctx.stroke(); ctx.setLineDash([]);
    [[tx, ty, C.padTx], [rx, ry, C.padRx]].forEach(([x, y, c]) => { ctx.beginPath(); ctx.arc(x, y, 4, 0, TAU); ctx.fillStyle = C.ink; ctx.fill(); });
    // labels
    ctx.font = '600 12px system-ui, sans-serif'; ctx.fillStyle = C.ink; ctx.textBaseline = 'top'; ctx.textAlign = 'left';
    halo(ctx, 'Top view (looking down through the car)', 10, 8);
    ctx.font = '12px system-ui, sans-serif'; ctx.fillStyle = C.ink2;
    halo(ctx, 'drag to move the car  ·  car front →', 10, 25);
    ctx.textBaseline = 'bottom';
    const rel = bayRel();
    halo(ctx, `transmitter-to-receiver offset  x ${Math.round(rel.x * 1000)} mm, y ${Math.round(rel.y * 1000)} mm`, 10, h - 8);
    // legend for dots
    if (BAY.dots.length) { const lx = w - 150, ly = h - 30;
      for (let i = 0; i < 40; i++) { ctx.fillStyle = seqColor(.08 + .92 * i / 39); ctx.fillRect(lx + i * 3.5, ly, 3.6, 8); }
      ctx.fillStyle = C.ink2; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom'; ctx.fillText('mapped |M|  low → high', lx, ly - 3); }
    // mini map of the bay
    if (w < 480) return;
    const mw = 92, mh = 50, mx0 = w - mw - 10, my0 = 10, ms = mw / 5.4;
    ctx.fillStyle = rgba(C.surface, .92); ctx.fillRect(mx0 - 4, my0 - 4, mw + 8, mh + 8);
    ctx.strokeStyle = C.grid; ctx.strokeRect(mx0, my0, mw, mh);
    const M0 = (x, y) => [mx0 + mw / 2 + x * ms, my0 + mh / 2 - y * ms];
    const [c0x, c0y] = M0(BAY.car.x - 2.2, BAY.car.y + .9); ctx.fillStyle = rgba(C.ink2, .45); roundRect(ctx, c0x, c0y, 4.4 * ms, 1.8 * ms, 3); ctx.fill();
    const [zx, zy] = M0(-.75, .5); ctx.strokeStyle = C.accent; ctx.lineWidth = 1.5; ctx.strokeRect(zx, zy, 1.5 * ms, 1 * ms);
  }
  function bayCurrents() {
    const M = bayM();
    let on = false, amp = 1, set = BAY.set;
    if (BAY.phase === 'charge' && set) on = true;
    else if (BAY.phase === 'idle' && set) on = true;
    else if (BAY.plan && BAY.pi < BAY.plan.ev.length) {
      const e = BAY.plan.ev[BAY.pi];
      if (['meas', 'tune', 'move'].includes(e.type)) { on = true; amp = LIM.PROBE_AMP;
        const d = D.pads[BAY.pad].design;
        set = e.type === 'tune' && BAY.probes.length ? BAY.probes[BAY.probes.length - 1] : { C1: d.C1_bank[d.C1_bank.length >> 1], f: LIM.F0 };
        set = { C1: set.C1, f: set.f, s: amp }; }
    }
    if (!on || !set) return { M, r: null };
    const r = BAY.phase === 'charge' || BAY.phase === 'idle' ? operate(BAY.pad, M, set.C1, set.f, set.s)
                                                              : W.solve(W.truthP(BAY.pad), set.C1, set.f, M, D.pads[BAY.pad].RL, amp);
    return { M, r, set };
  }
  function bayDrawSide(dt) {
    const { ctx, w, h } = fit(cvSide, .36, 170, 330);
    const { r } = bayCurrents();
    BAY.theta += dt * TAU * (reduceMotion ? 0 : .45);
    const z1 = BAY.clear[BAY.pad] + .17;
    drawSide(ctx, w, h, { pad: BAY.pad, txX: BAY.tx.x, lift: BAY.tx.lift, rxX: BAY.car.x, clear: BAY.clear[BAY.pad], tilt: BAY.tilt,
      i1: r ? { re: r.i1r, im: r.i1i, on: true } : { on: false }, i2: r ? { re: r.i2r, im: r.i2i, on: true } : { on: false },
      theta: BAY.theta, stage: true, car: true, view: { x0: -.75, x1: .75, z0: -.13, z1 } });
    ctx.font = '600 12px system-ui, sans-serif'; ctx.fillStyle = C.ink; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    halo(ctx, 'Side view (x–z section)', 10, 8);
    ctx.font = '12px system-ui, sans-serif'; ctx.fillStyle = C.ink2;
    ctx.fillText(!r ? 'inverter off' : (['charge', 'idle'].includes(BAY.phase) ? 'magnetic field lines, slowed down' : 'low-power probing, slowed down'), 10, 25);
  }
  function bayDrawTune() {
    const { ctx, w, h } = fit(cvTune, .44, 230, 320), pad = BAY.pad, d = D.pads[pad].design;
    ctx.clearRect(0, 0, w, h);
    const box = { x: 52, y: 30, w: w - 66, h: h - 86 };
    const ymax = 8;
    const { X, Y } = axes(ctx, box, [79, 90], [0, ymax], [79, 81, 83, 85, 87, 89], [0, 2, 4, 6, 8], 'Frequency f (kHz)', 'Power (kW)');
    ctx.font = '600 12px system-ui, sans-serif'; ctx.fillStyle = C.ink; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.fillText('(C1, f) search: measured power at full duty', 10, 6);
    // rated line
    ctx.strokeStyle = C.ink2; ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.moveTo(box.x, Y(PR / 1000)); ctx.lineTo(box.x + box.w, Y(PR / 1000)); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = C.ink2; ctx.font = '12px system-ui, sans-serif'; ctx.textAlign = 'right'; ctx.textBaseline = 'bottom';
    halo(ctx, 'rated 3.3 kW', box.x + box.w - 2, Y(PR / 1000) - 2);
    const e = BAY.tune;
    if (!e) { ctx.fillStyle = C.ink2; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(BAY.phase === 'idle' ? 'Press “Align & tune” to start' : 'waiting for the transmitter to settle…', box.x + box.w / 2, box.y + box.h * .25); return; }
    const order = []; e.ms.roots.forEach(r => order.push(r.b));
    const col = b => [C.s1, C.s2, C.s3][Math.max(0, order.indexOf(b))] || C.ink2;
    // mode roots
    e.ms.roots.forEach(r => r.f.forEach(fr => { const fk = fr / 1000; if (fk < 79 || fk > 90) return;
      ctx.strokeStyle = col(r.b); ctx.lineWidth = 1; ctx.setLineDash([2, 3]);
      ctx.beginPath(); ctx.moveTo(X(fk), box.y); ctx.lineTo(X(fk), box.y + box.h); ctx.stroke(); ctx.setLineDash([]); }));
    // probes
    for (const p of BAY.probes) {
      const ok = p.eta >= LIM.ETA_MIN && p.phi >= 0 && p.phi <= LIM.PHI_MAX && p.I1 <= LIM.I1_MAX && p.VC1 <= LIM.VC1_MAX;
      const x = X(p.f / 1000), yv = p.PL / 1000, y = Y(Math.min(yv, ymax));
      ctx.beginPath();
      if (p.tag === 'fallback') { ctx.moveTo(x, y - 5); ctx.lineTo(x + 5, y); ctx.lineTo(x, y + 5); ctx.lineTo(x - 5, y); ctx.closePath(); }
      else ctx.arc(x, y, 4, 0, TAU);
      if (ok) { ctx.fillStyle = col(p.b); ctx.fill(); ctx.strokeStyle = C.surface; ctx.lineWidth = 1.5; ctx.stroke(); }
      else { ctx.strokeStyle = col(p.b); ctx.lineWidth = 1.5; ctx.stroke(); }
    }
    // chosen
    if (BAY.phase === 'charge' && BAY.set) { const x = X(BAY.set.f / 1000), y = Y(Math.min(BAY.result.P / 1000, ymax));
      star(ctx, x, y, 10, C.accent, C.surface); }
    // legend
    let lx = box.x; const ly = h - 14; ctx.font = '12px system-ui, sans-serif'; ctx.textBaseline = 'middle'; ctx.textAlign = 'left';
    e.ms.roots.forEach(r => { ctx.fillStyle = col(r.b); ctx.beginPath(); ctx.arc(lx + 5, ly, 4.5, 0, TAU); ctx.fill();
      ctx.fillStyle = C.ink2; const t = 'C1 ' + bankLabel(pad, r.b); ctx.fillText(t, lx + 13, ly); lx += ctx.measureText(t).width + 28; });
    if (lx < w - 140) { ctx.strokeStyle = C.ink2; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(lx + 5, ly, 4, 0, TAU); ctx.stroke();
      ctx.fillStyle = C.ink2; ctx.fillText('breaks a limit', lx + 13, ly); }
  }
  function star(ctx, x, y, r, fill, edge) {
    ctx.beginPath(); for (let i = 0; i < 10; i++) { const a = -Math.PI / 2 + i * Math.PI / 5, rr = i % 2 ? r * .45 : r;
      ctx.lineTo(x + rr * Math.cos(a), y + rr * Math.sin(a)); } ctx.closePath();
    ctx.fillStyle = fill; ctx.fill(); ctx.strokeStyle = edge; ctx.lineWidth = 2; ctx.stroke();
  }

  // ---- DOM read-outs
  const PHASES = ['locate', 'refine', 'lift', 'tune', 'charge'];
  let uiTick = 0;
  function bayUI() {
    const pad = BAY.pad, M = bayM(), k = W.kOf(pad, M), rel = bayRel();
    const { r, set } = bayCurrents();
    // stepper
    $$('.step', bayEl).forEach(s => { const i = PHASES.indexOf(s.dataset.p), c = PHASES.indexOf(BAY.phase);
      s.classList.toggle('done', BAY.phase !== 'idle' && i < c); s.classList.toggle('now', i === c && BAY.phase !== 'idle'); });
    // status
    let st;
    if (BAY.phase === 'idle') st = 'Fixed tuning, transmitter at home. Press “Align & tune”.';
    else if (BAY.phase === 'locate') st = 'Locating the receiver: coarse scan, |M| from transmitter current at each stop.';
    else if (BAY.phase === 'refine') st = 'Refining the transmitter position around the best stop.';
    else if (BAY.phase === 'lift') st = 'Choosing the lift height.';
    else if (BAY.phase === 'tune') st = 'Tuning (C1, f): probing narrow windows around the predicted coupled modes.';
    else if (BAY.changed) st = 'The car moved. Settings are now stale; press “Align & tune” again.';
    else st = BAY.set ? `Charging. Setting found by the ${BAY.set.label.replace('mode-', '')} ${BAY.set.label === 'fallback' ? 'window' : 'mode window'}.` : 'No safe setting at this position.';
    $('#bay-status', bayEl).textContent = st;
    const busy = !!(BAY.plan && BAY.pi < BAY.plan.ev.length);
    $('#bay-go', bayEl).disabled = busy; bayEl.classList.toggle('busy', busy);
    // required settings
    const out = (id, v) => { const el = $(id, bayEl); if (el.textContent !== v) el.textContent = v; };
    out('#o-txx', `${Math.round(BAY.tx.x * 1000)} mm`); out('#o-txy', `${Math.round(BAY.tx.y * 1000)} mm`);
    out('#o-lift', `${Math.round(BAY.tx.lift * 1000)} mm`); out('#o-gap', `${Math.round(rel.gap * 1000)} mm`);
    out('#o-k', fmt(Math.abs(k), 3));
    const showSet = (BAY.phase === 'charge' || BAY.phase === 'idle') ? BAY.set : null;
    out('#o-c1', showSet ? (showSet.b == null ? `${fmt(showSet.C1 * 1e9, 2)} nF (nominal)` : bankLabel(pad, showSet.b)) : '…');
    out('#o-f', showSet ? `${fmt(showSet.f / 1000, 1)} kHz` : '…');
    const live = (BAY.phase === 'charge' || BAY.phase === 'idle') && r;
    out('#o-p', live ? `${fmt(r.PL / 1000, 2)} kW` : (BAY.phase === 'charge' ? '0 kW' : '…'));
    out('#o-eta', live ? `${fmt(r.eta * 100, 1)} %` : '…');
    out('#o-i1', live ? `${fmt(r.I1, 1)} A` : '…'); out('#o-vc1', live ? `${fmt(r.VC1 / 1000, 2)} kV` : '…');
    const warn = live ? [r.derated ? 'derated to stay within limits' : '', !r.zvs ? 'no soft switching' : '', !r.effOk ? 'efficiency < 85 %' : ''].filter(Boolean).join(' · ') : '';
    out('#o-warn', warn);
    const R = BAY.result;
    out('#o-mmap', R ? `${fmt(R.Me * 1e6, 2)} µH (true ${fmt(Math.abs(R.M) * 1e6, 2)})` : '…');
    out('#o-probes', R ? `${R.nMeas} stops · ${R.nProbes} tuning probes` : (busy ? `${BAY.dots.length} stops…` : '…'));
    out('#o-time', busy || R ? `${fmt(BAY.simClock, 1)} s` : '…');
    // comparison bars
    const Mh = bayM({ x: 0, y: 0, lift: 0 }), fx = W.fixedTuning(pad, Mh), op = W.trueOptimum(pad, Mh);
    const ours = R && !BAY.changed ? Math.min(R.P, PR * LIM.P_OVER) : null;
    const rows = [[fx.P, fx.eta, 'Fixed tuning, transmitter at home'], [op ? op.P : 0, op ? op.eta : NaN, 'Best (C1, f), transmitter at home'],
                  [ours, R ? R.eta : NaN, 'Transmitter positioned + (C1, f) tuned']];
    $$('.cmp-row', bayEl).forEach((el, i) => { const [p, e] = rows[i], pct = p == null ? 0 : clamp(p / PR, 0, 1.05);
      $('.bar i', el).style.width = (pct / 1.05 * 100) + '%';
      $('.v', el).textContent = p == null ? '–' : `${fmt(p / 1000, 2)} kW${isFinite(e) && p > 1 ? ` · η ${fmt(e * 100, 1)} %` : ''}`; });
  }

  // ---- inputs
  seg($('#bay-pad', bayEl), v => { BAY.pad = v; $('#bay-clear', bayEl).value = Math.round(BAY.clear[v] * 1000); bayInputs(); bayReset(); });
  function bayInputs() {
    $('#bay-clear-v', bayEl).textContent = $('#bay-clear', bayEl).value + ' mm';
    $('#bay-tilt-v', bayEl).textContent = $('#bay-tilt', bayEl).value + '°';
    $('#bay-cx-v', bayEl).textContent = Math.round(BAY.car.x * 1000) + ' mm';
    $('#bay-cy-v', bayEl).textContent = Math.round(BAY.car.y * 1000) + ' mm';
    $('#bay-cx', bayEl).value = Math.round(BAY.car.x * 1000); $('#bay-cy', bayEl).value = Math.round(BAY.car.y * 1000);
  }
  function carMoved() { if (BAY.phase === 'charge' && BAY.result) BAY.changed = true; bayInputs(); }
  $('#bay-cx', bayEl).addEventListener('input', e => { BAY.car.x = +e.target.value / 1000; carMoved(); });
  $('#bay-cy', bayEl).addEventListener('input', e => { BAY.car.y = +e.target.value / 1000; carMoved(); });
  $('#bay-clear', bayEl).addEventListener('input', e => { BAY.clear[BAY.pad] = +e.target.value / 1000; carMoved(); });
  $('#bay-tilt', bayEl).addEventListener('input', e => { BAY.tilt = +e.target.value; carMoved(); });
  $('#bay-liftok', bayEl).addEventListener('change', e => { BAY.lift = e.target.checked; });
  seg($('#bay-speed', bayEl), v => { BAY.speed = +v; });
  $('#bay-go', bayEl).addEventListener('click', bayStart);
  $('#bay-reset', bayEl).addEventListener('click', () => bayReset());
  $('#bay-park', bayEl).addEventListener('click', () => {
    if (BAY.plan && BAY.pi < BAY.plan.ev.length) return;
    const R = W.rng(Date.now() & 0xffff);
    BAY.car.x = Math.round((R.u() * .46 - .23) * 100) / 100; BAY.car.y = Math.round((R.u() * .4 - .2) * 100) / 100;
    BAY.tilt = Math.round(R.u() * 6); $('#bay-tilt', bayEl).value = BAY.tilt;
    bayReset(); bayInputs(); });
  // drag the car on the top view
  (function () {
    let drag = null;
    cvTop.addEventListener('pointerdown', e => { if (BAY.plan && BAY.pi < BAY.plan.ev.length) return;
      drag = { x: e.clientX, y: e.clientY, cx: BAY.car.x, cy: BAY.car.y }; cvTop.setPointerCapture(e.pointerId); });
    cvTop.addEventListener('pointermove', e => { if (!drag) return; const s = cvTop.clientWidth / 1.5;
      BAY.car.x = clamp(Math.round((drag.cx + (e.clientX - drag.x) / s) * 200) / 200, -.25, .25);
      BAY.car.y = clamp(Math.round((drag.cy - (e.clientY - drag.y) / s) * 200) / 200, -.25, .25); carMoved(); });
    const end = () => { drag = null; }; cvTop.addEventListener('pointerup', end); cvTop.addEventListener('pointercancel', end);
  })();
  // tune chart hover
  (function () {
    const tip = $('#bay-tip', bayEl), host = cvTune.parentElement;
    cvTune.addEventListener('pointermove', e => {
      if (!BAY.probes.length) return tooltip(tip, null);
      const rc = cvTune.getBoundingClientRect(), mx = e.clientX - rc.left, my = e.clientY - rc.top, w = rc.width, h = rc.height;
      const box = { x: 52, y: 30, w: w - 66, h: h - 86 };
      let best = null, bd = 18;
      for (const p of BAY.probes) { const x = box.x + (p.f / 1000 - 79) / 11 * box.w, y = box.y + box.h - Math.min(p.PL / 1000, 8) / 8 * box.h;
        const dd = Math.hypot(x - mx, y - my); if (dd < bd) { bd = dd; best = p; } }
      if (!best) return tooltip(tip, null);
      tooltip(tip, `<b>${fmt(best.f / 1000, 2)} kHz</b> · C1 ${bankLabel(BAY.pad, best.b)}<br>P ${fmt(best.PL / 1000, 2)} kW · η ${fmt(best.eta * 100, 1)} %<br>I1 ${fmt(best.I1, 1)} A · V<sub>C1</sub> ${fmt(best.VC1 / 1000, 2)} kV · φ ${fmt(best.phi, 0)}°<br><span class="muted">window: ${best.tag.replace('mode-', '')}</span>`,
        mx, my, host);
    });
    cvTune.addEventListener('pointerleave', () => tooltip(tip, null));
  })();

  loops.bay = (dt) => {
    bayStep(dt);
    bayDrawTop(); bayDrawSide(dt); bayDrawTune();
    if ((uiTick += dt) > .1) { uiTick = 0; bayUI(); }
  };
  bayInputs(); bayReset();

  // ====================================================================================
  // SIM 2 - RESONANCE LAB
  // ====================================================================================
  const LAB = { pad: 'circular', x: .06, gap: .15, b: 7, f: 85000, theta: 0, slow: true };
  const labEl = $('#sim-lab');
  const cvScope = $('#lab-scope', labEl), cvField = $('#lab-field', labEl), cvPf = $('#lab-pf', labEl);
  function labState() {
    const pad = LAB.pad, d = D.pads[pad].design, C1 = d.C1_bank[LAB.b], M = W.interpM(pad, LAB.x, 0, LAB.gap, 0);
    const r = W.solve(W.truthP(pad), C1, LAB.f, M, D.pads[pad].RL, 1);
    return { pad, C1, M, r, k: W.kOf(pad, M) };
  }
  function limitsOf(r) {
    const v = [];
    if (r.I1 > LIM.I1_MAX) v.push('I1 > 25 A');
    if (r.VC1 > LIM.VC1_MAX) v.push('V_C1 > 3 kV');
    if (r.PL > LIM.P_OVER * PR) v.push('P > rated');
    if (r.phi < 0) v.push('capacitive (no ZVS)');
    else if (r.phi > LIM.PHI_MAX) v.push('φ > 60°');
    if (r.eta < LIM.ETA_MIN) v.push('η < 85 %');
    return v;
  }
  function labUI() {
    const S = labState(), r = S.r, v = limitsOf(r), t = S.pad === 'circular';
    $('#lab-x-v', labEl).textContent = Math.round(LAB.x * 1000) + ' mm';
    $('#lab-gap-v', labEl).textContent = Math.round(LAB.gap * 1000) + ' mm';
    $('#lab-b-v', labEl).textContent = bankLabel(S.pad, LAB.b);
    $('#lab-f-v', labEl).textContent = fmt(LAB.f / 1000, 1) + ' kHz';
    $('#lab-k', labEl).textContent = fmt(Math.abs(S.k), 3);
    $('#lab-p', labEl).textContent = fmt(r.PL / 1000, 2) + ' kW';
    $('#lab-eta', labEl).textContent = fmt(r.eta * 100, 1) + ' %';
    $('#lab-i1', labEl).textContent = fmt(r.I1, 1) + ' A';
    $('#lab-vc1', labEl).textContent = fmt(r.VC1 / 1000, 2) + ' kV';
    $('#lab-phi', labEl).textContent = fmt(r.phi, 0) + '°';
    const st = $('#lab-state', labEl);
    st.className = 'badge ' + (v.length ? 'bad' : 'ok');
    st.innerHTML = v.length ? '<span aria-hidden="true">✕</span> ' + v.join(' · ') : '<span aria-hidden="true">✓</span> within all limits';
  }
  function labDrawScope(dt) {
    const { ctx, w, h } = fit(cvScope, .5, 190, 320), S = labState(), r = S.r;
    LAB.theta += dt * TAU * (reduceMotion ? 0 : (LAB.slow ? .25 : 1.2));
    const scr = '#0c1512'; ctx.fillStyle = scr; roundRect(ctx, 0, 0, w, h, 10); ctx.fill();
    const pad = 12, bx = pad, by = 26, bw = w - 2 * pad, bh = h - by - 30;
    ctx.strokeStyle = 'rgba(160,200,180,.16)'; ctx.lineWidth = 1;
    for (let i = 0; i <= 10; i++) { ctx.beginPath(); ctx.moveTo(bx + i * bw / 10, by); ctx.lineTo(bx + i * bw / 10, by + bh); ctx.stroke(); }
    for (let j = 0; j <= 8; j++) { ctx.beginPath(); ctx.moveTo(bx, by + j * bh / 8); ctx.lineTo(bx + bw, by + j * bh / 8); ctx.stroke(); }
    ctx.strokeStyle = 'rgba(160,200,180,.35)'; ctx.beginPath(); ctx.moveTo(bx, by + bh / 2); ctx.lineTo(bx + bw, by + bh / 2); ctx.stroke();
    const periods = 2.5, T = 1 / LAB.f, tspan = periods * T, Vdc = D.pads[S.pad].true.V1 * Math.PI / (2 * Math.SQRT2);
    const nice = v => { for (const s of [1, 2, 5, 10, 20, 50, 100, 200]) if (v <= s * 3.6) return s; return 500; };
    const sV = 100, sI = nice(Math.max(r.I1, r.I2) * Math.SQRT2);
    const yv = (val, sc) => by + bh / 2 - val / sc * bh / 8;
    const tr = (fn, col, sc) => { ctx.beginPath();
      for (let i = 0; i <= bw; i += 1.5) { const ph = TAU * periods * i / bw + LAB.theta; const y = clamp(yv(fn(ph), sc), by, by + bh);
        i ? ctx.lineTo(bx + i, y) : ctx.moveTo(bx + i, y); }
      ctx.strokeStyle = rgba(col, .25); ctx.lineWidth = 5; ctx.stroke(); ctx.strokeStyle = col; ctx.lineWidth = 1.8; ctx.stroke(); };
    const a1 = Math.atan2(r.i1i, r.i1r), a2 = Math.atan2(r.i2i, r.i2r);
    tr(ph => Math.cos(ph) >= 0 ? Vdc : -Vdc, '#f2c94c', sV);
    tr(ph => Math.SQRT2 * r.I1 * Math.cos(ph + a1), '#4dd8e6', sI);
    tr(ph => Math.SQRT2 * r.I2 * Math.cos(ph + a2), '#ff6fae', sI);
    ctx.font = '600 12px system-ui, sans-serif'; ctx.textBaseline = 'middle'; ctx.textAlign = 'left';
    const lab = [['#f2c94c', `CH1 v₁ inverter  ${sV} V/div`], ['#4dd8e6', `CH2 i₁ transmitter  ${sI} A/div`], ['#ff6fae', `CH3 i₂ receiver  ${sI} A/div`]];
    let lx = bx; lab.forEach(([c, t]) => { ctx.fillStyle = c; ctx.fillText(t, lx, 13); lx += ctx.measureText(t).width + 16; });
    ctx.fillStyle = 'rgba(200,230,215,.75)'; ctx.font = '12px system-ui, sans-serif';
    ctx.fillText(`${fmt(tspan / 10 * 1e6, 2)} µs/div  ·  f = ${fmt(LAB.f / 1000, 1)} kHz  ·  shown in slow motion`, bx, h - 14);
  }
  function labDrawField() {
    const { ctx, w, h } = fit(cvField, .5, 170, 300), S = labState(), r = S.r;
    drawSide(ctx, w, h, { pad: S.pad, txX: -LAB.x / 2, lift: 0, rxX: LAB.x / 2, clear: LAB.gap, tilt: 0,
      i1: { re: r.i1r, im: r.i1i, on: true }, i2: { re: r.i2r, im: r.i2i, on: true }, theta: LAB.theta, stage: false, car: false,
      fieldAlpha: 1, view: { x0: -.62, x1: .62, z0: -.04, z1: LAB.gap + .2 } });
    ctx.font = '600 12px system-ui, sans-serif'; ctx.fillStyle = C.ink; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.fillText('Field lines between the pads', 10, 8);
    ctx.font = '12px system-ui, sans-serif'; ctx.fillStyle = C.ink2; ctx.fillText('transmitter below, receiver above', 10, 25);
  }
  let pfCache = null;
  function labCurve() {
    const key = [LAB.pad, LAB.x, LAB.gap, LAB.b].join();
    if (pfCache && pfCache.key === key) return pfCache;
    const pad = LAB.pad, d = D.pads[pad].design, C1 = d.C1_bank[LAB.b], M = W.interpM(pad, LAB.x, 0, LAB.gap, 0), p = W.truthP(pad), pts = [];
    for (let f = 79000; f <= 90000; f += 50) { const r = W.solve(p, C1, f, M, D.pads[pad].RL, 1); pts.push({ f, r, bad: limitsOf(r).length > 0 }); }
    const roots = W.modeRoots(p.L1, p.L2, C1, p.C2, M);
    return (pfCache = { key, pts, roots });
  }
  function labDrawPf() {
    const { ctx, w, h } = fit(cvPf, .45, 210, 320), cv = labCurve();
    ctx.clearRect(0, 0, w, h);
    const ymax = 8, box = { x: 52, y: 30, w: w - 66, h: h - 92 };
    const { X, Y } = axes(ctx, box, [79, 90], [0, ymax], [79, 81, 83, 85, 87, 89], [0, 2, 4, 6, 8], 'Frequency f (kHz)', 'Power (kW)');
    ctx.font = '600 12px system-ui, sans-serif'; ctx.fillStyle = C.ink; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.fillText(`Power vs frequency at C1 ${bankLabel(LAB.pad, LAB.b)}`, 10, 6);
    // limit strip
    const sy = box.y + box.h + 38;
    ctx.font = '12px system-ui, sans-serif'; ctx.textBaseline = 'middle'; ctx.fillStyle = C.ink2; ctx.textAlign = 'right';
    ctx.fillText('limits', box.x - 6, sy + 5);
    for (let i = 0; i + 1 < cv.pts.length; i++) { const a = cv.pts[i];
      ctx.fillStyle = a.bad ? C.bad : C.good; ctx.fillRect(X(a.f / 1000), sy, X(cv.pts[i + 1].f / 1000) - X(a.f / 1000) + .6, 10); }
    // rated
    ctx.strokeStyle = C.ink2; ctx.setLineDash([4, 4]); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(box.x, Y(PR / 1000)); ctx.lineTo(box.x + box.w, Y(PR / 1000)); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = C.ink2; ctx.textAlign = 'right'; ctx.textBaseline = 'bottom'; ctx.fillText('rated 3.3 kW', box.x + box.w - 2, Y(PR / 1000) - 2);
    // modes
    cv.roots.forEach((fr, i) => { const fk = fr / 1000; if (fk < 79 || fk > 90) return;
      ctx.strokeStyle = C.ink2; ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(X(fk), box.y); ctx.lineTo(X(fk), box.y + box.h); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = C.ink2; ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
      ctx.fillText(cv.roots.length === 2 ? (i ? 'f₊' : 'f₋') : 'f₀', X(fk), box.y - 2); });
    // curve
    ctx.save(); ctx.beginPath(); ctx.rect(box.x, box.y, box.w, box.h); ctx.clip();
    ctx.beginPath(); cv.pts.forEach((p, i) => { const x = X(p.f / 1000), y = Y(Math.min(p.r.PL / 1000, ymax + 1)); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.strokeStyle = C.s1; ctx.lineWidth = 2; ctx.stroke(); ctx.restore();
    // current point
    const S = labState(), x = X(LAB.f / 1000), y = Y(Math.min(S.r.PL / 1000, ymax));
    ctx.strokeStyle = C.ink; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x, box.y); ctx.lineTo(x, box.y + box.h); ctx.stroke();
    ctx.beginPath(); ctx.arc(x, y, 6, 0, TAU); ctx.fillStyle = C.s1; ctx.fill(); ctx.strokeStyle = C.surface; ctx.lineWidth = 2; ctx.stroke();
    LAB.box = box;
  }
  (function () {
    const tip = $('#lab-tip', labEl), host = cvPf.parentElement;
    cvPf.addEventListener('pointermove', e => {
      const rc = cvPf.getBoundingClientRect(), mx = e.clientX - rc.left, my = e.clientY - rc.top, b = LAB.box;
      if (!b || mx < b.x || mx > b.x + b.w) return tooltip(tip, null);
      const fk = 79 + (mx - b.x) / b.w * 11, cv = labCurve();
      const p = cv.pts[clamp(Math.round((fk * 1000 - 79000) / 50), 0, cv.pts.length - 1)], r = p.r, v = limitsOf(r);
      tooltip(tip, `<b>${fmt(p.f / 1000, 2)} kHz</b><br>P ${fmt(r.PL / 1000, 2)} kW · η ${fmt(r.eta * 100, 1)} %<br>I1 ${fmt(r.I1, 1)} A · V<sub>C1</sub> ${fmt(r.VC1 / 1000, 2)} kV · φ ${fmt(r.phi, 0)}°<br>${v.length ? '<span class="tbad">✕ ' + v.join(', ') + '</span>' : '<span class="tok">✓ within limits</span>'}<br><span class="muted">click to set f</span>`, mx, my, host);
    });
    cvPf.addEventListener('pointerleave', () => tooltip(tip, null));
    cvPf.addEventListener('click', e => { const rc = cvPf.getBoundingClientRect(), b = LAB.box, mx = e.clientX - rc.left;
      if (!b) return; LAB.f = clamp(Math.round((79 + (mx - b.x) / b.w * 11) * 10) * 100, 79000, 90000); $('#lab-f', labEl).value = LAB.f / 1000; labUI(); });
  })();
  seg($('#lab-pad', labEl), v => { LAB.pad = v; LAB.gap = D.pads[v].nominal_gap; $('#lab-gap', labEl).value = LAB.gap * 1000; labUI(); });
  $('#lab-x', labEl).addEventListener('input', e => { LAB.x = +e.target.value / 1000; labUI(); });
  $('#lab-gap', labEl).addEventListener('input', e => { LAB.gap = +e.target.value / 1000; labUI(); });
  $('#lab-b', labEl).addEventListener('input', e => { LAB.b = +e.target.value - 1; labUI(); });
  $('#lab-f', labEl).addEventListener('input', e => { LAB.f = Math.round(+e.target.value * 10) * 100; labUI(); });
  $('#lab-slow', labEl).addEventListener('change', e => { LAB.slow = e.target.checked; });
  let glide = null;
  function glideTo(b, f) { glide = { b0: LAB.b, b1: b, f0: LAB.f, f1: f, t: 0 }; }
  $('#lab-best', labEl).addEventListener('click', () => { const M = W.interpM(LAB.pad, LAB.x, 0, LAB.gap, 0), o = W.trueOptimum(LAB.pad, M);
    if (o) glideTo(o.b, o.f); else $('#lab-state', labEl).textContent = 'No setting meets every limit here.'; });
  $('#lab-fixed', labEl).addEventListener('click', () => { const d = D.pads[LAB.pad].design;
    let b = 0; d.C1_bank.forEach((c, i) => { if (Math.abs(c - d.C1_nom) < Math.abs(d.C1_bank[b] - d.C1_nom)) b = i; }); glideTo(b, LIM.F0); });
  loops.lab = dt => {
    if (glide) { glide.t = Math.min(1, glide.t + dt / (reduceMotion ? .01 : .8)); const e = glide.t < .5 ? 2 * glide.t * glide.t : 1 - Math.pow(-2 * glide.t + 2, 2) / 2;
      LAB.b = Math.round(glide.b0 + (glide.b1 - glide.b0) * e); LAB.f = Math.round((glide.f0 + (glide.f1 - glide.f0) * e) / 100) * 100;
      $('#lab-b', labEl).value = LAB.b + 1; $('#lab-f', labEl).value = LAB.f / 1000; labUI(); if (glide.t >= 1) glide = null; }
    labDrawScope(dt); labDrawField(); labDrawPf();
  };
  labUI();

  // ====================================================================================
  // SIM 3 - RESULTS EXPLORER (recorded runs)
  // ====================================================================================
  const RX = { pad: 'circular', method: 'mode', tilt: 0, y: 0, sel: null };
  const rxEl = $('#sim-res'), cvHeat = $('#res-heat', rxEl);
  const col = pad => { const c = D.pads[pad].cols, o = {}; c.forEach((n, i) => o[n] = i); return o; };
  function slice() {
    const P = D.pads[RX.pad], c = col(RX.pad);
    return P.rows.filter(r => Math.abs(r[c.tilt] - RX.tilt) < 1e-6 && Math.abs(r[c.y] - RX.y) < 1e-6);
  }
  function resDraw() {
    const { ctx, w, h } = fit(cvHeat, .62, 240, 460), P = D.pads[RX.pad], c = col(RX.pad), G = P.grid, rows = slice();
    ctx.clearRect(0, 0, w, h);
    const box = { x: 60, y: 14, w: w - 74, h: h - 66 }, nx = G.x.length, ng = G.gap.length, cw = box.w / nx, ch = box.h / ng;
    RX.cells = [];
    for (const r of rows) { const i = G.x.findIndex(v => Math.abs(v - r[c.x]) < 1e-6), j = G.gap.findIndex(v => Math.abs(v - r[c.gap]) < 1e-6);
      const Pm = r[c[RX.method + '_P']], t = clamp(Math.min(Pm, PR) / PR, 0, 1);
      const x = box.x + i * cw, y = box.y + box.h - (j + 1) * ch;
      ctx.fillStyle = Pm <= 1 ? C.grid : seqColor(.06 + .94 * t); ctx.fillRect(x + 1, y + 1, cw - 2, ch - 2);
      RX.cells.push({ x, y, w: cw, h: ch, r });
      if (RX.sel === r) { ctx.strokeStyle = C.ink; ctx.lineWidth = 2.5; ctx.strokeRect(x + 1.5, y + 1.5, cw - 3, ch - 3); }
    }
    ctx.font = '12px system-ui, sans-serif'; ctx.fillStyle = C.ink2; ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    G.x.forEach((v, i) => { if (i % 2 === 0) ctx.fillText(Math.round(v * 1000), box.x + (i + .5) * cw, box.y + box.h + 5); });
    ctx.fillText('Relative offset x (mm)', box.x + box.w / 2, box.y + box.h + 21);
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    G.gap.forEach((v, j) => { if (j % 2 === 0) ctx.fillText(Math.round(v * 1000), box.x - 6, box.y + box.h - (j + .5) * ch); });
    ctx.save(); ctx.translate(14, box.y + box.h / 2); ctx.rotate(-Math.PI / 2); ctx.textAlign = 'center'; ctx.fillText('Air gap (mm)', 0, 0); ctx.restore();
    // summary
    const m = RX.method, mean = rows.reduce((a, r) => a + Math.min(r[c[m + '_P']], PR), 0) / rows.length / PR * 100,
          pr = rows.reduce((a, r) => a + r[c[m + '_probes']], 0) / rows.length;
    $('#res-sum', rxEl).textContent = `Slice mean: ${fmt(mean, 0)} % of rated · ${fmt(pr, 0)} probes per pose`;
  }
  function resDetail(r) {
    const c = col(RX.pad), el = $('#res-detail', rxEl);
    if (!r) { el.innerHTML = '<p class="muted">Tap or hover a cell to see the recorded pose.</p>'; return; }
    const row = (m, name) => { const P = r[c[m + '_P']], b = r[c[m + '_C1i']], f = r[c[m + '_f']];
      const cl = m === 'fixed' ? `nominal · ${fmt(D.pads[RX.pad].design.C1_nom * 1e9, 2)} nF` : (b >= 0 ? bankLabel(RX.pad, b) : '–');
      return `<tr><th>${name}</th><td>${cl}</td><td>${f ? fmt(f / 1000, 1) + ' kHz' : '–'}</td><td>${fmt(P / 1000, 2)} kW</td><td>${r[c[m + '_eta']] ? fmt(r[c[m + '_eta']] * 100, 1) + ' %' : '–'}</td><td>${r[c[m + '_probes']]}</td></tr>`; };
    const Mt = Math.abs(r[c.M_true_uH]), Mh = r[c.M_hat_uH];
    el.innerHTML = `<p><b>Pose:</b> x ${Math.round(r[c.x] * 1000)} mm · y ${Math.round(r[c.y] * 1000)} mm · gap ${Math.round(r[c.gap] * 1000)} mm · tilt ${r[c.tilt]}°</p>
      <p><b>|M|:</b> true ${fmt(Mt, 2)} µH · mapped ${fmt(Mh, 2)} µH (${Mt > .05 ? fmt((Mh - Mt) / Mt * 100, 1) + ' %' : 'weak coupling'}) · best possible ${fmt(Math.min(r[c.P_opt], PR) / 1000, 2)} kW</p>
      <div class="tablewrap"><table><tr><th>Method</th><th>C1 state</th><th>f</th><th>Power</th><th>η</th><th>Probes</th></tr>
      ${row('mode', 'Proposed')}${row('full', 'Full sweep')}${row('fixed', 'Fixed')}</table></div>
      <p class="muted">Proposed setting found in: ${r[c.mode_src].replace('mode-', '') }${r[c.mode_src].startsWith('mode') ? ' mode window' : ''}.</p>`;
  }
  (function () {
    const tip = $('#res-tip', rxEl), host = cvHeat.parentElement;
    const hit = e => { const rc = cvHeat.getBoundingClientRect(), mx = e.clientX - rc.left, my = e.clientY - rc.top;
      return [(RX.cells || []).find(q => mx >= q.x && mx < q.x + q.w && my >= q.y && my < q.y + q.h), mx, my]; };
    cvHeat.addEventListener('pointermove', e => { const [q, mx, my] = hit(e); const c = col(RX.pad);
      if (!q) return tooltip(tip, null);
      tooltip(tip, `x ${Math.round(q.r[c.x] * 1000)} mm · gap ${Math.round(q.r[c.gap] * 1000)} mm<br><b>${fmt(q.r[c[RX.method + '_P']] / 1000, 2)} kW</b>`, mx, my, host); });
    cvHeat.addEventListener('pointerleave', () => tooltip(tip, null));
    cvHeat.addEventListener('click', e => { const [q] = hit(e); if (q) { RX.sel = q.r; resDetail(q.r); resDraw(); } });
  })();
  seg($('#res-pad', rxEl), v => { RX.pad = v; RX.sel = null; resDetail(null); resDraw(); });
  seg($('#res-method', rxEl), v => { RX.method = v; resDraw(); });
  seg($('#res-tilt', rxEl), v => { RX.tilt = +v; RX.sel = null; resDetail(null); resDraw(); });
  seg($('#res-y', rxEl), v => { RX.y = +v / 1000; RX.sel = null; resDetail(null); resDraw(); });
  let resW = 0;
  loops.res = () => { if (cvHeat.clientWidth !== resW) { resW = cvHeat.clientWidth; resDraw(); } };
  resDetail(null);
  window.addEventListener('resize', () => { resW = 0; });
  if (window.matchMedia) matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => setTimeout(() => { resW = 0; }, 80));

  requestAnimationFrame(frame);
})();
