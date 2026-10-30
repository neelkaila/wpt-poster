/* WPT physics + algorithm engine for the website.
 * A direct port of the project's Python code:
 *   circuit.py (SS link, FHA), cf_search.py (select rule, mode-centred search, full sweep),
 *   resonance.py (coupled-mode quadratic), mapping.py (|M| from transmitter current).
 * M(x, y, gap, tilt) comes from dense tables computed with em_coupling.py / em_dd.py.
 * x, y = relative offset between the pad centres (receiver minus transmitter), metres.
 */
(function (root) {
  'use strict';
  let D = null;
  const TWO_PI = 2 * Math.PI;

  function init(data) { D = data; return api; }

  // ---------------------------------------------------------------- random numbers
  function rng(seed) {                       // mulberry32 + Box-Muller
    let a = (seed >>> 0) || 1;
    const u = () => { a |= 0; a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a);
      t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; };
    return { u, n: () => { const r = Math.sqrt(-2 * Math.log(u() + 1e-12)); return r * Math.cos(TWO_PI * u()); } };
  }

  // ---------------------------------------------------------------- M(x, y, gap, tilt)
  function idx(axis, v) {                    // index + fraction for linear interpolation (clamped)
    const n = axis.length;
    if (v <= axis[0]) return [0, 0];
    if (v >= axis[n - 1]) return [n - 2, 1];
    let i = 0; while (axis[i + 1] < v) i++;
    return [i, (v - axis[i]) / (axis[i + 1] - axis[i])];
  }
  function interpM(pad, x, y, gap, tilt) {
    const T = D.mtable, A = T.M[pad];
    const [it, ft] = idx(T.tilts, tilt), [ig, fg] = idx(T.gaps, gap);
    const [ix, fx] = idx(T.xs, x), [iy, fy] = idx(T.ys, Math.abs(y));
    let s = 0;
    for (let a = 0; a < 2; a++) for (let b = 0; b < 2; b++) for (let c = 0; c < 2; c++) for (let d = 0; d < 2; d++) {
      const w = (a ? ft : 1 - ft) * (b ? fg : 1 - fg) * (c ? fx : 1 - fx) * (d ? fy : 1 - fy);
      if (w) s += w * A[it + a][ig + b][ix + c][iy + d];
    }
    return s * 1e-6;                          // table is in uH
  }
  function kOf(pad, M) { const t = D.pads[pad].true; return M / Math.sqrt(t.L1 * t.L2); }

  // ---------------------------------------------------------------- circuit (complex arithmetic inline)
  // Generic SS solve. p = {L1, L2, C2, R1_0, R2_0, r_inv, V1}. Returns phasors and powers.
  function solve(p, C1, f, M, RL, amp) {
    amp = amp === undefined ? 1 : amp;
    const w = TWO_PI * f, sq = Math.sqrt(f / D.limits.F0);
    const z1r = p.R1_0 * sq + p.r_inv, z1i = w * p.L1 - 1 / (w * C1);
    const z2r = p.R2_0 * sq + RL, z2i = w * p.L2 - 1 / (w * p.C2);
    const wM2 = (w * M) * (w * M), d2 = z2r * z2r + z2i * z2i;
    const zr = wM2 * z2r / d2, zi = -wM2 * z2i / d2;            // (wM)^2 / Z2
    const zinr = z1r + zr, zini = z1i + zi, dz = zinr * zinr + zini * zini;
    const V = p.V1 * amp;
    const i1r = V * zinr / dz, i1i = -V * zini / dz;             // I1 = V / Zin
    // I2 = -j w M I1 / Z2
    const nr = w * M * i1i, ni = -w * M * i1r;                  // -j*wM*I1
    const i2r = (nr * z2r + ni * z2i) / d2, i2i = (ni * z2r - nr * z2i) / d2;
    const I2m2 = i2r * i2r + i2i * i2i;
    const PL = I2m2 * RL, Pin = V * i1r, I1 = Math.hypot(i1r, i1i);
    return { I1, i1r, i1i, i2r, i2i, I2: Math.sqrt(I2m2), PL, Pin, eta: PL / Math.max(Pin, 1e-12),
             phi: Math.atan2(zini, zinr) * 180 / Math.PI, VC1: I1 / (w * C1), Zinr: zinr, Zini: zini };
  }
  const truthP = pad => D.pads[pad].true;
  function modelP(pad) {                      // what the algorithm knows: calibrated Tx + datasheet Rx
    const P = D.pads[pad];
    return { L1: P.cal.L1, R1_0: P.cal.R1_0, r_inv: P.design.r_inv, L2: P.design.L2_nom, C2: P.design.C2_nom,
             R2_0: P.design.R2_0_nom, V1: P.design.V1 };
  }
  function fGrid(pad) {
    const d = D.pads[pad].design, out = [];
    for (let f = d.f_lo; f <= d.f_hi + 1; f += d.f_step) out.push(Math.round(f));
    return out;
  }

  // ---------------------------------------------------------------- the common decision rule (cf_search.select)
  function select(q, enforceZvs) {
    if (enforceZvs === undefined) enforceZvs = true;
    const L = D.limits, g = L.GUARD, n = q.length, Pr = L.P_RATED;
    const base = new Array(n), full = new Array(n);
    let anyFull = false, anyBase = false;
    for (let i = 0; i < n; i++) {
      const r = q[i];
      base[i] = r.eta >= L.ETA_MIN && r.phi <= L.PHI_MAX && (!enforceZvs || r.phi >= L.PHI_MIN);
      full[i] = base[i] && r.I1 <= g * L.I1_MAX && r.VC1 <= g * L.VC1_MAX && r.PL <= L.P_OVER * Pr;
      anyFull = anyFull || full[i]; anyBase = anyBase || base[i];
    }
    if (!anyBase) return null;
    const mask = anyFull ? full : base, s = new Array(n), score = new Array(n);
    let best = -Infinity;
    for (let i = 0; i < n; i++) {
      const r = q[i];
      s[i] = anyFull ? 1 : Math.min(1, g * L.I1_MAX / Math.max(r.I1, 1e-9), g * L.VC1_MAX / Math.max(r.VC1, 1e-9),
                                    r.PL > L.P_OVER * Pr ? Math.sqrt(Pr / Math.max(r.PL, 1e-9)) : 1);
      score[i] = mask[i] ? Math.min(r.PL * s[i] * s[i], Pr) : -Infinity;
      if (score[i] > best) best = score[i];
    }
    let bi = -1, be = -Infinity;
    for (let i = 0; i < n; i++) if (mask[i] && score[i] >= best - 0.005 * Pr && q[i].eta > be) { be = q[i].eta; bi = i; }
    return { i: bi, s: s[bi], full: anyFull };
  }

  // ---------------------------------------------------------------- noise-free optimum over the full grid (truth)
  function trueOptimum(pad, M, enforceZvs) {
    const p = truthP(pad), RL = D.pads[pad].RL, bank = D.pads[pad].design.C1_bank, q = [];
    for (const f of fGrid(pad)) for (let b = 0; b < bank.length; b++) {
      const r = solve(p, bank[b], f, M, RL); r.f = f; r.C1 = bank[b]; r.b = b; q.push(r);
    }
    const sel = select(q, enforceZvs);
    if (!sel) return null;
    const r = q[sel.i];
    return Object.assign(finalOutcome(pad, M, r.C1, r.f, sel.s), { b: r.b, full: sel.full });
  }
  function finalOutcome(pad, M, C1, f, s) {
    const r = solve(truthP(pad), C1, f, M, D.pads[pad].RL, s);
    return { C1, f, s, P: r.PL, eta: r.eta, I1: r.I1, VC1: r.VC1, phi: r.phi, I2: r.I2, res: r };
  }
  function fixedTuning(pad, M, enforceZvs) {
    const d = D.pads[pad].design, f = D.limits.F0, r = solve(truthP(pad), d.C1_nom, f, M, D.pads[pad].RL);
    r.C1 = d.C1_nom; r.f = f;
    const sel = select([r], enforceZvs);
    if (!sel) return { C1: d.C1_nom, f, s: 0, P: 0, eta: NaN, I1: 0, VC1: 0, phi: r.phi, ok: false };
    return Object.assign(finalOutcome(pad, M, d.C1_nom, f, sel.s), { ok: true });
  }

  // ---------------------------------------------------------------- measurement (plant.probe + cf_search.derive)
  function probe(pad, M, C1, f, amp, R) {
    const L = D.limits, p = truthP(pad), r = solve(p, C1, f, M, D.pads[pad].RL, amp);
    const vr = p.V1 * amp * (1 + L.V_NOISE * R.n());
    const g = 1 + L.I_NOISE * R.n(), th = L.PH_NOISE * Math.PI / 180 * R.n();
    const ir = g * (r.i1r * Math.cos(th) - r.i1i * Math.sin(th)), ii = g * (r.i1r * Math.sin(th) + r.i1i * Math.cos(th));
    const Pin = vr * ir, PL = r.PL * (1 + L.TEL_NOISE * R.n());
    // derive(): scale to full duty
    const I = Math.hypot(ir, ii) / amp;
    return { C1, f, I1: I, Pin: Pin / (amp * amp), PL: PL / (amp * amp), eta: PL / Math.max(Pin, 1e-9),
             phi: Math.atan2(-ii, ir) * 180 / Math.PI, VC1: I / (TWO_PI * f * C1), V: vr, ir, ii };
  }

  // ---------------------------------------------------------------- step 2: |M| from transmitter current
  // Three low-power probes; Zr = Zin - Z1(cal); least squares for m = (M^2) with R_L held at the
  // identified commissioning load (mapping.estimate_coupling, conditioned branch).
  function estimateM(pad, M, R) {
    const P = D.pads[pad], m = modelP(pad), bank = P.design.C1_bank, C1 = bank[bank.length >> 1];
    let num = 0, den = 0;
    for (const f of D.limits.ID_FREQS) {
      const q = probe(pad, M, C1, f, D.limits.PROBE_AMP, R);
      const dz = q.ir * q.ir + q.ii * q.ii, zr = q.V * q.ir / dz, zi = -q.V * q.ii / dz;     // Zin = V/I
      const w = TWO_PI * f, sq = Math.sqrt(f / D.limits.F0);
      const Zrr = zr - (m.R1_0 * sq + m.r_inv), Zri = zi - (w * m.L1 - 1 / (w * C1));
      const z2r = m.R2_0 * sq + P.RL_hat, z2i = w * m.L2 - 1 / (w * m.C2), d2 = z2r * z2r + z2i * z2i;
      const ar = w * w * z2r / d2, ai = -w * w * z2i / d2;                                   // w^2 / Z2
      num += ar * Zrr + ai * Zri; den += ar * ar + ai * ai;
    }
    return Math.sqrt(Math.max(num / den, 0));
  }

  // ---------------------------------------------------------------- model prediction with the mapped |M|
  function modelGrid(pad, Mest, bankSubset) {
    const P = D.pads[pad], m = modelP(pad), bank = P.design.C1_bank, q = [];
    const bs = bankSubset || bank.map((_, i) => i);
    for (const f of fGrid(pad)) for (const b of bs) {
      const r = solve(m, bank[b], f, Mest, P.RL_hat); r.f = f; r.C1 = bank[b]; r.b = b; q.push(r);
    }
    return q;
  }
  function bankScore(sub) {
    const sel = select(sub);
    if (!sel) return -Infinity;
    const r = sub[sel.i];
    return Math.min(r.PL * sel.s * sel.s, D.limits.P_RATED) + 50 * r.eta + (sel.full ? 200 : 0);
  }
  function shortlist(pad, Mest, K) {
    const nb = D.pads[pad].design.C1_bank.length, sc = [];
    for (let b = 0; b < nb; b++) sc.push([bankScore(modelGrid(pad, Mest, [b])), b]);
    sc.sort((a, b) => b[0] - a[0]);
    return sc.slice(0, K).map(s => s[1]);
  }
  // predicted outcome at a pose (used to rank transmitter positions)
  function predict(pad, Mest) {
    const q = modelGrid(pad, Mest), sel = select(q);
    if (!sel) return { score: -Infinity, P: 0, eta: 0 };
    const r = q[sel.i];
    return { score: bankScore(q), P: Math.min(r.PL * sel.s * sel.s, D.limits.P_RATED), eta: r.eta, C1: r.C1, f: r.f };
  }

  // ---------------------------------------------------------------- coupled-mode resonances (resonance.py)
  function modeRoots(L1, L2, C1, C2, M) {
    const u0 = 1 / Math.sqrt(L1 * C1 * L2 * C2), r = Math.sqrt(L1 * C1 / (L2 * C2));
    const k2 = M * M / (L1 * L2), a = 1 - k2, b = -(r + 1 / r), c = 1;
    const disc = Math.max(b * b - 4 * a * c, 0);
    if (a < 1e-12) return [Math.sqrt(1 / (L1 * C1 + L2 * C2)) / TWO_PI];
    const q = -(b + Math.sign(b) * Math.sqrt(disc)) / 2;
    const xs = [q / a, c / q].filter(x => x > 0).sort((p, s) => p - s);
    return xs.map(x => Math.sqrt(x * u0) / TWO_PI);
  }

  // ---------------------------------------------------------------- steps 3-4: mode-centred search (cf_search.mode_search)
  function windowF(pad, anchor) {
    const L = D.limits, d = D.pads[pad].design, set = new Set();
    for (let f = anchor - L.W; f <= anchor + L.W + L.DF / 2; f += L.DF) {
      const g = Math.min(Math.max(Math.round(f / d.f_step) * d.f_step, d.f_lo), d.f_hi);
      set.add(Math.round(g));
    }
    return [...set].sort((a, b) => a - b);
  }
  function modeSearch(pad, M, Mest, R) {
    const P = D.pads[pad], L = D.limits, d = P.design, m = modelP(pad);
    const windows = [], roots = [], order = [], tags = new Map();
    for (const b of shortlist(pad, Mest, L.K)) {
      const C1 = d.C1_bank[b], rts = modeRoots(m.L1, m.L2, C1, m.C2, Mest);
      roots.push({ b, C1, f: rts });
      rts.forEach((fr, j) => {
        const label = rts.length === 2 && j === 0 ? 'mode-lower' : 'mode-upper';
        if (fr < d.f_lo - L.FAR || fr > d.f_hi + L.FAR) return;
        const anchor = Math.min(Math.max(fr, d.f_lo), d.f_hi), fw = windowF(pad, anchor);
        windows.push({ b, C1, root: fr, anchor, f: fw, label });
        for (const f of fw) { const key = b + ':' + f; if (!tags.has(key)) { tags.set(key, label); order.push([b, f]); } }
      });
    }
    const probes = order.map(([b, f]) => Object.assign(probe(pad, M, d.C1_bank[b], f, L.PROBE_AMP, R), { b, tag: tags.get(b + ':' + f) }));
    let sel = probes.length ? select(probes) : null, source = 'none';
    if (sel && sel.full) source = probes[sel.i].tag;
    if (source === 'none') {                                   // fallback around the receiver resonance
      const f2 = 1 / (TWO_PI * Math.sqrt(m.L2 * m.C2)), fw = windowF(pad, f2), b = shortlist(pad, Mest, 1)[0];
      windows.push({ b, C1: d.C1_bank[b], root: f2, anchor: f2, f: fw, label: 'fallback' });
      for (const f of fw) probes.push(Object.assign(probe(pad, M, d.C1_bank[b], f, L.PROBE_AMP, R), { b, tag: 'fallback' }));
      sel = select(probes);
      if (sel) source = sel.full ? probes[sel.i].tag : 'fallback';
    }
    if (!sel) return { source: 'none', probes, roots, windows, nProbes: probes.length + 3 };
    const ch = probes[sel.i];
    const v = verify(pad, M, ch.C1, ch.f, sel.s, R);
    return Object.assign(finalOutcome(pad, M, ch.C1, ch.f, v.s), { b: ch.b, source, probes, roots, windows, chosen: sel.i,
      nProbes: probes.length + 3 + v.n, predP: Math.min(ch.PL * sel.s * sel.s, L.P_RATED * L.P_OVER) });
  }
  function fullSweep(pad, M, R) {
    const d = D.pads[pad].design, q = [];
    for (const f of fGrid(pad)) for (let b = 0; b < d.C1_bank.length; b++)
      q.push(Object.assign(probe(pad, M, d.C1_bank[b], f, D.limits.PROBE_AMP, R), { b }));
    const sel = select(q);
    if (!sel) return { source: 'none', nProbes: q.length, P: 0 };
    const ch = q[sel.i], v = verify(pad, M, ch.C1, ch.f, sel.s, R);
    return Object.assign(finalOutcome(pad, M, ch.C1, ch.f, v.s), { b: ch.b, source: 'grid', nProbes: q.length + v.n });
  }
  function verify(pad, M, C1, f, s, R) {                     // apply at full power, back off if a limit trips
    const L = D.limits, g = L.GUARD;
    let n = 0;
    for (let k = 0; k <= 15; k++) {
      const q = probe(pad, M, C1, f, s, R); n++;
      const I1 = q.I1 * s, PL = q.PL * s * s, VC1 = I1 / (TWO_PI * f * C1);
      if (I1 <= g * L.I1_MAX && VC1 <= g * L.VC1_MAX && PL <= L.P_OVER * L.P_RATED) break;
      s *= 0.97;
    }
    return { s, n };
  }

  const api = { init, rng, interpM, kOf, solve, truthP, modelP, fGrid, select, trueOptimum, fixedTuning, probe,
                estimateM, predict, shortlist, modeRoots, modeSearch, fullSweep, finalOutcome, get data() { return D; } };
  root.WPT = api;
  if (typeof module !== 'undefined') module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
