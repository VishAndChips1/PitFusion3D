// <tier-controller>
// Adaptive quality: ask the server for fewer points when downloads are slow, and creep back up when the link is fast.
const TierController = {
  levels: [8000, 20000, 40000, 60000, 80000, 120000],   // the server clamps to its own ceiling
  index: 5, goodRun: 0, blockedUntil: 0,
  requested() { return this.levels[this.index]; },
  // Report one finished request (download time in ms, or aborted); returns the level index to use next.
  report(downloadMs, aborted, nowMs) {
    if (aborted) {
      this.index = Math.max(0, this.index - 2); this.goodRun = 0; this.blockedUntil = nowMs + 60000;
    } else if (downloadMs > 900) {
      if (this.index > 0) { this.index -= 1; this.blockedUntil = nowMs + 60000; }
      this.goodRun = 0;
    } else if (downloadMs < 400) {
      this.goodRun += 1;
      if (this.goodRun >= 8 && this.index < this.levels.length - 1 && nowMs >= this.blockedUntil) { this.index += 1; this.goodRun = 0; }
    } else {
      this.goodRun = 0;
    }
    return this.index;
  },
};

let pass = 0, fail = 0;
function check(name, cond) { if (cond) pass++; else { fail++; print('FAIL: ' + name); } }
function fresh() { const c = Object.assign({}, TierController); c.index = 5; c.goodRun = 0; c.blockedUntil = 0; return c; }
let t;
// 1 fast link stays at the top
t = fresh(); for (let i = 0; i < 50; i++) t.report(200, false, i * 700); check('fast link stays top', t.index === 5);
// 2 slow link walks down to the floor
t = fresh(); for (let i = 0; i < 10; i++) t.report(1500, false, i * 700); check('slow link reaches floor', t.index === 0);
// 3 an abort drops two levels at once
t = fresh(); t.report(null, true, 0); check('abort drops two', t.index === 3); t.report(null, true, 1); check('second abort', t.index === 1); t.report(null, true, 2); check('floor holds', t.index === 0);
// 4 after a drop, climbing is blocked for 60 s, then needs 8 good updates
t = fresh(); t.report(1500, false, 0); check('dropped one', t.index === 4);
for (let i = 1; i <= 20; i++) t.report(100, false, i * 1000); check('blocked within 60 s', t.index === 4);
for (let i = 61; i <= 70; i++) t.report(100, false, i * 1000); check('climbs after 60 s + 8 good', t.index === 5);
// 5 middle band (400-900 ms) neither climbs nor drops, and resets the good run
t = fresh(); t.index = 2; for (let i = 0; i < 7; i++) t.report(100, false, 100000 + i); t.report(600, false, 100010); for (let i = 0; i < 7; i++) t.report(100, false, 100020 + i);
check('middle band resets run', t.index === 2); t.report(100, false, 100100); check('8th good in a row climbs', t.index === 3);
// 6 requested() follows the index
t = fresh(); t.index = 2; check('requested level', t.requested() === 40000);
print('controller tests: ' + pass + ' passed, ' + fail + ' failed');

// Decode cost of the page's own loop at several sizes (SpiderMonkey on this Pi: a pessimistic stand-in for a laptop/phone)
function decodeMs(n) {
  const buf = new ArrayBuffer(24 + n * 11); const dv = new DataView(buf);
  dv.setUint32(0, n, true); dv.setFloat32(8, 1, true); dv.setFloat32(12, 2, true); dv.setFloat32(16, 0, true); dv.setFloat32(20, 0.001, true);
  for (let i = 0; i < n; i++) { const p = 24 + i * 11; dv.setInt16(p, i % 30000, true); dv.setUint8(p + 6, i & 255); dv.setInt16(p + 9, -32768, true); }
  const t0 = Date.now(); const ox = dv.getFloat32(8, true), oy = dv.getFloat32(12, true), oz = dv.getFloat32(16, true), scale = dv.getFloat32(20, true);
          
          const xyz = new Float32Array(n * 3);
          const rgb = new Float32Array(n * 3);
          const thermal = new Float32Array(n);
          let p = 24;
          for (let i = 0; i < n; i++) {
            xyz[i*3]   = ox + dv.getInt16(p, true) * scale;
            xyz[i*3+1] = oy + dv.getInt16(p + 2, true) * scale;
            xyz[i*3+2] = oz + dv.getInt16(p + 4, true) * scale;
            rgb[i*3]   = dv.getUint8(p + 6) / 255;
            rgb[i*3+1] = dv.getUint8(p + 7) / 255;
            rgb[i*3+2] = dv.getUint8(p + 8) / 255;
            const tq = dv.getInt16(p + 9, true);
            thermal[i] = (tq === -32768) ? NaN : tq / 100;
            p += 11;
          }

  return Date.now() - t0;
}
for (const n of [20000, 40000, 80000, 120000]) { decodeMs(n); print('decode ' + n + ' points: ' + decodeMs(n) + ' ms'); }
