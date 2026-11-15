const fs = imports.gi.GLib;
function load(path) { const [ok, data] = fs.file_get_contents(path); return data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength); }
function decode(buf) {
  const dv = new DataView(buf); const n = dv.getUint32(0, true);
          const ox = dv.getFloat32(8, true);
          const oy = dv.getFloat32(12, true);
          const oz = dv.getFloat32(16, true);
          const q = dv.getFloat32(20, true);
          const flags = dv.getUint32(24, true);
          const decodeStart = Date.now();
          let off = 28;
          const dx = new Int16Array(buf, off, n); off += 2 * n;
          const dy = new Int16Array(buf, off, n); off += 2 * n;
          const dz = new Int16Array(buf, off, n); off += 2 * n;
          let tq = null;
          if (flags & 1) { tq = new Int16Array(buf, off, n); off += 2 * n; }
          const dr = new Uint8Array(buf, off, n); off += n;
          const dg = new Uint8Array(buf, off, n); off += n;
          const db = new Uint8Array(buf, off, n);
          const xyz = new Float32Array(n * 3);
          const rgb = new Float32Array(n * 3);
          const thermal = new Float32Array(n);
          let ix = 0, iy = 0, iz = 0, r = 0, g = 0, b = 0;
          for (let i = 0; i < n; i++) {
            ix += dx[i]; iy += dy[i]; iz += dz[i];
            xyz[i*3] = ox + ix * q; xyz[i*3+1] = oy + iy * q; xyz[i*3+2] = oz + iz * q;
            r = (r + dr[i]) & 255; g = (g + dg[i]) & 255; b = (b + db[i]) & 255;
            rgb[i*3] = r / 255; rgb[i*3+1] = g / 255; rgb[i*3+2] = b / 255;
            thermal[i] = (tq === null || tq[i] === -32768) ? NaN : tq[i] / 100;
          }

  let sx = 0, sy = 0, sz = 0, sr = 0, sg = 0, sb = 0, st = 0;
  for (let i = 0; i < n; i++) { sx += xyz[i*3]; sy += xyz[i*3+1]; sz += xyz[i*3+2]; sr += rgb[i*3]*255; sg += rgb[i*3+1]*255; sb += rgb[i*3+2]*255; if (!isNaN(thermal[i])) st += thermal[i]; }
  return {n, xyz_sum: [sx, sy, sz], rgb_sum: [Math.round(sr), Math.round(sg), Math.round(sb)], x0: [xyz[0], xyz[1], xyz[2]], xl: [xyz[n*3-3], xyz[n*3-2], xyz[n*3-1]], rgb0: [Math.round(rgb[0]*255), Math.round(rgb[1]*255), Math.round(rgb[2]*255)], rgbl: [Math.round(rgb[n*3-3]*255), Math.round(rgb[n*3-2]*255), Math.round(rgb[n*3-1]*255)], thermal_sum: st, ms: Date.now() - decodeStart};
}
const res = {}; for (const k of ['plain']) res[k] = decode(load('v2_' + k + '.bin'));
print(JSON.stringify(res));
