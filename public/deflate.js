// public/deflate.js
const fs = require('fs');

class BitWriter {
  constructor() { this.buf = []; this.acc = 0; this.n = 0; }
  write(val, len) {
    this.acc |= (val & ((1 << len) - 1)) << this.n;
    this.n += len;
    while (this.n >= 8) {
      this.buf.push(this.acc & 0xFF);
      this.acc >>= 8;
      this.n -= 8;
    }
  }
  flush() { if (this.n) this.buf.push(this.acc & 0xFF); return Buffer.from(this.buf); }
}

function lz77(data) {
  const tokens = [];
  let pos = 0;
  while (pos < data.length) {
    let bestLen = 0, bestDist = 0;
    const limit = Math.min(pos, 32768);
    for (let d = 1; d <= limit; d++) {
      let l = 0;
      while (pos + l < data.length && l < 258 && data[pos + l] === data[pos - d + l]) l++;
      if (l > bestLen) { bestLen = l; bestDist = d; }
    }
    if (bestLen >= 3) {
      tokens.push({ t: 'm', l: bestLen, d: bestDist });
      pos += bestLen;
    } else {
      tokens.push({ t: 'l', v: data[pos] });
      pos++;
    }
  }
  return tokens;
}

// Shared static code writer (LSB-first)
function writeLitLen(bw, code) {
  let msb, len;
  if (code <= 143)      { msb = 0x30 + code;           len = 8; }
  else if (code <= 255) { msb = 0x190 + (code - 144);  len = 9; }
  else if (code <= 279) { msb = 0x00 + (code - 256);   len = 7; }
  else                  { msb = 0xC0 + (code - 280);   len = 8; }
  let lsb = 0;
  for (let i = 0; i < len; i++) lsb |= ((msb >> i) & 1) << (len - 1 - i);
  bw.write(lsb, len);
}

function encode(tokens) {
  const bw = new BitWriter();
  bw.write(1, 1); // BFINAL
  bw.write(1, 2); // BTYPE=01

  const LEN_BASE = [3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,35,43,51,59,67,83,99,115,131,163,195,227,258];
  const LEN_EXT  = [0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0];
  const DIST_BASE= [1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,257,385,513,769,1025,1537,2049,3073,4097,6145,8193,12289,16385,24577];
  const DIST_EXT = [0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13];

  for (const t of tokens) {
    if (t.t === 'l') {
      writeLitLen(bw, t.v);
    } else {
      let lc = 257;
      while (lc < 285 && LEN_BASE[lc - 257] <= t.l) lc++;
      lc--;
      writeLitLen(bw, lc);
      if (LEN_EXT[lc - 257]) bw.write(t.l - LEN_BASE[lc - 257], LEN_EXT[lc - 257]);
      
      let dc = 0;
      while (dc < 29 && DIST_BASE[dc + 1] <= t.d) dc++;
      bw.write(dc, 5);
      if (DIST_EXT[dc]) bw.write(t.d - DIST_BASE[dc], DIST_EXT[dc]);
    }
  }
  writeLitLen(bw, 256); // EOB
  return bw.flush();
}

const inFile = process.argv[2], outFile = process.argv[3];
if (!inFile || !outFile) { console.error('Usage: node deflate.js in out'); process.exit(1); }
const tokens = lz77(fs.readFileSync(inFile));
const halfwayLines = tokens.map((t, i) => {
  if (t.t === 'l') {
    const ch = t.v >= 32 && t.v < 127 ? String.fromCharCode(t.v) : '?';
    return `${i}\tLIT\t${t.v}\t'${ch}'`;
  } else {
    return `${i}\tMATCH\tlen=${t.l}\tdist=${t.d}`;
  }
});
fs.writeFileSync('test.halfway', halfwayLines.join('\n') + '\n');
console.log(`📄 ${tokens.length} tokens → test.halfway`);
const out = encode(tokens);
fs.writeFileSync(outFile, out);
console.log(`✅ ${fs.statSync(inFile).size} → ${out.length} bytes`);