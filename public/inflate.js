// public/inflate.js
const fs = require('fs');
class BitReader {
  constructor(buf) { this.buf = buf; this.pos = 0; this.bit = 0; }
  read(n) {
    let val = 0;
    for (let i = 0; i < n; i++) {
      if (this.pos >= this.buf.length) throw new Error("Unexpected EOF");
      val |= ((this.buf[this.pos] >> this.bit) & 1) << i;
      this.bit++;
      if (this.bit === 8) { this.bit = 0; this.pos++; }
    }
    return val;
  }
  peek(n) {
    const savedPos = this.pos, savedBit = this.bit;
    const val = this.read(n);
    this.pos = savedPos; this.bit = savedBit;
    return val;
  }
  skip(n) { this.read(n); }
  align() { if (this.bit) { this.bit = 0; this.pos++; } }
}
const STATIC_TBL = new Array(512).fill(null);
function addCode(msb, len, sym) {
  let lsb = 0;
  for (let i = 0; i < len; i++) lsb |= ((msb >> i) & 1) << (len - 1 - i);
  const pad = 9 - len;
  for (let i = 0; i < (1 << pad); i++) STATIC_TBL[lsb | (i << len)] = { sym, len };
}
for (let i = 0; i <= 143; i++) addCode(0x30 + i, 8, i);
for (let i = 144; i <= 255; i++) addCode(0x190 + (i - 144), 9, i);
for (let i = 256; i <= 279; i++) addCode(0x00 + (i - 256), 7, i);
for (let i = 280; i <= 287; i++) addCode(0xC0 + (i - 280), 8, i);
const LEN_BASE = [3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,35,43,51,59,67,83,99,115,131,163,195,227,258];
const LEN_EXT  = [0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0];
const DIST_BASE= [1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,257,385,513,769,1025,1537,2049,3073,4097,6145,8193,12289,16385,24577];
const DIST_EXT = [0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13];
function decode(data) {
  const br = new BitReader(data);
  const out = [];
  while (br.pos < data.length) {
    const bfinal = br.read(1);
    const btype = br.read(2);
    if (btype === 0) {
      br.align();
      const len = br.read(16);
      const nlen = br.read(16);
      if ((len ^ 0xFFFF) !== nlen) throw new Error('Stored block checksum failed');
      for (let i = 0; i < len; i++) out.push(data[br.pos + i]);
      br.pos += len; br.bit = 0;
    } else if (btype === 1) {
      while (true) {
        const entry = STATIC_TBL[br.peek(9)];
        if (!entry) throw new Error(`Invalid static code at ${br.pos}:${br.bit}`);
        br.skip(entry.len);
        const sym = entry.sym;
        if (sym === 256) break;
        if (sym < 256) {
          console.log(`lit=${sym} (${String.fromCharCode(sym)}) out.length=${out.length}`);
          out.push(sym);
          continue;
        }
        const lidx = sym - 257;
        const matchLen = LEN_BASE[lidx] + (LEN_EXT[lidx] ? br.read(LEN_EXT[lidx]) : 0);
        const dcode = br.read(5);
        const matchDist = DIST_BASE[dcode] + (DIST_EXT[dcode] ? br.read(DIST_EXT[dcode]) : 0);
        console.log(`sym=${sym} lidx=${lidx} matchLen=${matchLen} dcode=${dcode} matchDist=${matchDist} out.length=${out.length}`);
        if (matchDist > out.length) throw new Error(`Invalid back-reference: dist=${matchDist} > out.length=${out.length}`);
        for (let i = 0; i < matchLen; i++) out.push(out[out.length - matchDist]);
      }
    } else {
      throw new Error(`Unsupported BTYPE=${btype}`);
    }
    if (bfinal) break;
  }
  return Buffer.from(out);
}
const inFile = process.argv[2], outFile = process.argv[3];
if (!inFile || !outFile) { console.error('Usage: node inflate.js in out'); process.exit(1); }
try {
  const data = fs.readFileSync(inFile);
  const decoded = decode(data);
  fs.writeFileSync(outFile, decoded);
  console.log(`✅ ${data.length} → ${decoded.length} bytes`);
  const origPath = inFile.replace('.deflate', '.txt');
  try {
    const orig = fs.readFileSync(origPath);
    console.log(decoded.equals(orig) ? '🔍 PERFECT MATCH' : '⚠️ MISMATCH');
  } catch(e) {}
} catch (e) { console.error('❌', e.message); process.exit(1); }