// Runs the script of tools/setup.html in a tiny fake browser against a simulated tracker (tks_sim, the real settings engine).
//   node test_setup_page.js path/to/tks_sim
// Checks: connect + read, the form, save with read-back, refused values, the radio-apply flow, the actions, and that a read
// or save still works when characters are lost from the replies (the page reads twice and compares).
const fs = require("fs"), vm = require("vm"), path = require("path"), { spawn } = require("child_process");
const simPath = process.argv[2];
const html = fs.readFileSync(path.join(__dirname, "../../tools/setup.html"), "utf8");
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

let fails = 0, checks = 0;
const CHECK = (c, what) => { checks++; if (!c) { fails++; console.log("FAIL:", what); } };
const sleep = ms => new Promise(r => setTimeout(r, ms));

class El {
  constructor(tag) { this.tag = tag; this.children = []; this.dataset = {}; this.style = {}; this.value = ""; this.hidden = false; this.textContent = "";
                     this.disabled = false; this.options = []; this.checked = false; this.className = ""; this.open = false; this.parent = null; }
  append(...c) { c.forEach(x => { this.children.push(x); if (x instanceof El) x.parent = this; }); }
  add(o) { this.options.push(o); if (this.options.length === 1 && !this.value) this.value = o.value; }
  closest() { return this.parent || new El("label"); }
  set innerHTML(v) {}
}
class Option { constructor(text, value) { this.text = text; this.value = value; } }

function makeEnv(sim, lossRate) {
  const els = {}, created = [];
  const document = {
    getElementById: id => els[id] || (els[id] = new El(id)),
    createElement: t => { const e = new El(t); created.push(e); return e; },
    querySelectorAll: sel => created.filter(e => e.dataset && e.dataset.name),
  };
  // the fake serial port: what the page writes goes to the simulated tracker; its output comes back, with characters lost at lossRate
  let queue = [], wake = null, closed = false;
  sim.stdout.on("data", d => {
    let b = Buffer.from(d);
    if (lossRate) { const keep = []; for (const x of b) { if (Math.random() < lossRate && x >= 0x20) continue; keep.push(x); } b = Buffer.from(keep); }
    queue.push(new Uint8Array(b)); if (wake) wake();
  });
  const port = {
    open: async () => {}, close: async () => {},
    readable: { getReader: () => ({
      read: async () => { while (!queue.length) { if (closed) return { done: true }; await new Promise(r => { wake = r; setTimeout(r, 50); }); } return { value: queue.shift(), done: false }; },
      cancel: async () => { closed = true; }, releaseLock: () => {} }) },
    writable: { getWriter: () => ({ write: async u => { sim.stdin.write(Buffer.from(u)); }, releaseLock: () => {} }) },
  };
  const navigator = { serial: { requestPort: async () => port } };
  const ctx = { document, navigator, Option, TextDecoder, TextEncoder, setTimeout, clearTimeout, console, Promise, Uint8Array, crypto: {}, fetch: undefined, JSON, Number, Object, Math, Date };
  vm.createContext(ctx);
  vm.runInContext(script + "\n;globalThis.__api = { get settings() { return settings; }, readSettings, save, wantedPairs };", ctx);
  return { els, created, document, ctx, api: ctx.__api };
}

async function session(lossRate, label, call) {
  const sim = spawn(simPath, call ? [call] : []);
  const env = makeEnv(sim, lossRate);
  const { els, created, api } = env;
  await els.connect.onclick();
  await sleep(100);
  CHECK(Object.keys(api.settings).length >= 26, label + ": settings read (" + Object.keys(api.settings).length + ")");
  CHECK(api.settings.call === "N0CALL" || !!call, label + ": callsign read");
  const field = n => created.find(e => e.dataset.name === n);
  CHECK(!!field("call") && !!field("horus_call") && !!field("bw") && !!field("cr") && !!field("horus_ver"), label + ": form built");
  CHECK(field("bw").options.length >= 10 && field("horus_ver").options.length === 2, label + ": selects filled");
  CHECK(!els.n0call.hidden === !call, label + ": N0CALL warning " + (call ? "hidden" : "shown"));

  // change a few settings (one of them a radio setting) and save
  field("call").value = "w2hab"; field("ssid").value = "11"; field("beacon_s").value = "60";
  field("horus_call").value = "K6ATV/Test"; field("freq").value = "439.9125"; field("cr").value = "4/6";
  field("horus_sync").value = "10"; field("horus_off").value = "5"; field("horus").value = "on";
  const pairs = api.wantedPairs();
  CHECK(pairs.length >= 7, label + ": changed fields are sent (" + pairs.length + ")");
  await els.save.onclick();
  CHECK(/Saved to the tracker and checked/.test(els.status.textContent), label + ": saved and read back (" + els.status.textContent + ")");
  CHECK(api.settings.call === "W2HAB" && api.settings.ssid === "11" && api.settings.freq.startsWith("439.9125") && api.settings.cr === "4/6" && api.settings.horus_call === "K6ATV/Test", label + ": values on the tracker");
  CHECK(els.formerr.hidden === true, label + ": no error shown");

  // a refused value: nothing is saved, the error says why
  field("sf").value = "13";
  await els.save.onclick();
  CHECK(els.formerr.hidden === false && /sf/.test(els.formerr.textContent), label + ": refused value is reported (" + els.formerr.textContent + ")");   // "out of range", or the read-back mismatch when the reply lost a character
  CHECK(!/Saved to the tracker/.test(els.status.textContent), label + ": a refused value is not reported as saved");
  await els.reread.onclick();
  CHECK(api.settings.sf === "12", label + ": the refused value did not change the tracker");

  // actions
  await els.actbeacon.onclick(); CHECK(/beacon cycle starts/.test(els.status.textContent), label + ": beacon action");
  await els.acthorus.onclick();  CHECK(/Horus packet follows/.test(els.status.textContent), label + ": horus action");
  await els.actrun.onclick();    CHECK(/running/.test(els.status.textContent), label + ": exit action");
  sim.stdin.end(); sim.kill();
}

(async () => {
  await session(0, "clean line", null);
  await session(0, "clean line, call set", "K6ATV");
  for (let i = 1; i <= 4; i++) await session(0.0005, "0.05% of characters lost (run " + i + ")", null);
  console.log(checks + " checks, " + fails + " failures");
  process.exit(fails ? 1 : 0);
})().catch(e => { console.log("test crashed:", e); process.exit(2); });
