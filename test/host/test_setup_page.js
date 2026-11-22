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
  addEventListener() {}
  querySelector(tag) { return this.children.find(c => c instanceof El && c.tag === tag) || new El(tag); }
  set innerHTML(v) {}
}
class Option { constructor(text, value) { this.text = text; this.value = value; } }

function makeEnv(sim, lossRate) {
  const els = {}, created = [];
  const document = {
    getElementById: id => els[id] || (els[id] = new El(id)),
    createElement: t => { const e = new El(t); created.push(e); return e; },
    querySelectorAll: sel => created.filter(e => e.dataset && e.dataset.name),
    querySelector: sel => { const m = /data-name="([^"]+)"/.exec(sel); return m ? created.find(e => e.dataset && e.dataset.name === m[1]) || null : null; },
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
  vm.runInContext(script + "\n;globalThis.__api = { get settings() { return settings; }, readSettings, save, wantedPairs, dualLabel, symbolPreview, get advDetails() { return advDetails; } };", ctx);
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

  // the "also transmit on the other SF" setting is named after the SF it adds, and is off for other main SFs
  const dualName = () => field("dual_sf").parent.children[0].textContent;
  api.dualLabel();
  CHECK(field("sf").value === "12" && dualName() === "Also transmit on SF7" && field("dual_sf").disabled === false, label + ": dual_sf named SF7 when the main SF is 12 (" + dualName() + ")");
  field("sf").value = "7";  api.dualLabel(); CHECK(dualName() === "Also transmit on SF12" && !field("dual_sf").disabled, label + ": dual_sf named SF12 when the main SF is 7");
  field("sf").value = "9";  api.dualLabel(); CHECK(dualName() === "Also transmit on the other SF" && field("dual_sf").disabled, label + ": dual_sf off for SF9");
  field("sf").value = "12"; api.dualLabel();

  // basic settings first; the rest sits behind the closed "Advanced settings" fold
  const inAdv = n => { for (let e = field(n); e; e = e.parent) if (e === api.advDetails) return true; return false; };
  CHECK(!!api.advDetails && api.advDetails.className === "adv" && api.advDetails.open === false, label + ": the Advanced fold exists and starts closed");
  const basic = ["call", "ssid", "comment", "sym_table", "sym_code", "aprs", "beacon_s", "dual_sf", "horus", "horus_call", "horus_note", "horus_freq"];
  const advanced = ["path", "comment2", "comment2_every", "gps_timeout", "freq", "bw", "sf", "cr", "power", "sync", "preamble", "aprs_desc", "aprs_gnd",
                    "horus_asc", "horus_desc", "horus_gnd", "horus_ver", "horus_id", "horus_power", "horus_sync", "horus_off"];
  CHECK(basic.every(n => field(n) && !inAdv(n)), label + ": the basic settings are outside the fold (" + basic.filter(n => !field(n) || inAdv(n)).join(",") + ")");
  CHECK(advanced.every(n => field(n) && inAdv(n)), label + ": the other settings are inside the fold (" + advanced.filter(n => !field(n) || !inAdv(n)).join(",") + ")");
  CHECK(!field("tocall") && !field("horus_div"), label + ": tocall and the unused horus_div are not on the page");

  // the APRS symbol preview: which cell of which sheet the table and code make (cell = ord(c) - 33, 16 per row, 48 px)
  const box = created.find(e => e.className === "sym");
  CHECK(!!box, label + ": symbol preview built");
  const setSym = (t, c) => { field("sym_table").value = t; field("sym_code").value = c; api.symbolPreview(); };
  setSym("/", "O"); CHECK(box.style.backgroundImage.includes("24-0@2x") && box.style.backgroundPosition === "-672px -96px", label + ": / O is the balloon cell (" + box.style.backgroundPosition + ")");
  setSym("/", "0"); CHECK(box.style.backgroundImage.includes("24-0@2x") && box.style.backgroundPosition === "-720px -0px", label + ": / 0 is a different cell from / O (" + box.style.backgroundPosition + ")");
  setSym("\\", "#"); CHECK(box.style.backgroundImage.includes("24-1@2x") && !box.style.backgroundImage.includes("24-2@2x") && box.style.backgroundPosition === "-96px -0px", label + ": the alternate table uses sheet 1");
  setSym("L", "#"); CHECK(box.style.backgroundImage.startsWith('url("aprs-symbols/aprs-symbols-24-2@2x.png")') && box.style.backgroundImage.includes("24-1@2x") && box.style.backgroundPosition === "-528px -96px, -96px -0px", label + ": an overlay letter is drawn on top (" + box.style.backgroundPosition + ")");
  setSym("/", ""); CHECK(box.style.backgroundImage === "none", label + ": no code, no picture");
  setSym("%", "O"); CHECK(box.style.backgroundImage === "none", label + ": a table that is not / or \\ or an overlay gives no picture");
  setSym("/", "O");

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
  CHECK(api.advDetails.open === true, label + ": the Advanced fold opens when a value inside it is refused");
  await els.reread.onclick();
  CHECK(api.settings.sf === "12", label + ": the refused value did not change the tracker");
  CHECK(api.advDetails.open === true, label + ": reading again keeps the fold as the user left it");

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
