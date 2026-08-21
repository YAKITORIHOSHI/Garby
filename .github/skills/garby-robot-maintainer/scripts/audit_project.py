#!/usr/bin/env python3
"""Static coordinated audit for the active GARBY Pi -> BLE bridge -> MCU release.

This is a source-level regression check. It is not an Arduino compiler and it
cannot prove electrical or physical safety.
"""
from __future__ import annotations
import argparse, json, re, sys
from dataclasses import dataclass, asdict
from pathlib import Path

@dataclass
class Result:
    severity: str
    check: str
    detail: str
    file: str = ""

class Audit:
    def __init__(self, root: Path):
        self.root = root.resolve(); self.results=[]
    def add(self, sev, check, detail, path=None):
        rel=""
        if path:
            try: rel=str(path.resolve().relative_to(self.root))
            except ValueError: rel=str(path)
        self.results.append(Result(sev,check,detail,rel))
    def ok(self,c,d,p=None): self.add("PASS",c,d,p)
    def warn(self,c,d,p=None): self.add("WARN",c,d,p)
    def fail(self,c,d,p=None): self.add("FAIL",c,d,p)

def text(p): return p.read_text(encoding="utf-8", errors="replace") if p and p.exists() else ""
def number(src,name):
    for pat in (rf"#define\s+{re.escape(name)}\s+([0-9]+(?:\.[0-9]+)?)", rf"\b{re.escape(name)}\s*=\s*([0-9]+(?:\.[0-9]+)?)"):
        m=re.search(pat,src)
        if m: return float(m.group(1))
    return None

def exact_or_find(root, relative, patterns):
    p=root/relative
    if p.is_file(): return p
    candidates=[]
    for pattern in patterns: candidates.extend(root.rglob(pattern))
    candidates=[c for c in candidates if c.is_file() and "__pycache__" not in c.parts]
    return sorted(candidates, key=lambda x:("simulator" in x.name.lower(),len(x.parts),str(x)))[0] if candidates else None

def required(a):
    files={
      "arch": exact_or_find(a.root,"SYSTEM_ARCHITECTURE.md",["SYSTEM_ARCHITECTURE*.md"]),
      "pi": exact_or_find(a.root,"RasPi/final_w_serial.py",["final_w_serial.py"]),
      "core": exact_or_find(a.root,"RasPi/bridge_core.py",["bridge_core.py"]),
      "tests": exact_or_find(a.root,"RasPi/test_bridge_core.py",["test_bridge_core.py"]),
      "bridge": exact_or_find(a.root,"BLE_Receiver-Final/BLE_Receiver-Final.ino",["BLE_Receiver-Final.ino"]),
      "mcu_h": exact_or_find(a.root,"NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.h",["NAPHTALI_CODE_V2.h"]),
      "mcu_cpp": exact_or_find(a.root,"NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.cpp",["NAPHTALI_CODE_V2.cpp"]),
      "mcu_ino": exact_or_find(a.root,"NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.ino",["NAPHTALI_CODE_V2.ino"]),
      "points": exact_or_find(a.root,"NAPHTALI_CODE_V2/pointsRun.ino",["pointsRun.ino"]),
    }
    for k,p in files.items():
        (a.ok if p else a.fail)("required file", f"{'Found' if p else 'Missing'} {k}", p)
    return files

def audit_pi(a,p,core,tests):
    if not p: return
    s=text(p)
    stale=number(s,"LIDAR_STALE_TIMEOUT_S")
    if stale is not None and stale<=1.0: a.ok("Pi LiDAR stale watchdog",f"{stale:g} s",p)
    else: a.fail("Pi LiDAR stale watchdog",f"Missing/slow timeout: {stale}",p)
    checks=[
      ("from bridge_core import", "Pi helper source", "Production imports bridge_core source"),
      ("CoalescingBleQueue", "Pi BLE queue", "Bounded/coalescing BLE mailbox present"),
      ('f"P:{seq}', "Pi sequenced path", "Produces P:<seq> packets"),
      ('f"S:{seq}', "Pi sequenced steering", "Produces matching S:<seq> packets"),
      ("F=S|B=S", "Pi stale output", "LiDAR loss emits explicit stale path"),
      ("asyncio.Lock", "Pi BLE serialization", "GATT writes share an asyncio lock"),
      ("response=acknowledged", "Pi acknowledged safety writes", "P/reset/readiness use acknowledged writes"),
      ("ALLOW_RUNTIME_SAFETY_DISABLE = False", "Pi safety bypass", "Runtime bypass disabled by default"),
    ]
    for token,c,d in checks: (a.ok if token in s else a.fail)(c,d,p)
    (a.ok if s.count("create_subscription(LaserScan") == 1 else a.fail)(
        "Pi ROS subscription", "Exactly one production LaserScan subscription", p)
    (a.ok if "if not self._first_scan_received" in s else a.fail)(
        "Pi first-scan safety", "No first scan is treated as stale/STOP", p)
    (a.ok if "self._front_data_valid" in s and "LIDAR_UNAVAILABLE" in s else a.fail)(
        "Pi scan-quality gate", "Incomplete safety sectors fail closed as unavailable", p)
    (a.ok if "_connect_guarded" in s else a.fail)(
        "Pi BLE connect recovery", "Unexpected connect-task failures reschedule recovery", p)
    (a.ok if "FIREBASE_IMPORT_ERROR" in s else a.fail)(
        "Pi Firebase isolation", "Missing firebase-admin cannot crash the safety bridge", p)
    if core:
        cs=text(core)
        a.ok("Pi helper completeness","bridge_core.py is source-controlled, not pyc-only",core)
        (a.ok if "math.isinf(distance) and distance > 0.0" in cs else a.fail)(
            "Pi LaserScan infinity", "+inf is represented as range_max", core)
        (a.ok if "return round(min(values) * 100.0, 1)" in cs else a.fail)(
            "Pi collision representative", "Safety uses the closest valid LiDAR return", core)
    if tests: a.ok("Pi host tests","test_bridge_core.py is included",tests)

def audit_bridge(a,p):
    if not p:return
    s=text(p)
    pt=number(s,"PATH_DATA_TIMEOUT_MS"); lt=number(s,"LINK_DATA_TIMEOUT_MS"); cc=number(s,"CLEAR_CONFIRM_PACKETS")
    (a.ok if pt is not None and pt<=1000 else a.fail)("bridge path watchdog",f"{pt} ms",p)
    (a.ok if lt is not None and lt<=10000 else a.warn)("bridge link recovery",f"{lt} ms",p)
    (a.ok if cc is not None and cc>=2 else a.fail)("bridge STOP clearance",f"{cc} clear packets",p)
    for cond,c,d in [
      ('volatile bool mcuReady            = false' in s,"bridge MCU readiness","Fail closed until explicit [MCU READY]"),
      (bool(re.search(r"stopLatched\s*=\s*true",s)),"bridge STOP default","STOP latch defaults true"),
      ('raw.startsWith("SENSOR:")' in s and 'sendMcuLine(raw, true);' in s,"bridge telemetry isolation","SENSOR is explicitly relayed"),
      ("parseSequencedPacket" in s and "sequenceIsNewer" in s,"bridge sequence freshness","Old path sequences rejected"),
      ("seq != lastPathSeq" in s,"bridge steering binding","S sequence must match latest P sequence"),
      ('STOP:LINK' in s and 'onDisconnect' in s,"bridge disconnect STOP","Disconnect has stationary STOP path"),
      ('ESP_Serial.begin(115200' in s,"bridge UART baud","UART is 115200"),
      ('if (mcuUnackedCommands > 0) mcuUnackedCommands--;' in s,"bridge ACK accounting","One generic ACK consumes one queued command"),
      ("parsePathBody" in s and "parseUint32Decimal" in s,"bridge strict path parser","Exact fields and uint32 sequence validation present"),
      ('pathVal == "CLEAR"' in s and 'backPathVal == "CLEAR"' in s,"bridge legacy parser","Legacy path values are explicitly validated"),
    ]:
        (a.ok if cond else a.fail)(c,d,p)

def audit_mcu(a,h,cpp,ino,points):
    hs,cs,isrc=text(h),text(cpp),text(ino)
    if cpp:
      for cond,c,d in [
        (bool(re.search(r"bool\s+shouldStop\s*=\s*true",cs)),"MCU STOP default","Controller starts fail closed"),
        ("!shouldStop && pathCommandFresh()" in cs,"MCU nudge guard","Nudges require fresh GO state"),
        ("controlledStopMotors" in cs and "SAFETY_STOP_DECEL" in cs,"MCU controlled braking","Safety stop decelerates before force fallback"),
        ("parseSensorNumber" in cs and "Commit atomically" in cs,"MCU sensor parser","Telemetry validates completely before commit"),
        ("startModemInitialization" in cs and "modemInitializationTask" in cs,"MCU modem isolation","Cellular init runs in background task"),
        ("return smsWorkerBusy || modemServiceBusy;" in cs,"MCU modem ownership","Main pass-through cannot steal modem task responses"),
        ("parseUnsignedToken" in cs,"MCU strict nudge parser","Nudge numeric fields reject junk/overflow"),
        ("recordFrontSample(999.0f, false)" in cs and "latestFrontSampleValid" in cs,"MCU sonar no-echo safety","No-echo cannot become a fresh clear sample"),
        ("requestStatus();" in cs[cs.find("void movementGate"):cs.find("void safeMoveDistance")],"MCU movement gate requests","Gate actively requests fresh status while waiting"),
      ]: (a.ok if cond else a.fail)(c,d,cpp)
      brake=cs[cs.find("void activeBrakeStopMotors"):cs.find("void startStraight")]
      (a.warn if re.search(r"\bmove\s*\(\s*-",brake) else a.ok)("MCU reverse brake pulse","No reverse pulse detected" if not re.search(r"\bmove\s*\(\s*-",brake) else "Reverse move detected",cpp)
    if h:
      pt=number(hs,"PATH_COMMAND_TIMEOUT_MS"); gc=number(hs,"MCU_GO_CONFIRM_PACKETS"); ut=number(hs,"ULTRASONIC_TIMEOUT_US"); side=number(hs,"ENABLE_ULTRASONIC_SIDE_NUDGE"); gate=number(hs,"MOTION_GATE_TIMEOUT_MS")
      (a.ok if pt is not None and pt<=1000 else a.fail)("MCU path watchdog",f"{pt} ms",h)
      (a.ok if gc is not None and gc>=2 else a.fail)("MCU GO confirmation",f"{gc} GO packets",h)
      (a.ok if ut is not None and ut<=20000 else a.fail)("MCU sonar timeout",f"{ut} us",h)
      (a.ok if side==0 else a.warn)("MCU steering authority",f"side-sonar nudge={side}",h)
      (a.ok if gate is not None and gate>=800 else a.fail)("MCU movement gate timing",f"{gate} ms",h)
    if ino:
      blocking = any(x in isrc for x in ('powerOnAir780();','waitForModule(10)','waitForNetwork(10)'))
      # Calls inside the background task live in .cpp; setup must not contain them.
      (a.fail if blocking else a.ok)("MCU startup gating","setup() does not wait for modem/network",ino)
      (a.ok if 'startModemInitialization();' in isrc else a.fail)("MCU async modem start","Modem task starts after [MCU READY]",ino)
      (a.ok if 'ESP_Serial.println("[MCU READY]")' in isrc else a.fail)("MCU readiness handshake","[MCU READY] emitted",ino)
    if points:
      ps=text(points)
      rm=re.search(r"runStart\(\).*?safeMoveDistance\(([-0-9]+)",ps,re.S); rr=re.search(r"returnToPointB\(\).*?safeMoveDistance\(([-0-9]+)",ps,re.S)
      if rm and rr and int(rm.group(1))>0 and int(rr.group(1))>0:
          a.warn("route direction","Outbound and return contain positive long moves; physical direction remains hardware-unverified",points)
      else: a.ok("route direction","Route sign pattern is explicit",points)

def audit_docs(a,p):
    if not p:return
    s=text(p)
    topics=["850 ms","10 s","5600","40°","120°","FRONT | 0°","background","bridge_core.py","hardware-unverified","900 ms","no echo"]
    for term in topics:
        (a.ok if term.lower() in s.lower() else a.warn)("architecture sync",f"Documents '{term}'",p)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("project_root",type=Path); ap.add_argument("--json",action="store_true"); ns=ap.parse_args()
    if not ns.project_root.is_dir(): print("project root must be a directory",file=sys.stderr); return 2
    a=Audit(ns.project_root); f=required(a); audit_pi(a,f['pi'],f['core'],f['tests']); audit_bridge(a,f['bridge']); audit_mcu(a,f['mcu_h'],f['mcu_cpp'],f['mcu_ino'],f['points']); audit_docs(a,f['arch'])
    pyc=list(ns.project_root.rglob("*.pyc"))
    (a.warn if pyc else a.ok)("package hygiene", f"Compiled Python cache files present: {len(pyc)}" if pyc else "No compiled Python cache files in release")
    launch=text(ns.project_root/".vscode/launch.json")
    (a.fail if re.search(r"[A-Za-z]:[/\\]", launch) else a.ok)("editor portability", "No machine-specific absolute Windows path in launch.json")
    counts={k:sum(r.severity==k for r in a.results) for k in ('PASS','WARN','FAIL')}
    if ns.json: print(json.dumps({'root':str(a.root),'summary':counts,'results':[asdict(r) for r in a.results]},indent=2))
    else:
      for r in a.results: print(f"{r.severity:4} {r.check}: {r.detail}"+(f" [{r.file}]" if r.file else ""))
      print(f"\nSummary: {counts['PASS']} passed, {counts['WARN']} warnings, {counts['FAIL']} failed")
      print("Static source audit only; compile and physical tests are still required.")
    return 1 if counts['FAIL'] else 0
if __name__=='__main__': raise SystemExit(main())
