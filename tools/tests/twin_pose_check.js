const { chromium } = require('playwright');
(async () => {
  const b = await chromium.launch({args:['--use-gl=swiftshader','--enable-webgl','--ignore-gpu-blocklist']});
  const p = await b.newPage({viewport:{width:1400,height:860}});
  const errs=[]; p.on('pageerror',e=>errs.push(e.message));
  await p.goto('http://localhost:8765/test.html'); await p.waitForTimeout(7000);
  const poses = {home:'home', desk_open:'move 107 131 98 60', desk_closed:'move 107 131 98 90', desk_front:'move 81 135 93 60', desk_right:'move 48 122 112 60', reach_out:'move 90 130 60 65', tucked:'move 90 55 150 90', fold_hard:'move 90 30 170 90'};
  for (const [n,c] of Object.entries(poses)){ await p.evaluate(c => window.__twin.exec(c), c); await p.waitForTimeout(4500); console.log(n.padEnd(12), await p.textContent('#status')); }
  console.log('errors:', errs.join(' | ')||'none'); await b.close();
})();
