const { chromium } = require('playwright');
(async () => {
  const b = await chromium.launch({args:['--use-gl=swiftshader','--enable-webgl','--ignore-gpu-blocklist']});
  const p = await b.newPage({viewport:{width:1400,height:860}});
  const errs=[]; p.on('pageerror',e=>errs.push(e.message));
  await p.goto('http://localhost:8765/test.html'); await p.waitForTimeout(7000);
  const check = () => p.evaluate(() => {
    const {THREE, meshes, cableMeshes, zroot} = window.__twin;
    const inv = new THREE.Matrix4().copy(zroot.matrixWorld).invert();
    const ray = new THREE.Raycaster(); const out = {};
    const list = Object.entries(meshes).map(([n,m]) => { m.material.side = THREE.DoubleSide; return [n,m]; });
    for (const [k, cm] of Object.entries(cableMeshes)){
      let bad = 0; const hits = {};
      for (const pt of cm.userData.pts){
        const w = pt.clone().applyMatrix4(zroot.matrixWorld);
        for (const [n,m] of list){
          if (!m.geometry.boundingBox) m.geometry.computeBoundingBox();
          const local = w.clone().applyMatrix4(new THREE.Matrix4().copy(m.matrixWorld).invert());
          if (!m.geometry.boundingBox.containsPoint(local)) continue;
          ray.set(w, new THREE.Vector3(0.577,0.577,0.577)); const c = ray.intersectObject(m).length;
          if (c % 2 === 1){ bad++; hits[n]=(hits[n]||0)+1; break; }
        }
      }
      out[k] = {bad, n: cm.userData.pts.length, hits};
    }
    return out;
  });
  const poses = ['home','move 107 131 98 60','move 30 150 40 70','move 160 60 150 90','move 90 40 170 90','move 0 120 120 55','move 140 160 20 90','move 60 100 60 70'];
  for (const c of poses){ await p.evaluate(c => window.__twin.exec(c), c); await p.waitForTimeout(4500); console.log(c.padEnd(22), JSON.stringify(await check())); }
  await p.evaluate(() => window.__twin.exec('move 107 131 98 60')); await p.waitForTimeout(4500);
  await p.screenshot({path:'/home/claude/arm/twin_c1.png'});
  console.log('errors:', errs.join(' | ')||'none'); await b.close();
})();
