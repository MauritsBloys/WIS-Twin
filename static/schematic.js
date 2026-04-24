// ── Popover state ─────────────────────────────────────────────────────────────
let _popType = null;  // 'gate' | 'valve'
let _popId   = null;  // node number (201–204) or valve number (1–6)

function _positionPopover(svgX, svgY) {
  const pop   = document.getElementById('ctrl-popover');
  const svgEl = document.getElementById('schematic');
  if (!pop || !svgEl) return;
  const wrap  = svgEl.closest('.schem-body') || document.body;
  const sr    = svgEl.getBoundingClientRect();
  const wr    = wrap.getBoundingClientRect();
  const vw    = svgEl.viewBox.baseVal.width  || 1400;
  const vh    = svgEl.viewBox.baseVal.height || 620;

  let left = (svgX / vw) * sr.width  + (sr.left - wr.left);
  let top  = (svgY / vh) * sr.height + (sr.top  - wr.top);

  const pw = 270, ph = 140;
  if (left + pw > wr.width  - 8) left = wr.width  - pw - 8;
  if (left < 5)                  left = 5;
  if (top  + ph > wr.height - 8) top  = top - ph - 36;
  if (top  < 5)                  top  = 5;

  pop.style.left = left + 'px';
  pop.style.top  = top  + 'px';
  pop.classList.remove('hidden');
}

function _wireSlider() {
  const s = document.getElementById('pop-slider');
  const n = document.getElementById('pop-num');
  if (!s || !n) return;
  s.oninput = () => { n.value = s.value; };
  n.oninput = () => { s.value = n.value; };
}

// ── Gate popover ──────────────────────────────────────────────────────────────
function openGatePopover(node, svgX, svgY) {
  _popType = 'gate';
  _popId   = node;
  const num = node - 200;
  document.getElementById('pop-title').textContent    = `GV-0${num}`;
  document.getElementById('pop-subtitle').textContent = `Gate ${num} · node ${node}`;
  document.getElementById('pop-min').textContent      = '0';
  document.getElementById('pop-max').textContent      = '255';
  document.getElementById('pop-mid').textContent      = '128';
  document.getElementById('pop-max-btn').textContent  = 'Max';
  const s = document.getElementById('pop-slider');
  const n = document.getElementById('pop-num');
  s.min = 0; s.max = 255; s.value = 0;
  n.min = 0; n.max = 255; n.value = 0;
  _wireSlider();
  _positionPopover(svgX, svgY);
}

// ── Valve popover ─────────────────────────────────────────────────────────────
function openValvePopover(valve, svgX, svgY) {
  _popType = 'valve';
  _popId   = valve;
  document.getElementById('pop-title').textContent    = `FV-0${valve}`;
  document.getElementById('pop-subtitle').textContent = `Valve ${valve} · 0–90°`;
  document.getElementById('pop-min').textContent      = '0°';
  document.getElementById('pop-max').textContent      = '90°';
  document.getElementById('pop-mid').textContent      = '45°';
  document.getElementById('pop-max-btn').textContent  = '90°';
  const s = document.getElementById('pop-slider');
  const n = document.getElementById('pop-num');
  s.min = 0; s.max = 90; s.value = 0;
  n.min = 0; n.max = 90; n.value = 0;
  _wireSlider();
  _positionPopover(svgX, svgY);
}

// ── Popover controls ──────────────────────────────────────────────────────────
function popSet(val) {
  const s = document.getElementById('pop-slider');
  const n = document.getElementById('pop-num');
  if (!s || !n) return;
  if (val === 'max') { s.value = s.max; n.value = n.max; }
  else if (val === 'mid') { const m = Math.round(parseInt(s.max) / 2); s.value = m; n.value = m; }
  else { s.value = val; n.value = val; }
}

function popApply() {
  const val = parseInt(document.getElementById('pop-num').value);
  if (isNaN(val)) return;

  if (_popType === 'gate') {
    fetch(`/api/firefly/gate/${_popId}/${val}`, { method: 'POST' })
      .then(r => r.json())
      .then(d => {
        if (d.error) { showToast('Fout: ' + d.error, true); return; }
        showToast(`Gate ${_popId - 200} → ${val}`);
        if (typeof logEvent !== 'undefined') logEvent(`GV-0${_popId - 200} → ${val}`, 'ctrl');
        const ro = document.getElementById('gate-readout-' + _popId);
        if (ro) ro.textContent = `G${_popId - 200} · ${val}`;
        closePopover();
      })
      .catch(() => showToast('Geen verbinding', true));

  } else if (_popType === 'valve') {
    fetch('/api/valve', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ valve: _popId, position: val })
    })
      .then(r => r.json())
      .then(d => {
        if (d.error) { showToast('Fout: ' + d.error, true); return; }
        showToast(`Valve ${_popId} → ${val}°`);
        if (typeof logEvent !== 'undefined') logEvent(`FV-0${_popId} → ${val}°`, 'ctrl');
        // Update SVG readout
        const ro = document.getElementById('valve-readout-' + _popId);
        if (ro) ro.textContent = val + '°';
        // Update valve table row
        const rows = document.querySelectorAll('#valve-tbody tr');
        const row  = rows[_popId - 1];
        if (row) {
          const cells = row.querySelectorAll('td');
          const pct   = Math.round(val / 90 * 100);
          const bar   = cells[1]?.querySelector('span');
          if (bar)     bar.style.width = pct + '%';
          if (cells[2]) cells[2].textContent = val + '°';
          if (cells[3]) cells[3].textContent = pct + '%';
        }
        closePopover();
      })
      .catch(() => showToast('Geen verbinding', true));
  }
}

function closePopover() {
  const pop = document.getElementById('ctrl-popover');
  if (pop) pop.classList.add('hidden');
  _popType = null;
  _popId   = null;
}

// Close on outside click
document.addEventListener('click', e => {
  const pop = document.getElementById('ctrl-popover');
  if (pop && !pop.classList.contains('hidden') &&
      !pop.contains(e.target) &&
      !e.target.closest('#schematic')) {
    closePopover();
  }
});
