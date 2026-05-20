// ── Clock ─────────────────────────────────────────────────────────────────────
function padZ(n) { return String(n).padStart(2, '0'); }
function updateClock() {
  const d = new Date();
  const el = document.getElementById('tb-clock');
  if (el) el.textContent = `${padZ(d.getHours())}:${padZ(d.getMinutes())}:${padZ(d.getSeconds())}`;
}
setInterval(updateClock, 1000);
updateClock();

// ── Event log ─────────────────────────────────────────────────────────────────
function logEvent(msg, level = 'info') {
  const log = document.getElementById('event-log');
  const cnt = document.getElementById('log-count');
  if (!log) return;
  const d = new Date();
  const t = `${padZ(d.getHours())}:${padZ(d.getMinutes())}:${padZ(d.getSeconds())}`;
  const entry = document.createElement('div');
  entry.className = 'log-entry';
  entry.innerHTML = `<span class="log-t">${t}</span><span class="log-lv lv-${level}">${level}</span><span class="log-msg">${msg}</span>`;
  log.insertBefore(entry, log.firstChild);
  const entries = log.querySelectorAll('.log-entry');
  if (entries.length > 100) log.removeChild(log.lastChild);
  if (cnt) cnt.textContent = `${entries.length} events`;
}

// ── Toast ─────────────────────────────────────────────────────────────────────
function showToast(msg, isError = false) {
  const t = document.getElementById('toast');
  if (!t) return;
  t.textContent = msg;
  t.className = 'toast' + (isError ? ' error' : '');
  t.style.display = 'block';
  clearTimeout(t._to);
  t._to = setTimeout(() => t.style.display = 'none', 3000);
}

// ── Relay state tracking ──────────────────────────────────────────────────────
const _relayState = {};

function _parseRelays(relayStr) {
  if (!relayStr) return;
  relayStr.split(' ').forEach(pair => {
    const [name, state] = pair.split('=');
    if (name && state) {
      const s = state.trim().toLowerCase();
      if (_relayState[name] !== s && s !== 'on' && s !== 'off')
        logEvent(`${name} = ${s}`, 'info');
      _relayState[name] = s;
    }
  });
}

function _applyRelayUI() {
  document.querySelectorAll('.relay-cell').forEach(cell => {
    const nameEl = cell.querySelector('.relay-name');
    if (!nameEl) return;
    const name = nameEl.textContent.trim();
    if (!(name in _relayState)) return;
    const state = _relayState[name];
    const counting = state.includes('count');
    const on = !counting && state.startsWith('on');
    const stateEl = cell.querySelector('.relay-state');
    if (stateEl) {
      stateEl.textContent = on ? 'ON' : counting ? 'COUNTING' : 'OFF';
      stateEl.style.color = on ? 'var(--green)' : counting ? 'var(--amber)' : 'var(--ink-mute)';
    }
    cell.classList.toggle('relay-on', on);
    cell.classList.toggle('relay-counting', counting);
  });
  _applyPumpVisual('well-pump');
  _applyPumpVisual('rain-pump');
}

function _applyPumpVisual(relay) {
  const on = (_relayState[relay] || '').startsWith('on');
  if (relay === 'well-pump') {
    const c = document.getElementById('well-pump-circle');
    const l = document.getElementById('well-pump-lbl');
    if (c) c.setAttribute('fill', on ? '#2e7d32' : '#1b5e20');
    if (l) l.textContent = 'WELL · ' + (on ? 'ON' : 'OFF');
  } else {
    const c = document.getElementById('rain-pump-circle');
    const l = document.getElementById('rain-pump-lbl');
    if (c) c.setAttribute('fill', on ? '#c62828' : '#3e0000');
    if (l) l.textContent = 'RAIN · ' + (on ? 'ON' : 'OFF');
  }
}

// ── Pump toggle (SVG click) ───────────────────────────────────────────────────
function togglePump(relay) {
  const next = !(_relayState[relay] || '').startsWith('on');
  const action = next ? 'on' : 'off';
  fetch(`/api/relay/${relay}/${action}`, { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      if (d.error) { showToast('Fout: ' + d.error, true); logEvent(relay + ' error: ' + d.error, 'warn'); return; }
      _relayState[relay] = next ? 'on' : 'off';
      _applyRelayUI();
      showToast(`${relay} → ${action.toUpperCase()}`);
      logEvent(`${relay} → ${action.toUpperCase()}`, 'ctrl');
    })
    .catch(() => showToast('Geen verbinding', true));
}

// ── Relay action (panel buttons) ──────────────────────────────────────────────
function relayAction(relay, action) {
  fetch(`/api/relay/${relay}/${action}`, { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      if (d.error) { showToast('Fout: ' + d.error, true); logEvent(relay + ' error: ' + d.error, 'warn'); return; }
      _relayState[relay] = action;
      _applyRelayUI();
      showToast(`${relay} → ${action.toUpperCase()}`);
      logEvent(`${relay} → ${action.toUpperCase()}`, 'ctrl');
    })
    .catch(() => showToast('Geen verbinding', true));
}

// ── Firefly repeat last command ───────────────────────────────────────────────
let _lastFireflyUrl = null;
let _repeatTimer    = null;

function sendFireflyCmd(url, label) {
  fetch(url, { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      if (d.error) { showToast('Fout: ' + d.error, true); return; }
      _lastFireflyUrl = url;
    })
    .catch(() => {});
  return fetch(url, { method: 'POST' });
}

function toggleRepeat(on) {
  clearInterval(_repeatTimer);
  _repeatTimer = null;
  const btn = document.getElementById('btn-repeat');
  if (on) {
    _repeatTimer = setInterval(() => {
      if (_lastFireflyUrl) fetch(_lastFireflyUrl, { method: 'POST' }).catch(() => {});
    }, 5000);
    if (btn) { btn.classList.add('active'); btn.textContent = 'Sleep Mode: OFF'; }
    logEvent('Sleep mode uitgeschakeld · Firefly herhaal aan', 'info');
  } else {
    if (btn) { btn.classList.remove('active'); btn.textContent = 'Sleep Mode: ON'; }
    logEvent('Sleep mode ingeschakeld', 'info');
  }
}

// ── Gate mode ─────────────────────────────────────────────────────────────────
function setMode(mode) {
  const url = `/api/firefly/mode/${mode}`;
  fetch(url, { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      if (d.error) { showToast('Fout: ' + d.error, true); return; }
      _lastFireflyUrl = url;
      document.getElementById('mode-manual')?.classList.toggle('active', mode === 'manual');
      document.getElementById('mode-auto')?.classList.toggle('active',   mode === 'auto');
      showToast('Mode: ' + mode);
      logEvent('Gate mode → ' + mode, 'ctrl');
    })
    .catch(() => showToast('Geen verbinding', true));
}

// ── Status fetch ──────────────────────────────────────────────────────────────
function fetchStatus() {
  fetch('/api/status')
    .then(r => r.json())
    .then(d => {
      if (d.error) { logEvent('Status fout: ' + d.error, 'warn'); return; }
      _parseRelays(d.relays);
      _applyRelayUI();
    })
    .catch(() => {});
}

// ── Close-all valve UI reset ──────────────────────────────────────────────────
function _applyCloseAllValveUI() {
  for (let i = 1; i <= 6; i++) {
    const ro = document.getElementById('valve-readout-' + i);
    if (ro) ro.textContent = '0°';
  }
  document.querySelectorAll('#valve-tbody tr').forEach(row => {
    const cells = row.querySelectorAll('td');
    const bar = cells[1]?.querySelector('span');
    if (bar) bar.style.width = '0%';
    if (cells[2]) cells[2].textContent = '0°';
    if (cells[3]) cells[3].textContent = '0%';
  });
}

// ── sendPost utility ──────────────────────────────────────────────────────────
function sendPost(url, body, label) {
  const opts = { method: 'POST', headers: { 'Content-Type': 'application/json' } };
  if (body) opts.body = JSON.stringify(body);
  fetch(url, opts)
    .then(r => r.json())
    .then(d => {
      if (d.error) { showToast('Fout: ' + d.error, true); logEvent((label || url) + ' error: ' + d.error, 'warn'); return; }
      showToast((label || url) + ' — OK');
      logEvent(label || url, 'ctrl');
      fetchStatus();
      if (url === '/api/close-all') _applyCloseAllValveUI();
    })
    .catch(() => showToast('Geen verbinding', true));
}

// ── Shutdown ──────────────────────────────────────────────────────────────────
function confirmShutdown() {
  if (confirm('Weet je zeker dat je het systeem wilt uitschakelen?')) {
    sendPost('/api/shutdown', null, 'Noodstop');
  }
}

// ── Sparkline history ─────────────────────────────────────────────────────────
const _sparkHistory = Array.from({ length: 7 }, () => []);
const SPARK_MAX_PTS = 30;

function _pushSpark(i, val) {
  const h = _sparkHistory[i];
  h.push(val);
  if (h.length > SPARK_MAX_PTS) h.shift();
}

function _drawSpark(i) {
  const h = _sparkHistory[i];
  if (h.length < 2) return;
  const rows = document.querySelectorAll('#bin-body .bin-row');
  const row = rows[i];
  if (!row) return;
  const fillEl = row.querySelector('.spark-fill');
  const lineEl = row.querySelector('.spark-line');
  if (!fillEl || !lineEl) return;
  const maxVal = Math.max(...h, 1);
  const pts = h.map((v, idx) => {
    const x = (idx / (SPARK_MAX_PTS - 1)) * 100;
    const y = 22 - (v / maxVal) * 20;
    return `${x.toFixed(1)},${y.toFixed(1)}`;
  });
  const lineD = 'M ' + pts.join(' L ');
  const first = pts[0].split(',');
  const last  = pts[pts.length - 1].split(',');
  const fillD = `M ${first[0]},22 L ` + pts.join(' L ') + ` L ${last[0]},22 Z`;
  lineEl.setAttribute('d', lineD);
  fillEl.setAttribute('d', fillD);
}

// ── Firefly status poll ───────────────────────────────────────────────────────
const TANK_BOTTOM = 520;
const TANK_HEIGHT = 240;
const CM_MAX      = 60;

function fetchFirefly() {
  fetch('/api/firefly/status')
    .then(r => r.json())
    .then(d => {
      // Connection pill
      const pill = document.getElementById('ff-pill');
      if (pill) {
        pill.textContent = d.connected ? 'Firefly · OK' : 'Firefly · offline';
        pill.className   = d.connected ? 'pill ok' : 'pill err';
      }

      // Sensors 1–7
      for (let i = 1; i <= 7; i++) {
        const info  = d.sensors[String(i)];
        const cm    = (info && info.cm != null) ? info.cm : null;
        const cmStr = cm != null ? cm.toFixed(1) + ' cm' : '— cm';

        // PT chip readout
        const chip = document.getElementById('pt-chip-' + i);
        if (chip) chip.textContent = cmStr;

        // SVG bin text
        const binCm = document.getElementById('bin-cm-' + i);
        if (binCm) binCm.textContent = cmStr;

        // Bin-body row cm span
        const rows = document.querySelectorAll('#bin-body .bin-row');
        const row  = rows[i - 1];
        if (row) {
          const cmEl = row.querySelector('.cm');
          if (cmEl) cmEl.textContent = cmStr;
        }

        // Sparkline
        if (cm != null) {
          _pushSpark(i - 1, cm);
          _drawSpark(i - 1);
        }

        // Water level animation
        if (cm != null) {
          const pct      = Math.min(cm / CM_MAX, 1);
          const heightPx = Math.max(pct * TANK_HEIGHT, 1);
          const yPx      = TANK_BOTTOM - heightPx;
          const waterRect = document.getElementById('water-' + i);
          const waterLine = document.getElementById('water-surface-' + i);
          if (waterRect) { waterRect.setAttribute('height', heightPx); waterRect.setAttribute('y', yPx); }
          if (waterLine) { waterLine.setAttribute('y1', yPx); waterLine.setAttribute('y2', yPx); }
        }
      }

      // Gates 201–204
      const GATES    = [201, 202, 203, 204];
      const gateRows = document.querySelectorAll('#gate-tbody tr');
      GATES.forEach((node, idx) => {
        const val = d.actuators[String(node)];
        const pct = val != null ? Math.round(val / 255 * 100) : 0;

        // Table row
        const tr = gateRows[idx];
        if (tr) {
          const cells = tr.querySelectorAll('td');
          const bar   = cells[2]?.querySelector('span');
          if (bar)     bar.style.width = pct + '%';
          if (cells[3]) cells[3].textContent = val != null ? val : '—';
          if (cells[4]) cells[4].textContent = pct + '%';
        }

        // SVG blade
        const blade = document.getElementById('gate-blade-' + node);
        if (blade && val != null) {
          const h = Math.max(10, Math.round(val / 255 * 90));
          blade.setAttribute('height', h);
        }

        // SVG readout
        const readout = document.getElementById('gate-readout-' + node);
        if (readout) readout.textContent = `G${node - 200} · ${val != null ? val : '—'}`;

        // SVG motor stroke colour
        const motor = document.getElementById('gate-motor-' + node);
        if (motor && val != null) {
          motor.setAttribute('stroke', pct > 50 ? '#4fc3f7' : pct > 10 ? '#ffd54f' : '#ef5350');
        }
      });
    })
    .catch(() => {});
}

// ── Setpoint panel ────────────────────────────────────────────────────────────
document.querySelectorAll('[data-sp]').forEach(el => {
  el.addEventListener('input', () => {
    const sp    = el.getAttribute('data-sp');
    const type  = el.type === 'range' ? 'number' : 'range';
    const other = document.querySelector(`[data-sp="${sp}"][type="${type}"]`);
    if (other) other.value = el.value;
  });
});

document.querySelectorAll('[data-apply]').forEach(btn => {
  btn.addEventListener('click', () => {
    const idx   = btn.getAttribute('data-apply');
    const numEl = document.querySelector(`input[type="number"][data-sp="${idx}"]`);
    if (!numEl) return;
    const tag = `PT-0${parseInt(idx) + 1}`;
    showToast(`Setpoint ${tag} → ${numEl.value} cm`);
    logEvent(`Setpoint ${tag} → ${numEl.value} cm`, 'info');
  });
});

// ── Boot ──────────────────────────────────────────────────────────────────────
toggleRepeat(true);
fetchStatus();
fetchFirefly();
setInterval(fetchStatus,  2000);
setInterval(fetchFirefly,  2000);
logEvent('SCADA gestart · verbinding maken met backend', 'info');
