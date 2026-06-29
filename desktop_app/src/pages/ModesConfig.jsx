import React, { useState, useEffect, useRef, useCallback } from 'react';

const MODES = [
  { id: 0,  name: 'Manual' },
  { id: 1,  name: 'Stabilize' },
  { id: 2,  name: 'Altitude Hold' },
  { id: 3,  name: 'Position Hold' },
  { id: 4,  name: 'RTH' },
  { id: 5,  name: 'Headless' },
  { id: 6,  name: 'Acro Plus' },
  { id: 7,  name: 'Sport' },
  { id: 8,  name: 'Cinematic' },
  { id: 9,  name: 'GPS Rescue' },
  { id: 10, name: 'Turtle' },
  { id: 11, name: 'Launch Assist' },
  { id: 12, name: 'Land Assist' },
];

const AUX_CHANNELS = ['AUX1', 'AUX2', 'AUX3', 'AUX4'];

const PWM_MIN = 900;
const PWM_MAX = 2100;
const PWM_RANGE = PWM_MAX - PWM_MIN;

const TICK_MARKS = [900, 1000, 1200, 1400, 1500, 1600, 1800, 2000, 2100];

const RANGE_COLORS = [
  'bg-green-500',
  'bg-blue-500',
  'bg-purple-500',
  'bg-amber-500',
];

const DEFAULT_ARMING_FLAGS = [
  { name: 'Gyro not calibrated',         status: 'ok', message: 'Gyro calibration OK' },
  { name: 'Accelerometer not calibrated', status: 'ok', message: 'Accel calibration OK' },
  { name: 'RC signal lost',              status: 'ok', message: 'RC link active' },
  { name: 'Battery low',                 status: 'ok', message: 'Battery voltage OK' },
  { name: 'Motor test active',           status: 'ok', message: 'Motors idle' },
  { name: 'System not ready',            status: 'ok', message: 'System ready' },
];

const pwmToPercent = (pwm) => ((pwm - PWM_MIN) / PWM_RANGE) * 100;

const clampPwm = (pwm) => Math.max(PWM_MIN, Math.min(PWM_MAX, Math.round(pwm)));

const RangeHandle = ({ side, onMouseDown }) => (
  <div
    onMouseDown={onMouseDown}
    className={`absolute top-0 bottom-0 w-2.5 cursor-col-resize z-10 group
      ${side === 'left' ? '-left-1' : '-right-1'}`}
  >
    <div className="absolute inset-y-0 left-1/2 -translate-x-1/2 w-1
      bg-white/80 group-hover:bg-white rounded-sm shadow" />
  </div>
);

const RangeBar = ({ range, channelIndex, barRef, onChange, onRemove }) => {
  const dragState = useRef(null);

  const startDrag = useCallback((type, e) => {
    e.preventDefault();
    e.stopPropagation();
    if (!barRef.current) return;
    const rect = barRef.current.getBoundingClientRect();
    dragState.current = {
      type,
      startX: e.clientX,
      startMin: range.min,
      startMax: range.max,
      barLeft: rect.left,
      barWidth: rect.width,
    };

    const onMove = (ev) => {
      if (!dragState.current) return;
      const { type: dt, startX, startMin, startMax, barWidth } = dragState.current;
      const dx = ev.clientX - startX;
      const dPwm = (dx / barWidth) * PWM_RANGE;

      if (dt === 'left') {
        onChange({ ...range, min: clampPwm(Math.min(startMin + dPwm, range.max - 10)) });
      } else if (dt === 'right') {
        onChange({ ...range, max: clampPwm(Math.max(startMax + dPwm, range.min + 10)) });
      } else {
        const newMin = clampPwm(startMin + dPwm);
        const newMax = clampPwm(startMax + dPwm);
        const width = startMax - startMin;
        if (newMin >= PWM_MIN && newMax <= PWM_MAX) {
          onChange({ ...range, min: newMin, max: newMax });
        } else if (newMin < PWM_MIN) {
          onChange({ ...range, min: PWM_MIN, max: PWM_MIN + width });
        } else {
          onChange({ ...range, max: PWM_MAX, min: PWM_MAX - width });
        }
      }
    };

    const onUp = () => {
      dragState.current = null;
      window.removeEventListener('mousemove', onMove);
      window.removeEventListener('mouseup', onUp);
    };

    window.addEventListener('mousemove', onMove);
    window.addEventListener('mouseup', onUp);
  }, [range, barRef, onChange]);

  const left = pwmToPercent(range.min);
  const width = pwmToPercent(range.max) - left;
  const colorClass = RANGE_COLORS[channelIndex % RANGE_COLORS.length];

  return (
    <div
      className={`absolute top-0.5 bottom-0.5 rounded ${colorClass} opacity-85
        hover:opacity-100 transition-opacity cursor-grab active:cursor-grabbing`}
      style={{ left: `${left}%`, width: `${width}%` }}
      onMouseDown={(e) => startDrag('move', e)}
    >
      <RangeHandle side="left" onMouseDown={(e) => startDrag('left', e)} />
      <RangeHandle side="right" onMouseDown={(e) => startDrag('right', e)} />

      <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
        <span className="text-[10px] font-bold text-white drop-shadow select-none truncate px-1">
          {range.min}–{range.max}
        </span>
      </div>

      <button
        onClick={(e) => { e.stopPropagation(); onRemove(); }}
        className="absolute -top-2 -right-2 w-4 h-4 bg-red-500 text-white rounded-full
          text-[9px] leading-none flex items-center justify-center opacity-0
          hover:opacity-100 group-hover:opacity-100 transition-opacity z-20 shadow"
        title="Remove range"
      >
        ✕
      </button>
    </div>
  );
};

const ChannelLane = ({ label, ranges, channelIndex, onChangeRange, onRemoveRange }) => {
  const barRef = useRef(null);

  return (
    <div className="flex items-center gap-2 h-6">
      <span className="text-[10px] font-medium text-gray-500 w-8 shrink-0 text-right">
        {label}
      </span>
      <div ref={barRef} className="relative flex-1 h-full bg-gray-200 rounded group">
        {ranges.map((range, idx) => (
          <RangeBar
            key={idx}
            range={range}
            channelIndex={channelIndex}
            barRef={barRef}
            onChange={(updated) => onChangeRange(idx, updated)}
            onRemove={() => onRemoveRange(idx)}
          />
        ))}
      </div>
    </div>
  );
};

const TickLabels = () => (
  <div className="flex items-center gap-2 h-4 mb-1">
    <span className="w-8 shrink-0" />
    <div className="relative flex-1">
      {TICK_MARKS.map((tick) => (
        <span
          key={tick}
          className="absolute text-[9px] text-gray-400 -translate-x-1/2 select-none"
          style={{ left: `${pwmToPercent(tick)}%` }}
        >
          {tick}
        </span>
      ))}
    </div>
  </div>
);

const TickLines = () => (
  <div className="flex items-center gap-2 pointer-events-none" style={{ height: 0 }}>
    <span className="w-8 shrink-0" />
    <div className="relative flex-1">
      {TICK_MARKS.map((tick) => (
        <div
          key={tick}
          className="absolute top-0 w-px bg-gray-300"
          style={{
            left: `${pwmToPercent(tick)}%`,
            height: `${AUX_CHANNELS.length * 28}px`,
          }}
        />
      ))}
    </div>
  </div>
);

const ModeRow = ({ mode, onChange }) => {
  const addRange = (channelIndex) => {
    const newRange = { channel: channelIndex, min: 1400, max: 1600 };
    onChange({
      ...mode,
      ranges: [...mode.ranges, newRange],
    });
  };

  const updateRange = (rangeIdx, updated) => {
    const next = [...mode.ranges];
    next[rangeIdx] = updated;
    onChange({ ...mode, ranges: next });
  };

  const removeRange = (rangeIdx) => {
    onChange({
      ...mode,
      ranges: mode.ranges.filter((_, i) => i !== rangeIdx),
    });
  };

  const rangesForChannel = (ch) =>
    mode.ranges
      .map((r, idx) => ({ ...r, _idx: idx }))
      .filter((r) => r.channel === ch);

  return (
    <div className="bg-white rounded-lg shadow border border-gray-100 p-4 hover:shadow-md transition-shadow">
      <div className="flex items-start gap-4">
        {/* Mode name */}
        <div className="w-32 shrink-0 pt-1">
          <h3 className="text-sm font-semibold text-gray-800">{mode.name}</h3>
          <span className="text-[10px] text-gray-400">ID {mode.id}</span>
        </div>

        {/* Channel lanes */}
        <div className="flex-1 min-w-0">
          <TickLabels />
          <div className="relative">
            <TickLines />
            <div className="space-y-1">
              {AUX_CHANNELS.map((ch, chIdx) => (
                <ChannelLane
                  key={ch}
                  label={ch}
                  channelIndex={chIdx}
                  ranges={rangesForChannel(chIdx)}
                  onChangeRange={(localIdx, updated) => {
                    const globalIdx = rangesForChannel(chIdx)[localIdx]._idx;
                    updateRange(globalIdx, updated);
                  }}
                  onRemoveRange={(localIdx) => {
                    const globalIdx = rangesForChannel(chIdx)[localIdx]._idx;
                    removeRange(globalIdx);
                  }}
                />
              ))}
            </div>
          </div>
        </div>

        {/* Add range dropdown */}
        <div className="shrink-0 pt-1">
          <AddRangeButton onAdd={addRange} />
        </div>
      </div>
    </div>
  );
};

const AddRangeButton = ({ onAdd }) => {
  const [open, setOpen] = useState(false);
  const ref = useRef(null);

  useEffect(() => {
    const close = (e) => {
      if (ref.current && !ref.current.contains(e.target)) setOpen(false);
    };
    document.addEventListener('mousedown', close);
    return () => document.removeEventListener('mousedown', close);
  }, []);

  return (
    <div ref={ref} className="relative">
      <button
        onClick={() => setOpen(!open)}
        className="px-3 py-1.5 bg-green-500 text-white text-xs font-medium rounded
          hover:bg-green-600 transition-colors whitespace-nowrap"
      >
        + Add Range
      </button>
      {open && (
        <div className="absolute right-0 mt-1 bg-white border border-gray-200 rounded shadow-lg z-30 py-1 w-28">
          {AUX_CHANNELS.map((ch, idx) => (
            <button
              key={ch}
              onClick={() => { onAdd(idx); setOpen(false); }}
              className="block w-full text-left px-3 py-1.5 text-xs text-gray-700
                hover:bg-gray-100 transition-colors"
            >
              {ch}
            </button>
          ))}
        </div>
      )}
    </div>
  );
};

const ArmingFlagRow = ({ flag }) => {
  const statusConfig = {
    ok:      { bg: 'bg-green-100', text: 'text-green-800', dot: 'bg-green-500', label: 'OK' },
    warning: { bg: 'bg-yellow-100', text: 'text-yellow-800', dot: 'bg-yellow-500', label: 'WARN' },
    error:   { bg: 'bg-red-100', text: 'text-red-800', dot: 'bg-red-500', label: 'FAIL' },
  };
  const cfg = statusConfig[flag.status] || statusConfig.ok;

  return (
    <div className="flex items-center justify-between p-3 bg-gray-50 rounded hover:bg-gray-100 transition-colors">
      <div className="flex items-center gap-2">
        <div className={`w-2.5 h-2.5 rounded-full ${cfg.dot}`} />
        <span className="text-sm font-medium text-gray-800">{flag.name}</span>
      </div>
      <div className="flex items-center gap-3">
        <span className="text-xs text-gray-500">{flag.message}</span>
        <span className={`inline-flex items-center px-2 py-0.5 rounded-full text-[10px] font-semibold ${cfg.bg} ${cfg.text}`}>
          {cfg.label}
        </span>
      </div>
    </div>
  );
};

const ModesConfig = ({ sendCommand, connected }) => {
  const [modes, setModes] = useState(
    MODES.map((m) => ({ ...m, ranges: [] }))
  );

  const [armingFlags, setArmingFlags] = useState(DEFAULT_ARMING_FLAGS);
  const [saving, setSaving] = useState(false);
  const [loading, setLoading] = useState(false);
  const [statusMessage, setStatusMessage] = useState(null);

  useEffect(() => {
    const handleSerialData = (event, data) => {
      try {
        if (typeof data === 'string' && data.startsWith('{')) {
          const json = JSON.parse(data);

          if (json.modes) {
            setModes((prev) =>
              prev.map((m) => {
                const incoming = json.modes.find((im) => im.id === m.id);
                return incoming ? { ...m, ranges: incoming.ranges || [] } : m;
              })
            );
          }

          if (json.arming_flags) {
            setArmingFlags(
              json.arming_flags.map((f) => ({
                name: f.name,
                status: f.status || 'ok',
                message: f.message || '',
              }))
            );
          }
        }

        if (typeof data === 'string' && data.includes('mode_save_ok')) {
          showStatus('Modes saved to flight controller', 'success');
        }
        if (typeof data === 'string' && data.includes('mode_load_ok')) {
          showStatus('Modes loaded from flight controller', 'success');
        }
      } catch (e) {
        // non-JSON serial traffic is expected
      }
    };

    if (window.electronAPI) {
      window.electronAPI.onSerialData(handleSerialData);
    }

    return () => {
      if (window.electronAPI) {
        window.electronAPI.removeAllListeners('serial-data');
      }
    };
  }, []);

  useEffect(() => {
    if (connected) {
      loadFromFC();
    }
  }, [connected]);

  const showStatus = (msg, type = 'info') => {
    setStatusMessage({ msg, type });
    setTimeout(() => setStatusMessage(null), 3000);
  };

  const saveToFC = async () => {
    if (!connected) {
      showStatus('Not connected to flight controller', 'error');
      return;
    }
    setSaving(true);
    try {
      for (const mode of modes) {
        for (const range of mode.ranges) {
          await sendCommand(`mode set ${mode.id} ${range.channel} ${range.min} ${range.max}`);
        }
      }
      await sendCommand('mode save');
      showStatus('Modes saved to flight controller', 'success');
    } catch (err) {
      showStatus('Failed to save modes', 'error');
    } finally {
      setSaving(false);
    }
  };

  const loadFromFC = async () => {
    if (!connected) {
      showStatus('Not connected to flight controller', 'error');
      return;
    }
    setLoading(true);
    try {
      await sendCommand('mode list json');
      await sendCommand('arming_flags json');
      showStatus('Requested mode configuration from FC', 'info');
    } catch (err) {
      showStatus('Failed to load modes', 'error');
    } finally {
      setLoading(false);
    }
  };

  const resetDefaults = () => {
    setModes(MODES.map((m) => ({ ...m, ranges: [] })));
    showStatus('Modes reset to defaults', 'info');
  };

  const updateMode = (idx, updated) => {
    setModes((prev) => prev.map((m, i) => (i === idx ? updated : m)));
  };

  const anyBlocker = armingFlags.some((f) => f.status === 'error');

  return (
    <div className="p-6 space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-2xl font-bold text-gray-800">Modes &amp; AUX Channels</h1>
          <p className="text-sm text-gray-500 mt-1">
            Assign flight modes to AUX channel ranges. Drag handles to adjust activation windows.
          </p>
        </div>
        <div className={`inline-flex items-center px-3 py-1 rounded-full text-xs font-medium ${
          connected ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'
        }`}>
          <div className={`w-2 h-2 rounded-full mr-2 ${connected ? 'bg-green-500' : 'bg-red-500'}`} />
          {connected ? 'Connected' : 'Disconnected'}
        </div>
      </div>

      {/* Status toast */}
      {statusMessage && (
        <div className={`p-3 rounded-lg text-sm font-medium border ${
          statusMessage.type === 'success' ? 'bg-green-50 text-green-800 border-green-200' :
          statusMessage.type === 'error'   ? 'bg-red-50 text-red-800 border-red-200' :
                                             'bg-blue-50 text-blue-800 border-blue-200'
        }`}>
          {statusMessage.msg}
        </div>
      )}

      {/* Mode rows */}
      <div className="space-y-3">
        {modes.map((mode, idx) => (
          <ModeRow
            key={mode.id}
            mode={mode}
            onChange={(updated) => updateMode(idx, updated)}
          />
        ))}
      </div>

      {/* Arming Prevention Flags */}
      <div className="bg-white rounded-lg shadow p-6 border border-gray-100">
        <div className="flex items-center justify-between mb-4">
          <div className="flex items-center gap-2">
            <h2 className="text-lg font-semibold text-gray-800">Arming Prevention Flags</h2>
            {anyBlocker && (
              <span className="inline-flex items-center px-2 py-0.5 rounded-full text-[10px]
                font-semibold bg-red-100 text-red-800">
                BLOCKED
              </span>
            )}
          </div>
          <button
            onClick={() => sendCommand('arming_flags json')}
            disabled={!connected}
            className="text-xs text-blue-600 hover:text-blue-800 disabled:text-gray-400
              transition-colors font-medium"
          >
            Refresh
          </button>
        </div>
        <div className="space-y-2">
          {armingFlags.map((flag, idx) => (
            <ArmingFlagRow key={idx} flag={flag} />
          ))}
        </div>
      </div>

      {/* Action Buttons */}
      <div className="bg-white rounded-lg shadow p-6 border border-gray-100">
        <h2 className="text-lg font-semibold text-gray-800 mb-4">Actions</h2>
        <div className="flex flex-wrap gap-3">
          <button
            onClick={saveToFC}
            disabled={!connected || saving}
            className="px-5 py-2.5 bg-blue-600 text-white rounded-lg hover:bg-blue-700
              disabled:bg-gray-300 disabled:cursor-not-allowed transition-colors font-medium text-sm"
          >
            {saving ? 'Saving…' : 'Save to FC'}
          </button>
          <button
            onClick={loadFromFC}
            disabled={!connected || loading}
            className="px-5 py-2.5 bg-green-600 text-white rounded-lg hover:bg-green-700
              disabled:bg-gray-300 disabled:cursor-not-allowed transition-colors font-medium text-sm"
          >
            {loading ? 'Loading…' : 'Load from FC'}
          </button>
          <button
            onClick={resetDefaults}
            className="px-5 py-2.5 bg-gray-200 text-gray-700 rounded-lg hover:bg-gray-300
              transition-colors font-medium text-sm"
          >
            Reset to Defaults
          </button>
        </div>
      </div>

      {/* PWM Range Legend */}
      <div className="bg-white rounded-lg shadow p-4 border border-gray-100">
        <h3 className="text-sm font-semibold text-gray-700 mb-3">Channel Color Legend</h3>
        <div className="flex flex-wrap gap-4">
          {AUX_CHANNELS.map((ch, idx) => (
            <div key={ch} className="flex items-center gap-2">
              <div className={`w-4 h-3 rounded ${RANGE_COLORS[idx]}`} />
              <span className="text-xs text-gray-600">{ch}</span>
            </div>
          ))}
        </div>
        <p className="text-[11px] text-gray-400 mt-2">
          Click "Add Range" to create an activation window, then drag the bar or its handles
          to set the PWM range ({PWM_MIN}–{PWM_MAX}).
        </p>
      </div>
    </div>
  );
};

export default ModesConfig;
