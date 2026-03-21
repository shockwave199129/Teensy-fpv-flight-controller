import React, { useState, useEffect, useRef, useCallback } from 'react';

const PIDTuning = ({ sendCommand, connected }) => {
  const [pidValues, setPidValues] = useState({
    rate: {
      roll: { kp: 1.0, ki: 0.0, kd: 0.1 },
      pitch: { kp: 1.0, ki: 0.0, kd: 0.1 },
      yaw: { kp: 1.0, ki: 0.0, kd: 0.1 }
    },
    angle: {
      roll: { kp: 2.0, ki: 0.0, kd: 0.0 },
      pitch: { kp: 2.0, ki: 0.0, kd: 0.0 }
    },
    altitude: {
      rate: { kp: 2.0, ki: 0.5, kd: 0.1 },
      position: { kp: 1.0, ki: 0.0, kd: 0.0 }
    }
  });

  const [telemetryData, setTelemetryData] = useState({
    roll: 0,
    pitch: 0,
    yaw: 0,
    altitude: 0,
    armed: false
  });

  const [tuningMode, setTuningMode] = useState('rate');
  const [selectedAxis, setSelectedAxis] = useState('roll');
  const [autoTuneActive, setAutoTuneActive] = useState(false);

  const [activeProfile, setActiveProfile] = useState(1);
  const [profiles, setProfiles] = useState({
    1: { name: 'Default' },
    2: { name: 'Sport' },
    3: { name: 'Cinematic' }
  });

  const [rateSettings, setRateSettings] = useState({
    maxRollRate: 360, maxPitchRate: 360, maxYawRate: 200,
    rollExpo: 0.0, pitchExpo: 0.0, yawExpo: 0.0
  });

  const rateCurveCanvasRef = useRef(null);
  const stepResponseCanvasRef = useRef(null);

  useEffect(() => {
    if (connected) {
      // Request current PID values
      sendCommand('get pid');
    }

    // Listen for serial data
    const handleSerialData = (event, data) => {
      parsePidData(data);
      parseTelemetryData(data);
    };

    window.electronAPI.onSerialData(handleSerialData);

    return () => {
      window.electronAPI.removeAllListeners('serial-data');
    };
  }, [connected]);

  const switchProfile = useCallback((profileNum) => {
    setProfiles(prev => ({
      ...prev,
      [activeProfile]: { ...prev[activeProfile], pidValues: JSON.parse(JSON.stringify(pidValues)) }
    }));

    const target = profiles[profileNum];
    if (target.pidValues) {
      setPidValues(target.pidValues);
    }
    setActiveProfile(profileNum);
    if (connected) {
      sendCommand(`set pid_profile ${profileNum}`);
    }
  }, [activeProfile, pidValues, profiles, connected, sendCommand]);

  const computeRateCurve = useCallback((rcCommand, maxRate, expo) => {
    const rc = rcCommand / 100;
    const rcRate = 1.0;
    return maxRate * (rc * rcRate + rc * rc * rc * expo * (1 - rcRate));
  }, []);

  useEffect(() => {
    const canvas = rateCurveCanvasRef.current;
    if (!canvas || tuningMode !== 'rate') return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;
    const pad = { top: 20, right: 20, bottom: 40, left: 55 };
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;

    ctx.clearRect(0, 0, w, h);

    ctx.fillStyle = '#f9fafb';
    ctx.fillRect(pad.left, pad.top, plotW, plotH);

    ctx.strokeStyle = '#e5e7eb';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 5; i++) {
      const y = pad.top + (plotH / 5) * i;
      ctx.beginPath();
      ctx.moveTo(pad.left, y);
      ctx.lineTo(pad.left + plotW, y);
      ctx.stroke();
    }
    for (let i = 0; i <= 5; i++) {
      const x = pad.left + (plotW / 5) * i;
      ctx.beginPath();
      ctx.moveTo(x, pad.top);
      ctx.lineTo(x, pad.top + plotH);
      ctx.stroke();
    }

    ctx.strokeStyle = '#9ca3af';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(pad.left, pad.top, plotW, plotH);

    const maxDisplayRate = Math.max(rateSettings.maxRollRate, rateSettings.maxPitchRate, rateSettings.maxYawRate, 100);

    ctx.fillStyle = '#6b7280';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    for (let i = 0; i <= 5; i++) {
      ctx.fillText(`${i * 20}%`, pad.left + (plotW / 5) * i, pad.top + plotH + 16);
    }
    ctx.fillText('Stick Input', pad.left + plotW / 2, pad.top + plotH + 34);

    ctx.textAlign = 'right';
    for (let i = 0; i <= 5; i++) {
      const val = Math.round((maxDisplayRate / 5) * (5 - i));
      ctx.fillText(`${val}`, pad.left - 6, pad.top + (plotH / 5) * i + 4);
    }
    ctx.save();
    ctx.translate(14, pad.top + plotH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center';
    ctx.fillText('Rate (deg/s)', 0, 0);
    ctx.restore();

    const curves = [
      { key: 'Roll', color: '#3b82f6', maxRate: rateSettings.maxRollRate, expo: rateSettings.rollExpo },
      { key: 'Pitch', color: '#22c55e', maxRate: rateSettings.maxPitchRate, expo: rateSettings.pitchExpo },
      { key: 'Yaw', color: '#a855f7', maxRate: rateSettings.maxYawRate, expo: rateSettings.yawExpo },
    ];

    curves.forEach(({ color, maxRate, expo }) => {
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (let i = 0; i <= 100; i++) {
        const rate = computeRateCurve(i, maxRate, expo);
        const x = pad.left + (i / 100) * plotW;
        const y = pad.top + plotH - (rate / maxDisplayRate) * plotH;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    });

    const legendX = pad.left + 10;
    const legendY = pad.top + 10;
    curves.forEach(({ key, color }, idx) => {
      const y = legendY + idx * 18;
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(legendX, y);
      ctx.lineTo(legendX + 20, y);
      ctx.stroke();
      ctx.fillStyle = '#374151';
      ctx.font = '12px sans-serif';
      ctx.textAlign = 'left';
      ctx.fillText(key, legendX + 26, y + 4);
    });
  }, [rateSettings, tuningMode, computeRateCurve]);

  useEffect(() => {
    const canvas = stepResponseCanvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;
    const pad = { top: 20, right: 20, bottom: 40, left: 55 };
    const plotW = w - pad.left - pad.right;
    const plotH = h - pad.top - pad.bottom;

    const pid = pidValues.rate[selectedAxis] || pidValues.rate.roll;
    const Kp = pid.kp;
    const Ki = pid.ki;
    const Kd = pid.kd;

    const dt = 0.001;
    const steps = 500;
    const setpoint = 1.0;
    let output = 0;
    let iTerm = 0;
    let prevError = 0;
    const response = [];

    for (let i = 0; i < steps; i++) {
      const error = setpoint - output;
      const pTerm = Kp * error;
      iTerm += Ki * error * dt;
      iTerm = Math.max(-2, Math.min(2, iTerm));
      const dTerm = i === 0 ? 0 : Kd * (error - prevError) / dt;
      output += (pTerm + iTerm + dTerm) * dt;
      output = Math.max(-3, Math.min(3, output));
      prevError = error;
      response.push(output);
    }

    const maxVal = Math.max(setpoint * 1.5, ...response.map(Math.abs));

    ctx.clearRect(0, 0, w, h);

    ctx.fillStyle = '#f9fafb';
    ctx.fillRect(pad.left, pad.top, plotW, plotH);

    ctx.strokeStyle = '#e5e7eb';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = pad.top + (plotH / 4) * i;
      ctx.beginPath();
      ctx.moveTo(pad.left, y);
      ctx.lineTo(pad.left + plotW, y);
      ctx.stroke();
    }

    ctx.strokeStyle = '#9ca3af';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(pad.left, pad.top, plotW, plotH);

    const setpointY = pad.top + plotH - (setpoint / maxVal) * plotH;
    ctx.setLineDash([6, 4]);
    ctx.strokeStyle = '#9ca3af';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(pad.left, setpointY);
    ctx.lineTo(pad.left + plotW, setpointY);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.fillStyle = '#6b7280';
    ctx.font = '10px sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText('setpoint', pad.left + plotW - 50, setpointY - 5);

    ctx.strokeStyle = '#3b82f6';
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < steps; i++) {
      const x = pad.left + (i / steps) * plotW;
      const y = pad.top + plotH - (response[i] / maxVal) * plotH;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    ctx.fillStyle = '#6b7280';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    for (let i = 0; i <= 5; i++) {
      const t = ((steps * dt) / 5) * i;
      ctx.fillText(`${t.toFixed(1)}s`, pad.left + (plotW / 5) * i, pad.top + plotH + 16);
    }
    ctx.fillText('Time (s)', pad.left + plotW / 2, pad.top + plotH + 34);

    ctx.textAlign = 'right';
    for (let i = 0; i <= 4; i++) {
      const val = (maxVal / 4) * (4 - i);
      ctx.fillText(val.toFixed(2), pad.left - 6, pad.top + (plotH / 4) * i + 4);
    }
    ctx.save();
    ctx.translate(14, pad.top + plotH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center';
    ctx.fillText('Response', 0, 0);
    ctx.restore();
  }, [pidValues, selectedAxis]);

  const parsePidData = (data) => {
    // Parse PID values from serial data
    const pidMatch = data.match(/PID\s+(\w+)\s+(\w+):\s+P=([0-9.-]+)\s+I=([0-9.-]+)\s+D=([0-9.-]+)/);
    if (pidMatch) {
      const [, mode, axis, kp, ki, kd] = pidMatch;
      setPidValues(prev => ({
        ...prev,
        [mode.toLowerCase()]: {
          ...prev[mode.toLowerCase()],
          [axis.toLowerCase()]: {
            kp: parseFloat(kp),
            ki: parseFloat(ki),
            kd: parseFloat(kd)
          }
        }
      }));
    }
  };

  const parseTelemetryData = (data) => {
    // Parse telemetry data for real-time display
    if (data.includes('"telemetry":')) {
      try {
        const telemetry = JSON.parse(data).telemetry;
        setTelemetryData(telemetry);
      } catch (error) {
        // Ignore parsing errors
      }
    }
  };

  const updatePidValue = (mode, axis, param, value) => {
    const numValue = parseFloat(value) || 0;
    setPidValues(prev => ({
      ...prev,
      [mode]: {
        ...prev[mode],
        [axis]: {
          ...prev[mode][axis],
          [param]: numValue
        }
      }
    }));
  };

  const applyPidValues = async (mode, axis) => {
    if (!connected) return;

    try {
      const pid = pidValues[mode][axis];
      await sendCommand(`set pid ${mode} ${axis} ${pid.kp} ${pid.ki} ${pid.kd}`);
      console.log(`Applied PID values for ${mode} ${axis}:`, pid);
    } catch (error) {
      console.error('Failed to apply PID values:', error);
    }
  };

  const applyAllPidValues = async () => {
    if (!connected) return;

    try {
      // Apply all rate PID values
      for (const axis of ['roll', 'pitch', 'yaw']) {
        await applyPidValues('rate', axis);
        await new Promise(resolve => setTimeout(resolve, 100)); // Small delay
      }

      // Apply angle PID values
      for (const axis of ['roll', 'pitch']) {
        await applyPidValues('angle', axis);
        await new Promise(resolve => setTimeout(resolve, 100));
      }

      // Apply altitude PID values
      await applyPidValues('altitude', 'rate');
      await applyPidValues('altitude', 'position');

      console.log('All PID values applied successfully');
    } catch (error) {
      console.error('Failed to apply all PID values:', error);
    }
  };

  const resetPidValues = async () => {
    if (!connected) return;

    try {
      await sendCommand('reset pid');
      // Reload default values
      await sendCommand('get pid');
    } catch (error) {
      console.error('Failed to reset PID values:', error);
    }
  };

  const startAutoTune = async (axis) => {
    if (!connected) return;

    try {
      setAutoTuneActive(true);
      await sendCommand(`autotune ${axis}`);
    } catch (error) {
      console.error('Failed to start auto-tune:', error);
      setAutoTuneActive(false);
    }
  };

  const stopAutoTune = async () => {
    if (!connected) return;

    try {
      await sendCommand('autotune stop');
      setAutoTuneActive(false);
    } catch (error) {
      console.error('Failed to stop auto-tune:', error);
    }
  };

  const PidSlider = ({ label, value, onChange, min = 0, max = 10, step = 0.1 }) => (
    <div className="space-y-2">
      <div className="flex justify-between items-center">
        <label className="text-sm font-medium text-gray-700">{label}</label>
        <span className="text-sm text-gray-600">{value.toFixed(3)}</span>
      </div>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        className="w-full"
      />
      <input
        type="number"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value) || 0)}
        className="w-full px-3 py-1 border border-gray-300 rounded text-sm"
      />
    </div>
  );

  const AxisPidPanel = ({ mode, axis, pid }) => (
    <div className="border border-gray-200 rounded-lg p-4">
      <div className="flex justify-between items-center mb-4">
        <h3 className="font-semibold text-gray-900 capitalize">{axis} {mode}</h3>
        <button
          onClick={() => applyPidValues(mode, axis)}
          className="bg-blue-600 hover:bg-blue-700 text-white text-sm px-3 py-1 rounded"
        >
          Apply
        </button>
      </div>
      <div className="space-y-4">
        <PidSlider
          label="P (Proportional)"
          value={pid.kp}
          onChange={(value) => updatePidValue(mode, axis, 'kp', value)}
          max={mode === 'rate' ? 5 : 10}
        />
        <PidSlider
          label="I (Integral)"
          value={pid.ki}
          onChange={(value) => updatePidValue(mode, axis, 'ki', value)}
          max={2}
        />
        <PidSlider
          label="D (Derivative)"
          value={pid.kd}
          onChange={(value) => updatePidValue(mode, axis, 'kd', value)}
          max={1}
        />
      </div>
    </div>
  );

  if (!connected) {
    return (
      <div className="p-6">
        <div className="bg-yellow-100 border border-yellow-400 text-yellow-700 px-4 py-3 rounded">
          <strong>Warning:</strong> Please connect to the flight controller to tune PID values.
        </div>
      </div>
    );
  }

  return (
    <div className="p-6 max-w-7xl mx-auto">
      <div className="mb-6">
        <h1 className="text-3xl font-bold text-gray-900 mb-2">PID Tuning</h1>
        <p className="text-gray-600">
          Fine-tune the PID controllers for optimal flight performance. Start with rate PIDs, then angle PIDs.
        </p>
      </div>

      {/* Safety Warning */}
      <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-6">
        <div className="flex items-center">
          <span className="text-2xl mr-3">⚠️</span>
          <div>
            <strong>SAFETY WARNING:</strong> PID tuning affects flight stability.
            <br />
            Always test changes carefully and be prepared for unstable flight behavior.
          </div>
        </div>
      </div>

      {/* PID Profile Switching */}
      <div className="bg-white rounded-lg shadow-md mb-6">
        <div className="px-6 py-4 border-b border-gray-200">
          <h2 className="text-xl font-semibold">PID Profile</h2>
          <p className="text-sm text-gray-600 mt-1">
            Switch between profiles for different flight styles. Values are saved per profile.
          </p>
        </div>
        <div className="p-6">
          <div className="flex space-x-2">
            {[1, 2, 3].map(num => (
              <button
                key={num}
                onClick={() => switchProfile(num)}
                className={`flex-1 py-2 px-4 rounded-t-lg font-medium border-b-2 transition-colors ${
                  activeProfile === num
                    ? 'bg-blue-50 text-blue-700 border-blue-600'
                    : 'bg-gray-50 text-gray-600 border-transparent hover:bg-gray-100 hover:text-gray-800'
                }`}
              >
                <div className="text-sm font-semibold">Profile {num}</div>
                <div className="text-xs opacity-75">{profiles[num].name}</div>
              </button>
            ))}
          </div>
          <div className="mt-3 text-sm text-gray-500">
            Active: <span className="font-medium text-gray-700">Profile {activeProfile} — {profiles[activeProfile].name}</span>
          </div>
        </div>
      </div>

      {/* Telemetry Display */}
      <div className="bg-white rounded-lg shadow-md mb-6">
        <div className="px-6 py-4 border-b border-gray-200">
          <h2 className="text-xl font-semibold">Real-time Telemetry</h2>
        </div>
        <div className="p-6">
          <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
            <div className="text-center">
              <div className="text-2xl font-bold text-blue-600">{telemetryData.roll?.toFixed(1) || '0.0'}°</div>
              <div className="text-sm text-gray-600">Roll</div>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-green-600">{telemetryData.pitch?.toFixed(1) || '0.0'}°</div>
              <div className="text-sm text-gray-600">Pitch</div>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-purple-600">{telemetryData.yaw?.toFixed(1) || '0.0'}°</div>
              <div className="text-sm text-gray-600">Yaw</div>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-orange-600">{telemetryData.altitude?.toFixed(1) || '0.0'}m</div>
              <div className="text-sm text-gray-600">Altitude</div>
            </div>
          </div>
          <div className="mt-4 text-center">
            <span className={`inline-flex items-center px-3 py-1 rounded-full text-sm font-medium ${
              telemetryData.armed ? 'bg-red-100 text-red-800' : 'bg-green-100 text-green-800'
            }`}>
              {telemetryData.armed ? '🔴 ARMED' : '🟢 DISARMED'}
            </span>
          </div>
        </div>
      </div>

      {/* PID Mode Selection */}
      <div className="bg-white rounded-lg shadow-md mb-6">
        <div className="px-6 py-4 border-b border-gray-200">
          <h2 className="text-xl font-semibold">PID Controller Selection</h2>
        </div>
        <div className="p-6">
          <div className="flex space-x-4">
            <button
              onClick={() => setTuningMode('rate')}
              className={`px-4 py-2 rounded font-medium ${
                tuningMode === 'rate' 
                  ? 'bg-blue-600 text-white' 
                  : 'bg-gray-200 text-gray-700 hover:bg-gray-300'
              }`}
            >
              Rate Control (Inner Loop)
            </button>
            <button
              onClick={() => setTuningMode('angle')}
              className={`px-4 py-2 rounded font-medium ${
                tuningMode === 'angle' 
                  ? 'bg-blue-600 text-white' 
                  : 'bg-gray-200 text-gray-700 hover:bg-gray-300'
              }`}
            >
              Angle Control (Outer Loop)
            </button>
            <button
              onClick={() => setTuningMode('altitude')}
              className={`px-4 py-2 rounded font-medium ${
                tuningMode === 'altitude' 
                  ? 'bg-blue-600 text-white' 
                  : 'bg-gray-200 text-gray-700 hover:bg-gray-300'
              }`}
            >
              Altitude Control
            </button>
          </div>
        </div>
      </div>

      {/* Rate Control PIDs */}
      {tuningMode === 'rate' && (
        <div className="bg-white rounded-lg shadow-md mb-6">
          <div className="px-6 py-4 border-b border-gray-200">
            <h2 className="text-xl font-semibold">Rate Control PIDs</h2>
            <p className="text-sm text-gray-600 mt-1">
              Controls the rate of rotation (degrees/second). Tune these first for stability.
            </p>
          </div>
          <div className="p-6">
            <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
              {['roll', 'pitch', 'yaw'].map(axis => (
                <AxisPidPanel
                  key={axis}
                  mode="rate"
                  axis={axis}
                  pid={pidValues.rate[axis]}
                />
              ))}
            </div>
          </div>
        </div>
      )}

      {/* Rate Curve Visualizer */}
      {tuningMode === 'rate' && (
        <div className="bg-white rounded-lg shadow-md mb-6">
          <div className="px-6 py-4 border-b border-gray-200">
            <h2 className="text-xl font-semibold">Rate Curve Preview</h2>
            <p className="text-sm text-gray-600 mt-1">
              Betaflight-style rate curves showing stick-to-rate mapping for each axis.
            </p>
          </div>
          <div className="p-6">
            <div className="flex justify-center mb-6">
              <canvas
                ref={rateCurveCanvasRef}
                width={500}
                height={300}
                className="border border-gray-200 rounded"
              />
            </div>
            <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
              <div className="space-y-4">
                <h3 className="font-semibold text-blue-600">Roll</h3>
                <PidSlider
                  label="Max Rate (deg/s)"
                  value={rateSettings.maxRollRate}
                  onChange={(v) => setRateSettings(prev => ({ ...prev, maxRollRate: v }))}
                  min={100} max={1000} step={10}
                />
                <PidSlider
                  label="Expo"
                  value={rateSettings.rollExpo}
                  onChange={(v) => setRateSettings(prev => ({ ...prev, rollExpo: v }))}
                  min={0} max={1} step={0.01}
                />
              </div>
              <div className="space-y-4">
                <h3 className="font-semibold text-green-600">Pitch</h3>
                <PidSlider
                  label="Max Rate (deg/s)"
                  value={rateSettings.maxPitchRate}
                  onChange={(v) => setRateSettings(prev => ({ ...prev, maxPitchRate: v }))}
                  min={100} max={1000} step={10}
                />
                <PidSlider
                  label="Expo"
                  value={rateSettings.pitchExpo}
                  onChange={(v) => setRateSettings(prev => ({ ...prev, pitchExpo: v }))}
                  min={0} max={1} step={0.01}
                />
              </div>
              <div className="space-y-4">
                <h3 className="font-semibold text-purple-600">Yaw</h3>
                <PidSlider
                  label="Max Rate (deg/s)"
                  value={rateSettings.maxYawRate}
                  onChange={(v) => setRateSettings(prev => ({ ...prev, maxYawRate: v }))}
                  min={100} max={500} step={10}
                />
                <PidSlider
                  label="Expo"
                  value={rateSettings.yawExpo}
                  onChange={(v) => setRateSettings(prev => ({ ...prev, yawExpo: v }))}
                  min={0} max={1} step={0.01}
                />
              </div>
            </div>
          </div>
        </div>
      )}

      {/* PID Step Response Graph */}
      <div className="bg-white rounded-lg shadow-md mb-6">
        <div className="px-6 py-4 border-b border-gray-200">
          <h2 className="text-xl font-semibold">PID Step Response</h2>
          <p className="text-sm text-gray-600 mt-1">
            Simulated step response for the selected axis ({selectedAxis}). Shows how the PID controller would respond to a unit step input.
          </p>
        </div>
        <div className="p-6">
          <div className="flex space-x-2 mb-4">
            {['roll', 'pitch', 'yaw'].map(axis => (
              <button
                key={axis}
                onClick={() => setSelectedAxis(axis)}
                className={`px-3 py-1 rounded text-sm font-medium capitalize ${
                  selectedAxis === axis
                    ? 'bg-blue-600 text-white'
                    : 'bg-gray-200 text-gray-700 hover:bg-gray-300'
                }`}
              >
                {axis}
              </button>
            ))}
          </div>
          <div className="flex justify-center">
            <canvas
              ref={stepResponseCanvasRef}
              width={500}
              height={250}
              className="border border-gray-200 rounded"
            />
          </div>
          <div className="mt-3 text-xs text-gray-500 text-center">
            P={pidValues.rate[selectedAxis]?.kp.toFixed(2)} | I={pidValues.rate[selectedAxis]?.ki.toFixed(2)} | D={pidValues.rate[selectedAxis]?.kd.toFixed(3)}
          </div>
        </div>
      </div>

      {/* Angle Control PIDs */}
      {tuningMode === 'angle' && (
        <div className="bg-white rounded-lg shadow-md mb-6">
          <div className="px-6 py-4 border-b border-gray-200">
            <h2 className="text-xl font-semibold">Angle Control PIDs</h2>
            <p className="text-sm text-gray-600 mt-1">
              Controls the angle (degrees). Tune after rate PIDs are stable.
            </p>
          </div>
          <div className="p-6">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              {['roll', 'pitch'].map(axis => (
                <AxisPidPanel
                  key={axis}
                  mode="angle"
                  axis={axis}
                  pid={pidValues.angle[axis]}
                />
              ))}
            </div>
          </div>
        </div>
      )}

      {/* Altitude Control PIDs */}
      {tuningMode === 'altitude' && (
        <div className="bg-white rounded-lg shadow-md mb-6">
          <div className="px-6 py-4 border-b border-gray-200">
            <h2 className="text-xl font-semibold">Altitude Control PIDs</h2>
            <p className="text-sm text-gray-600 mt-1">
              Controls altitude hold and vertical velocity.
            </p>
          </div>
          <div className="p-6">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <AxisPidPanel
                mode="altitude"
                axis="rate"
                pid={pidValues.altitude.rate}
              />
              <AxisPidPanel
                mode="altitude"
                axis="position"
                pid={pidValues.altitude.position}
              />
            </div>
          </div>
        </div>
      )}

      {/* Auto-tune Section */}
      <div className="bg-white rounded-lg shadow-md mb-6">
        <div className="px-6 py-4 border-b border-gray-200">
          <h2 className="text-xl font-semibold">Auto-tune</h2>
          <p className="text-sm text-gray-600 mt-1">
            Automatically tune PID values through flight testing.
          </p>
        </div>
        <div className="p-6">
          <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
            {['roll', 'pitch', 'yaw'].map(axis => (
              <button
                key={axis}
                onClick={() => startAutoTune(axis)}
                disabled={autoTuneActive}
                className="bg-orange-600 hover:bg-orange-700 disabled:bg-gray-400 text-white font-medium py-2 px-4 rounded capitalize"
              >
                Auto-tune {axis}
              </button>
            ))}
          </div>
          {autoTuneActive && (
            <div className="mt-4 p-4 bg-orange-100 border border-orange-300 rounded">
              <div className="flex justify-between items-center">
                <span className="text-orange-800">Auto-tune in progress...</span>
                <button
                  onClick={stopAutoTune}
                  className="bg-red-600 hover:bg-red-700 text-white text-sm px-3 py-1 rounded"
                >
                  Stop Auto-tune
                </button>
              </div>
            </div>
          )}
        </div>
      </div>

      {/* Actions */}
      <div className="bg-white rounded-lg shadow-md">
        <div className="px-6 py-4 border-b border-gray-200">
          <h2 className="text-xl font-semibold">Actions</h2>
        </div>
        <div className="p-6">
          <div className="flex space-x-4">
            <button
              onClick={applyAllPidValues}
              className="bg-green-600 hover:bg-green-700 text-white font-medium py-2 px-6 rounded"
            >
              Apply All PID Values
            </button>
            <button
              onClick={resetPidValues}
              className="bg-red-600 hover:bg-red-700 text-white font-medium py-2 px-6 rounded"
            >
              Reset to Defaults
            </button>
            <button
              onClick={() => sendCommand('get pid')}
              className="bg-blue-600 hover:bg-blue-700 text-white font-medium py-2 px-6 rounded"
            >
              Refresh Values
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default PIDTuning;
