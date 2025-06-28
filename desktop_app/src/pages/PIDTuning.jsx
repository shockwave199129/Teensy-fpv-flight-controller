import React, { useState, useEffect } from 'react';

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
