import React, { useState, useEffect, useCallback } from 'react';

const FEATURE_FLAGS = [
  { key: 'BLACKBOX', label: 'Blackbox', description: 'Data logging to onboard flash' },
  { key: 'GPS', label: 'GPS', description: 'GPS navigation support' },
  { key: 'OSD', label: 'OSD', description: 'On-screen display overlay' },
  { key: 'TELEMETRY', label: 'Telemetry', description: 'Telemetry output to radio' },
  { key: 'MOTOR_STOP', label: 'Motor Stop', description: 'Stop motors when disarmed' },
  { key: 'AIRMODE', label: 'Air Mode', description: 'Full PID authority at zero throttle' },
  { key: 'ANTI_GRAVITY', label: 'Anti-Gravity', description: 'I-term boost on throttle changes' },
  { key: 'DYNAMIC_FILTER', label: 'Dynamic Filter', description: 'Adaptive notch filtering' },
  { key: 'RPM_FILTER', label: 'RPM Filter', description: 'Motor RPM-based noise filtering' },
  { key: 'DUAL_IMU', label: 'Dual IMU', description: 'Dual IMU sensor fusion' },
  { key: 'SENSOR_REDUNDANCY', label: 'Sensor Redundancy', description: 'Redundant sensor failover' },
  { key: 'OPTICAL_FLOW', label: 'Optical Flow', description: 'Optical flow position hold' },
  { key: 'LED_STRIP', label: 'LED Strip', description: 'Addressable LED strip control' },
];

const MIXER_TYPES = [
  { value: 'QUAD_X', label: 'Quad X', motors: 4 },
  { value: 'QUAD_PLUS', label: 'Quad +', motors: 4 },
  { value: 'HEX_X', label: 'Hex X', motors: 6 },
  { value: 'HEX_PLUS', label: 'Hex +', motors: 6 },
  { value: 'OCTO_X', label: 'Octo X', motors: 8 },
  { value: 'CUSTOM', label: 'Custom', motors: 0 },
];

const MIXER_MOTOR_POSITIONS = {
  QUAD_X: [
    { x: 75, y: 25, label: '1', rotation: 'CW' },
    { x: 25, y: 25, label: '2', rotation: 'CCW' },
    { x: 25, y: 75, label: '3', rotation: 'CW' },
    { x: 75, y: 75, label: '4', rotation: 'CCW' },
  ],
  QUAD_PLUS: [
    { x: 50, y: 15, label: '1', rotation: 'CW' },
    { x: 15, y: 50, label: '2', rotation: 'CCW' },
    { x: 50, y: 85, label: '3', rotation: 'CW' },
    { x: 85, y: 50, label: '4', rotation: 'CCW' },
  ],
  HEX_X: [
    { x: 75, y: 20, label: '1', rotation: 'CW' },
    { x: 25, y: 20, label: '2', rotation: 'CCW' },
    { x: 10, y: 50, label: '3', rotation: 'CW' },
    { x: 25, y: 80, label: '4', rotation: 'CCW' },
    { x: 75, y: 80, label: '5', rotation: 'CW' },
    { x: 90, y: 50, label: '6', rotation: 'CCW' },
  ],
  HEX_PLUS: [
    { x: 50, y: 12, label: '1', rotation: 'CW' },
    { x: 15, y: 32, label: '2', rotation: 'CCW' },
    { x: 15, y: 68, label: '3', rotation: 'CW' },
    { x: 50, y: 88, label: '4', rotation: 'CCW' },
    { x: 85, y: 68, label: '5', rotation: 'CW' },
    { x: 85, y: 32, label: '6', rotation: 'CCW' },
  ],
  OCTO_X: [
    { x: 70, y: 15, label: '1', rotation: 'CW' },
    { x: 30, y: 15, label: '2', rotation: 'CCW' },
    { x: 12, y: 35, label: '3', rotation: 'CW' },
    { x: 12, y: 65, label: '4', rotation: 'CCW' },
    { x: 30, y: 85, label: '5', rotation: 'CW' },
    { x: 70, y: 85, label: '6', rotation: 'CCW' },
    { x: 88, y: 65, label: '7', rotation: 'CW' },
    { x: 88, y: 35, label: '8', rotation: 'CCW' },
  ],
  CUSTOM: [],
};

const DEFAULT_MOTOR_ASSIGNMENTS = [
  { motor: 1, pin: 'D2', timer: 'FTM0_CH0', protocol: 'DSHOT600' },
  { motor: 2, pin: 'D3', timer: 'FTM0_CH1', protocol: 'DSHOT600' },
  { motor: 3, pin: 'D4', timer: 'FTM0_CH2', protocol: 'DSHOT600' },
  { motor: 4, pin: 'D5', timer: 'FTM0_CH3', protocol: 'DSHOT600' },
  { motor: 5, pin: 'D6', timer: 'FTM1_CH0', protocol: 'DSHOT600' },
  { motor: 6, pin: 'D7', timer: 'FTM1_CH1', protocol: 'DSHOT600' },
  { motor: 7, pin: 'D8', timer: 'FTM2_CH0', protocol: 'DSHOT600' },
  { motor: 8, pin: 'D9', timer: 'FTM2_CH1', protocol: 'DSHOT600' },
];

const CUBE_FACES = [
  { name: 'Front', color: 'rgba(59,130,246,0.85)', transform: 'rotateY(0deg) translateZ(60px)' },
  { name: 'Back', color: 'rgba(239,68,68,0.85)', transform: 'rotateY(180deg) translateZ(60px)' },
  { name: 'Top', color: 'rgba(34,197,94,0.85)', transform: 'rotateX(90deg) translateZ(60px)' },
  { name: 'Bottom', color: 'rgba(234,179,8,0.85)', transform: 'rotateX(-90deg) translateZ(60px)' },
  { name: 'Left', color: 'rgba(249,115,22,0.85)', transform: 'rotateY(-90deg) translateZ(60px)' },
  { name: 'Right', color: 'rgba(168,85,247,0.85)', transform: 'rotateY(90deg) translateZ(60px)' },
];

const ToggleSwitch = ({ checked, onChange, disabled }) => (
  <button
    onClick={() => !disabled && onChange(!checked)}
    disabled={disabled}
    className={`relative w-11 h-6 rounded-full transition-colors duration-200 ${
      checked ? 'bg-blue-600' : 'bg-gray-300'
    } ${disabled ? 'opacity-50 cursor-not-allowed' : 'cursor-pointer'}`}
  >
    <div className={`absolute top-0.5 left-0.5 w-5 h-5 bg-white rounded-full shadow transition-transform duration-200 ${
      checked ? 'translate-x-5' : 'translate-x-0'
    }`} />
  </button>
);

const MixerDiagram = ({ mixerType }) => {
  const motors = MIXER_MOTOR_POSITIONS[mixerType] || [];

  if (motors.length === 0) {
    return (
      <div className="w-48 h-48 mx-auto bg-gray-100 rounded-lg border-2 border-dashed border-gray-300 flex items-center justify-center">
        <span className="text-sm text-gray-500">Custom layout</span>
      </div>
    );
  }

  const cx = 50, cy = 50;

  return (
    <div className="w-48 h-48 mx-auto relative">
      <svg viewBox="0 0 100 100" className="w-full h-full">
        {motors.map((m) => (
          <line
            key={`arm-${m.label}`}
            x1={cx} y1={cy} x2={m.x} y2={m.y}
            stroke="#9CA3AF" strokeWidth="2"
          />
        ))}
        <rect x={cx - 8} y={cy - 10} width="16" height="20" rx="3" fill="#D1D5DB" stroke="#6B7280" strokeWidth="1" />
        <polygon points={`${cx},${cy - 14} ${cx - 4},${cy - 10} ${cx + 4},${cy - 10}`} fill="#3B82F6" />
        {motors.map((m) => (
          <g key={`motor-${m.label}`}>
            <circle cx={m.x} cy={m.y} r="8" fill={m.rotation === 'CW' ? '#DBEAFE' : '#FEE2E2'} stroke={m.rotation === 'CW' ? '#3B82F6' : '#EF4444'} strokeWidth="1.5" />
            <text x={m.x} y={m.y + 1} textAnchor="middle" dominantBaseline="middle" fontSize="6" fontWeight="bold" fill="#1F2937">{m.label}</text>
            <text x={m.x} y={m.y + 9} textAnchor="middle" fontSize="4" fill="#6B7280">{m.rotation === 'CW' ? '↻' : '↺'}</text>
          </g>
        ))}
        <text x={cx} y="6" textAnchor="middle" fontSize="5" fontWeight="bold" fill="#6B7280">FRONT</text>
      </svg>
    </div>
  );
};

const OrientationCube = ({ roll, pitch, yaw }) => (
  <div className="flex items-center justify-center" style={{ perspective: '500px' }}>
    <div
      className="relative"
      style={{
        width: '120px',
        height: '120px',
        transformStyle: 'preserve-3d',
        transform: `rotateX(${-pitch}deg) rotateY(${yaw}deg) rotateZ(${-roll}deg)`,
        transition: 'transform 0.15s ease-out',
      }}
    >
      {CUBE_FACES.map((face) => (
        <div
          key={face.name}
          className="absolute inset-0 flex items-center justify-center border border-white/40 text-white text-xs font-bold select-none backface-visible"
          style={{
            width: '120px',
            height: '120px',
            backgroundColor: face.color,
            transform: face.transform,
            backfaceVisibility: 'visible',
          }}
        >
          {face.name}
        </div>
      ))}
    </div>
  </div>
);

function BoardConfig({ sendCommand, connected }) {
  const [features, setFeatures] = useState(
    Object.fromEntries(FEATURE_FLAGS.map(f => [f.key, false]))
  );
  const [mixerType, setMixerType] = useState('QUAD_X');
  const [boardOrientation, setBoardOrientation] = useState({ roll: 0, pitch: 0, yaw: 0 });
  const [motorAssignment, setMotorAssignment] = useState(DEFAULT_MOTOR_ASSIGNMENTS);
  const [saving, setSaving] = useState(false);
  const [rebooting, setRebooting] = useState(false);
  const [dirty, setDirty] = useState(false);

  const loadConfig = useCallback(() => {
    if (connected) {
      sendCommand('config get');
    }
  }, [connected, sendCommand]);

  useEffect(() => {
    loadConfig();

    const handleConfigData = (data) => {
      if (data.features) {
        setFeatures(prev => ({ ...prev, ...data.features }));
      }
      if (data.mixer) setMixerType(data.mixer);
      if (data.boardAlignment) setBoardOrientation(data.boardAlignment);
      if (data.motorAssignment) setMotorAssignment(data.motorAssignment);
    };

    window.electronAPI?.on('config-data', handleConfigData);
    return () => {
      window.electronAPI?.off('config-data', handleConfigData);
    };
  }, [loadConfig]);

  const handleFeatureToggle = (key, value) => {
    setFeatures(prev => ({ ...prev, [key]: value }));
    sendCommand(`set feature ${key} ${value ? 'on' : 'off'}`);
    setDirty(true);
  };

  const handleMixerChange = (type) => {
    setMixerType(type);
    sendCommand(`set mixer ${type}`);
    const motorCount = MIXER_TYPES.find(m => m.value === type)?.motors || 4;
    setMotorAssignment(DEFAULT_MOTOR_ASSIGNMENTS.slice(0, Math.max(motorCount, 4)));
    setDirty(true);
  };

  const handleOrientationChange = (axis, value) => {
    const newOrientation = { ...boardOrientation, [axis]: value };
    setBoardOrientation(newOrientation);
    sendCommand(`set board_align ${newOrientation.roll} ${newOrientation.pitch} ${newOrientation.yaw}`);
    setDirty(true);
  };

  const handleSave = async () => {
    if (!connected) return;
    setSaving(true);
    try {
      await sendCommand('config save');
      setDirty(false);
    } finally {
      setTimeout(() => setSaving(false), 1000);
    }
  };

  const handleReboot = async () => {
    if (!connected) return;
    if (!window.confirm('Reboot the flight controller? Unsaved changes will be lost.')) return;
    setRebooting(true);
    await sendCommand('reboot');
    setTimeout(() => setRebooting(false), 5000);
  };

  const activeMixer = MIXER_TYPES.find(m => m.value === mixerType);
  const activeMotorCount = activeMixer?.motors || 0;

  return (
    <div className="max-w-6xl mx-auto space-y-6 p-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold text-gray-900">Board Configuration</h1>
          <p className="text-sm text-gray-500 mt-1">System features, frame geometry, and board alignment</p>
        </div>
        <div className={`px-3 py-1.5 rounded-full text-sm font-medium ${
          connected ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'
        }`}>
          {connected ? 'Connected' : 'Disconnected'}
        </div>
      </div>

      {dirty && (
        <div className="bg-amber-50 border border-amber-300 text-amber-800 px-4 py-2 rounded-lg text-sm flex items-center justify-between">
          <span>You have unsaved configuration changes.</span>
          <button onClick={handleSave} className="font-medium underline hover:no-underline">Save now</button>
        </div>
      )}

      {/* Feature Flags */}
      <div className="bg-white rounded-lg shadow p-6">
        <h2 className="text-xl font-semibold text-gray-900 mb-4">Feature Flags</h2>
        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-x-6 gap-y-3">
          {FEATURE_FLAGS.map(({ key, label, description }) => (
            <div key={key} className="flex items-center justify-between py-2 px-3 rounded-lg hover:bg-gray-50 transition-colors">
              <div className="min-w-0 mr-3">
                <div className="text-sm font-medium text-gray-900">{label}</div>
                <div className="text-xs text-gray-500 truncate">{description}</div>
              </div>
              <ToggleSwitch
                checked={features[key]}
                onChange={(val) => handleFeatureToggle(key, val)}
                disabled={!connected}
              />
            </div>
          ))}
        </div>
      </div>

      {/* Mixer Type + Motor Diagram */}
      <div className="bg-white rounded-lg shadow p-6">
        <h2 className="text-xl font-semibold text-gray-900 mb-4">Mixer / Frame Type</h2>
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-8">
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-2">Frame Geometry</label>
            <select
              value={mixerType}
              onChange={(e) => handleMixerChange(e.target.value)}
              disabled={!connected}
              className="w-full p-2.5 border border-gray-300 rounded-lg text-sm focus:ring-2 focus:ring-blue-500 focus:border-blue-500 disabled:opacity-50"
            >
              {MIXER_TYPES.map(m => (
                <option key={m.value} value={m.value}>{m.label} {m.motors > 0 ? `(${m.motors} motors)` : ''}</option>
              ))}
            </select>
            <div className="mt-4 space-y-2 text-sm text-gray-600">
              <div className="flex justify-between"><span>Motor count:</span><span className="font-mono">{activeMotorCount || '—'}</span></div>
              <div className="flex items-center space-x-2">
                <span className="inline-block w-3 h-3 rounded-full bg-blue-200 border border-blue-500" />
                <span>CW rotation</span>
                <span className="inline-block w-3 h-3 rounded-full bg-red-200 border border-red-500 ml-3" />
                <span>CCW rotation</span>
              </div>
            </div>
          </div>
          <div className="flex flex-col items-center">
            <span className="text-sm font-medium text-gray-700 mb-2">Motor Layout</span>
            <MixerDiagram mixerType={mixerType} />
          </div>
        </div>
      </div>

      {/* Board Orientation */}
      <div className="bg-white rounded-lg shadow p-6">
        <h2 className="text-xl font-semibold text-gray-900 mb-4">Board Orientation</h2>
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-8">
          <div className="space-y-5">
            {[
              { axis: 'roll', label: 'Roll', color: 'blue' },
              { axis: 'pitch', label: 'Pitch', color: 'green' },
              { axis: 'yaw', label: 'Yaw', color: 'purple' },
            ].map(({ axis, label, color }) => (
              <div key={axis}>
                <div className="flex items-center justify-between mb-1">
                  <label className="text-sm font-medium text-gray-700">{label}</label>
                  <span className="text-sm font-mono text-gray-900">{boardOrientation[axis]}°</span>
                </div>
                <input
                  type="range"
                  min="-180"
                  max="180"
                  step="1"
                  value={boardOrientation[axis]}
                  onChange={(e) => handleOrientationChange(axis, parseInt(e.target.value))}
                  disabled={!connected}
                  className={`w-full h-2 rounded-lg appearance-none cursor-pointer accent-${color}-600 disabled:opacity-50`}
                />
                <div className="flex justify-between text-xs text-gray-400 mt-0.5">
                  <span>-180°</span>
                  <button
                    className="text-blue-600 hover:underline"
                    onClick={() => handleOrientationChange(axis, 0)}
                    disabled={!connected}
                  >
                    Reset
                  </button>
                  <span>+180°</span>
                </div>
              </div>
            ))}
            <button
              onClick={() => {
                setBoardOrientation({ roll: 0, pitch: 0, yaw: 0 });
                sendCommand('set board_align 0 0 0');
                setDirty(true);
              }}
              disabled={!connected}
              className="text-sm text-blue-600 hover:text-blue-800 font-medium disabled:opacity-50"
            >
              Reset all to 0°
            </button>
          </div>
          <div className="flex flex-col items-center">
            <span className="text-sm font-medium text-gray-700 mb-4">Live Preview</span>
            <div className="bg-gray-900 rounded-xl p-6 w-52 h-52 flex items-center justify-center">
              <OrientationCube
                roll={boardOrientation.roll}
                pitch={boardOrientation.pitch}
                yaw={boardOrientation.yaw}
              />
            </div>
            <div className="mt-3 flex space-x-4 text-xs text-gray-500">
              <span>R: {boardOrientation.roll}°</span>
              <span>P: {boardOrientation.pitch}°</span>
              <span>Y: {boardOrientation.yaw}°</span>
            </div>
          </div>
        </div>
      </div>

      {/* ESC / Motor Output Assignment */}
      <div className="bg-white rounded-lg shadow p-6">
        <h2 className="text-xl font-semibold text-gray-900 mb-4">ESC / Motor Output Assignment</h2>
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
          <div className="lg:col-span-2 overflow-x-auto">
            <table className="w-full text-sm">
              <thead>
                <tr className="border-b border-gray-200">
                  <th className="text-left py-2 px-3 font-medium text-gray-600">Motor</th>
                  <th className="text-left py-2 px-3 font-medium text-gray-600">Output Pin</th>
                  <th className="text-left py-2 px-3 font-medium text-gray-600">Timer Group</th>
                  <th className="text-left py-2 px-3 font-medium text-gray-600">Protocol</th>
                  <th className="text-left py-2 px-3 font-medium text-gray-600">Status</th>
                </tr>
              </thead>
              <tbody>
                {motorAssignment.map((row, idx) => {
                  const isActive = idx < activeMotorCount || mixerType === 'CUSTOM';
                  return (
                    <tr key={row.motor} className={`border-b border-gray-100 ${!isActive ? 'opacity-40' : ''}`}>
                      <td className="py-2.5 px-3 font-medium text-gray-900">M{row.motor}</td>
                      <td className="py-2.5 px-3 font-mono text-gray-700">{row.pin}</td>
                      <td className="py-2.5 px-3 font-mono text-gray-700">{row.timer}</td>
                      <td className="py-2.5 px-3">
                        <span className="inline-block bg-blue-100 text-blue-800 text-xs font-medium px-2 py-0.5 rounded">
                          {row.protocol}
                        </span>
                      </td>
                      <td className="py-2.5 px-3">
                        {isActive ? (
                          <span className="inline-block w-2 h-2 rounded-full bg-green-500" title="Active" />
                        ) : (
                          <span className="inline-block w-2 h-2 rounded-full bg-gray-300" title="Unused" />
                        )}
                      </td>
                    </tr>
                  );
                })}
              </tbody>
            </table>
          </div>
          <div className="flex flex-col items-center">
            <span className="text-sm font-medium text-gray-700 mb-2">Output Map</span>
            <MixerDiagram mixerType={mixerType} />
          </div>
        </div>
      </div>

      {/* Action Buttons */}
      <div className="flex items-center justify-end space-x-4">
        <button
          onClick={loadConfig}
          disabled={!connected}
          className="px-5 py-2.5 border border-gray-300 rounded-lg text-sm font-medium text-gray-700 hover:bg-gray-50 disabled:opacity-50 transition-colors"
        >
          Reload
        </button>
        <button
          onClick={handleReboot}
          disabled={!connected || rebooting}
          className="px-5 py-2.5 bg-red-600 hover:bg-red-700 disabled:bg-gray-400 text-white rounded-lg text-sm font-medium transition-colors"
        >
          {rebooting ? 'Rebooting…' : 'Reboot FC'}
        </button>
        <button
          onClick={handleSave}
          disabled={!connected || saving}
          className="px-6 py-2.5 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white rounded-lg text-sm font-medium transition-colors"
        >
          {saving ? 'Saving…' : 'Save Configuration'}
        </button>
      </div>
    </div>
  );
}

export default BoardConfig;
