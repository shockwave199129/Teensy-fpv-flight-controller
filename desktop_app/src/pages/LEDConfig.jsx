import React, { useState, useEffect } from 'react';

const LEDConfig = ({ sendCommand, connected }) => {
  const [ledPatterns, setLedPatterns] = useState({
    disarmed: { color: '#FF0000', pattern: 'solid' },
    armed: { color: '#00FF00', pattern: 'breathing' },
    gps_lock: { color: '#0000FF', pattern: 'pulse' },
    low_battery: { color: '#FFA500', pattern: 'flash' },
    error: { color: '#FF0000', pattern: 'fast_flash' }
  });

  const [customColor, setCustomColor] = useState('#FFFFFF');
  const [brightness, setBrightness] = useState(50);

  const patterns = [
    { id: 'solid', name: 'Solid' },
    { id: 'breathing', name: 'Breathing' },
    { id: 'pulse', name: 'Pulse' },
    { id: 'flash', name: 'Flash' },
    { id: 'fast_flash', name: 'Fast Flash' },
    { id: 'rainbow', name: 'Rainbow' },
    { id: 'chase', name: 'Chase' }
  ];

  const presetColors = [
    { name: 'Red', color: '#FF0000' },
    { name: 'Green', color: '#00FF00' },
    { name: 'Blue', color: '#0000FF' },
    { name: 'Yellow', color: '#FFFF00' },
    { name: 'Purple', color: '#800080' },
    { name: 'Orange', color: '#FFA500' },
    { name: 'White', color: '#FFFFFF' }
  ];

  const applyLedConfig = async () => {
    if (!connected) return;
    
    try {
      // Apply brightness
      await sendCommand(`led brightness ${brightness}`);
      
      // Apply patterns for different states
      for (const [state, config] of Object.entries(ledPatterns)) {
        await sendCommand(`led pattern ${state} ${config.color} ${config.pattern}`);
      }
      
      console.log('LED configuration applied successfully');
    } catch (error) {
      console.error('Failed to apply LED config:', error);
    }
  };

  const testPattern = async (pattern) => {
    if (!connected) return;
    
    try {
      await sendCommand(`led test ${customColor} ${pattern}`);
    } catch (error) {
      console.error('Failed to test LED pattern:', error);
    }
  };

  const updatePattern = (state, field, value) => {
    setLedPatterns(prev => ({
      ...prev,
      [state]: {
        ...prev[state],
        [field]: value
      }
    }));
  };

  if (!connected) {
    return (
      <div className="p-6">
        <div className="bg-yellow-100 border border-yellow-400 text-yellow-700 px-4 py-3 rounded">
          <strong>Warning:</strong> Please connect to the flight controller to configure LEDs.
        </div>
      </div>
    );
  }

  return (
    <div className="p-6 max-w-6xl mx-auto">
      <div className="mb-6">
        <h1 className="text-3xl font-bold text-gray-900 mb-2">LED Configuration</h1>
        <p className="text-gray-600">
          Configure LED patterns and colors for different flight states and status indicators.
        </p>
      </div>

      {/* Brightness Control */}
      <div className="bg-white rounded-lg shadow-md mb-6">
        <div className="px-6 py-4 border-b border-gray-200">
          <h2 className="text-xl font-semibold">Brightness Control</h2>
        </div>
        <div className="p-6">
          <div className="flex items-center space-x-4">
            <label className="text-sm font-medium text-gray-700 w-20">Brightness:</label>
            <input
              type="range"
              min="0"
              max="100"
              value={brightness}
              onChange={(e) => setBrightness(e.target.value)}
              className="flex-1"
            />
            <span className="text-sm text-gray-600 w-12">{brightness}%</span>
          </div>
        </div>
      </div>

      {/* Status LED Patterns */}
      <div className="bg-white rounded-lg shadow-md mb-6">
        <div className="px-6 py-4 border-b border-gray-200">
          <h2 className="text-xl font-semibold">Status LED Patterns</h2>
        </div>
        <div className="p-6">
          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
            {Object.entries(ledPatterns).map(([state, config]) => (
              <div key={state} className="border border-gray-200 rounded-lg p-4">
                <h3 className="font-semibold text-gray-900 mb-3 capitalize">
                  {state.replace('_', ' ')}
                </h3>
                <div className="space-y-3">
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Color</label>
                    <div className="flex items-center space-x-2">
                      <input
                        type="color"
                        value={config.color}
                        onChange={(e) => updatePattern(state, 'color', e.target.value)}
                        className="w-12 h-10 rounded border border-gray-300"
                      />
                      <input
                        type="text"
                        value={config.color}
                        onChange={(e) => updatePattern(state, 'color', e.target.value)}
                        className="flex-1 px-3 py-2 border border-gray-300 rounded text-sm"
                      />
                    </div>
                  </div>
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Pattern</label>
                    <select
                      value={config.pattern}
                      onChange={(e) => updatePattern(state, 'pattern', e.target.value)}
                      className="w-full px-3 py-2 border border-gray-300 rounded"
                    >
                      {patterns.map(pattern => (
                        <option key={pattern.id} value={pattern.id}>
                          {pattern.name}
                        </option>
                      ))}
                    </select>
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>

      {/* Test Patterns */}
      <div className="bg-white rounded-lg shadow-md mb-6">
        <div className="px-6 py-4 border-b border-gray-200">
          <h2 className="text-xl font-semibold">Test Patterns</h2>
        </div>
        <div className="p-6">
          <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">Test Color</label>
              <div className="flex items-center space-x-2 mb-4">
                <input
                  type="color"
                  value={customColor}
                  onChange={(e) => setCustomColor(e.target.value)}
                  className="w-12 h-10 rounded border border-gray-300"
                />
                <input
                  type="text"
                  value={customColor}
                  onChange={(e) => setCustomColor(e.target.value)}
                  className="flex-1 px-3 py-2 border border-gray-300 rounded"
                />
              </div>
              <div className="grid grid-cols-4 gap-2">
                {presetColors.map(preset => (
                  <button
                    key={preset.name}
                    onClick={() => setCustomColor(preset.color)}
                    className="px-3 py-2 text-xs border border-gray-300 rounded hover:bg-gray-50"
                    style={{ backgroundColor: preset.color, color: preset.color === '#FFFFFF' ? '#000' : '#FFF' }}
                  >
                    {preset.name}
                  </button>
                ))}
              </div>
            </div>
            <div>
              <label className="block text-sm font-medium text-gray-700 mb-2">Test Patterns</label>
              <div className="grid grid-cols-2 gap-2">
                {patterns.map(pattern => (
                  <button
                    key={pattern.id}
                    onClick={() => testPattern(pattern.id)}
                    className="px-4 py-2 bg-blue-600 hover:bg-blue-700 text-white text-sm rounded"
                  >
                    {pattern.name}
                  </button>
                ))}
              </div>
            </div>
          </div>
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
              onClick={applyLedConfig}
              className="bg-green-600 hover:bg-green-700 text-white font-medium py-2 px-6 rounded"
            >
              Apply Configuration
            </button>
            <button
              onClick={() => sendCommand('led off')}
              className="bg-gray-600 hover:bg-gray-700 text-white font-medium py-2 px-6 rounded"
            >
              Turn Off LEDs
            </button>
            <button
              onClick={() => sendCommand('led rainbow')}
              className="bg-purple-600 hover:bg-purple-700 text-white font-medium py-2 px-6 rounded"
            >
              Rainbow Test
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default LEDConfig;
