import React, { useState, useEffect } from 'react';

// Channel function definitions matching firmware
const CHANNEL_FUNCTIONS = {
  CHAN_FUNC_THROTTLE: 0,
  CHAN_FUNC_ROLL: 1,
  CHAN_FUNC_PITCH: 2,
  CHAN_FUNC_YAW: 3,
  CHAN_FUNC_ARM_DISARM: 4,
  CHAN_FUNC_FLIGHT_MODE: 5,
  CHAN_FUNC_RTH: 6,
  CHAN_FUNC_ALTITUDE_HOLD: 7,
  CHAN_FUNC_POSITION_HOLD: 8,
  CHAN_FUNC_HEADLESS_MODE: 9,
  CHAN_FUNC_TURTLE_MODE: 10,
  CHAN_FUNC_BEEPER: 11,
  CHAN_FUNC_LED_CONTROL: 12,
  CHAN_FUNC_CAMERA_TILT: 13,
  CHAN_FUNC_RATE_PROFILE: 14,
  CHAN_FUNC_GPS_RESCUE: 15,
  CHAN_FUNC_LAUNCH_ASSIST: 16,
  CHAN_FUNC_BLACKBOX: 17,
  CHAN_FUNC_NONE: 18
};

const FUNCTION_NAMES = {
  [CHANNEL_FUNCTIONS.CHAN_FUNC_THROTTLE]: 'Throttle',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_ROLL]: 'Roll',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_PITCH]: 'Pitch',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_YAW]: 'Yaw',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_ARM_DISARM]: 'Arm/Disarm',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_FLIGHT_MODE]: 'Flight Mode',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_RTH]: 'Return to Home',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_ALTITUDE_HOLD]: 'Altitude Hold',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_POSITION_HOLD]: 'Position Hold',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_HEADLESS_MODE]: 'Headless Mode',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_TURTLE_MODE]: 'Turtle Mode',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_BEEPER]: 'Beeper',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_LED_CONTROL]: 'LED Control',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_CAMERA_TILT]: 'Camera Tilt',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_RATE_PROFILE]: 'Rate Profile',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_GPS_RESCUE]: 'GPS Rescue',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_LAUNCH_ASSIST]: 'Launch Assist',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_BLACKBOX]: 'Blackbox',
  [CHANNEL_FUNCTIONS.CHAN_FUNC_NONE]: 'None'
};

const RC_PROTOCOLS = {
  RC_PROTOCOL_PPM: 0,
  RC_PROTOCOL_IBUS: 1,
  RC_PROTOCOL_SBUS: 2,
  RC_PROTOCOL_ELRS: 3
};

const PROTOCOL_NAMES = {
  [RC_PROTOCOLS.RC_PROTOCOL_PPM]: 'PPM',
  [RC_PROTOCOLS.RC_PROTOCOL_IBUS]: 'iBUS',
  [RC_PROTOCOLS.RC_PROTOCOL_SBUS]: 'SBUS',
  [RC_PROTOCOLS.RC_PROTOCOL_ELRS]: 'ExpressLRS'
};

// Protocol-specific channel limits and characteristics
const PROTOCOL_INFO = {
  [RC_PROTOCOLS.RC_PROTOCOL_PPM]: {
    name: 'PPM',
    maxChannels: 8,
    minChannels: 4,
    description: 'Pulse Position Modulation - Simple, reliable, limited channels',
    updateRate: '22-50Hz',
    resolution: '1μs',
    latency: '22ms',
    advantages: ['Simple wiring', 'Wide compatibility', 'Reliable'],
    limitations: ['Limited channels', 'Higher latency', 'Susceptible to noise']
  },
  [RC_PROTOCOLS.RC_PROTOCOL_IBUS]: {
    name: 'iBUS',
    maxChannels: 14,
    minChannels: 4,
    description: 'FlySky digital protocol with telemetry support',
    updateRate: '50Hz',
    resolution: '1μs',
    latency: '20ms',
    advantages: ['More channels', 'Digital protocol', 'Telemetry support', 'Error checking'],
    limitations: ['FlySky specific', 'Moderate latency']
  },
  [RC_PROTOCOLS.RC_PROTOCOL_SBUS]: {
    name: 'SBUS',
    maxChannels: 16,
    minChannels: 4,
    description: 'FrSky/Futaba digital protocol - industry standard',
    updateRate: '100Hz',
    resolution: '0.5μs',
    latency: '10ms',
    advantages: ['High channel count', 'Low latency', 'Industry standard', 'Precise'],
    limitations: ['Inverted signal', 'Requires converter for some RX']
  },
  [RC_PROTOCOLS.RC_PROTOCOL_ELRS]: {
    name: 'ExpressLRS',
    maxChannels: 16,
    minChannels: 4,
    description: 'Ultra-low latency open-source protocol',
    updateRate: '50-1000Hz',
    resolution: '0.1μs',
    latency: '1-10ms',
    advantages: ['Extremely low latency', 'Long range', 'Open source', 'High precision'],
    limitations: ['Newer protocol', 'Requires compatible hardware']
  }
};

function ReceiverConfig({ config, setConfig, sendCommand, connected }) {
  const [rcConfig, setRcConfig] = useState({
    protocol: RC_PROTOCOLS.RC_PROTOCOL_PPM,
    channelCount: 8,
    channels: Array(16).fill().map((_, i) => ({
      channelNumber: 0,
      function: CHANNEL_FUNCTIONS.CHAN_FUNC_NONE,
      reversed: false,
      minValue: 1000,
      centerValue: 1500,
      maxValue: 2000,
      deadband: 10,
      isSwitch: false,
      switchPositions: 2,
      switchThresholds: [1300, 1700]
    })),
    failsafeEnabled: true,
    failsafeThrottle: 1000,
    failsafeMode: 'STABILIZE',
    rthOnFailsafe: false,
    stickArmingEnabled: true,
    switchArmingEnabled: true,
    armChannel: 5,
    armSequenceTimeMs: 2000
  });

  const [rateProfiles, setRateProfiles] = useState([
    {
      name: 'Beginner',
      maxRollRate: 200,
      maxPitchRate: 200,
      maxYawRate: 180,
      rollExpo: 0.0,
      pitchExpo: 0.0,
      yawExpo: 0.0,
      rcSmoothingFactor: 0.7,
      throttleExpo: 0.0,
      throttleMid: 0.5
    },
    {
      name: 'Sport',
      maxRollRate: 400,
      maxPitchRate: 400,
      maxYawRate: 360,
      rollExpo: 0.3,
      pitchExpo: 0.3,
      yawExpo: 0.2,
      rcSmoothingFactor: 0.3,
      throttleExpo: 0.2,
      throttleMid: 0.5
    },
    {
      name: 'Acro',
      maxRollRate: 800,
      maxPitchRate: 800,
      maxYawRate: 720,
      rollExpo: 0.5,
      pitchExpo: 0.5,
      yawExpo: 0.4,
      rcSmoothingFactor: 0.1,
      throttleExpo: 0.3,
      throttleMid: 0.4
    }
  ]);

  const [currentRateProfile, setCurrentRateProfile] = useState(0);
  const [channelData, setChannelData] = useState(Array(16).fill(1500));
  const [channelStats, setChannelStats] = useState(Array(16).fill().map(() => ({
    min: 1500,
    max: 1500,
    jitter: 0,
    updateCount: 0,
    lastUpdate: 0,
    health: 'unknown'
  })));
  const [isMonitoring, setIsMonitoring] = useState(false);
  const [selectedTab, setSelectedTab] = useState('mapping');
  const [rcSignalStats, setRcSignalStats] = useState({
    rssi: 0,
    linkQuality: 0,
    packetsReceived: 0,
    packetsLost: 0,
    frameRate: 0
  });

  // Get protocol-specific info
  const currentProtocolInfo = PROTOCOL_INFO[rcConfig.protocol];
  const maxChannelsForProtocol = currentProtocolInfo.maxChannels;

  // Initialize default channel mapping
  useEffect(() => {
    initializeDefaultMapping();
  }, []);

  // Monitor RC channels when enabled
  useEffect(() => {
    let interval;
    if (isMonitoring && connected) {
      interval = setInterval(() => {
        sendCommand('get_rc_data');
      }, 50); // 20Hz monitoring rate
    }
    return () => clearInterval(interval);
  }, [isMonitoring, connected, sendCommand]);

  // Listen for RC data updates from the flight controller
  useEffect(() => {
    const handleRcData = (event, data) => {
      try {
        const rcData = JSON.parse(data);
        if (rcData.type === 'rc_data') {
          updateChannelData(rcData.channels);
          setRcSignalStats(rcData.stats || {});
        }
      } catch (e) {
        // Handle non-JSON or other data
      }
    };

    if (window.electronAPI) {
      window.electronAPI.onSerialData(handleRcData);
    }

    return () => {
      if (window.electronAPI) {
        window.electronAPI.removeAllListeners('serial-data');
      }
    };
  }, []);

  const updateChannelData = (newChannelData) => {
    const now = Date.now();
    setChannelData(newChannelData);
    
    // Update channel statistics
    setChannelStats(prev => prev.map((stat, index) => {
      const value = newChannelData[index] || 1500;
      const timeDiff = now - stat.lastUpdate;
      
      return {
        min: Math.min(stat.min, value),
        max: Math.max(stat.max, value),
        jitter: timeDiff > 0 ? Math.abs(value - (channelData[index] || 1500)) : stat.jitter,
        updateCount: stat.updateCount + 1,
        lastUpdate: now,
        health: getChannelHealth(value, timeDiff, stat.updateCount)
      };
    }));
  };

  const getChannelHealth = (value, timeDiff, updateCount) => {
    if (updateCount < 5) return 'unknown';
    if (value < 900 || value > 2100) return 'error';
    if (timeDiff > 100) return 'timeout';
    if (value >= 1000 && value <= 2000) return 'good';
    return 'warning';
  };

  const initializeDefaultMapping = () => {
    const defaultChannels = [...rcConfig.channels];
    
    // Standard AETR mapping
    defaultChannels[0] = {
      ...defaultChannels[0],
      channelNumber: 1,
      function: CHANNEL_FUNCTIONS.CHAN_FUNC_ROLL
    };
    defaultChannels[1] = {
      ...defaultChannels[1],
      channelNumber: 2,
      function: CHANNEL_FUNCTIONS.CHAN_FUNC_PITCH,
      reversed: true
    };
    defaultChannels[2] = {
      ...defaultChannels[2],
      channelNumber: 3,
      function: CHANNEL_FUNCTIONS.CHAN_FUNC_THROTTLE,
      centerValue: 1000
    };
    defaultChannels[3] = {
      ...defaultChannels[3],
      channelNumber: 4,
      function: CHANNEL_FUNCTIONS.CHAN_FUNC_YAW
    };
    defaultChannels[4] = {
      ...defaultChannels[4],
      channelNumber: 5,
      function: CHANNEL_FUNCTIONS.CHAN_FUNC_ARM_DISARM,
      isSwitch: true,
      switchPositions: 2
    };
    defaultChannels[5] = {
      ...defaultChannels[5],
      channelNumber: 6,
      function: CHANNEL_FUNCTIONS.CHAN_FUNC_FLIGHT_MODE,
      isSwitch: true,
      switchPositions: 3
    };
    defaultChannels[6] = {
      ...defaultChannels[6],
      channelNumber: 7,
      function: CHANNEL_FUNCTIONS.CHAN_FUNC_RTH,
      isSwitch: true,
      switchPositions: 2
    };
    defaultChannels[7] = {
      ...defaultChannels[7],
      channelNumber: 8,
      function: CHANNEL_FUNCTIONS.CHAN_FUNC_BEEPER,
      isSwitch: true,
      switchPositions: 2
    };

    setRcConfig(prev => ({ ...prev, channels: defaultChannels }));
  };

  const updateChannelMapping = (index, field, value) => {
    const newChannels = [...rcConfig.channels];
    newChannels[index] = { ...newChannels[index], [field]: value };
    setRcConfig(prev => ({ ...prev, channels: newChannels }));
  };

  const updateRateProfile = (profileIndex, field, value) => {
    const newProfiles = [...rateProfiles];
    newProfiles[profileIndex] = { ...newProfiles[profileIndex], [field]: value };
    setRateProfiles(newProfiles);
  };

  const handleProtocolChange = (newProtocol) => {
    const protocolInfo = PROTOCOL_INFO[newProtocol];
    setRcConfig(prev => ({
      ...prev,
      protocol: newProtocol,
      channelCount: Math.min(prev.channelCount, protocolInfo.maxChannels)
    }));
  };

  const applyRcConfig = () => {
    if (!connected) return;

    // Send protocol change
    sendCommand(`set rc protocol ${rcConfig.protocol}`);
    
    // Send channel mappings
    rcConfig.channels.forEach((channel, index) => {
      if (channel.channelNumber > 0 && channel.function !== CHANNEL_FUNCTIONS.CHAN_FUNC_NONE) {
        sendCommand(`set channel ${channel.channelNumber} function ${channel.function} ${channel.reversed ? 'reversed' : 'normal'}`);
        if (channel.isSwitch) {
          sendCommand(`set channel ${channel.channelNumber} switch ${channel.switchPositions} ${channel.switchThresholds[0]} ${channel.switchThresholds[1]}`);
        }
      }
    });

    // Send failsafe config
    sendCommand(`set failsafe enabled ${rcConfig.failsafeEnabled}`);
    sendCommand(`set failsafe throttle ${rcConfig.failsafeThrottle}`);
    sendCommand(`set failsafe mode ${rcConfig.failsafeMode}`);
    sendCommand(`set failsafe rth ${rcConfig.rthOnFailsafe}`);

    // Send rate profile
    const profile = rateProfiles[currentRateProfile];
    sendCommand(`set rate profile ${currentRateProfile}`);
    Object.keys(profile).forEach(key => {
      if (key !== 'name') {
        sendCommand(`set rate ${key} ${profile[key]}`);
      }
    });
  };

  const resetToDefaults = () => {
    initializeDefaultMapping();
    setCurrentRateProfile(0);
  };

  const getChannelBar = (value) => {
    const percentage = ((value - 1000) / 1000) * 100;
    return Math.max(0, Math.min(100, percentage));
  };

  const getSwitchPosition = (value, positions) => {
    if (positions === 2) {
      return value < 1300 ? 'LOW' : 'HIGH';
    } else {
      if (value < 1300) return 'LOW';
      if (value < 1700) return 'MID';
      return 'HIGH';
    }
  };

  const getHealthColor = (health) => {
    switch (health) {
      case 'good': return 'text-green-600';
      case 'warning': return 'text-yellow-600';
      case 'error': return 'text-red-600';
      case 'timeout': return 'text-orange-600';
      default: return 'text-gray-600';
    }
  };

  const getHealthIcon = (health) => {
    switch (health) {
      case 'good': return '✅';
      case 'warning': return '⚠️';
      case 'error': return '❌';
      case 'timeout': return '⏰';
      default: return '❓';
    }
  };

  const renderProtocolInfo = () => (
    <div className="card mb-6">
      <h3 className="text-lg font-semibold mb-4">Protocol Information</h3>
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        <div>
          <div className="bg-blue-50 border border-blue-200 rounded-lg p-4">
            <h4 className="font-semibold text-blue-800 mb-2">{currentProtocolInfo.name}</h4>
            <p className="text-sm text-blue-700 mb-3">{currentProtocolInfo.description}</p>
            <div className="grid grid-cols-2 gap-2 text-xs">
              <div><strong>Update Rate:</strong> {currentProtocolInfo.updateRate}</div>
              <div><strong>Resolution:</strong> {currentProtocolInfo.resolution}</div>
              <div><strong>Latency:</strong> {currentProtocolInfo.latency}</div>
              <div><strong>Max Channels:</strong> {currentProtocolInfo.maxChannels}</div>
            </div>
          </div>
        </div>
        <div>
          <div className="space-y-3">
            <div>
              <h5 className="font-medium text-green-700 mb-1">Advantages:</h5>
              <ul className="text-sm text-green-600 list-disc list-inside">
                {currentProtocolInfo.advantages.map((adv, i) => (
                  <li key={i}>{adv}</li>
                ))}
              </ul>
            </div>
            <div>
              <h5 className="font-medium text-orange-700 mb-1">Limitations:</h5>
              <ul className="text-sm text-orange-600 list-disc list-inside">
                {currentProtocolInfo.limitations.map((lim, i) => (
                  <li key={i}>{lim}</li>
                ))}
              </ul>
            </div>
          </div>
        </div>
      </div>
    </div>
  );

  const renderChannelMapping = () => (
    <div className="space-y-6">
      {renderProtocolInfo()}
      
      <div className="card">
        <h3 className="text-lg font-semibold mb-4">Protocol Configuration</h3>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          <div>
            <label className="block text-sm font-medium mb-2">RC Protocol:</label>
            <select 
              value={rcConfig.protocol}
              onChange={(e) => handleProtocolChange(parseInt(e.target.value))}
              className="w-full px-3 py-2 border rounded-lg bg-white dark:bg-gray-700"
            >
              {Object.entries(PROTOCOL_NAMES).map(([value, name]) => (
                <option key={value} value={value}>{name}</option>
              ))}
            </select>
          </div>
          <div>
            <label className="block text-sm font-medium mb-2">
              Active Channels (Max {maxChannelsForProtocol}):
            </label>
            <input
              type="number"
              min={currentProtocolInfo.minChannels}
              max={maxChannelsForProtocol}
              value={rcConfig.channelCount}
              onChange={(e) => setRcConfig(prev => ({ 
                ...prev, 
                channelCount: Math.min(parseInt(e.target.value), maxChannelsForProtocol)
              }))}
              className="w-full px-3 py-2 border rounded-lg"
            />
          </div>
        </div>
      </div>

      <div className="card">
        <h3 className="text-lg font-semibold mb-4">Channel Mapping</h3>
        <div className="overflow-x-auto">
          <table className="w-full text-sm">
            <thead>
              <tr className="border-b">
                <th className="text-left p-2">Ch#</th>
                <th className="text-left p-2">Function</th>
                <th className="text-left p-2">Type</th>
                <th className="text-left p-2">Reversed</th>
                <th className="text-left p-2">Min</th>
                <th className="text-left p-2">Center</th>
                <th className="text-left p-2">Max</th>
                <th className="text-left p-2">Deadband</th>
              </tr>
            </thead>
            <tbody>
              {rcConfig.channels.slice(0, rcConfig.channelCount).map((channel, index) => (
                <tr key={index} className="border-b">
                  <td className="p-2">
                    <input
                      type="number"
                      min="0"
                      max={maxChannelsForProtocol}
                      value={channel.channelNumber}
                      onChange={(e) => updateChannelMapping(index, 'channelNumber', parseInt(e.target.value))}
                      className="w-16 px-2 py-1 border rounded text-center"
                    />
                  </td>
                  <td className="p-2">
                    <select
                      value={channel.function}
                      onChange={(e) => updateChannelMapping(index, 'function', parseInt(e.target.value))}
                      className="w-full px-2 py-1 border rounded bg-white dark:bg-gray-700 text-xs"
                    >
                      {Object.entries(FUNCTION_NAMES).map(([value, name]) => (
                        <option key={value} value={value}>{name}</option>
                      ))}
                    </select>
                  </td>
                  <td className="p-2">
                    <select
                      value={channel.isSwitch ? 'switch' : 'analog'}
                      onChange={(e) => updateChannelMapping(index, 'isSwitch', e.target.value === 'switch')}
                      className="w-20 px-2 py-1 border rounded bg-white dark:bg-gray-700 text-xs"
                    >
                      <option value="analog">Analog</option>
                      <option value="switch">Switch</option>
                    </select>
                  </td>
                  <td className="p-2">
                    <input
                      type="checkbox"
                      checked={channel.reversed}
                      onChange={(e) => updateChannelMapping(index, 'reversed', e.target.checked)}
                      className="w-4 h-4"
                    />
                  </td>
                  <td className="p-2">
                    <input
                      type="number"
                      value={channel.minValue}
                      onChange={(e) => updateChannelMapping(index, 'minValue', parseInt(e.target.value))}
                      className="w-16 px-2 py-1 border rounded text-center text-xs"
                    />
                  </td>
                  <td className="p-2">
                    <input
                      type="number"
                      value={channel.centerValue}
                      onChange={(e) => updateChannelMapping(index, 'centerValue', parseInt(e.target.value))}
                      className="w-16 px-2 py-1 border rounded text-center text-xs"
                      disabled={channel.function === CHANNEL_FUNCTIONS.CHAN_FUNC_THROTTLE}
                    />
                  </td>
                  <td className="p-2">
                    <input
                      type="number"
                      value={channel.maxValue}
                      onChange={(e) => updateChannelMapping(index, 'maxValue', parseInt(e.target.value))}
                      className="w-16 px-2 py-1 border rounded text-center text-xs"
                    />
                  </td>
                  <td className="p-2">
                    <input
                      type="number"
                      value={channel.deadband}
                      onChange={(e) => updateChannelMapping(index, 'deadband', parseInt(e.target.value))}
                      className="w-16 px-2 py-1 border rounded text-center text-xs"
                    />
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
        
        <div className="mt-4 flex gap-2">
          <button onClick={resetToDefaults} className="btn-secondary">
            Reset to Defaults
          </button>
          <button onClick={applyRcConfig} disabled={!connected} className="btn-primary">
            Apply Configuration
          </button>
        </div>
      </div>
    </div>
  );

  const renderChannelMonitor = () => (
    <div className="space-y-6">
      {/* Signal Quality Overview */}
      <div className="card">
        <h3 className="text-lg font-semibold mb-4">RC Signal Status</h3>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{rcSignalStats.rssi || 0}</div>
            <div className="text-sm text-gray-600">RSSI (dBm)</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{rcSignalStats.linkQuality || 0}%</div>
            <div className="text-sm text-gray-600">Link Quality</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">{rcSignalStats.frameRate || 0}Hz</div>
            <div className="text-sm text-gray-600">Frame Rate</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-orange-600">
              {rcSignalStats.packetsLost || 0}/{rcSignalStats.packetsReceived || 0}
            </div>
            <div className="text-sm text-gray-600">Lost/Total</div>
          </div>
        </div>
        
        <div className="flex items-center gap-4">
          <button
            onClick={() => setIsMonitoring(!isMonitoring)}
            disabled={!connected}
            className={`btn-${isMonitoring ? 'danger' : 'primary'}`}
          >
            {isMonitoring ? '⏹️ Stop Monitoring' : '▶️ Start Monitoring'}
          </button>
          
          {isMonitoring && (
            <div className="flex items-center text-green-600">
              <div className="w-3 h-3 bg-green-500 rounded-full mr-2 animate-pulse"></div>
              Live monitoring at 20Hz
            </div>
          )}
        </div>
      </div>

      {/* Enhanced Channel Display */}
      <div className="card">
        <h3 className="text-lg font-semibold mb-4">RC Channel Monitor</h3>
        
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-4">
          {channelData.slice(0, rcConfig.channelCount).map((value, index) => {
            const channel = rcConfig.channels[index];
            const stats = channelStats[index];
            const functionName = FUNCTION_NAMES[channel.function] || `Ch${index + 1}`;
            
            return (
              <div key={index} className="border-2 rounded-lg p-4 bg-white dark:bg-gray-800">
                <div className="flex items-center justify-between mb-2">
                  <div className="text-sm font-medium">
                    Ch{index + 1}: {functionName}
                  </div>
                  <div className={`text-lg ${getHealthColor(stats.health)}`}>
                    {getHealthIcon(stats.health)}
                  </div>
                </div>
                
                <div className="text-xl font-bold mb-2 font-mono">{value}</div>
                
                {/* Value bar with colored zones */}
                <div className="bg-gray-200 dark:bg-gray-700 rounded-full h-3 mb-2 relative overflow-hidden">
                  <div
                    className="bg-blue-500 h-3 rounded-full transition-all duration-100"
                    style={{ width: `${getChannelBar(value)}%` }}
                  />
                  {/* Min/Max indicators */}
                  <div 
                    className="absolute top-0 w-0.5 h-3 bg-red-400"
                    style={{ left: `${getChannelBar(channel.minValue)}%` }}
                  />
                  <div 
                    className="absolute top-0 w-0.5 h-3 bg-red-400"
                    style={{ left: `${getChannelBar(channel.maxValue)}%` }}
                  />
                  {channel.function !== CHANNEL_FUNCTIONS.CHAN_FUNC_THROTTLE && (
                    <div 
                      className="absolute top-0 w-0.5 h-3 bg-yellow-400"
                      style={{ left: `${getChannelBar(channel.centerValue)}%` }}
                    />
                  )}
                </div>

                {/* Channel statistics */}
                <div className="text-xs text-gray-600 space-y-1">
                  <div className="flex justify-between">
                    <span>Range:</span>
                    <span className="font-mono">{stats.min}-{stats.max}</span>
                  </div>
                  <div className="flex justify-between">
                    <span>Jitter:</span>
                    <span className="font-mono">{stats.jitter}μs</span>
                  </div>
                  <div className="flex justify-between">
                    <span>Updates:</span>
                    <span className="font-mono">{stats.updateCount}</span>
                  </div>
                </div>

                {/* Switch position display */}
                {channel.isSwitch && (
                  <div className="mt-2 p-2 bg-gray-100 dark:bg-gray-700 rounded text-center">
                    <div className="text-xs text-gray-600 mb-1">Switch Position</div>
                    <div className="font-semibold">
                      {getSwitchPosition(value, channel.switchPositions)}
                    </div>
                  </div>
                )}

                {/* Health status */}
                <div className={`mt-2 text-xs font-medium ${getHealthColor(stats.health)}`}>
                  {stats.health.toUpperCase()}
                </div>
              </div>
            );
          })}
        </div>

        {/* Channel Range Calibration Helper */}
        {isMonitoring && (
          <div className="mt-6 p-4 bg-yellow-50 border border-yellow-200 rounded-lg">
            <h4 className="font-semibold text-yellow-800 mb-2">📋 Calibration Helper</h4>
            <div className="text-sm text-yellow-700 space-y-1">
              <p>• Move all sticks and switches to their extreme positions</p>
              <p>• The min/max values will be automatically detected</p>
              <p>• Red lines on the bars show configured limits</p>
              <p>• Yellow line shows center position (for non-throttle channels)</p>
            </div>
          </div>
        )}
      </div>
    </div>
  );

  const renderRateProfiles = () => (
    <div className="space-y-6">
      <div className="card">
        <h3 className="text-lg font-semibold mb-4">Rate Profiles</h3>
        <div className="mb-4">
          <label className="block text-sm font-medium mb-2">Active Profile:</label>
          <select
            value={currentRateProfile}
            onChange={(e) => setCurrentRateProfile(parseInt(e.target.value))}
            className="px-3 py-2 border rounded-lg bg-white dark:bg-gray-700"
          >
            {rateProfiles.map((profile, index) => (
              <option key={index} value={index}>{profile.name}</option>
            ))}
          </select>
        </div>

        {rateProfiles.map((profile, profileIndex) => (
          <div key={profileIndex} className={`border rounded-lg p-4 ${profileIndex === currentRateProfile ? 'border-blue-500' : ''}`}>
            <h4 className="font-semibold mb-3">{profile.name}</h4>
            <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
              <div>
                <label className="block text-sm font-medium mb-1">Max Roll Rate (°/s):</label>
                <input
                  type="number"
                  value={profile.maxRollRate}
                  onChange={(e) => updateRateProfile(profileIndex, 'maxRollRate', parseFloat(e.target.value))}
                  className="w-full px-3 py-2 border rounded"
                />
              </div>
              <div>
                <label className="block text-sm font-medium mb-1">Max Pitch Rate (°/s):</label>
                <input
                  type="number"
                  value={profile.maxPitchRate}
                  onChange={(e) => updateRateProfile(profileIndex, 'maxPitchRate', parseFloat(e.target.value))}
                  className="w-full px-3 py-2 border rounded"
                />
              </div>
              <div>
                <label className="block text-sm font-medium mb-1">Max Yaw Rate (°/s):</label>
                <input
                  type="number"
                  value={profile.maxYawRate}
                  onChange={(e) => updateRateProfile(profileIndex, 'maxYawRate', parseFloat(e.target.value))}
                  className="w-full px-3 py-2 border rounded"
                />
              </div>
              <div>
                <label className="block text-sm font-medium mb-1">Roll Expo:</label>
                <input
                  type="number"
                  step="0.1"
                  min="0"
                  max="1"
                  value={profile.rollExpo}
                  onChange={(e) => updateRateProfile(profileIndex, 'rollExpo', parseFloat(e.target.value))}
                  className="w-full px-3 py-2 border rounded"
                />
              </div>
              <div>
                <label className="block text-sm font-medium mb-1">Pitch Expo:</label>
                <input
                  type="number"
                  step="0.1"
                  min="0"
                  max="1"
                  value={profile.pitchExpo}
                  onChange={(e) => updateRateProfile(profileIndex, 'pitchExpo', parseFloat(e.target.value))}
                  className="w-full px-3 py-2 border rounded"
                />
              </div>
              <div>
                <label className="block text-sm font-medium mb-1">Yaw Expo:</label>
                <input
                  type="number"
                  step="0.1"
                  min="0"
                  max="1"
                  value={profile.yawExpo}
                  onChange={(e) => updateRateProfile(profileIndex, 'yawExpo', parseFloat(e.target.value))}
                  className="w-full px-3 py-2 border rounded"
                />
              </div>
              <div>
                <label className="block text-sm font-medium mb-1">Throttle Expo:</label>
                <input
                  type="number"
                  step="0.1"
                  min="0"
                  max="1"
                  value={profile.throttleExpo}
                  onChange={(e) => updateRateProfile(profileIndex, 'throttleExpo', parseFloat(e.target.value))}
                  className="w-full px-3 py-2 border rounded"
                />
              </div>
              <div>
                <label className="block text-sm font-medium mb-1">Throttle Mid:</label>
                <input
                  type="number"
                  step="0.1"
                  min="0"
                  max="1"
                  value={profile.throttleMid}
                  onChange={(e) => updateRateProfile(profileIndex, 'throttleMid', parseFloat(e.target.value))}
                  className="w-full px-3 py-2 border rounded"
                />
              </div>
              <div>
                <label className="block text-sm font-medium mb-1">RC Smoothing:</label>
                <input
                  type="number"
                  step="0.1"
                  min="0"
                  max="1"
                  value={profile.rcSmoothingFactor}
                  onChange={(e) => updateRateProfile(profileIndex, 'rcSmoothingFactor', parseFloat(e.target.value))}
                  className="w-full px-3 py-2 border rounded"
                />
              </div>
            </div>
          </div>
        ))}
      </div>
    </div>
  );

  const renderFailsafeConfig = () => (
    <div className="space-y-6">
      <div className="card">
        <h3 className="text-lg font-semibold mb-4">Failsafe Configuration</h3>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          <div>
            <label className="flex items-center mb-4">
              <input
                type="checkbox"
                checked={rcConfig.failsafeEnabled}
                onChange={(e) => setRcConfig(prev => ({ ...prev, failsafeEnabled: e.target.checked }))}
                className="mr-2"
              />
              Enable Failsafe
            </label>
          </div>
          <div>
            <label className="flex items-center mb-4">
              <input
                type="checkbox"
                checked={rcConfig.rthOnFailsafe}
                onChange={(e) => setRcConfig(prev => ({ ...prev, rthOnFailsafe: e.target.checked }))}
                className="mr-2"
              />
              RTH on Signal Loss
            </label>
          </div>
          <div>
            <label className="block text-sm font-medium mb-2">Failsafe Throttle:</label>
            <input
              type="number"
              min="1000"
              max="2000"
              value={rcConfig.failsafeThrottle}
              onChange={(e) => setRcConfig(prev => ({ ...prev, failsafeThrottle: parseInt(e.target.value) }))}
              className="w-full px-3 py-2 border rounded"
            />
          </div>
          <div>
            <label className="block text-sm font-medium mb-2">Failsafe Flight Mode:</label>
            <select
              value={rcConfig.failsafeMode}
              onChange={(e) => setRcConfig(prev => ({ ...prev, failsafeMode: e.target.value }))}
              className="w-full px-3 py-2 border rounded bg-white dark:bg-gray-700"
            >
              <option value="MANUAL">Manual</option>
              <option value="STABILIZE">Stabilize</option>
              <option value="ALTITUDE_HOLD">Altitude Hold</option>
              <option value="RETURN_TO_HOME">Return to Home</option>
            </select>
          </div>
        </div>
      </div>

      <div className="card">
        <h3 className="text-lg font-semibold mb-4">Arming Configuration</h3>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          <div>
            <label className="flex items-center mb-4">
              <input
                type="checkbox"
                checked={rcConfig.stickArmingEnabled}
                onChange={(e) => setRcConfig(prev => ({ ...prev, stickArmingEnabled: e.target.checked }))}
                className="mr-2"
              />
              Enable Stick Arming
            </label>
          </div>
          <div>
            <label className="flex items-center mb-4">
              <input
                type="checkbox"
                checked={rcConfig.switchArmingEnabled}
                onChange={(e) => setRcConfig(prev => ({ ...prev, switchArmingEnabled: e.target.checked }))}
                className="mr-2"
              />
              Enable Switch Arming
            </label>
          </div>
          <div>
            <label className="block text-sm font-medium mb-2">Arm Switch Channel:</label>
            <input
              type="number"
              min="1"
              max="16"
              value={rcConfig.armChannel}
              onChange={(e) => setRcConfig(prev => ({ ...prev, armChannel: parseInt(e.target.value) }))}
              className="w-full px-3 py-2 border rounded"
            />
          </div>
          <div>
            <label className="block text-sm font-medium mb-2">Arm Sequence Time (ms):</label>
            <input
              type="number"
              min="500"
              max="5000"
              value={rcConfig.armSequenceTimeMs}
              onChange={(e) => setRcConfig(prev => ({ ...prev, armSequenceTimeMs: parseInt(e.target.value) }))}
              className="w-full px-3 py-2 border rounded"
            />
          </div>
        </div>
      </div>
    </div>
  );

  return (
    <div className="space-y-6">
      <h1 className="text-3xl font-bold">RC Receiver Configuration</h1>
      
      {/* Navigation Tabs */}
      <div className="border-b border-gray-200 dark:border-gray-700">
        <nav className="-mb-px flex space-x-8">
          {[
            { id: 'mapping', name: 'Channel Mapping' },
            { id: 'monitor', name: 'Channel Monitor' },
            { id: 'rates', name: 'Rate Profiles' },
            { id: 'failsafe', name: 'Failsafe & Arming' }
          ].map((tab) => (
            <button
              key={tab.id}
              onClick={() => setSelectedTab(tab.id)}
              className={`py-2 px-1 border-b-2 font-medium text-sm ${
                selectedTab === tab.id
                  ? 'border-blue-500 text-blue-600'
                  : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
              }`}
            >
              {tab.name}
            </button>
          ))}
        </nav>
      </div>

      {/* Tab Content */}
      {selectedTab === 'mapping' && renderChannelMapping()}
      {selectedTab === 'monitor' && renderChannelMonitor()}
      {selectedTab === 'rates' && renderRateProfiles()}
      {selectedTab === 'failsafe' && renderFailsafeConfig()}
    </div>
  );
}

export default ReceiverConfig;
