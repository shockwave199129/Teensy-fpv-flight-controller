import React, { useState, useEffect } from 'react';

const SafetyStatus = ({ sendCommand, connected }) => {
  const [safetyStatus, setSafetyStatus] = useState({
    safeToArm: false,
    calibrationComplete: false,
    batteryConnected: false,
    batteryVoltage: 0,
    batteryStatus: 'UNKNOWN',
    gpsHealthy: false,
    gpsFix: false,
    gpsSatellites: 0,
    gpsRequired: false,
    rcSignalValid: false,
    sensorsHealthy: false,
    lastSafetyCheck: null,
    gpsConfiguredFunctions: []
  });

  const [batteryDetails, setBatteryDetails] = useState({
    mainVoltage: 0,
    escVoltage: 0,
    current: 0,
    escTelemetryAvailable: false,
    lowVoltageThreshold: 11.5,
    criticalVoltageThreshold: 10.5
  });

  const [warnings, setWarnings] = useState([]);
  const [lastUpdate, setLastUpdate] = useState(null);
  const [autoRefresh, setAutoRefresh] = useState(true);

  useEffect(() => {
    if (connected && autoRefresh) {
      // Auto-refresh every 3 seconds
      const interval = setInterval(() => {
        performComprehensiveSafetyCheck();
      }, 3000);

      // Initial check
      performComprehensiveSafetyCheck();

      return () => clearInterval(interval);
    }
  }, [connected, autoRefresh]);

  useEffect(() => {
    // Listen for serial data
    const handleSerialData = (event, data) => {
      parseSafetyData(data);
    };

    window.electronAPI.onSerialData(handleSerialData);

    return () => {
      window.electronAPI.removeAllListeners('serial-data');
    };
  }, []);

  const performComprehensiveSafetyCheck = async () => {
    if (!connected) return;
    
    try {
      await sendCommand('safety check');
      await sendCommand('battery status');
      await sendCommand('gps status');
      await sendCommand('calibration check');
      setLastUpdate(new Date());
    } catch (error) {
      console.error('Error performing safety check:', error);
    }
  };

  const parseSafetyData = (data) => {
    try {
      // Parse comprehensive safety check results
      if (data.includes('=== Pre-Flight Safety Check ===')) {
        setSafetyStatus(prev => ({ ...prev, lastSafetyCheck: new Date() }));
      }

      if (data.includes('✓ SAFE TO ARM')) {
        setSafetyStatus(prev => ({ ...prev, safeToArm: true }));
      } else if (data.includes('✗ NOT SAFE TO ARM')) {
        setSafetyStatus(prev => ({ ...prev, safeToArm: false }));
      }

      // Parse specific safety components
      if (data.includes('✓ All sensors calibrated')) {
        setSafetyStatus(prev => ({ ...prev, calibrationComplete: true }));
      } else if (data.includes('✗ Calibration incomplete')) {
        setSafetyStatus(prev => ({ ...prev, calibrationComplete: false }));
      }

      // Battery parsing
      if (data.includes('Battery Connected: YES')) {
        setSafetyStatus(prev => ({ ...prev, batteryConnected: true }));
      } else if (data.includes('Battery Connected: NO')) {
        setSafetyStatus(prev => ({ ...prev, batteryConnected: false }));
      }

      const voltageMatch = data.match(/Battery Voltage: ([0-9.]+)/);
      if (voltageMatch) {
        const voltage = parseFloat(voltageMatch[1]);
        setSafetyStatus(prev => ({ 
          ...prev, 
          batteryVoltage: voltage,
          batteryStatus: voltage > 11.5 ? 'GOOD' : voltage > 10.5 ? 'LOW' : 'CRITICAL'
        }));
        setBatteryDetails(prev => ({ ...prev, mainVoltage: voltage }));
      }

      const currentMatch = data.match(/Current Draw: ([0-9.]+)/);
      if (currentMatch) {
        setBatteryDetails(prev => ({ ...prev, current: parseFloat(currentMatch[1]) }));
      }

      // ESC voltage telemetry
      if (data.includes('ESC Voltage Telemetry: AVAILABLE')) {
        setBatteryDetails(prev => ({ ...prev, escTelemetryAvailable: true }));
      } else if (data.includes('ESC Voltage Telemetry: NOT AVAILABLE')) {
        setBatteryDetails(prev => ({ ...prev, escTelemetryAvailable: false }));
      }

      // GPS status parsing
      if (data.includes('GPS Detected: YES')) {
        setSafetyStatus(prev => ({ ...prev, gpsHealthy: true }));
      } else if (data.includes('GPS Detected: NO')) {
        setSafetyStatus(prev => ({ ...prev, gpsHealthy: false }));
      }

      if (data.includes('GPS Fix: YES')) {
        setSafetyStatus(prev => ({ ...prev, gpsFix: true }));
      } else if (data.includes('GPS Fix: NO')) {
        setSafetyStatus(prev => ({ ...prev, gpsFix: false }));
      }

      const satMatch = data.match(/Satellites: (\d+)/);
      if (satMatch) {
        setSafetyStatus(prev => ({ ...prev, gpsSatellites: parseInt(satMatch[1]) }));
      }

      // GPS-dependent functions
      if (data.includes('Configured GPS functions:')) {
        setSafetyStatus(prev => ({ ...prev, gpsRequired: true }));
        // Parse individual functions
        const functions = [];
        if (data.includes('Return to Home')) functions.push('RTH');
        if (data.includes('GPS Rescue')) functions.push('GPS Rescue');
        if (data.includes('Position Hold')) functions.push('Position Hold');
        setSafetyStatus(prev => ({ ...prev, gpsConfiguredFunctions: functions }));
      } else if (data.includes('No GPS-dependent functions configured')) {
        setSafetyStatus(prev => ({ ...prev, gpsRequired: false, gpsConfiguredFunctions: [] }));
      }

      // RC and sensor status
      if (data.includes('✓ RC signal good')) {
        setSafetyStatus(prev => ({ ...prev, rcSignalValid: true }));
      } else if (data.includes('✗ No RC signal')) {
        setSafetyStatus(prev => ({ ...prev, rcSignalValid: false }));
      }

      if (data.includes('✓ Core sensors healthy')) {
        setSafetyStatus(prev => ({ ...prev, sensorsHealthy: true }));
      } else if (data.includes('✗ Critical sensor failure')) {
        setSafetyStatus(prev => ({ ...prev, sensorsHealthy: false }));
      }

      // Collect warnings and errors
      if (data.includes('WARNING:') || data.includes('ERROR:') || data.includes('⚠️')) {
        setWarnings(prev => {
          const newWarnings = [...prev, { message: data, timestamp: new Date() }];
          // Keep only last 10 warnings
          return newWarnings.slice(-10);
        });
      }

    } catch (error) {
      console.error('Error parsing safety data:', error);
    }
  };

  const clearWarnings = () => {
    setWarnings([]);
  };

  const handleManualSafetyCheck = async () => {
    await performComprehensiveSafetyCheck();
  };

  const SafetyOverviewCard = () => (
    <div className={`p-6 rounded-lg border-2 ${
      safetyStatus.safeToArm ? 'border-green-500 bg-green-50' : 'border-red-500 bg-red-50'
    }`}>
      <div className="flex items-center justify-between mb-4">
        <h2 className="text-2xl font-bold">
          {safetyStatus.safeToArm ? '✅ SAFE TO ARM' : '❌ NOT SAFE TO ARM'}
        </h2>
        <div className={`px-4 py-2 rounded-full font-bold ${
          safetyStatus.safeToArm ? 'bg-green-200 text-green-800' : 'bg-red-200 text-red-800'
        }`}>
          {safetyStatus.safeToArm ? 'READY' : 'NOT READY'}
        </div>
      </div>
      
      {safetyStatus.lastSafetyCheck && (
        <p className="text-sm text-gray-600">
          Last safety check: {safetyStatus.lastSafetyCheck.toLocaleString()}
        </p>
      )}
      
      {!safetyStatus.safeToArm && (
        <div className="mt-4 p-4 bg-yellow-100 border border-yellow-300 rounded">
          <h3 className="font-semibold text-yellow-800 mb-2">Issues to resolve:</h3>
          <ul className="text-sm text-yellow-700 space-y-1">
            {!safetyStatus.calibrationComplete && <li>• Complete sensor calibration</li>}
            {!safetyStatus.batteryConnected && <li>• Connect battery</li>}
            {!safetyStatus.rcSignalValid && <li>• Establish RC signal</li>}
            {!safetyStatus.sensorsHealthy && <li>• Fix sensor failures</li>}
            {safetyStatus.gpsRequired && (!safetyStatus.gpsHealthy || !safetyStatus.gpsFix || safetyStatus.gpsSatellites < 6) && (
              <li>• GPS fix required for configured functions</li>
            )}
          </ul>
        </div>
      )}
    </div>
  );

  const DetailedStatusGrid = () => (
    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
      {/* Calibration Status */}
      <div className={`p-4 rounded-lg border ${
        safetyStatus.calibrationComplete ? 'border-green-500 bg-green-50' : 'border-red-500 bg-red-50'
      }`}>
        <h3 className="text-lg font-semibold mb-3">📐 Calibration Status</h3>
        <div className="space-y-2">
          <div className="flex justify-between">
            <span>Overall Status:</span>
            <span className={safetyStatus.calibrationComplete ? 'text-green-600 font-bold' : 'text-red-600 font-bold'}>
              {safetyStatus.calibrationComplete ? '✅ Complete' : '❌ Required'}
            </span>
          </div>
          <button
            onClick={() => sendCommand('calibration check')}
            disabled={!connected}
            className="w-full mt-2 px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600 disabled:opacity-50 text-sm"
          >
            Check Calibration Details
          </button>
        </div>
      </div>

      {/* Battery Status */}
      <div className={`p-4 rounded-lg border ${
        safetyStatus.batteryStatus === 'CRITICAL' ? 'border-red-500 bg-red-50' :
        safetyStatus.batteryStatus === 'LOW' ? 'border-yellow-500 bg-yellow-50' :
        safetyStatus.batteryConnected ? 'border-green-500 bg-green-50' : 'border-red-500 bg-red-50'
      }`}>
        <h3 className="text-lg font-semibold mb-3">🔋 Battery Safety</h3>
        <div className="space-y-2 text-sm">
          <div className="flex justify-between">
            <span>Connected:</span>
            <span className={safetyStatus.batteryConnected ? 'text-green-600' : 'text-red-600'}>
              {safetyStatus.batteryConnected ? '✅ YES' : '❌ NO'}
            </span>
          </div>
          <div className="flex justify-between">
            <span>Voltage:</span>
            <span className={`font-mono ${
              safetyStatus.batteryStatus === 'CRITICAL' ? 'text-red-600' :
              safetyStatus.batteryStatus === 'LOW' ? 'text-yellow-600' : 'text-green-600'
            }`}>
              {safetyStatus.batteryVoltage.toFixed(2)}V
            </span>
          </div>
          <div className="flex justify-between">
            <span>Status:</span>
            <span className={`font-bold ${
              safetyStatus.batteryStatus === 'CRITICAL' ? 'text-red-600' :
              safetyStatus.batteryStatus === 'LOW' ? 'text-yellow-600' : 'text-green-600'
            }`}>
              {safetyStatus.batteryStatus}
            </span>
          </div>
          <div className="flex justify-between">
            <span>ESC Telemetry:</span>
            <span className={batteryDetails.escTelemetryAvailable ? 'text-green-600' : 'text-gray-500'}>
              {batteryDetails.escTelemetryAvailable ? '✅ Available' : '➖ N/A'}
            </span>
          </div>
        </div>
      </div>

      {/* GPS Status */}
      <div className={`p-4 rounded-lg border ${
        safetyStatus.gpsRequired 
          ? (safetyStatus.gpsHealthy && safetyStatus.gpsFix && safetyStatus.gpsSatellites >= 6 
             ? 'border-green-500 bg-green-50' : 'border-red-500 bg-red-50')
          : 'border-gray-300 bg-gray-50'
      }`}>
        <h3 className="text-lg font-semibold mb-3">🛰️ GPS Safety</h3>
        <div className="space-y-2 text-sm">
          <div className="flex justify-between">
            <span>Required:</span>
            <span className={safetyStatus.gpsRequired ? 'text-orange-600' : 'text-gray-500'}>
              {safetyStatus.gpsRequired ? 'YES' : 'NO'}
            </span>
          </div>
          <div className="flex justify-between">
            <span>Detected:</span>
            <span className={safetyStatus.gpsHealthy ? 'text-green-600' : 'text-red-600'}>
              {safetyStatus.gpsHealthy ? '✅ YES' : '❌ NO'}
            </span>
          </div>
          <div className="flex justify-between">
            <span>Fix:</span>
            <span className={safetyStatus.gpsFix ? 'text-green-600' : 'text-red-600'}>
              {safetyStatus.gpsFix ? '✅ YES' : '❌ NO'}
            </span>
          </div>
          <div className="flex justify-between">
            <span>Satellites:</span>
            <span className={safetyStatus.gpsSatellites >= 6 ? 'text-green-600' : 'text-red-600'}>
              {safetyStatus.gpsSatellites}/6+
            </span>
          </div>
          {safetyStatus.gpsConfiguredFunctions.length > 0 && (
            <div className="mt-2 p-2 bg-blue-100 rounded text-xs">
              <div className="font-semibold text-blue-800">Configured Functions:</div>
              {safetyStatus.gpsConfiguredFunctions.map((func, index) => (
                <div key={index} className="text-blue-600">• {func}</div>
              ))}
            </div>
          )}
        </div>
      </div>

      {/* RC Signal Status */}
      <div className={`p-4 rounded-lg border ${
        safetyStatus.rcSignalValid ? 'border-green-500 bg-green-50' : 'border-red-500 bg-red-50'
      }`}>
        <h3 className="text-lg font-semibold mb-3">📡 RC Signal</h3>
        <div className="space-y-2">
          <div className="flex justify-between">
            <span>Signal Status:</span>
            <span className={safetyStatus.rcSignalValid ? 'text-green-600 font-bold' : 'text-red-600 font-bold'}>
              {safetyStatus.rcSignalValid ? '✅ Good' : '❌ Lost'}
            </span>
          </div>
          <button
            onClick={() => sendCommand('get_rc_data')}
            disabled={!connected}
            className="w-full mt-2 px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600 disabled:opacity-50 text-sm"
          >
            Test RC Data
          </button>
        </div>
      </div>

      {/* Sensor Health */}
      <div className={`p-4 rounded-lg border ${
        safetyStatus.sensorsHealthy ? 'border-green-500 bg-green-50' : 'border-red-500 bg-red-50'
      }`}>
        <h3 className="text-lg font-semibold mb-3">📊 Sensor Health</h3>
        <div className="space-y-2">
          <div className="flex justify-between">
            <span>Core Sensors:</span>
            <span className={safetyStatus.sensorsHealthy ? 'text-green-600 font-bold' : 'text-red-600 font-bold'}>
              {safetyStatus.sensorsHealthy ? '✅ Healthy' : '❌ Failure'}
            </span>
          </div>
          <button
            onClick={() => sendCommand('sensor_status')}
            disabled={!connected}
            className="w-full mt-2 px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600 disabled:opacity-50 text-sm"
          >
            Check Sensor Details
          </button>
        </div>
      </div>

      {/* System Status */}
      <div className="p-4 rounded-lg border border-blue-500 bg-blue-50">
        <h3 className="text-lg font-semibold mb-3">🖥️ System Status</h3>
        <div className="space-y-2 text-sm">
          <div className="flex justify-between">
            <span>Connection:</span>
            <span className={connected ? 'text-green-600' : 'text-red-600'}>
              {connected ? '✅ Connected' : '❌ Disconnected'}
            </span>
          </div>
          <div className="flex justify-between">
            <span>Auto Refresh:</span>
            <label className="flex items-center">
              <input
                type="checkbox"
                checked={autoRefresh}
                onChange={(e) => setAutoRefresh(e.target.checked)}
                className="mr-1"
              />
              <span className={autoRefresh ? 'text-green-600' : 'text-gray-600'}>
                {autoRefresh ? 'ON' : 'OFF'}
              </span>
            </label>
          </div>
          {lastUpdate && (
            <div className="text-xs text-gray-500">
              Last update: {lastUpdate.toLocaleTimeString()}
            </div>
          )}
        </div>
      </div>
    </div>
  );

  return (
    <div className="p-6 space-y-6">
      <div className="flex justify-between items-center">
        <h1 className="text-3xl font-bold">🛡️ Safety Status</h1>
        <div className="flex space-x-2">
          <button
            onClick={handleManualSafetyCheck}
            disabled={!connected}
            className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 disabled:opacity-50"
          >
            🔄 Refresh Status
          </button>
          <button
            onClick={clearWarnings}
            disabled={warnings.length === 0}
            className="px-4 py-2 bg-yellow-500 text-white rounded hover:bg-yellow-600 disabled:opacity-50"
          >
            🗑️ Clear Warnings
          </button>
        </div>
      </div>

      {!connected && (
        <div className="p-4 bg-yellow-100 border border-yellow-400 text-yellow-700 rounded">
          ⚠️ Not connected to flight controller. Connect via serial port to view safety status.
        </div>
      )}

      {/* Active Warnings */}
      {warnings.length > 0 && (
        <div className="space-y-2">
          <h2 className="text-xl font-semibold text-red-700">⚠️ Active Warnings & Alerts</h2>
          {warnings.slice(-5).map((warning, index) => (
            <div key={index} className="p-3 bg-red-100 border border-red-400 text-red-700 rounded">
              <div className="flex justify-between items-start">
                <span>{warning.message}</span>
                <span className="text-xs text-red-500">
                  {warning.timestamp.toLocaleTimeString()}
                </span>
              </div>
            </div>
          ))}
        </div>
      )}

      {/* Safety Overview */}
      <SafetyOverviewCard />

      {/* Detailed Status */}
      <DetailedStatusGrid />

      {/* Quick Actions */}
      <div className="bg-white p-6 rounded-lg border border-gray-300">
        <h2 className="text-xl font-semibold mb-4">🚀 Quick Safety Actions</h2>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
          <button
            onClick={() => sendCommand('safety check')}
            disabled={!connected}
            className="p-3 bg-blue-500 text-white rounded hover:bg-blue-600 disabled:opacity-50 text-sm"
          >
            🛡️ Full Safety Check
          </button>
          <button
            onClick={() => sendCommand('calibration check')}
            disabled={!connected}
            className="p-3 bg-green-500 text-white rounded hover:bg-green-600 disabled:opacity-50 text-sm"
          >
            📐 Check Calibration
          </button>
          <button
            onClick={() => sendCommand('battery status')}
            disabled={!connected}
            className="p-3 bg-yellow-500 text-white rounded hover:bg-yellow-600 disabled:opacity-50 text-sm"
          >
            🔋 Battery Status
          </button>
          <button
            onClick={() => sendCommand('gps status')}
            disabled={!connected}
            className="p-3 bg-purple-500 text-white rounded hover:bg-purple-600 disabled:opacity-50 text-sm"
          >
            🛰️ GPS Status
          </button>
        </div>
      </div>

      {/* Safety Information */}
      <div className="bg-blue-50 p-6 rounded-lg border border-blue-200">
        <h2 className="text-xl font-semibold mb-4 text-blue-800">📋 Enhanced Safety Features</h2>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-6 text-sm">
          <div>
            <h3 className="font-semibold text-blue-700 mb-3">🛡️ Pre-Flight Safety System</h3>
            <ul className="space-y-2 text-blue-600">
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Comprehensive pre-flight validation system</span>
              </li>
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Automatic arming prevention when unsafe</span>
              </li>
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Real-time safety monitoring during flight</span>
              </li>
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Multiple redundancy layers for safety</span>
              </li>
            </ul>
          </div>
          <div>
            <h3 className="font-semibold text-blue-700 mb-3">🔋 Advanced Battery Safety</h3>
            <ul className="space-y-2 text-blue-600">
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Automatic battery connection detection</span>
              </li>
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Dual voltage monitoring (sensor + ESC telemetry)</span>
              </li>
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Emergency RTH trigger on critical voltage</span>
              </li>
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Three-tier warning system (Good/Low/Critical)</span>
              </li>
            </ul>
          </div>
          <div>
            <h3 className="font-semibold text-blue-700 mb-3">🛰️ GPS Requirement Enforcement</h3>
            <ul className="space-y-2 text-blue-600">
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>GPS-dependent functions require 6+ satellites</span>
              </li>
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Automatic blocking of GPS modes without fix</span>
              </li>
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Configuration warnings for GPS dependencies</span>
              </li>
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Home position automatic setup and validation</span>
              </li>
            </ul>
          </div>
          <div>
            <h3 className="font-semibold text-blue-700 mb-3">🚨 Emergency Systems</h3>
            <ul className="space-y-2 text-blue-600">
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Automatic failsafe activation on signal loss</span>
              </li>
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Emergency stop capability</span>
              </li>
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Critical voltage automatic RTH</span>
              </li>
              <li className="flex items-start">
                <span className="mr-2">•</span>
                <span>Real-time warning and alert system</span>
              </li>
            </ul>
          </div>
        </div>
      </div>
    </div>
  );
};

export default SafetyStatus; 