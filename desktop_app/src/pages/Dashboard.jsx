import React, { useState, useEffect } from 'react';

function Dashboard({ telemetry, connected, sendCommand }) {
  const [safetyStatus, setSafetyStatus] = useState({
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
    safeToArm: false,
    lastSafetyCheck: null
  });

  const [escVoltage, setEscVoltage] = useState({
    available: false,
    voltage: 0
  });

  const [warnings, setWarnings] = useState([]);

  useEffect(() => {
    if (connected) {
      // Request safety status every 5 seconds
      const interval = setInterval(() => {
        performSafetyCheck();
      }, 5000);

      // Initial safety check
      performSafetyCheck();

      return () => clearInterval(interval);
    }
  }, [connected]);

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

  const performSafetyCheck = async () => {
    if (!connected) return;
    
    try {
      await sendCommand('safety check');
      await sendCommand('battery status');
      await sendCommand('gps status');
    } catch (error) {
      console.error('Error performing safety check:', error);
    }
  };

  const parseSafetyData = (data) => {
    try {
      // Parse safety check results
      if (data.includes('=== Pre-Flight Safety Check ===')) {
        setSafetyStatus(prev => ({ ...prev, lastSafetyCheck: new Date() }));
      }

      if (data.includes('✓ SAFE TO ARM')) {
        setSafetyStatus(prev => ({ ...prev, safeToArm: true }));
      } else if (data.includes('✗ NOT SAFE TO ARM')) {
        setSafetyStatus(prev => ({ ...prev, safeToArm: false }));
      }

      // Parse calibration status
      if (data.includes('✓ All sensors calibrated')) {
        setSafetyStatus(prev => ({ ...prev, calibrationComplete: true }));
      } else if (data.includes('✗ Calibration incomplete')) {
        setSafetyStatus(prev => ({ ...prev, calibrationComplete: false }));
      }

      // Parse battery status
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
      }

      // Parse ESC voltage telemetry
      if (data.includes('ESC Voltage Telemetry: AVAILABLE')) {
        setEscVoltage(prev => ({ ...prev, available: true }));
      } else if (data.includes('ESC Voltage Telemetry: NOT AVAILABLE')) {
        setEscVoltage(prev => ({ ...prev, available: false }));
      }

      // Parse GPS status (enhanced with synthetic GPS support)
      if (data.includes('GPS ready - Dedicated GPS')) {
        setSafetyStatus(prev => ({ ...prev, gpsHealthy: true, gpsFix: true, gpsType: 'dedicated' }));
      } else if (data.includes('GPS ready - Synthetic GPS')) {
        setSafetyStatus(prev => ({ ...prev, gpsHealthy: true, gpsFix: true, gpsType: 'synthetic' }));
        const confidenceMatch = data.match(/(\d+\.?\d*)% confidence/);
        if (confidenceMatch) {
          setSafetyStatus(prev => ({ ...prev, syntheticGpsConfidence: parseFloat(confidenceMatch[1]) }));
        }
      } else if (data.includes('GPS functionality required but not available')) {
        setSafetyStatus(prev => ({ ...prev, gpsHealthy: false, gpsFix: false, gpsType: 'none' }));
      }

      // Legacy GPS parsing for compatibility
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

      // Check for GPS-dependent functions
      if (data.includes('Configured GPS functions:')) {
        setSafetyStatus(prev => ({ ...prev, gpsRequired: true }));
      } else if (data.includes('No GPS-dependent functions configured')) {
        setSafetyStatus(prev => ({ ...prev, gpsRequired: false }));
      }

      // Parse RC signal status
      if (data.includes('✓ RC signal good')) {
        setSafetyStatus(prev => ({ ...prev, rcSignalValid: true }));
      } else if (data.includes('✗ No RC signal')) {
        setSafetyStatus(prev => ({ ...prev, rcSignalValid: false }));
      }

      // Parse sensor health
      if (data.includes('✓ Core sensors healthy')) {
        setSafetyStatus(prev => ({ ...prev, sensorsHealthy: true }));
      } else if (data.includes('✗ Critical sensor failure')) {
        setSafetyStatus(prev => ({ ...prev, sensorsHealthy: false }));
      }

      // Parse warnings
      if (data.includes('WARNING:') || data.includes('ERROR:')) {
        setWarnings(prev => {
          const newWarnings = [...prev, data];
          // Keep only last 5 warnings and remove after 10 seconds
          setTimeout(() => {
            setWarnings(current => current.filter(w => w !== data));
          }, 10000);
          return newWarnings.slice(-5);
        });
      }

    } catch (error) {
      console.error('Error parsing safety data:', error);
    }
  };

  const handleEmergencyStop = async () => {
    if (window.confirm('Are you sure you want to trigger emergency stop?')) {
      await sendCommand('disarm');
    }
  };

  const handleGetStatus = async () => {
    await sendCommand('status');
  };

  const handleSafetyCheck = async () => {
    await performSafetyCheck();
  };

  const SafetyStatusCard = () => (
    <div className={`card ${safetyStatus.safeToArm ? 'border-green-500' : 'border-red-500'}`}>
      <h3 className="text-lg font-semibold mb-4">🛡️ Safety Status</h3>
      <div className="space-y-2">
        <div className="flex justify-between">
          <span>Safe to Arm:</span>
          <span className={safetyStatus.safeToArm ? 'text-green-600 font-bold' : 'text-red-600 font-bold'}>
            {safetyStatus.safeToArm ? '✅ YES' : '❌ NO'}
          </span>
        </div>
        <div className="flex justify-between">
          <span>Calibration:</span>
          <span className={safetyStatus.calibrationComplete ? 'text-green-600' : 'text-red-600'}>
            {safetyStatus.calibrationComplete ? '✅ Complete' : '❌ Required'}
          </span>
        </div>
        <div className="flex justify-between">
          <span>Battery:</span>
          <span className={safetyStatus.batteryConnected ? 'text-green-600' : 'text-red-600'}>
            {safetyStatus.batteryConnected ? '✅ Connected' : '❌ Disconnected'}
          </span>
        </div>
        <div className="flex justify-between">
          <span>RC Signal:</span>
          <span className={safetyStatus.rcSignalValid ? 'text-green-600' : 'text-red-600'}>
            {safetyStatus.rcSignalValid ? '✅ Good' : '❌ Lost'}
          </span>
        </div>
        <div className="flex justify-between">
          <span>Sensors:</span>
          <span className={safetyStatus.sensorsHealthy ? 'text-green-600' : 'text-red-600'}>
            {safetyStatus.sensorsHealthy ? '✅ Healthy' : '❌ Failure'}
          </span>
        </div>
        {safetyStatus.gpsRequired && (
          <div className="flex justify-between">
            <span>GPS Ready:</span>
            <span className={
              safetyStatus.gpsHealthy && safetyStatus.gpsFix && safetyStatus.gpsSatellites >= 6 
                ? 'text-green-600' : 'text-red-600'
            }>
              {safetyStatus.gpsHealthy && safetyStatus.gpsFix && safetyStatus.gpsSatellites >= 6 
                ? '✅ Ready' : '❌ Not Ready'}
            </span>
          </div>
        )}
        {safetyStatus.lastSafetyCheck && (
          <div className="text-xs text-gray-500 mt-2">
            Last check: {safetyStatus.lastSafetyCheck.toLocaleTimeString()}
          </div>
        )}
      </div>
    </div>
  );

  const BatteryStatusCard = () => (
    <div className={`card ${
      safetyStatus.batteryStatus === 'CRITICAL' ? 'border-red-500' :
      safetyStatus.batteryStatus === 'LOW' ? 'border-yellow-500' : 'border-green-500'
    }`}>
      <h3 className="text-lg font-semibold mb-4">🔋 Battery Status</h3>
      <div className="space-y-2">
        <div className="flex justify-between">
          <span>Connection:</span>
          <span className={safetyStatus.batteryConnected ? 'text-green-600' : 'text-red-600'}>
            {safetyStatus.batteryConnected ? 'Connected' : 'Disconnected'}
          </span>
        </div>
        <div className="flex justify-between">
          <span>Main Voltage:</span>
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
          <span className={escVoltage.available ? 'text-green-600' : 'text-gray-500'}>
            {escVoltage.available ? '✅ Available' : '➖ N/A'}
          </span>
        </div>
        {safetyStatus.batteryStatus === 'CRITICAL' && (
          <div className="mt-2 p-2 bg-red-100 border border-red-300 rounded text-red-700 text-sm">
            ⚠️ CRITICAL: Land immediately!
          </div>
        )}
        {safetyStatus.batteryStatus === 'LOW' && (
          <div className="mt-2 p-2 bg-yellow-100 border border-yellow-300 rounded text-yellow-700 text-sm">
            ⚠️ LOW: Prepare to land
          </div>
        )}
      </div>
    </div>
  );

  const GPSStatusCard = () => (
    <div className={`card ${
      safetyStatus.gpsRequired 
        ? (safetyStatus.gpsHealthy && safetyStatus.gpsFix 
           ? 'border-green-500' : 'border-red-500')
        : 'border-gray-300'
    }`}>
      <h3 className="text-lg font-semibold mb-4">🛰️ GPS Status</h3>
      <div className="space-y-2">
        <div className="flex justify-between">
          <span>GPS Required:</span>
          <span className={safetyStatus.gpsRequired ? 'text-orange-600' : 'text-gray-500'}>
            {safetyStatus.gpsRequired ? 'YES' : 'NO'}
          </span>
        </div>
        
        {/* Enhanced GPS Type Display */}
        <div className="flex justify-between">
          <span>GPS Type:</span>
          <span className={
            safetyStatus.gpsType === 'dedicated' ? 'text-green-600' :
            safetyStatus.gpsType === 'synthetic' ? 'text-blue-600' : 'text-red-600'
          }>
            {safetyStatus.gpsType === 'dedicated' ? '🛰️ Dedicated' :
             safetyStatus.gpsType === 'synthetic' ? '🔄 Synthetic' : '❌ None'}
          </span>
        </div>
        
        {/* Show synthetic GPS confidence if applicable */}
        {safetyStatus.gpsType === 'synthetic' && (
          <div className="flex justify-between">
            <span>Confidence:</span>
            <span className={
              safetyStatus.syntheticGpsConfidence >= 70 ? 'text-green-600' :
              safetyStatus.syntheticGpsConfidence >= 30 ? 'text-yellow-600' : 'text-red-600'
            }>
              {safetyStatus.syntheticGpsConfidence?.toFixed(1) || '--'}%
            </span>
          </div>
        )}
        
        {/* Show satellite count for dedicated GPS */}
        {safetyStatus.gpsType === 'dedicated' && (
          <div className="flex justify-between">
            <span>Satellites:</span>
            <span className={safetyStatus.gpsSatellites >= 6 ? 'text-green-600' : 'text-red-600'}>
              {safetyStatus.gpsSatellites}/6+
            </span>
          </div>
        )}
        
        <div className="flex justify-between">
          <span>Functionality:</span>
          <span className={safetyStatus.gpsHealthy && safetyStatus.gpsFix ? 'text-green-600' : 'text-red-600'}>
            {safetyStatus.gpsHealthy && safetyStatus.gpsFix ? '✅ Available' : '❌ Unavailable'}
          </span>
        </div>
        
        {safetyStatus.gpsRequired && !safetyStatus.gpsHealthy && (
          <div className="mt-2 p-2 bg-red-100 border border-red-300 rounded text-red-700 text-sm">
            ⚠️ GPS functions require either dedicated GPS (6+ satellites) or synthetic GPS (&gt;30% confidence)
          </div>
        )}
        {safetyStatus.gpsType === 'synthetic' && (
          <div className="mt-2 p-2 bg-blue-100 border border-blue-300 rounded text-blue-700 text-sm">
            ℹ️ Using synthetic GPS generated from IMU + Magnetometer + Barometer data
          </div>
        )}
      </div>
    </div>
  );

  return (
    <div className="space-y-6">
      <div className="flex justify-between items-center">
        <h1 className="text-3xl font-bold">Flight Controller Dashboard</h1>
        <div className="flex space-x-2">
          <button onClick={handleSafetyCheck} className="btn-secondary" disabled={!connected}>
            🛡️ Safety Check
          </button>
          <button onClick={handleGetStatus} className="btn-secondary" disabled={!connected}>
            📊 Get Status
          </button>
          <button onClick={handleEmergencyStop} className="btn-danger" disabled={!connected}>
            🚨 Emergency Stop
          </button>
        </div>
      </div>

      {!connected && (
        <div className="status-warning p-4 rounded-lg">
          ⚠️ Not connected to flight controller. Connect via serial port to view telemetry.
        </div>
      )}

      {/* Active Warnings */}
      {warnings.length > 0 && (
        <div className="space-y-2">
          {warnings.map((warning, index) => (
            <div key={index} className="p-3 bg-red-100 border border-red-400 text-red-700 rounded">
              ⚠️ {warning}
            </div>
          ))}
        </div>
      )}

      {/* Enhanced Safety Status Section */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
        <SafetyStatusCard />
        <BatteryStatusCard />
        <GPSStatusCard />
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
        {/* Flight Status */}
        <div className="card">
          <h3 className="text-lg font-semibold mb-4">Flight Status</h3>
          <div className="space-y-2">
            <div className="flex justify-between">
              <span>Armed:</span>
              <span className={telemetry?.armed ? 'text-red-600 font-bold' : 'text-green-600'}>
                {telemetry?.armed ? '🔴 ARMED' : '🟢 DISARMED'}
              </span>
            </div>
            <div className="flex justify-between">
              <span>Flight Mode:</span>
              <span className="font-mono">{telemetry?.mode || 'Unknown'}</span>
            </div>
            <div className="flex justify-between">
              <span>RC Signal:</span>
              <span className={telemetry?.rc_valid ? 'text-green-600' : 'text-red-600'}>
                {telemetry?.rc_valid ? '✅ Good' : '❌ Lost'}
              </span>
            </div>
          </div>
        </div>

        {/* Attitude */}
        <div className="card">
          <h3 className="text-lg font-semibold mb-4">Attitude</h3>
          <div className="space-y-2">
            <div className="flex justify-between">
              <span>Roll:</span>
              <span className="font-mono">{telemetry?.roll?.toFixed(1) || '0.0'}°</span>
            </div>
            <div className="flex justify-between">
              <span>Pitch:</span>
              <span className="font-mono">{telemetry?.pitch?.toFixed(1) || '0.0'}°</span>
            </div>
            <div className="flex justify-between">
              <span>Yaw:</span>
              <span className="font-mono">{telemetry?.yaw?.toFixed(1) || '0.0'}°</span>
            </div>
          </div>
        </div>

        {/* Power */}
        <div className="card">
          <h3 className="text-lg font-semibold mb-4">Power System</h3>
          <div className="space-y-2">
            <div className="flex justify-between">
              <span>Battery:</span>
              <span className={`font-mono ${
                telemetry?.battery < 11.0 ? 'text-red-600' : 
                telemetry?.battery < 11.5 ? 'text-yellow-600' : 'text-green-600'
              }`}>
                {telemetry?.battery?.toFixed(2) || '0.00'}V
              </span>
            </div>
            <div className="flex justify-between">
              <span>Altitude:</span>
              <span className="font-mono">{telemetry?.altitude?.toFixed(1) || '0.0'}m</span>
            </div>
            <div className="flex justify-between">
              <span>GPS Fix:</span>
              <span className={telemetry?.gps_fix ? 'text-green-600' : 'text-red-600'}>
                {telemetry?.gps_fix ? '🛰️ Lock' : '📡 Searching'}
              </span>
            </div>
          </div>
        </div>
      </div>

      {/* Visual Attitude Indicator */}
      <div className="card">
        <h3 className="text-lg font-semibold mb-4">Attitude Indicator</h3>
        <div className="flex justify-center">
          <div className="w-48 h-48 bg-gradient-to-b from-blue-400 to-green-400 rounded-full relative overflow-hidden">
            <div 
              className="absolute inset-0 bg-gradient-to-b from-transparent to-amber-600 transition-transform duration-200"
              style={{
                transform: `rotate(${telemetry?.roll || 0}deg) translateY(${(telemetry?.pitch || 0) * 2}px)`
              }}
            />
            <div className="absolute inset-0 flex items-center justify-center">
              <div className="w-1 h-24 bg-white"></div>
              <div className="w-24 h-1 bg-white absolute"></div>
            </div>
          </div>
        </div>
      </div>

      {/* Enhanced Quick Actions */}
      <div className="card">
        <h3 className="text-lg font-semibold mb-4">Quick Actions</h3>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
          <button 
            onClick={() => sendCommand('safety check')} 
            className="btn-secondary"
            disabled={!connected}
          >
            🛡️ Safety Check
          </button>
          <button 
            onClick={() => sendCommand('calibration check')} 
            className="btn-secondary"
            disabled={!connected}
          >
            📐 Calibration Status
          </button>
          <button 
            onClick={() => sendCommand('gps status')} 
            className="btn-secondary"
            disabled={!connected}
          >
            🛰️ GPS Status
          </button>
          <button 
            onClick={() => sendCommand('battery status')} 
            className="btn-secondary"
            disabled={!connected}
          >
            🔋 Battery Status
          </button>
          <button 
            onClick={() => sendCommand('calibrate esc')} 
            className="btn-secondary"
            disabled={!connected}
          >
            🔧 Calibrate ESCs
          </button>
          <button 
            onClick={() => sendCommand('reset pid')} 
            className="btn-secondary"
            disabled={!connected}
          >
            🔄 Reset PID
          </button>
          <button 
            onClick={() => sendCommand('led rainbow')} 
            className="btn-secondary"
            disabled={!connected}
          >
            🌈 Test LEDs
          </button>
          <button 
            onClick={() => sendCommand('reboot')} 
            className="btn-danger"
            disabled={!connected}
          >
            🔄 Reboot FC
          </button>
        </div>
      </div>

      {/* Safety Information Panel */}
      <div className="card bg-blue-50 border-blue-200">
        <h3 className="text-lg font-semibold mb-4 text-blue-800">🛡️ Enhanced Safety Features</h3>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4 text-sm">
          <div>
            <h4 className="font-semibold text-blue-700 mb-2">GPS Safety</h4>
            <ul className="space-y-1 text-blue-600">
              <li>• GPS functions require 6+ satellites</li>
              <li>• Automatic arming prevention without GPS</li>
              <li>• Mode switching blocked without GPS</li>
              <li>• RTH requires home position set</li>
            </ul>
          </div>
          <div>
            <h4 className="font-semibold text-blue-700 mb-2">Battery Safety</h4>
            <ul className="space-y-1 text-blue-600">
              <li>• Automatic battery connection detection</li>
              <li>• Dual voltage monitoring (sensor + ESC)</li>
              <li>• Emergency RTH on critical voltage</li>
              <li>• Real-time voltage level warnings</li>
            </ul>
          </div>
          <div>
            <h4 className="font-semibold text-blue-700 mb-2">Pre-Flight Safety</h4>
            <ul className="space-y-1 text-blue-600">
              <li>• Comprehensive safety check system</li>
              <li>• Calibration requirement enforcement</li>
              <li>• RC signal validation</li>
              <li>• Sensor health monitoring</li>
            </ul>
          </div>
          <div>
            <h4 className="font-semibold text-blue-700 mb-2">Emergency Features</h4>
            <ul className="space-y-1 text-blue-600">
              <li>• Automatic failsafe activation</li>
              <li>• Emergency stop capability</li>
              <li>• Real-time warning system</li>
              <li>• Multiple redundancy layers</li>
            </ul>
          </div>
        </div>
      </div>
    </div>
  );
}

export default Dashboard; 