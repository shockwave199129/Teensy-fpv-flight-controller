import React, { useState, useEffect } from 'react';

const FlightModes = ({ sendCommand, connected }) => {
  const [flightMode, setFlightMode] = useState('STABILIZE');
  const [gpsStatus, setGpsStatus] = useState({
    healthy: false,
    fix: false,
    satellites: 0,
    latitude: 0,
    longitude: 0,
    altitude: 0
  });
  const [batteryStatus, setBatteryStatus] = useState({
    connected: false,
    voltage: 0,
    current: 0,
    lowVoltage: false,
    critical: false
  });
  const [rthEnabled, setRthEnabled] = useState(false);
  const [homePosition, setHomePosition] = useState({ lat: 0, lon: 0, alt: 0, set: false });
  const [warnings, setWarnings] = useState([]);

  useEffect(() => {
    if (connected) {
      // Request initial status
      sendCommand('gps status');
      sendCommand('battery status');
      sendCommand('status');
    }

    // Listen for serial data
    const handleSerialData = (event, data) => {
      parseSerialData(data);
    };

    window.electronAPI.onSerialData(handleSerialData);

    return () => {
      window.electronAPI.removeAllListeners('serial-data');
    };
  }, [connected]);

  const parseSerialData = (data) => {
    try {
      // Parse GPS status
      if (data.includes('GPS Detected:')) {
        const gpsDetected = data.includes('GPS Detected: YES');
        const gpsFix = data.includes('GPS Fix: YES');
        const satMatch = data.match(/Satellites: (\d+)/);
        const satellites = satMatch ? parseInt(satMatch[1]) : 0;
        
        setGpsStatus(prev => ({
          ...prev,
          healthy: gpsDetected,
          fix: gpsFix,
          satellites: satellites
        }));
      }

      // Parse battery status
      if (data.includes('Battery Connected:')) {
        const batteryConnected = data.includes('Battery Connected: YES');
        const voltageMatch = data.match(/Battery Voltage: ([0-9.]+)/);
        const currentMatch = data.match(/Current Draw: ([0-9.]+)/);
        
        setBatteryStatus(prev => ({
          ...prev,
          connected: batteryConnected,
          voltage: voltageMatch ? parseFloat(voltageMatch[1]) : 0,
          current: currentMatch ? parseFloat(currentMatch[1]) : 0,
          lowVoltage: voltageMatch ? parseFloat(voltageMatch[1]) < 11.5 : false,
          critical: voltageMatch ? parseFloat(voltageMatch[1]) < 10.5 : false
        }));
      }

      // Parse flight mode changes
      if (data.includes('Flight mode changed to:')) {
        const modeMatch = data.match(/Flight mode changed to: (\w+)/);
        if (modeMatch) {
          setFlightMode(modeMatch[1]);
        }
      }

      // Parse mode switch blocking
      if (data.includes('Mode switch BLOCKED:')) {
        setWarnings(prev => [...prev, data]);
        setTimeout(() => {
          setWarnings(prev => prev.filter(w => w !== data));
        }, 5000);
      }

      // Parse home position
      if (data.includes('Home position set:')) {
        setHomePosition(prev => ({ ...prev, set: true }));
      }

    } catch (error) {
      console.error('Error parsing serial data:', error);
    }
  };

  const handleModeSwitch = async (mode) => {
    if (!connected) return;

    // Check GPS requirements
    if ((mode === 'POSITION_HOLD' || mode === 'RETURN_TO_HOME') && 
        (!gpsStatus.healthy || !gpsStatus.fix || gpsStatus.satellites < 6)) {
      setWarnings(prev => [...prev, `${mode} requires GPS fix with 6+ satellites`]);
      setTimeout(() => {
        setWarnings(prev => prev.filter(w => !w.includes(mode)));
      }, 5000);
      return;
    }

    if (mode === 'RETURN_TO_HOME' && !homePosition.set) {
      setWarnings(prev => [...prev, 'RTH requires home position to be set']);
      setTimeout(() => {
        setWarnings(prev => prev.filter(w => !w.includes('RTH')));
      }, 5000);
      return;
    }

    // Send mode switch command
    await sendCommand(`set flight mode ${mode}`);
  };

  const handleRthToggle = async () => {
    if (!connected) return;
    
    if (!rthEnabled && (!gpsStatus.healthy || !gpsStatus.fix || gpsStatus.satellites < 6)) {
      setWarnings(prev => [...prev, 'RTH requires GPS fix with 6+ satellites']);
      setTimeout(() => {
        setWarnings(prev => prev.filter(w => !w.includes('RTH')));
      }, 5000);
      return;
    }

    setRthEnabled(!rthEnabled);
    await sendCommand(`set failsafe rth ${!rthEnabled ? 'true' : 'false'}`);
  };

  const refreshStatus = async () => {
    if (!connected) return;
    await sendCommand('gps status');
    await sendCommand('battery status');
    await sendCommand('safety check');
  };

  const GPSStatusCard = () => (
    <div className={`p-4 rounded-lg border ${
      gpsStatus.healthy && gpsStatus.fix && gpsStatus.satellites >= 6 
        ? 'border-green-500 bg-green-50' 
        : 'border-red-500 bg-red-50'
    }`}>
      <h3 className="text-lg font-semibold mb-2">GPS Status</h3>
      <div className="grid grid-cols-2 gap-2 text-sm">
        <div>Detected: <span className={gpsStatus.healthy ? 'text-green-600' : 'text-red-600'}>
          {gpsStatus.healthy ? 'YES' : 'NO'}
        </span></div>
        <div>Fix: <span className={gpsStatus.fix ? 'text-green-600' : 'text-red-600'}>
          {gpsStatus.fix ? 'YES' : 'NO'}
        </span></div>
        <div>Satellites: <span className={gpsStatus.satellites >= 6 ? 'text-green-600' : 'text-red-600'}>
          {gpsStatus.satellites}
        </span></div>
        <div>Ready: <span className={
          gpsStatus.healthy && gpsStatus.fix && gpsStatus.satellites >= 6 
            ? 'text-green-600' : 'text-red-600'
        }>
          {gpsStatus.healthy && gpsStatus.fix && gpsStatus.satellites >= 6 ? 'YES' : 'NO'}
        </span></div>
      </div>
      {gpsStatus.fix && (
        <div className="mt-2 text-xs text-gray-600">
          Lat: {gpsStatus.latitude.toFixed(6)}, Lon: {gpsStatus.longitude.toFixed(6)}
        </div>
      )}
    </div>
  );

  const BatteryStatusCard = () => (
    <div className={`p-4 rounded-lg border ${
      batteryStatus.connected && !batteryStatus.critical 
        ? (batteryStatus.lowVoltage ? 'border-yellow-500 bg-yellow-50' : 'border-green-500 bg-green-50')
        : 'border-red-500 bg-red-50'
    }`}>
      <h3 className="text-lg font-semibold mb-2">Battery Status</h3>
      <div className="grid grid-cols-2 gap-2 text-sm">
        <div>Connected: <span className={batteryStatus.connected ? 'text-green-600' : 'text-red-600'}>
          {batteryStatus.connected ? 'YES' : 'NO'}
        </span></div>
        <div>Voltage: <span className={
          batteryStatus.critical ? 'text-red-600' : 
          batteryStatus.lowVoltage ? 'text-yellow-600' : 'text-green-600'
        }>
          {batteryStatus.voltage.toFixed(2)}V
        </span></div>
        <div>Current: <span className="text-blue-600">
          {batteryStatus.current.toFixed(2)}A
        </span></div>
        <div>Status: <span className={
          batteryStatus.critical ? 'text-red-600' : 
          batteryStatus.lowVoltage ? 'text-yellow-600' : 'text-green-600'
        }>
          {batteryStatus.critical ? 'CRITICAL' : 
           batteryStatus.lowVoltage ? 'LOW' : 'GOOD'}
        </span></div>
      </div>
    </div>
  );

  const flightModes = [
    { id: 'MANUAL', name: 'Manual', description: 'Direct gyro rate control', gpsRequired: false },
    { id: 'STABILIZE', name: 'Stabilize', description: 'Auto-level using IMU', gpsRequired: false },
    { id: 'ALTITUDE_HOLD', name: 'Altitude Hold', description: 'Maintain current altitude', gpsRequired: false },
    { id: 'POSITION_HOLD', name: 'Position Hold', description: 'GPS position lock', gpsRequired: true },
    { id: 'RETURN_TO_HOME', name: 'Return to Home', description: 'Automatic return to takeoff point', gpsRequired: true },
    { id: 'HEADLESS', name: 'Headless', description: 'Forward direction locked to home', gpsRequired: false }
  ];

  const gpsReady = gpsStatus.healthy && gpsStatus.fix && gpsStatus.satellites >= 6;

  return (
    <div className="p-6 space-y-6">
      <div className="flex justify-between items-center">
        <h1 className="text-2xl font-bold">Flight Modes</h1>
        <button
          onClick={refreshStatus}
          disabled={!connected}
          className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 disabled:opacity-50"
        >
          Refresh Status
        </button>
      </div>

      {/* Warnings */}
      {warnings.length > 0 && (
        <div className="space-y-2">
          {warnings.map((warning, index) => (
            <div key={index} className="p-3 bg-red-100 border border-red-400 text-red-700 rounded">
              ⚠️ {warning}
            </div>
          ))}
        </div>
      )}

      {/* Status Cards */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
        <GPSStatusCard />
        <BatteryStatusCard />
      </div>

      {/* Current Flight Mode */}
      <div className="p-4 bg-blue-50 border border-blue-200 rounded-lg">
        <h3 className="text-lg font-semibold mb-2">Current Flight Mode</h3>
        <div className="text-2xl font-bold text-blue-700">{flightMode}</div>
      </div>

      {/* Flight Mode Selection */}
      <div className="space-y-4">
        <h2 className="text-xl font-semibold">Available Flight Modes</h2>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
          {flightModes.map((mode) => {
            const isCurrentMode = flightMode === mode.id;
            const isDisabled = mode.gpsRequired && !gpsReady;
            
            return (
              <div
                key={mode.id}
                className={`p-4 border rounded-lg cursor-pointer transition-colors ${
                  isCurrentMode 
                    ? 'border-blue-500 bg-blue-50' 
                    : isDisabled 
                      ? 'border-gray-300 bg-gray-50 opacity-50 cursor-not-allowed'
                      : 'border-gray-300 hover:border-blue-400 hover:bg-blue-50'
                }`}
                onClick={() => !isDisabled && handleModeSwitch(mode.id)}
              >
                <div className="flex justify-between items-start mb-2">
                  <h3 className="font-semibold">{mode.name}</h3>
                  <div className="flex space-x-1">
                    {mode.gpsRequired && (
                      <span className={`text-xs px-2 py-1 rounded ${
                        gpsReady ? 'bg-green-100 text-green-700' : 'bg-red-100 text-red-700'
                      }`}>
                        GPS
                      </span>
                    )}
                    {isCurrentMode && (
                      <span className="text-xs px-2 py-1 bg-blue-100 text-blue-700 rounded">
                        ACTIVE
                      </span>
                    )}
                  </div>
                </div>
                <p className="text-sm text-gray-600">{mode.description}</p>
                {mode.gpsRequired && !gpsReady && (
                  <p className="text-xs text-red-600 mt-2">
                    Requires GPS fix with 6+ satellites
                  </p>
                )}
              </div>
            );
          })}
        </div>
      </div>

      {/* Return to Home Configuration */}
      <div className="space-y-4">
        <h2 className="text-xl font-semibold">Return to Home Settings</h2>
        <div className="p-4 border border-gray-300 rounded-lg">
          <div className="flex items-center justify-between mb-4">
            <div>
              <h3 className="font-semibold">Enable RTH on Failsafe</h3>
              <p className="text-sm text-gray-600">
                Automatically return home when RC signal is lost
              </p>
            </div>
            <button
              onClick={handleRthToggle}
              disabled={!connected || (!rthEnabled && !gpsReady)}
              className={`px-4 py-2 rounded ${
                rthEnabled 
                  ? 'bg-green-500 text-white hover:bg-green-600' 
                  : 'bg-gray-300 text-gray-700 hover:bg-gray-400'
              } disabled:opacity-50`}
            >
              {rthEnabled ? 'Enabled' : 'Disabled'}
            </button>
          </div>
          
          <div className="space-y-2 text-sm">
            <div className="flex justify-between">
              <span>Home Position Set:</span>
              <span className={homePosition.set ? 'text-green-600' : 'text-red-600'}>
                {homePosition.set ? 'YES' : 'NO'}
              </span>
            </div>
            <div className="flex justify-between">
              <span>GPS Ready:</span>
              <span className={gpsReady ? 'text-green-600' : 'text-red-600'}>
                {gpsReady ? 'YES' : 'NO'}
              </span>
            </div>
            {!gpsReady && (
              <p className="text-xs text-red-600">
                RTH requires GPS fix with 6+ satellites
              </p>
            )}
          </div>
        </div>
      </div>

      {/* Safety Information */}
      <div className="p-4 bg-yellow-50 border border-yellow-200 rounded-lg">
        <h3 className="font-semibold text-yellow-800 mb-2">⚠️ Important Safety Information</h3>
        <ul className="text-sm text-yellow-700 space-y-1">
          <li>• GPS-dependent modes require GPS fix with 6+ satellites</li>
          <li>• Home position is set automatically when GPS fix is acquired</li>
          <li>• Battery monitoring is active - critical voltage triggers warnings</li>
          <li>• Flight modes will automatically fallback to Stabilize if GPS is lost</li>
          <li>• Always maintain visual line of sight when using GPS modes</li>
        </ul>
      </div>
    </div>
  );
};

export default FlightModes;
