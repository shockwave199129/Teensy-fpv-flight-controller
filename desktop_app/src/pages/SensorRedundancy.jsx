import React, { useState, useEffect } from 'react';

const SensorRedundancy = () => {
  const [isConnected, setIsConnected] = useState(false);
  const [redundancyData, setRedundancyData] = useState({
    capability: 'FLIGHT_FULL_CAPABILITY',
    sensorStatus: {
      imu_health: 'SENSOR_HEALTHY',
      gps_health: 'SENSOR_HEALTHY',
      mag_health: 'SENSOR_HEALTHY',
      baro_health: 'SENSOR_HEALTHY',
      sonar_health: 'SENSOR_HEALTHY',
      optical_flow_health: 'SENSOR_MISSING',
      battery_health: 'SENSOR_HEALTHY',
      rc_health: 'SENSOR_HEALTHY',
      imu_quality: 0.95,
      gps_accuracy: 2.1,
      mag_calibration: 0.87,
      baro_stability: 0.92
    },
    syntheticData: {
      synthetic_gps_valid: false,
      synthetic_mag_valid: false,
      synthetic_baro_valid: false,
      gps_confidence: 0.0,
      mag_confidence: 0.0,
      baro_confidence: 0.0
    },
    alerts: []
  });
  const [lastUpdate, setLastUpdate] = useState(Date.now());

  useEffect(() => {
    // Listen for connection status
    const handleConnectionChange = (event, connected) => {
      setIsConnected(connected);
    };

    // Listen for sensor redundancy data
    const handleSerialData = (event, data) => {
      try {
        if (data.includes('"redundancy_status":')) {
          const response = JSON.parse(data);
          setRedundancyData(response.redundancy_status);
          setLastUpdate(Date.now());
        }
      } catch (error) {
        console.error('Error parsing redundancy data:', error);
      }
    };

    window.electronAPI.onConnectionChange(handleConnectionChange);
    window.electronAPI.onSerialData(handleSerialData);

    return () => {
      window.electronAPI.removeAllListeners('connection-change');
      window.electronAPI.removeAllListeners('serial-data');
    };
  }, []);

  // Auto-refresh redundancy status every 2 seconds
  useEffect(() => {
    if (!isConnected) return;

    const requestRedundancyStatus = () => {
      window.electronAPI.sendCommand('redundancy status json');
    };

    // Initial request
    requestRedundancyStatus();

    // Set up periodic refresh
    const interval = setInterval(requestRedundancyStatus, 2000);

    return () => clearInterval(interval);
  }, [isConnected]);

  const sendCommand = async (command) => {
    if (isConnected) {
      try {
        await window.electronAPI.sendCommand(command);
      } catch (error) {
        console.error('Error sending command:', error);
      }
    }
  };

  const getCapabilityColor = (capability) => {
    switch (capability) {
      case 'FLIGHT_FULL_CAPABILITY':
        return 'bg-green-100 text-green-800 border-green-200';
      case 'FLIGHT_DEGRADED_GPS':
      case 'FLIGHT_DEGRADED_MAG':
      case 'FLIGHT_DEGRADED_BARO':
        return 'bg-yellow-100 text-yellow-800 border-yellow-200';
      case 'FLIGHT_MINIMAL':
        return 'bg-orange-100 text-orange-800 border-orange-200';
      case 'FLIGHT_EMERGENCY':
        return 'bg-red-100 text-red-800 border-red-200';
      default:
        return 'bg-gray-100 text-gray-800 border-gray-200';
    }
  };

  const getSensorHealthColor = (health) => {
    switch (health) {
      case 'SENSOR_HEALTHY':
        return 'bg-green-100 text-green-800';
      case 'SENSOR_DEGRADED':
        return 'bg-yellow-100 text-yellow-800';
      case 'SENSOR_FAILED':
        return 'bg-red-100 text-red-800';
      case 'SENSOR_MISSING':
        return 'bg-gray-100 text-gray-600';
      default:
        return 'bg-gray-100 text-gray-600';
    }
  };

  const getCapabilityDescription = (capability) => {
    switch (capability) {
      case 'FLIGHT_FULL_CAPABILITY':
        return 'All systems operational - full flight capability';
      case 'FLIGHT_DEGRADED_GPS':
        return 'GPS unavailable - using synthetic position estimation';
      case 'FLIGHT_DEGRADED_MAG':
        return 'Magnetometer unavailable - using GPS/gyro heading';
      case 'FLIGHT_DEGRADED_BARO':
        return 'Barometer unavailable - using GPS/sonar altitude';
      case 'FLIGHT_MINIMAL':
        return 'Minimal sensors only - basic stabilization mode';
      case 'FLIGHT_EMERGENCY':
        return 'Critical sensor failure - emergency landing required';
      default:
        return 'Unknown flight capability status';
    }
  };

  const formatSensorName = (sensorKey) => {
    const nameMap = {
      imu_health: 'IMU',
      gps_health: 'GPS',
      mag_health: 'Magnetometer',
      baro_health: 'Barometer',
      sonar_health: 'Sonar',
      optical_flow_health: 'Optical Flow',
      battery_health: 'Battery',
      rc_health: 'RC Receiver'
    };
    return nameMap[sensorKey] || sensorKey;
  };

  const formatHealthStatus = (health) => {
    return health.replace('SENSOR_', '').toLowerCase()
      .split('_').map(word => word.charAt(0).toUpperCase() + word.slice(1)).join(' ');
  };

  const getQualityDisplay = (sensor, value) => {
    if (value === undefined || value === null) return '--';
    
    switch (sensor) {
      case 'imu_quality':
      case 'mag_calibration':
      case 'baro_stability':
        return `${(value * 100).toFixed(1)}%`;
      case 'gps_accuracy':
        return `${value.toFixed(1)}m`;
      default:
        return value.toString();
    }
  };

  const getQualityColor = (sensor, value) => {
    if (value === undefined || value === null) return 'text-gray-500';
    
    switch (sensor) {
      case 'imu_quality':
      case 'mag_calibration':
      case 'baro_stability':
        if (value >= 0.8) return 'text-green-600';
        if (value >= 0.6) return 'text-yellow-600';
        return 'text-red-600';
      case 'gps_accuracy':
        if (value <= 3) return 'text-green-600';
        if (value <= 10) return 'text-yellow-600';
        return 'text-red-600';
      default:
        return 'text-gray-600';
    }
  };

  const handleEmergencyMode = () => {
    if (window.confirm('Are you sure you want to activate emergency mode? This will apply maximum stability settings.')) {
      sendCommand('emergency mode');
    }
  };

  const handleRecoveryMode = () => {
    if (window.confirm('Are you sure you want to activate recovery mode? This will reset and recalibrate the redundancy system.')) {
      sendCommand('recovery mode');
    }
  };

  if (!isConnected) {
    return (
      <div className="p-6">
        <div className="bg-yellow-100 border border-yellow-400 text-yellow-700 px-4 py-3 rounded">
          <strong>Not Connected:</strong> Please connect to the drone to view sensor redundancy status.
        </div>
      </div>
    );
  }

  return (
    <div className="p-6 space-y-6">
      <div className="flex justify-between items-center">
        <h1 className="text-3xl font-bold text-gray-900">Sensor Redundancy System</h1>
        <div className="text-sm text-gray-500">
          Last updated: {new Date(lastUpdate).toLocaleTimeString()}
        </div>
      </div>

      {/* Flight Capability Status */}
      <div className="bg-white rounded-lg shadow p-6">
        <h2 className="text-xl font-semibold text-gray-900 mb-4">
          🛩️ Flight Capability Status
        </h2>
        <div className={`p-4 rounded-lg border ${getCapabilityColor(redundancyData.capability)}`}>
          <div className="flex items-center justify-between">
            <div>
              <h3 className="text-lg font-semibold">
                {redundancyData.capability?.replace('FLIGHT_', '').replace('_', ' ')}
              </h3>
              <p className="text-sm mt-1">
                {getCapabilityDescription(redundancyData.capability)}
              </p>
            </div>
            <div className="text-2xl">
              {redundancyData.capability === 'FLIGHT_FULL_CAPABILITY' && '✅'}
              {(redundancyData.capability?.includes('DEGRADED') || redundancyData.capability === 'FLIGHT_MINIMAL') && '⚠️'}
              {redundancyData.capability === 'FLIGHT_EMERGENCY' && '🚨'}
            </div>
          </div>
        </div>
      </div>

      {/* Sensor Health Grid */}
      <div className="bg-white rounded-lg shadow p-6">
        <h2 className="text-xl font-semibold text-gray-900 mb-4">
          📊 Sensor Health Status
        </h2>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
          {Object.entries(redundancyData.sensorStatus || {}).map(([sensorKey, health]) => {
            if (!sensorKey.endsWith('_health')) return null;
            
            const qualityKey = sensorKey.replace('_health', '_quality')
              .replace('mag_quality', 'mag_calibration')
              .replace('baro_quality', 'baro_stability')
              .replace('gps_quality', 'gps_accuracy');
            
            const qualityValue = redundancyData.sensorStatus[qualityKey];
            
            return (
              <div key={sensorKey} className={`p-3 rounded-lg ${getSensorHealthColor(health)}`}>
                <div className="font-semibold text-sm">
                  {formatSensorName(sensorKey)}
                </div>
                <div className="text-xs mt-1">
                  {formatHealthStatus(health)}
                </div>
                {qualityValue !== undefined && (
                  <div className={`text-xs mt-1 font-medium ${getQualityColor(qualityKey, qualityValue)}`}>
                    {getQualityDisplay(qualityKey, qualityValue)}
                  </div>
                )}
              </div>
            );
          })}
        </div>
      </div>

      {/* Synthetic Sensor Systems */}
      <div className="bg-white rounded-lg shadow p-6">
        <h2 className="text-xl font-semibold text-gray-900 mb-4">
          🔄 Synthetic Sensor Systems
        </h2>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
          <div className={`p-4 rounded-lg border ${
            redundancyData.syntheticData?.synthetic_gps_valid 
              ? 'bg-blue-100 text-blue-800 border-blue-200' 
              : 'bg-gray-100 text-gray-600 border-gray-200'
          }`}>
            <div className="flex items-center justify-between">
              <div>
                <h3 className="font-semibold">Synthetic GPS</h3>
                <p className="text-sm">
                  {redundancyData.syntheticData?.synthetic_gps_valid ? 'ACTIVE' : 'INACTIVE'}
                </p>
              </div>
              <div className="text-lg">
                {redundancyData.syntheticData?.synthetic_gps_valid ? '🔄' : '⏸️'}
              </div>
            </div>
            {redundancyData.syntheticData?.synthetic_gps_valid && (
              <div className="mt-2 text-sm">
                Confidence: {(redundancyData.syntheticData.gps_confidence * 100).toFixed(1)}%
              </div>
            )}
          </div>

          <div className={`p-4 rounded-lg border ${
            redundancyData.syntheticData?.synthetic_mag_valid 
              ? 'bg-blue-100 text-blue-800 border-blue-200' 
              : 'bg-gray-100 text-gray-600 border-gray-200'
          }`}>
            <div className="flex items-center justify-between">
              <div>
                <h3 className="font-semibold">Synthetic Magnetometer</h3>
                <p className="text-sm">
                  {redundancyData.syntheticData?.synthetic_mag_valid ? 'ACTIVE' : 'INACTIVE'}
                </p>
              </div>
              <div className="text-lg">
                {redundancyData.syntheticData?.synthetic_mag_valid ? '🔄' : '⏸️'}
              </div>
            </div>
            {redundancyData.syntheticData?.synthetic_mag_valid && (
              <div className="mt-2 text-sm">
                Confidence: {(redundancyData.syntheticData.mag_confidence * 100).toFixed(1)}%
              </div>
            )}
          </div>

          <div className={`p-4 rounded-lg border ${
            redundancyData.syntheticData?.synthetic_baro_valid 
              ? 'bg-blue-100 text-blue-800 border-blue-200' 
              : 'bg-gray-100 text-gray-600 border-gray-200'
          }`}>
            <div className="flex items-center justify-between">
              <div>
                <h3 className="font-semibold">Synthetic Barometer</h3>
                <p className="text-sm">
                  {redundancyData.syntheticData?.synthetic_baro_valid ? 'ACTIVE' : 'INACTIVE'}
                </p>
              </div>
              <div className="text-lg">
                {redundancyData.syntheticData?.synthetic_baro_valid ? '🔄' : '⏸️'}
              </div>
            </div>
            {redundancyData.syntheticData?.synthetic_baro_valid && (
              <div className="mt-2 text-sm">
                Confidence: {(redundancyData.syntheticData.baro_confidence * 100).toFixed(1)}%
              </div>
            )}
          </div>
        </div>
      </div>

      {/* Active Alerts */}
      <div className="bg-white rounded-lg shadow p-6">
        <h2 className="text-xl font-semibold text-gray-900 mb-4">
          🚨 Active Alerts
        </h2>
        {redundancyData.alerts && redundancyData.alerts.length > 0 ? (
          <div className="space-y-2">
            {redundancyData.alerts.map((alert, index) => (
              <div key={index} className="p-3 bg-red-100 text-red-800 rounded-lg border border-red-200">
                <div className="flex items-center justify-between">
                  <span className="font-medium">{alert.message}</span>
                  <span className="text-sm">
                    {new Date(alert.timestamp).toLocaleTimeString()}
                  </span>
                </div>
              </div>
            ))}
          </div>
        ) : (
          <div className="p-4 bg-green-100 text-green-700 rounded-lg border border-green-200">
            ✅ No active sensor alerts
          </div>
        )}
      </div>

      {/* Emergency Controls */}
      <div className="bg-white rounded-lg shadow p-6">
        <h2 className="text-xl font-semibold text-gray-900 mb-4">
          ⚡ Emergency Controls
        </h2>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          <button
            onClick={handleEmergencyMode}
            className="p-4 bg-red-600 text-white rounded-lg hover:bg-red-700 transition-colors"
          >
            <div className="text-lg font-semibold">🚨 Emergency Mode</div>
            <div className="text-sm mt-1">
              Activate maximum stability settings
            </div>
          </button>
          
          <button
            onClick={handleRecoveryMode}
            className="p-4 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition-colors"
          >
            <div className="text-lg font-semibold">🔄 Recovery Mode</div>
            <div className="text-sm mt-1">
              Reset and recalibrate redundancy system
            </div>
          </button>
        </div>
      </div>

      {/* Additional Commands */}
      <div className="bg-white rounded-lg shadow p-6">
        <h2 className="text-xl font-semibold text-gray-900 mb-4">
          🛠️ System Commands
        </h2>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
          <button
            onClick={() => sendCommand('sensor health')}
            className="p-3 bg-blue-500 text-white rounded hover:bg-blue-600 transition-colors text-sm"
          >
            Sensor Health Check
          </button>
          <button
            onClick={() => sendCommand('synthetic data')}
            className="p-3 bg-green-500 text-white rounded hover:bg-green-600 transition-colors text-sm"
          >
            View Synthetic Data
          </button>
          <button
            onClick={() => sendCommand('safety check')}
            className="p-3 bg-yellow-500 text-white rounded hover:bg-yellow-600 transition-colors text-sm"
          >
            Safety Analysis
          </button>
          <button
            onClick={() => sendCommand('redundancy status json')}
            className="p-3 bg-purple-500 text-white rounded hover:bg-purple-600 transition-colors text-sm"
          >
            Refresh Status
          </button>
        </div>
      </div>
    </div>
  );
};

export default SensorRedundancy; 