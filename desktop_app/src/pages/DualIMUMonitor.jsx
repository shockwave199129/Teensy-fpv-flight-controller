import React, { useState, useEffect } from 'react';

const DualIMUMonitor = () => {
  const [imuStatus, setImuStatus] = useState({
    primaryHealthy: false,
    secondaryHealthy: false,
    crossValidationPassed: false,
    usingPrimary: true,
    gyroDivergence: 0,
    primaryFailures: 0,
    secondaryFailures: 0,
    lastValidation: 0
  });

  const [imuConfig, setImuConfig] = useState({
    primaryType: 'MPU6050',
    secondaryType: 'ICM20948',
    autoFailoverEnabled: true,
    crossValidationEnabled: true,
    divergenceThreshold: 5.0,
    gyroDivergenceThreshold: 10.0
  });

  const [realtimeData, setRealtimeData] = useState({
    primary: {
      accel: { x: 0, y: 0, z: 0 },
      gyro: { x: 0, y: 0, z: 0 },
      temp: 0
    },
    secondary: {
      accel: { x: 0, y: 0, z: 0 },
      gyro: { x: 0, y: 0, z: 0 },
      temp: 0
    },
    fused: {
      accel: { x: 0, y: 0, z: 0 },
      gyro: { x: 0, y: 0, z: 0 },
      attitude: { roll: 0, pitch: 0, yaw: 0 }
    }
  });

  const [alerts, setAlerts] = useState([]);

  useEffect(() => {
    // Request IMU status every 500ms
    const interval = setInterval(() => {
      window.electronAPI.sendSerial('dual_imu status');
    }, 500);

    // Listen for IMU data updates
    const handleIMUData = (data) => {
      setImuStatus(data.status);
      setRealtimeData(data.realtimeData);
      
      // Check for alerts
      checkForAlerts(data.status);
    };

    window.electronAPI.on('imu-data', handleIMUData);

    return () => {
      clearInterval(interval);
      window.electronAPI.off('imu-data', handleIMUData);
    };
  }, []);

  const checkForAlerts = (status) => {
    const newAlerts = [];
    
    if (!status.primaryHealthy) {
      newAlerts.push({ type: 'error', message: 'Primary IMU not responding' });
    }
    if (!status.secondaryHealthy) {
      newAlerts.push({ type: 'warning', message: 'Secondary IMU not responding' });
    }
    if (!status.crossValidationPassed) {
      newAlerts.push({ type: 'warning', message: `IMU divergence detected: ${status.gyroDivergence.toFixed(1)}°/s` });
    }
    if (status.primaryFailures > 3) {
      newAlerts.push({ type: 'error', message: `Primary IMU has ${status.primaryFailures} consecutive failures` });
    }
    
    setAlerts(newAlerts);
  };

  const handleForceIMU = (imu) => {
    window.electronAPI.sendSerial(`dual_imu force_${imu}`);
  };

  const handleAutoFailover = () => {
    window.electronAPI.sendSerial('dual_imu auto');
  };

  const IMUStatusCard = ({ title, data, isActive, isHealthy }) => (
    <div className={`p-6 rounded-lg shadow-md border-2 ${
      isActive 
        ? 'border-blue-500 bg-blue-50' 
        : 'border-gray-200 bg-white'
    }`}>
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-lg font-semibold">{title}</h3>
        <div className={`px-3 py-1 rounded-full text-sm font-medium ${
          isHealthy 
            ? 'bg-green-100 text-green-800' 
            : 'bg-red-100 text-red-800'
        }`}>
          {isHealthy ? 'Healthy' : 'Failed'}
        </div>
      </div>
      
      <div className="grid grid-cols-2 gap-4 text-sm">
        <div>
          <h4 className="font-medium mb-2">Accelerometer (g)</h4>
          <div>X: {data.accel.x.toFixed(3)}</div>
          <div>Y: {data.accel.y.toFixed(3)}</div>
          <div>Z: {data.accel.z.toFixed(3)}</div>
        </div>
        <div>
          <h4 className="font-medium mb-2">Gyroscope (°/s)</h4>
          <div>X: {data.gyro.x.toFixed(1)}</div>
          <div>Y: {data.gyro.y.toFixed(1)}</div>
          <div>Z: {data.gyro.z.toFixed(1)}</div>
        </div>
      </div>
      
      <div className="mt-4 text-sm text-gray-600">
        Temperature: {data.temp.toFixed(1)}°C
      </div>
    </div>
  );

  const StatusIndicator = ({ label, status, goodValue = true }) => (
    <div className="flex items-center justify-between p-3 bg-gray-50 rounded">
      <span className="font-medium">{label}</span>
      <span className={`px-2 py-1 rounded text-sm ${
        (status === goodValue) 
          ? 'bg-green-100 text-green-800' 
          : 'bg-red-100 text-red-800'
      }`}>
        {status.toString()}
      </span>
    </div>
  );

  return (
    <div className="p-6">
      <h1 className="text-3xl font-bold mb-6">Dual IMU Monitoring</h1>

      {/* Alerts Section */}
      {alerts.length > 0 && (
        <div className="mb-6">
          {alerts.map((alert, index) => (
            <div
              key={index}
              className={`p-4 rounded-lg mb-2 ${
                alert.type === 'error' 
                  ? 'bg-red-100 border border-red-400 text-red-700'
                  : 'bg-yellow-100 border border-yellow-400 text-yellow-700'
              }`}
            >
              {alert.message}
            </div>
          ))}
        </div>
      )}

      {/* System Status Overview */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 mb-6">
        <StatusIndicator 
          label="Primary IMU" 
          status={imuStatus.primaryHealthy ? 'Healthy' : 'Failed'} 
          goodValue="Healthy"
        />
        <StatusIndicator 
          label="Secondary IMU" 
          status={imuStatus.secondaryHealthy ? 'Healthy' : 'Failed'} 
          goodValue="Healthy"
        />
        <StatusIndicator 
          label="Cross Validation" 
          status={imuStatus.crossValidationPassed ? 'Pass' : 'Fail'} 
          goodValue="Pass"
        />
        <StatusIndicator 
          label="Active IMU" 
          status={imuStatus.usingPrimary ? 'Primary' : 'Secondary'} 
          goodValue="Primary"
        />
      </div>

      {/* Divergence Monitor */}
      <div className="mb-6 p-4 bg-white rounded-lg shadow-md">
        <h3 className="text-lg font-semibold mb-3">Sensor Divergence</h3>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Gyro Divergence
            </label>
            <div className={`text-2xl font-bold ${
              imuStatus.gyroDivergence > imuConfig.gyroDivergenceThreshold 
                ? 'text-red-600' 
                : 'text-green-600'
            }`}>
              {imuStatus.gyroDivergence.toFixed(1)}°/s
            </div>
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Primary Failures
            </label>
            <div className="text-2xl font-bold text-gray-800">
              {imuStatus.primaryFailures}
            </div>
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Secondary Failures
            </label>
            <div className="text-2xl font-bold text-gray-800">
              {imuStatus.secondaryFailures}
            </div>
          </div>
        </div>
      </div>

      {/* IMU Data Display */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 mb-6">
        <IMUStatusCard
          title="Primary IMU"
          data={realtimeData.primary}
          isActive={imuStatus.usingPrimary}
          isHealthy={imuStatus.primaryHealthy}
        />
        <IMUStatusCard
          title="Secondary IMU"
          data={realtimeData.secondary}
          isActive={!imuStatus.usingPrimary}
          isHealthy={imuStatus.secondaryHealthy}
        />
        <div className="p-6 rounded-lg shadow-md border-2 border-green-500 bg-green-50">
          <h3 className="text-lg font-semibold mb-4">Fused Output</h3>
          <div className="space-y-3 text-sm">
            <div>
              <h4 className="font-medium mb-1">Attitude (°)</h4>
              <div>Roll: {realtimeData.fused.attitude.roll.toFixed(1)}</div>
              <div>Pitch: {realtimeData.fused.attitude.pitch.toFixed(1)}</div>
              <div>Yaw: {realtimeData.fused.attitude.yaw.toFixed(1)}</div>
            </div>
            <div>
              <h4 className="font-medium mb-1">Gyro Rates (°/s)</h4>
              <div>X: {realtimeData.fused.gyro.x.toFixed(1)}</div>
              <div>Y: {realtimeData.fused.gyro.y.toFixed(1)}</div>
              <div>Z: {realtimeData.fused.gyro.z.toFixed(1)}</div>
            </div>
          </div>
        </div>
      </div>

      {/* Control Buttons */}
      <div className="grid grid-cols-1 md:grid-cols-4 gap-4">
        <button
          onClick={() => handleForceIMU('primary')}
          className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded"
        >
          Force Primary
        </button>
        <button
          onClick={() => handleForceIMU('secondary')}
          className="bg-orange-500 hover:bg-orange-700 text-white font-bold py-2 px-4 rounded"
        >
          Force Secondary
        </button>
        <button
          onClick={handleAutoFailover}
          className="bg-green-500 hover:bg-green-700 text-white font-bold py-2 px-4 rounded"
        >
          Enable Auto Failover
        </button>
        <button
          onClick={() => window.electronAPI.sendSerial('dual_imu status')}
          className="bg-gray-500 hover:bg-gray-700 text-white font-bold py-2 px-4 rounded"
        >
          Refresh Status
        </button>
      </div>
    </div>
  );
};

export default DualIMUMonitor; 