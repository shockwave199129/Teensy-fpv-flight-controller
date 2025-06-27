import React, { useState, useEffect } from 'react';

const SystemInfo = () => {
  const [systemInfo, setSystemInfo] = useState({
    firmware_version: '',
    hardware_platform: 'Teensy 4.1',
    compilation_date: '',
    uptime: 0,
    cpu_usage: 0,
    memory_usage: 0,
    loop_frequency: 0
  });

  const [sensorDetection, setSensorDetection] = useState({
    detection_complete: false,
    last_scan: 0,
    imu: {},
    magnetometer: {},
    barometer: {},
    other: {},
    i2c_devices: []
  });

  const [isScanning, setIsScanning] = useState(false);
  const [i2cScanResults, setI2cScanResults] = useState([]);
  const [selectedTab, setSelectedTab] = useState('overview');

  useEffect(() => {
    // Request system info on component mount
    getSystemInfo();
    getSensorStatus();

    // Listen for serial data updates
    const handleSerialData = (event, data) => {
      try {
        // Try to parse JSON responses
        if (data.startsWith('{')) {
          const jsonData = JSON.parse(data);
          if (jsonData.sensors) {
            setSensorDetection(jsonData.sensors);
          }
          if (jsonData.system) {
            setSystemInfo(prev => ({ ...prev, ...jsonData.system }));
          }
        }
        // Handle sensor detection report
        else if (data.includes('=== Sensor Detection Report ===')) {
          // Parse text-based sensor report (fallback)
          console.log('Received sensor report:', data);
        }
        // Handle I2C scan results
        else if (data.startsWith('I2C devices found:')) {
          const devices = data.split(':')[1].split(',').map(addr => addr.trim()).filter(addr => addr);
          setSensorDetection(prev => ({ ...prev, i2c_devices: devices }));
        }
      } catch (e) {
        // Handle non-JSON responses
        console.log('Received data:', data);
      }
    };

    if (window.electronAPI) {
      window.electronAPI.onSerialData(handleSerialData);
    }

    return () => {
      if (window.electronAPI) {
        window.electronAPI.removeAllListeners('serial-data');
      }
    };
  }, []);

  const getSystemInfo = async () => {
    if (window.electronAPI) {
      try {
        await window.electronAPI.sendCommand('info');
        await window.electronAPI.sendCommand('status');
      } catch (error) {
        console.error('Error getting system info:', error);
      }
    }
  };

  const getSensorStatus = async () => {
    if (window.electronAPI) {
      try {
        await window.electronAPI.sendCommand('sensor_json');
      } catch (error) {
        console.error('Error getting sensor status:', error);
      }
    }
  };

  const performSensorScan = async () => {
    if (window.electronAPI) {
      setIsScanning(true);
      try {
        await window.electronAPI.sendCommand('detect_sensors');
        // Wait a moment for scan to complete, then get results
        setTimeout(async () => {
          await getSensorStatus();
          setIsScanning(false);
        }, 3000);
      } catch (error) {
        console.error('Error performing sensor scan:', error);
        setIsScanning(false);
      }
    }
  };

  const performI2CScan = async () => {
    if (window.electronAPI) {
      try {
        await window.electronAPI.sendCommand('i2c_scan');
      } catch (error) {
        console.error('Error performing I2C scan:', error);
      }
    }
  };

  const detectSpecificSensor = async (sensorType) => {
    if (window.electronAPI) {
      try {
        await window.electronAPI.sendCommand(`detect ${sensorType}`);
        // Update sensor status after detection
        setTimeout(() => getSensorStatus(), 1000);
      } catch (error) {
        console.error(`Error detecting ${sensorType} sensors:`, error);
      }
    }
  };

  const formatUptime = (seconds) => {
    const days = Math.floor(seconds / (24 * 3600));
    const hours = Math.floor((seconds % (24 * 3600)) / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = seconds % 60;
    
    if (days > 0) {
      return `${days}d ${hours}h ${minutes}m ${secs}s`;
    } else if (hours > 0) {
      return `${hours}h ${minutes}m ${secs}s`;
    } else if (minutes > 0) {
      return `${minutes}m ${secs}s`;
    } else {
      return `${secs}s`;
    }
  };

  const SensorStatusBadge = ({ present, name }) => (
    <div className={`inline-flex items-center px-3 py-1 rounded-full text-xs font-medium ${
      present 
        ? 'bg-green-100 text-green-800' 
        : 'bg-red-100 text-red-800'
    }`}>
      <div className={`w-2 h-2 rounded-full mr-2 ${
        present ? 'bg-green-500' : 'bg-red-500'
      }`}></div>
      {name}
    </div>
  );

  const SensorCard = ({ title, sensors, onDetect, icon }) => (
    <div className="bg-white p-6 rounded-lg shadow-md border">
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center">
          <span className="text-2xl mr-3">{icon}</span>
          <h3 className="text-lg font-semibold">{title}</h3>
        </div>
        <button
          onClick={onDetect}
          className="btn-secondary text-sm"
        >
          Detect
        </button>
      </div>
      
      <div className="space-y-2">
        {Object.entries(sensors).length > 0 ? (
          Object.entries(sensors).map(([key, present]) => (
            <SensorStatusBadge 
              key={key} 
              present={present} 
              name={key.toUpperCase().replace('_', ' ')} 
            />
          ))
        ) : (
          <div className="text-gray-500 text-sm">No detection data available</div>
        )}
      </div>
    </div>
  );

  const renderOverview = () => (
    <div className="space-y-6">
      {/* System Information */}
      <div className="bg-white p-6 rounded-lg shadow-md">
        <h3 className="text-lg font-semibold mb-4">System Information</h3>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Hardware Platform
            </label>
            <div className="text-lg">{systemInfo.hardware_platform}</div>
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Firmware Version
            </label>
            <div className="text-lg">{systemInfo.firmware_version || 'Unknown'}</div>
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Compilation Date
            </label>
            <div className="text-lg">{systemInfo.compilation_date || 'Unknown'}</div>
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Uptime
            </label>
            <div className="text-lg">{formatUptime(systemInfo.uptime)}</div>
          </div>
        </div>
      </div>

      {/* Performance Metrics */}
      <div className="bg-white p-6 rounded-lg shadow-md">
        <h3 className="text-lg font-semibold mb-4">Performance Metrics</h3>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{systemInfo.loop_frequency}Hz</div>
            <div className="text-sm text-gray-600">Loop Frequency</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{systemInfo.cpu_usage}%</div>
            <div className="text-sm text-gray-600">CPU Usage</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-orange-600">{systemInfo.memory_usage}%</div>
            <div className="text-sm text-gray-600">Memory Usage</div>
          </div>
        </div>
      </div>

      {/* Feature Status */}
      <div className="bg-white p-6 rounded-lg shadow-md">
        <h3 className="text-lg font-semibold mb-4">Active Features</h3>
        <div className="grid grid-cols-2 md:grid-cols-3 gap-4">
          <div className="flex items-center">
            <div className="w-3 h-3 bg-green-500 rounded-full mr-2"></div>
            <span className="text-sm">Cascaded PID Control</span>
          </div>
          <div className="flex items-center">
            <div className="w-3 h-3 bg-green-500 rounded-full mr-2"></div>
            <span className="text-sm">Mahony AHRS Filter</span>
          </div>
          <div className="flex items-center">
            <div className="w-3 h-3 bg-green-500 rounded-full mr-2"></div>
            <span className="text-sm">Dual IMU Fusion</span>
          </div>
          <div className="flex items-center">
            <div className="w-3 h-3 bg-green-500 rounded-full mr-2"></div>
            <span className="text-sm">Dynamic Filtering</span>
          </div>
          <div className="flex items-center">
            <div className="w-3 h-3 bg-green-500 rounded-full mr-2"></div>
            <span className="text-sm">Advanced Flight Modes</span>
          </div>
          <div className="flex items-center">
            <div className="w-3 h-3 bg-green-500 rounded-full mr-2"></div>
            <span className="text-sm">2kHz Control Loop</span>
          </div>
        </div>
      </div>
    </div>
  );

  const renderSensorDetection = () => (
    <div className="space-y-6">
      {/* Sensor Detection Controls */}
      <div className="bg-white p-6 rounded-lg shadow-md">
        <h3 className="text-lg font-semibold mb-4">Sensor Detection</h3>
        <div className="flex items-center space-x-4 mb-4">
          <button
            onClick={performSensorScan}
            disabled={isScanning}
            className={`btn-primary ${isScanning ? 'opacity-50 cursor-not-allowed' : ''}`}
          >
            {isScanning ? '🔍 Scanning...' : '🔍 Full Sensor Scan'}
          </button>
          <button
            onClick={performI2CScan}
            className="btn-secondary"
          >
            📡 I2C Bus Scan
          </button>
          <button
            onClick={getSensorStatus}
            className="btn-secondary"
          >
            🔄 Refresh Status
          </button>
        </div>
        
        {sensorDetection.detection_complete && (
          <div className="text-sm text-gray-600">
            Last scan: {new Date(sensorDetection.last_scan).toLocaleString()}
          </div>
        )}
      </div>

      {/* Sensor Categories */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
        <SensorCard
          title="IMU Sensors"
          sensors={sensorDetection.imu}
          onDetect={() => detectSpecificSensor('imu')}
          icon="⚖️"
        />
        
        <SensorCard
          title="Magnetometer"
          sensors={sensorDetection.magnetometer}
          onDetect={() => detectSpecificSensor('mag')}
          icon="🧭"
        />
        
        <SensorCard
          title="Barometer"
          sensors={sensorDetection.barometer}
          onDetect={() => detectSpecificSensor('baro')}
          icon="🌡️"
        />
        
        <SensorCard
          title="Other Sensors"
          sensors={sensorDetection.other}
          onDetect={() => detectSpecificSensor('power')}
          icon="🔌"
        />
      </div>

      {/* I2C Devices */}
      {sensorDetection.i2c_devices && sensorDetection.i2c_devices.length > 0 && (
        <div className="bg-white p-6 rounded-lg shadow-md">
          <h3 className="text-lg font-semibold mb-4">I2C Devices Found</h3>
          <div className="flex flex-wrap gap-2">
            {sensorDetection.i2c_devices.map((device, index) => (
              <div key={index} className="bg-blue-100 text-blue-800 px-3 py-1 rounded-full text-sm font-mono">
                {device}
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );

  const renderHardwareInfo = () => (
    <div className="space-y-6">
      {/* Pin Configuration */}
      <div className="bg-white p-6 rounded-lg shadow-md">
        <h3 className="text-lg font-semibold mb-4">Pin Configuration</h3>
        <div className="overflow-x-auto">
          <table className="w-full text-sm">
            <thead>
              <tr className="border-b">
                <th className="text-left py-2">Function</th>
                <th className="text-left py-2">Teensy 4.1 Pin</th>
                <th className="text-left py-2">Description</th>
              </tr>
            </thead>
            <tbody className="text-sm">
              <tr className="border-b">
                <td className="py-2 font-medium">Motor 1 PWM</td>
                <td className="py-2 font-mono">2</td>
                <td className="py-2">Front Right Motor</td>
              </tr>
              <tr className="border-b">
                <td className="py-2 font-medium">Motor 2 PWM</td>
                <td className="py-2 font-mono">3</td>
                <td className="py-2">Front Left Motor</td>
              </tr>
              <tr className="border-b">
                <td className="py-2 font-medium">Motor 3 PWM</td>
                <td className="py-2 font-mono">4</td>
                <td className="py-2">Rear Left Motor</td>
              </tr>
              <tr className="border-b">
                <td className="py-2 font-medium">Motor 4 PWM</td>
                <td className="py-2 font-mono">5</td>
                <td className="py-2">Rear Right Motor</td>
              </tr>
              <tr className="border-b">
                <td className="py-2 font-medium">RGB LED Strip</td>
                <td className="py-2 font-mono">6</td>
                <td className="py-2">Status LED Control</td>
              </tr>
              <tr className="border-b">
                <td className="py-2 font-medium">RC Receiver (UART)</td>
                <td className="py-2 font-mono">7 (RX)</td>
                <td className="py-2">SBUS/iBUS/ELRS Input</td>
              </tr>
              <tr className="border-b">
                <td className="py-2 font-medium">GPS (UART)</td>
                <td className="py-2 font-mono">0 (RX), 1 (TX)</td>
                <td className="py-2">GPS Communication</td>
              </tr>
              <tr className="border-b">
                <td className="py-2 font-medium">I2C SDA</td>
                <td className="py-2 font-mono">18</td>
                <td className="py-2">Sensor Communication</td>
              </tr>
              <tr className="border-b">
                <td className="py-2 font-medium">I2C SCL</td>
                <td className="py-2 font-mono">19</td>
                <td className="py-2">Sensor Communication</td>
              </tr>
              <tr className="border-b">
                <td className="py-2 font-medium">Sonar Trigger</td>
                <td className="py-2 font-mono">22</td>
                <td className="py-2">Ultrasonic Distance Sensor</td>
              </tr>
              <tr className="border-b">
                <td className="py-2 font-medium">Sonar Echo</td>
                <td className="py-2 font-mono">23</td>
                <td className="py-2">Ultrasonic Distance Sensor</td>
              </tr>
              <tr className="border-b">
                <td className="py-2 font-medium">Battery Voltage</td>
                <td className="py-2 font-mono">A0</td>
                <td className="py-2">Voltage Monitoring</td>
              </tr>
              <tr>
                <td className="py-2 font-medium">Current Sensor</td>
                <td className="py-2 font-mono">A1</td>
                <td className="py-2">Current Monitoring</td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>

      {/* Supported Hardware */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
        <div className="bg-white p-6 rounded-lg shadow-md">
          <h3 className="text-lg font-semibold mb-4">Supported IMU Sensors</h3>
          <div className="space-y-2 text-sm">
            <div>• MPU6050 - Basic 6-axis IMU</div>
            <div>• MPU9250 - 9-axis IMU with magnetometer</div>
            <div>• ICM20948 - Advanced 9-axis IMU</div>
            <div>• ICM42688P - High-performance 6-axis IMU</div>
            <div>• BMI270 - Low-power 6-axis IMU</div>
            <div>• LSM6DSO32 - High-g 6-axis IMU</div>
            <div>• BMI323 - Ultra-low power IMU</div>
            <div>• ICM20602 - Racing-grade IMU</div>
            <div>• LSM6DS33 - Compact 6-axis IMU</div>
          </div>
        </div>

        <div className="bg-white p-6 rounded-lg shadow-md">
          <h3 className="text-lg font-semibold mb-4">Supported RC Protocols</h3>
          <div className="space-y-2 text-sm">
            <div>• PPM - 4-8 channels, 22-50Hz</div>
            <div>• iBUS - 4-14 channels, 50Hz, telemetry</div>
            <div>• SBUS - 4-16 channels, 100Hz, digital</div>
            <div>• ExpressLRS - 4-16 channels, 50-1000Hz</div>
          </div>
        </div>
      </div>
    </div>
  );

  return (
    <div className="space-y-6">
      <h1 className="text-3xl font-bold">System Information</h1>

      {/* Navigation Tabs */}
      <div className="border-b border-gray-200">
        <nav className="-mb-px flex space-x-8">
          {[
            { id: 'overview', name: 'Overview', icon: '📊' },
            { id: 'sensors', name: 'Sensor Detection', icon: '🔍' },
            { id: 'hardware', name: 'Hardware Info', icon: '⚙️' }
          ].map((tab) => (
            <button
              key={tab.id}
              onClick={() => setSelectedTab(tab.id)}
              className={`py-2 px-1 border-b-2 font-medium text-sm flex items-center ${
                selectedTab === tab.id
                  ? 'border-blue-500 text-blue-600'
                  : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
              }`}
            >
              <span className="mr-2">{tab.icon}</span>
              {tab.name}
            </button>
          ))}
        </nav>
      </div>

      {/* Tab Content */}
      {selectedTab === 'overview' && renderOverview()}
      {selectedTab === 'sensors' && renderSensorDetection()}
      {selectedTab === 'hardware' && renderHardwareInfo()}
    </div>
  );
};

export default SystemInfo;
