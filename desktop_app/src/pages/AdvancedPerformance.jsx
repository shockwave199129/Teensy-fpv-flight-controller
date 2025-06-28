import React, { useState, useEffect } from 'react';

const AdvancedPerformance = ({ sendCommand, connected }) => {
  const [performanceData, setPerformanceData] = useState({
    pid: {
      adaptive_gains_enabled: false,
      current_phase: 'PHASE_GROUND',
      aggressiveness_level: 1.0,
      smoothness_factor: 1.0,
      control_effort: [0, 0, 0],
      efficiency_score: 0,
      oscillation_frequency: [0, 0, 0]
    },
    motors: {
      rpm_based_filtering: false,
      health_monitoring: true,
      motor_rpms: [0, 0, 0, 0],
      motor_temps: [0, 0, 0, 0],
      efficiency_scores: [0, 0, 0, 0],
      vibration_levels: [0, 0, 0, 0],
      warning_flags: [false, false, false, false]
    },
    sensors: {
      enhanced_fusion: false,
      imu_quality: 0,
      gps_accuracy: 0,
      mag_quality: 0,
      baro_stability: 0,
      fusion_weights: {
        imu: 1.0,
        gps: 0.0,
        mag: 0.0,
        baro: 0.0
      }
    }
  });

  const [activeTab, setActiveTab] = useState('pid');
  const [autoUpdate, setAutoUpdate] = useState(true);

  useEffect(() => {
    let interval;
    if (autoUpdate && connected) {
      interval = setInterval(() => {
        requestPerformanceData();
      }, 1000);
    }
    return () => {
      if (interval) clearInterval(interval);
    };
  }, [autoUpdate, connected]);

  const requestPerformanceData = () => {
    sendCommand('performance all');
  };

  const getQualityColor = (score) => {
    if (score >= 90) return 'text-green-600';
    if (score >= 70) return 'text-yellow-600';
    if (score >= 50) return 'text-orange-600';
    return 'text-red-600';
  };

  const PerformanceMetric = ({ label, value, unit, color = 'text-gray-900' }) => (
    <div className="bg-white p-4 rounded-lg shadow">
      <div className="text-sm text-gray-600">{label}</div>
      <div className={`text-2xl font-bold ${color}`}>
        {typeof value === 'number' ? value.toFixed(1) : value}{unit}
      </div>
    </div>
  );

  return (
    <div className="p-6 max-w-7xl mx-auto">
      <div className="mb-6">
        <h1 className="text-3xl font-bold text-gray-900 mb-2">Advanced Flight Performance Monitor</h1>
        <p className="text-gray-600">
          Monitor and configure Phase 3 advanced flight control features
        </p>
      </div>

      {/* Quick Actions */}
      <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-6">
        <button 
          onClick={() => sendCommand('pid adaptive enable')}
          className="bg-blue-600 text-white px-4 py-2 rounded-lg hover:bg-blue-700 transition-colors"
        >
          Enable Advanced Features
        </button>
        <button 
          onClick={() => sendCommand('pid characteristics aggressive 0.7 smooth 1.5')}
          className="bg-green-600 text-white px-4 py-2 rounded-lg hover:bg-green-700 transition-colors"
        >
          Conservative Setup
        </button>
        <button 
          onClick={() => sendCommand('pid characteristics aggressive 1.5 smooth 1.0')}
          className="bg-yellow-600 text-white px-4 py-2 rounded-lg hover:bg-yellow-700 transition-colors"
        >
          Sport Setup
        </button>
        <button 
          onClick={() => sendCommand('pid characteristics aggressive 2.0 smooth 0.8')}
          className="bg-red-600 text-white px-4 py-2 rounded-lg hover:bg-red-700 transition-colors"
        >
          Race Setup
        </button>
      </div>

      {/* PID Performance Section */}
      <div className="bg-white p-6 rounded-lg shadow mb-6">
        <h3 className="text-xl font-semibold mb-4">🧠 Adaptive PID Control Status</h3>
        
        <div className="grid grid-cols-2 md:grid-cols-3 gap-4 mb-6">
          <PerformanceMetric 
            label="Aggressiveness Level" 
            value={performanceData.pid.aggressiveness_level} 
            unit=""
            color={performanceData.pid.aggressiveness_level > 1.5 ? 'text-red-600' : 'text-green-600'}
          />
          <PerformanceMetric 
            label="Smoothness Factor" 
            value={performanceData.pid.smoothness_factor} 
            unit=""
            color={performanceData.pid.smoothness_factor > 1.5 ? 'text-green-600' : 'text-orange-600'}
          />
          <PerformanceMetric 
            label="Efficiency Score" 
            value={performanceData.pid.efficiency_score} 
            unit="%"
            color={getQualityColor(performanceData.pid.efficiency_score)}
          />
        </div>

        {/* Control Effort */}
        <div className="mb-4">
          <h4 className="font-semibold mb-2">Control Effort (% of available authority)</h4>
          <div className="grid grid-cols-3 gap-4">
            {['Roll', 'Pitch', 'Yaw'].map((axis, index) => (
              <div key={axis} className="bg-gray-50 p-3 rounded">
                <div className="text-sm text-gray-600">{axis}</div>
                <div className="flex items-center">
                  <div className="flex-1 bg-gray-200 rounded-full h-2 mr-2">
                    <div 
                      className={`h-2 rounded-full ${
                        performanceData.pid.control_effort[index] > 80 ? 'bg-red-500' :
                        performanceData.pid.control_effort[index] > 60 ? 'bg-yellow-500' : 'bg-green-500'
                      }`}
                      style={{ width: `${performanceData.pid.control_effort[index]}%` }}
                    ></div>
                  </div>
                  <span className="text-sm font-bold">{performanceData.pid.control_effort[index].toFixed(1)}%</span>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>

      {/* Motor Health Section */}
      <div className="bg-white p-6 rounded-lg shadow mb-6">
        <h3 className="text-xl font-semibold mb-4">🚁 Motor Health Monitoring</h3>
        
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 mb-6">
          {performanceData.motors.motor_rpms.map((rpm, index) => (
            <div key={index} className={`p-4 rounded-lg shadow ${performanceData.motors.warning_flags[index] ? 'bg-red-50 border-red-200' : 'bg-white'}`}>
              <div className="flex justify-between items-center mb-2">
                <h4 className="font-semibold">Motor {index + 1}</h4>
                {performanceData.motors.warning_flags[index] && <span className="text-red-600 text-sm">⚠️ Warning</span>}
              </div>
              <div className="grid grid-cols-2 gap-2 text-sm">
                <div>RPM: <span className="font-bold">{rpm}</span></div>
                <div>Temp: <span className="font-bold">{performanceData.motors.motor_temps[index]}°C</span></div>
                <div>Efficiency: <span className={`font-bold ${getQualityColor(performanceData.motors.efficiency_scores[index])}`}>{performanceData.motors.efficiency_scores[index].toFixed(1)}%</span></div>
                <div>Vibration: <span className="font-bold">{performanceData.motors.vibration_levels[index].toFixed(1)}%</span></div>
              </div>
            </div>
          ))}
        </div>
      </div>

      {/* Sensor Fusion Section */}
      <div className="bg-white p-6 rounded-lg shadow mb-6">
        <h3 className="text-xl font-semibold mb-4">🔄 Enhanced Sensor Fusion</h3>

        {/* Sensor Quality Scores */}
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-6">
          <PerformanceMetric 
            label="IMU Quality" 
            value={performanceData.sensors.imu_quality} 
            unit="%"
            color={getQualityColor(performanceData.sensors.imu_quality)}
          />
          <PerformanceMetric 
            label="GPS Accuracy" 
            value={performanceData.sensors.gps_accuracy} 
            unit="%"
            color={getQualityColor(performanceData.sensors.gps_accuracy)}
          />
          <PerformanceMetric 
            label="Magnetometer Quality" 
            value={performanceData.sensors.mag_quality} 
            unit="%"
            color={getQualityColor(performanceData.sensors.mag_quality)}
          />
          <PerformanceMetric 
            label="Barometer Stability" 
            value={performanceData.sensors.baro_stability} 
            unit="%"
            color={getQualityColor(performanceData.sensors.baro_stability)}
          />
        </div>

        {/* Fusion Weights */}
        <div className="mb-6">
          <h4 className="font-semibold mb-3">Adaptive Fusion Weights</h4>
          <div className="space-y-3">
            {Object.entries(performanceData.sensors.fusion_weights).map(([sensor, weight]) => (
              <div key={sensor} className="flex items-center">
                <div className="w-20 text-sm font-medium text-gray-700 capitalize">{sensor}:</div>
                <div className="flex-1 bg-gray-200 rounded-full h-3 mx-3">
                  <div 
                    className="h-3 bg-blue-500 rounded-full transition-all duration-300"
                    style={{ width: `${weight * 100}%` }}
                  ></div>
                </div>
                <div className="w-12 text-sm font-bold text-right">{(weight * 100).toFixed(0)}%</div>
              </div>
            ))}
          </div>
        </div>
      </div>

      {/* Real-time Commands */}
      <div className="bg-gray-50 p-4 rounded-lg">
        <h3 className="font-semibold mb-3">Quick Commands</h3>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-2">
          <button 
            onClick={() => sendCommand('pid performance')}
            className="bg-blue-500 text-white px-3 py-2 rounded text-sm hover:bg-blue-600 transition-colors"
          >
            PID Performance
          </button>
          <button 
            onClick={() => sendCommand('motor health')}
            className="bg-green-500 text-white px-3 py-2 rounded text-sm hover:bg-green-600 transition-colors"
          >
            Motor Health
          </button>
          <button 
            onClick={() => sendCommand('sensor quality')}
            className="bg-purple-500 text-white px-3 py-2 rounded text-sm hover:bg-purple-600 transition-colors"
          >
            Sensor Quality
          </button>
          <button 
            onClick={() => sendCommand('fusion weights')}
            className="bg-orange-500 text-white px-3 py-2 rounded text-sm hover:bg-orange-600 transition-colors"
          >
            Fusion Weights
          </button>
        </div>
      </div>

      {!connected && (
        <div className="bg-yellow-50 border border-yellow-200 rounded-lg p-4 mt-6">
          <p className="text-yellow-800">
            ⚠️ Flight controller not connected. Connect to view real-time performance data.
          </p>
        </div>
      )}
    </div>
  );
};

export default AdvancedPerformance; 