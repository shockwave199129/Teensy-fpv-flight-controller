import React, { useState, useEffect } from 'react';

const CalibrationWizard = ({ sendCommand, connected }) => {
  const [calibrationStatus, setCalibrationStatus] = useState({
    system_calibrated: false,
    gyro: { calibrated: false, quality: 0 },
    accelerometer: { calibrated: false, quality: 0 },
    magnetometer: { calibrated: false, quality: 0 },
    esc: { calibrated: false, quality: 0 },
    rc: { calibrated: false, quality: 0 }
  });

  const [accelPositions, setAccelPositions] = useState({
    level: false,        // Position 1: Normal flight orientation
    upsideDown: false,   // Position 2: Upside down (180° roll)
    rightSide: false,    // Position 3: Right side down (90° roll right)
    leftSide: false,     // Position 4: Left side down (90° roll left)
    noseDown: false,     // Position 5: Nose down (90° pitch forward)
    noseUp: false        // Position 6: Nose up (90° pitch backward)
  });

  const [showAccelPositions, setShowAccelPositions] = useState(false);
  
  const [currentStep, setCurrentStep] = useState(0);
  const [calibrating, setCalibrating] = useState(false);
  const [calibrationProgress, setCalibrationProgress] = useState('');
  const [calibrationLog, setCalibrationLog] = useState([]);

  const calibrationSteps = [
    {
      id: 'sensor_detection',
      title: 'Sensor Detection',
      description: 'Detect and verify all connected sensors',
      status: 'pending'
    },
    {
      id: 'gyro',
      title: 'Gyroscope Calibration',
      description: 'Calibrate gyroscope bias (keep drone still)',
      status: 'pending'
    },
    {
      id: 'accelerometer',
      title: 'Accelerometer Calibration',
      description: '6-position level calibration',
      status: 'pending'
    },
    {
      id: 'magnetometer',
      title: 'Magnetometer Calibration',
      description: 'Hard/soft iron calibration (rotate drone)',
      status: 'pending'
    },
    {
      id: 'rc',
      title: 'RC Receiver Setup',
      description: 'Channel mapping and range calibration',
      status: 'pending'
    },
    {
      id: 'esc',
      title: 'ESC Calibration',
      description: 'Motor and ESC setup',
      status: 'pending'
    },
    {
      id: 'final_check',
      title: 'Final Verification',
      description: 'Verify all calibrations and save to EEPROM',
      status: 'pending'
    }
  ];

  const [steps, setSteps] = useState(calibrationSteps);

  useEffect(() => {
    if (connected) {
      refreshCalibrationStatus();
    }

    // Listen for serial data
    window.electronAPI.onSerialData((event, data) => {
      handleSerialData(data);
    });

    return () => {
      window.electronAPI.removeAllListeners('serial-data');
    };
  }, [connected]);

  const handleSerialData = (data) => {
    // Add to calibration log
    setCalibrationLog(prev => [...prev.slice(-50), data]); // Keep last 50 lines

    // Parse calibration status updates
    if (data.includes('CALIBRATION_OK')) {
      setCalibrationStatus(prev => ({ ...prev, system_calibrated: true }));
    } else if (data.includes('calibration complete')) {
      setCalibrating(false);
      refreshCalibrationStatus();
    } else if (data.includes('Progress:') || data.includes('Position')) {
      setCalibrationProgress(data);
    }

    // Parse accelerometer position completion
    if (data.includes('Position 1 complete') || data.includes('Level position complete')) {
      setAccelPositions(prev => ({ ...prev, level: true }));
    } else if (data.includes('Position 2 complete') || data.includes('Upside down complete')) {
      setAccelPositions(prev => ({ ...prev, upsideDown: true }));
    } else if (data.includes('Position 3 complete') || data.includes('Right side complete')) {
      setAccelPositions(prev => ({ ...prev, rightSide: true }));
    } else if (data.includes('Position 4 complete') || data.includes('Left side complete')) {
      setAccelPositions(prev => ({ ...prev, leftSide: true }));
    } else if (data.includes('Position 5 complete') || data.includes('Nose down complete')) {
      setAccelPositions(prev => ({ ...prev, noseDown: true }));
    } else if (data.includes('Position 6 complete') || data.includes('Nose up complete')) {
      setAccelPositions(prev => ({ ...prev, noseUp: true }));
    }
  };

  const refreshCalibrationStatus = async () => {
    try {
      await sendCommand('calibration status');
      // Status will be updated via serial data handler
    } catch (error) {
      console.error('Failed to get calibration status:', error);
    }
  };

  const startCalibrationStep = async (stepId) => {
    setCalibrating(true);
    setCalibrationProgress('Starting calibration...');
    
    try {
      switch (stepId) {
        case 'sensor_detection':
          await sendCommand('detect_sensors');
          break;
        case 'gyro':
          await sendCommand('calibrate gyro');
          break;
        case 'accelerometer':
          setShowAccelPositions(true);
          await sendCommand('calibrate accel');
          break;
        case 'magnetometer':
          await sendCommand('calibrate mag');
          break;
        case 'rc':
          await sendCommand('calibrate rc');
          break;
        case 'esc':
          await sendCommand('calibrate esc');
          break;
        case 'final_check':
          await sendCommand('calibration check');
          await sendCommand('save calibration');
          break;
      }
    } catch (error) {
      console.error('Calibration step failed:', error);
      setCalibrating(false);
    }
  };

  const startAccelPositionCalibration = async (position) => {
    setCalibrating(true);
    setCalibrationProgress(`Calibrating ${position} position...`);
    
    try {
      await sendCommand(`calibrate accel position ${position}`);
    } catch (error) {
      console.error(`Position ${position} calibration failed:`, error);
      setCalibrating(false);
    }
  };

  const resetAccelCalibration = () => {
    setAccelPositions({
      level: false,
      upsideDown: false,
      rightSide: false,
      leftSide: false,
      noseDown: false,
      noseUp: false
    });
    setShowAccelPositions(false);
  };

  const startFullWizard = async () => {
    try {
      await sendCommand('calibration wizard');
    } catch (error) {
      console.error('Failed to start calibration wizard:', error);
    }
  };

  const resetAllCalibrations = async () => {
    if (window.confirm('This will reset ALL calibration data. Are you sure?')) {
      try {
        await sendCommand('reset calibration');
        refreshCalibrationStatus();
      } catch (error) {
        console.error('Failed to reset calibrations:', error);
      }
    }
  };

  const getStatusIcon = (calibrated, quality) => {
    if (!calibrated) return '❌';
    if (quality >= 90) return '✅';
    if (quality >= 75) return '⚠️';
    return '❌';
  };

  const getQualityColor = (quality) => {
    if (quality >= 90) return 'text-green-600';
    if (quality >= 75) return 'text-yellow-600';
    return 'text-red-600';
  };

  if (!connected) {
    return (
      <div className="p-6">
        <div className="bg-yellow-100 border border-yellow-400 text-yellow-700 px-4 py-3 rounded">
          <strong>Warning:</strong> Please connect to the flight controller to access calibration features.
        </div>
      </div>
    );
  }

  return (
    <div className="p-6 max-w-6xl mx-auto">
      <div className="mb-6">
        <h1 className="text-3xl font-bold text-gray-900 mb-2">Flight Controller Setup & Calibration</h1>
        <p className="text-gray-600">
          Complete all calibrations before first flight. This ensures safe and accurate flight performance.
        </p>
      </div>

      {/* Safety Warning */}
      {!calibrationStatus.system_calibrated && (
        <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-6">
          <div className="flex items-center">
            <span className="text-2xl mr-3">⚠️</span>
            <div>
              <strong>FLIGHT BLOCKED:</strong> System is not fully calibrated.
              <br />
              Complete all required calibrations below before attempting to arm the drone.
            </div>
          </div>
        </div>
      )}

      {/* System Status Overview */}
      <div className="bg-white rounded-lg shadow-md mb-6">
        <div className="px-6 py-4 border-b border-gray-200">
          <h2 className="text-xl font-semibold">Calibration Status Overview</h2>
        </div>
        <div className="p-6">
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
            <div className="flex items-center justify-between p-3 bg-gray-50 rounded">
              <span>System Ready</span>
              <span className="text-2xl">
                {calibrationStatus.system_calibrated ? '✅' : '❌'}
              </span>
            </div>
            <div className="flex items-center justify-between p-3 bg-gray-50 rounded">
              <span>Gyroscope</span>
              <div className="flex items-center">
                <span className="text-2xl mr-2">
                  {getStatusIcon(calibrationStatus.gyro.calibrated, calibrationStatus.gyro.quality)}
                </span>
                <span className={`font-medium ${getQualityColor(calibrationStatus.gyro.quality)}`}>
                  {calibrationStatus.gyro.quality}%
                </span>
              </div>
            </div>
            <div className="flex items-center justify-between p-3 bg-gray-50 rounded">
              <span>Accelerometer</span>
              <div className="flex items-center">
                <span className="text-2xl mr-2">
                  {getStatusIcon(calibrationStatus.accelerometer.calibrated, calibrationStatus.accelerometer.quality)}
                </span>
                <span className={`font-medium ${getQualityColor(calibrationStatus.accelerometer.quality)}`}>
                  {calibrationStatus.accelerometer.quality}%
                </span>
              </div>
            </div>
            <div className="flex items-center justify-between p-3 bg-gray-50 rounded">
              <span>Magnetometer</span>
              <div className="flex items-center">
                <span className="text-2xl mr-2">
                  {getStatusIcon(calibrationStatus.magnetometer.calibrated, calibrationStatus.magnetometer.quality)}
                </span>
                <span className={`font-medium ${getQualityColor(calibrationStatus.magnetometer.quality)}`}>
                  {calibrationStatus.magnetometer.quality}%
                </span>
              </div>
            </div>
            <div className="flex items-center justify-between p-3 bg-gray-50 rounded">
              <span>RC Receiver</span>
              <div className="flex items-center">
                <span className="text-2xl mr-2">
                  {getStatusIcon(calibrationStatus.rc.calibrated, calibrationStatus.rc.quality)}
                </span>
                <span className={`font-medium ${getQualityColor(calibrationStatus.rc.quality)}`}>
                  {calibrationStatus.rc.quality}%
                </span>
              </div>
            </div>
            <div className="flex items-center justify-between p-3 bg-gray-50 rounded">
              <span>ESC/Motors</span>
              <div className="flex items-center">
                <span className="text-2xl mr-2">
                  {getStatusIcon(calibrationStatus.esc.calibrated, calibrationStatus.esc.quality)}
                </span>
                <span className={`font-medium ${getQualityColor(calibrationStatus.esc.quality)}`}>
                  {calibrationStatus.esc.quality}%
                </span>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Quick Actions */}
      <div className="grid grid-cols-1 md:grid-cols-4 gap-4 mb-6">
        <button
          onClick={startFullWizard}
          disabled={calibrating}
          className="bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white font-medium py-3 px-4 rounded-lg flex items-center justify-center"
        >
          🧙‍♂️ Setup Wizard
        </button>
        <button
          onClick={refreshCalibrationStatus}
          disabled={calibrating}
          className="bg-green-600 hover:bg-green-700 disabled:bg-gray-400 text-white font-medium py-3 px-4 rounded-lg flex items-center justify-center"
        >
          🔄 Refresh Status
        </button>
        <button
          onClick={() => sendCommand('save calibration')}
          disabled={calibrating}
          className="bg-purple-600 hover:bg-purple-700 disabled:bg-gray-400 text-white font-medium py-3 px-4 rounded-lg flex items-center justify-center"
        >
          💾 Save All
        </button>
        <button
          onClick={resetAllCalibrations}
          disabled={calibrating}
          className="bg-red-600 hover:bg-red-700 disabled:bg-gray-400 text-white font-medium py-3 px-4 rounded-lg flex items-center justify-center"
        >
          🗑️ Reset All
        </button>
      </div>

      {/* Individual Calibration Steps */}
      <div className="bg-white rounded-lg shadow-md mb-6">
        <div className="px-6 py-4 border-b border-gray-200">
          <h2 className="text-xl font-semibold">Individual Calibrations</h2>
        </div>
        <div className="p-6">
          <div className="space-y-4">
            {steps.map((step, index) => (
              <div key={step.id} className="border border-gray-200 rounded-lg p-4">
                <div className="flex items-center justify-between">
                  <div className="flex items-center">
                    <span className="w-8 h-8 bg-blue-100 text-blue-800 rounded-full flex items-center justify-center text-sm font-medium mr-4">
                      {index + 1}
                    </span>
                    <div>
                      <h3 className="font-medium text-gray-900">{step.title}</h3>
                      <p className="text-sm text-gray-600">{step.description}</p>
                    </div>
                  </div>
                  <div className="flex space-x-2">
                    {step.id === 'accelerometer' && (
                      <button
                        onClick={() => setShowAccelPositions(!showAccelPositions)}
                        className="bg-purple-600 hover:bg-purple-700 text-white font-medium py-2 px-4 rounded"
                      >
                        {showAccelPositions ? 'Hide Positions' : 'Show Positions'}
                      </button>
                    )}
                    <button
                      onClick={() => startCalibrationStep(step.id)}
                      disabled={calibrating}
                      className="bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white font-medium py-2 px-4 rounded"
                    >
                      {calibrating ? 'Running...' : 'Start'}
                    </button>
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>

      {/* Accelerometer 6-Position Calibration */}
      {showAccelPositions && (
        <div className="bg-white rounded-lg shadow-md mb-6">
          <div className="px-6 py-4 border-b border-gray-200">
            <div className="flex items-center justify-between">
              <h2 className="text-xl font-semibold">Accelerometer 6-Position Calibration</h2>
              <button
                onClick={resetAccelCalibration}
                className="bg-red-600 hover:bg-red-700 text-white font-medium py-2 px-4 rounded text-sm"
              >
                Reset Positions
              </button>
            </div>
            <p className="text-sm text-gray-600 mt-2">
              Place the drone in each position and click "Calibrate" when stable. Hold each position steady for 10-15 seconds.
            </p>
          </div>
          <div className="p-6">
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
              
              {/* Position 1: Level */}
              <div className="border-2 border-gray-200 rounded-lg p-4 bg-gray-50">
                <div className="flex items-center justify-between mb-3">
                  <h3 className="font-semibold text-gray-900">Position 1: Level</h3>
                  <span className="text-2xl">
                    {accelPositions.level ? '✅' : '⏳'}
                  </span>
                </div>
                <div className="flex justify-center mb-4">
                  <div className="relative">
                    <div className="w-20 h-12 bg-blue-500 rounded border-2 border-gray-600 relative">
                      <div className="absolute top-1 left-1 w-3 h-3 bg-red-500 rounded-full"></div>
                      <div className="text-white text-xs text-center leading-8">LEVEL</div>
                    </div>
                    <div className="w-20 h-1 bg-green-500 mt-1"></div>
                  </div>
                </div>
                <p className="text-sm text-gray-600 mb-3">Normal flight orientation on flat surface</p>
                <button
                  onClick={() => startAccelPositionCalibration('level')}
                  disabled={calibrating || accelPositions.level}
                  className={`w-full py-2 px-4 rounded font-medium ${
                    accelPositions.level 
                      ? 'bg-green-500 text-white cursor-not-allowed' 
                      : 'bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white'
                  }`}
                >
                  {accelPositions.level ? 'Completed' : 'Calibrate'}
                </button>
              </div>

              {/* Position 2: Upside Down */}
              <div className="border-2 border-gray-200 rounded-lg p-4 bg-gray-50">
                <div className="flex items-center justify-between mb-3">
                  <h3 className="font-semibold text-gray-900">Position 2: Upside Down</h3>
                  <span className="text-2xl">
                    {accelPositions.upsideDown ? '✅' : '⏳'}
                  </span>
                </div>
                <div className="flex justify-center mb-4">
                  <div className="relative">
                    <div className="w-20 h-1 bg-green-500 mb-1"></div>
                    <div className="w-20 h-12 bg-blue-500 rounded border-2 border-gray-600 relative transform rotate-180">
                      <div className="absolute top-1 left-1 w-3 h-3 bg-red-500 rounded-full"></div>
                      <div className="text-white text-xs text-center leading-8 transform rotate-180">FLIP</div>
                    </div>
                  </div>
                </div>
                <p className="text-sm text-gray-600 mb-3">Flipped upside down (180° roll)</p>
                <button
                  onClick={() => startAccelPositionCalibration('upside_down')}
                  disabled={calibrating || accelPositions.upsideDown}
                  className={`w-full py-2 px-4 rounded font-medium ${
                    accelPositions.upsideDown 
                      ? 'bg-green-500 text-white cursor-not-allowed' 
                      : 'bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white'
                  }`}
                >
                  {accelPositions.upsideDown ? 'Completed' : 'Calibrate'}
                </button>
              </div>

              {/* Position 3: Right Side Down */}
              <div className="border-2 border-gray-200 rounded-lg p-4 bg-gray-50">
                <div className="flex items-center justify-between mb-3">
                  <h3 className="font-semibold text-gray-900">Position 3: Right Side</h3>
                  <span className="text-2xl">
                    {accelPositions.rightSide ? '✅' : '⏳'}
                  </span>
                </div>
                <div className="flex justify-center mb-4">
                  <div className="relative">
                    <div className="w-12 h-20 bg-blue-500 rounded border-2 border-gray-600 relative transform rotate-90">
                      <div className="absolute top-1 left-1 w-3 h-3 bg-red-500 rounded-full"></div>
                      <div className="text-white text-xs text-center leading-8 transform -rotate-90 mt-2">RIGHT</div>
                    </div>
                  </div>
                </div>
                <p className="text-sm text-gray-600 mb-3">Right side down (90° roll right)</p>
                <button
                  onClick={() => startAccelPositionCalibration('right_side')}
                  disabled={calibrating || accelPositions.rightSide}
                  className={`w-full py-2 px-4 rounded font-medium ${
                    accelPositions.rightSide 
                      ? 'bg-green-500 text-white cursor-not-allowed' 
                      : 'bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white'
                  }`}
                >
                  {accelPositions.rightSide ? 'Completed' : 'Calibrate'}
                </button>
              </div>

              {/* Position 4: Left Side Down */}
              <div className="border-2 border-gray-200 rounded-lg p-4 bg-gray-50">
                <div className="flex items-center justify-between mb-3">
                  <h3 className="font-semibold text-gray-900">Position 4: Left Side</h3>
                  <span className="text-2xl">
                    {accelPositions.leftSide ? '✅' : '⏳'}
                  </span>
                </div>
                <div className="flex justify-center mb-4">
                  <div className="relative">
                    <div className="w-12 h-20 bg-blue-500 rounded border-2 border-gray-600 relative transform -rotate-90">
                      <div className="absolute top-1 left-1 w-3 h-3 bg-red-500 rounded-full"></div>
                      <div className="text-white text-xs text-center leading-8 transform rotate-90 mt-2">LEFT</div>
                    </div>
                  </div>
                </div>
                <p className="text-sm text-gray-600 mb-3">Left side down (90° roll left)</p>
                <button
                  onClick={() => startAccelPositionCalibration('left_side')}
                  disabled={calibrating || accelPositions.leftSide}
                  className={`w-full py-2 px-4 rounded font-medium ${
                    accelPositions.leftSide 
                      ? 'bg-green-500 text-white cursor-not-allowed' 
                      : 'bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white'
                  }`}
                >
                  {accelPositions.leftSide ? 'Completed' : 'Calibrate'}
                </button>
              </div>

              {/* Position 5: Nose Down */}
              <div className="border-2 border-gray-200 rounded-lg p-4 bg-gray-50">
                <div className="flex items-center justify-between mb-3">
                  <h3 className="font-semibold text-gray-900">Position 5: Nose Down</h3>
                  <span className="text-2xl">
                    {accelPositions.noseDown ? '✅' : '⏳'}
                  </span>
                </div>
                <div className="flex justify-center mb-4">
                  <div className="relative">
                    <div className="w-20 h-12 bg-blue-500 rounded border-2 border-gray-600 relative transform -rotate-12">
                      <div className="absolute top-1 left-1 w-3 h-3 bg-red-500 rounded-full"></div>
                      <div className="text-white text-xs text-center leading-8 transform rotate-12">NOSE↓</div>
                    </div>
                  </div>
                </div>
                <p className="text-sm text-gray-600 mb-3">Nose pointing down (90° pitch forward)</p>
                <button
                  onClick={() => startAccelPositionCalibration('nose_down')}
                  disabled={calibrating || accelPositions.noseDown}
                  className={`w-full py-2 px-4 rounded font-medium ${
                    accelPositions.noseDown 
                      ? 'bg-green-500 text-white cursor-not-allowed' 
                      : 'bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white'
                  }`}
                >
                  {accelPositions.noseDown ? 'Completed' : 'Calibrate'}
                </button>
              </div>

              {/* Position 6: Nose Up */}
              <div className="border-2 border-gray-200 rounded-lg p-4 bg-gray-50">
                <div className="flex items-center justify-between mb-3">
                  <h3 className="font-semibold text-gray-900">Position 6: Nose Up</h3>
                  <span className="text-2xl">
                    {accelPositions.noseUp ? '✅' : '⏳'}
                  </span>
                </div>
                <div className="flex justify-center mb-4">
                  <div className="relative">
                    <div className="w-20 h-12 bg-blue-500 rounded border-2 border-gray-600 relative transform rotate-12">
                      <div className="absolute top-1 left-1 w-3 h-3 bg-red-500 rounded-full"></div>
                      <div className="text-white text-xs text-center leading-8 transform -rotate-12">NOSE↑</div>
                    </div>
                  </div>
                </div>
                <p className="text-sm text-gray-600 mb-3">Nose pointing up (90° pitch backward)</p>
                <button
                  onClick={() => startAccelPositionCalibration('nose_up')}
                  disabled={calibrating || accelPositions.noseUp}
                  className={`w-full py-2 px-4 rounded font-medium ${
                    accelPositions.noseUp 
                      ? 'bg-green-500 text-white cursor-not-allowed' 
                      : 'bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white'
                  }`}
                >
                  {accelPositions.noseUp ? 'Completed' : 'Calibrate'}
                </button>
              </div>

            </div>

            {/* Position Progress Summary */}
            <div className="mt-6 p-4 bg-blue-50 rounded-lg">
              <h4 className="font-semibold text-blue-900 mb-2">Calibration Progress</h4>
              <div className="flex flex-wrap gap-2">
                {Object.entries(accelPositions).map(([position, completed]) => (
                  <span 
                    key={position}
                    className={`px-3 py-1 rounded-full text-sm font-medium ${
                      completed 
                        ? 'bg-green-100 text-green-800' 
                        : 'bg-gray-100 text-gray-600'
                    }`}
                  >
                    {position.replace(/([A-Z])/g, ' $1').replace(/^./, str => str.toUpperCase())} {completed ? '✓' : '○'}
                  </span>
                ))}
              </div>
              <div className="mt-3">
                <div className="w-full bg-gray-200 rounded-full h-2">
                  <div 
                    className="bg-blue-600 h-2 rounded-full transition-all duration-300"
                    style={{ 
                      width: `${(Object.values(accelPositions).filter(Boolean).length / 6) * 100}%` 
                    }}
                  ></div>
                </div>
                <p className="text-sm text-gray-600 mt-1">
                  {Object.values(accelPositions).filter(Boolean).length} of 6 positions completed
                </p>
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Calibration Progress */}
      {calibrating && (
        <div className="bg-white rounded-lg shadow-md mb-6">
          <div className="px-6 py-4 border-b border-gray-200">
            <h2 className="text-xl font-semibold">Calibration in Progress</h2>
          </div>
          <div className="p-6">
            <div className="flex items-center mb-4">
              <div className="animate-spin rounded-full h-6 w-6 border-b-2 border-blue-600 mr-3"></div>
              <span className="text-gray-700">{calibrationProgress || 'Calibrating...'}</span>
            </div>
            <div className="bg-gray-100 rounded p-3">
              <p className="text-sm text-gray-600 mb-2">Follow the instructions displayed below:</p>
              <div className="bg-black text-green-400 p-3 rounded font-mono text-sm h-32 overflow-y-auto">
                {calibrationLog.slice(-10).map((line, index) => (
                  <div key={index}>{line}</div>
                ))}
              </div>
            </div>
          </div>
        </div>
      )}

      {/* System Ready Confirmation */}
      {calibrationStatus.system_calibrated && (
        <div className="bg-green-100 border border-green-400 text-green-700 px-4 py-3 rounded">
          <div className="flex items-center">
            <span className="text-2xl mr-3">✅</span>
            <div>
              <strong>SYSTEM READY FOR FLIGHT!</strong>
              <br />
              All required calibrations are complete. The drone can now be armed safely.
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default CalibrationWizard; 