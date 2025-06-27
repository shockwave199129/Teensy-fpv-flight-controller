import React, { useState, useEffect } from 'react';

const SensorCalibration = ({ sendCommand, connected }) => {
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
  const [currentAccelPosition, setCurrentAccelPosition] = useState(null);
  const [accelCalibrationActive, setAccelCalibrationActive] = useState(false);
  
  // Magnetometer calibration timer
  const [magCalibrationActive, setMagCalibrationActive] = useState(false);
  const [magTimer, setMagTimer] = useState(0);
  const [magCalibrationInstructions, setMagCalibrationInstructions] = useState('');
  
  const [currentStep, setCurrentStep] = useState(0);
  const [calibrating, setCalibrating] = useState(false);
  const [calibrationProgress, setCalibrationProgress] = useState('');
  const [calibrationLog, setCalibrationLog] = useState([]);

  const accelPositionData = [
    {
      key: 'level',
      name: 'Level',
      description: 'Place drone in normal flight orientation (level)',
      icon: '🛩️',
      instruction: 'Place the drone flat and level on a stable surface'
    },
    {
      key: 'upsideDown',
      name: 'Upside Down',
      description: 'Flip drone upside down (180° roll)',
      icon: '🔄',
      instruction: 'Carefully flip the drone completely upside down'
    },
    {
      key: 'rightSide',
      name: 'Right Side',
      description: 'Right side down (90° roll right)',
      icon: '↪️',
      instruction: 'Tilt drone 90° to the right side'
    },
    {
      key: 'leftSide',
      name: 'Left Side',
      description: 'Left side down (90° roll left)',
      icon: '↩️',
      instruction: 'Tilt drone 90° to the left side'
    },
    {
      key: 'noseDown',
      name: 'Nose Down',
      description: 'Nose down (90° pitch forward)',
      icon: '⬇️',
      instruction: 'Tilt drone nose down at 90° angle'
    },
    {
      key: 'noseUp',
      name: 'Nose Up',
      description: 'Nose up (90° pitch backward)',
      icon: '⬆️',
      instruction: 'Tilt drone nose up at 90° angle'
    }
  ];

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

  // Magnetometer timer effect
  useEffect(() => {
    let interval;
    if (magCalibrationActive && magTimer > 0) {
      interval = setInterval(() => {
        setMagTimer(prev => {
          if (prev <= 1) {
            setMagCalibrationActive(false);
            setMagCalibrationInstructions('Magnetometer calibration complete!');
            return 0;
          }
          return prev - 1;
        });
      }, 1000);
    }
    
    return () => {
      if (interval) clearInterval(interval);
    };
  }, [magCalibrationActive, magTimer]);

  const handleSerialData = (data) => {
    // Add to calibration log
    setCalibrationLog(prev => [...prev.slice(-50), data]); // Keep last 50 lines

    // Parse calibration status updates
    if (data.includes('CALIBRATION_OK')) {
      setCalibrationStatus(prev => ({ ...prev, system_calibrated: true }));
    } else if (data.includes('calibration complete')) {
      setCalibrating(false);
      setAccelCalibrationActive(false);
      setMagCalibrationActive(false);
      refreshCalibrationStatus();
    } else if (data.includes('Progress:') || data.includes('Position')) {
      setCalibrationProgress(data);
    }

    // Parse accelerometer position completion
    if (data.includes('Position 1 complete') || data.includes('Level position complete')) {
      setAccelPositions(prev => ({ ...prev, level: true }));
      setCurrentAccelPosition(null);
    } else if (data.includes('Position 2 complete') || data.includes('Upside down complete')) {
      setAccelPositions(prev => ({ ...prev, upsideDown: true }));
      setCurrentAccelPosition(null);
    } else if (data.includes('Position 3 complete') || data.includes('Right side complete')) {
      setAccelPositions(prev => ({ ...prev, rightSide: true }));
      setCurrentAccelPosition(null);
    } else if (data.includes('Position 4 complete') || data.includes('Left side complete')) {
      setAccelPositions(prev => ({ ...prev, leftSide: true }));
      setCurrentAccelPosition(null);
    } else if (data.includes('Position 5 complete') || data.includes('Nose down complete')) {
      setAccelPositions(prev => ({ ...prev, noseDown: true }));
      setCurrentAccelPosition(null);
    } else if (data.includes('Position 6 complete') || data.includes('Nose up complete')) {
      setAccelPositions(prev => ({ ...prev, noseUp: true }));
      setCurrentAccelPosition(null);
    }

    // Handle magnetometer calibration start
    if (data.includes('Magnetometer calibration started') || data.includes('calibrate mag')) {
      setMagCalibrationActive(true);
      setMagTimer(90); // 90 second timer
      setMagCalibrationInstructions('Move the drone in figure-8 patterns in all directions');
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
          setAccelCalibrationActive(true);
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
    setCurrentAccelPosition(position);
    setCalibrationProgress(`Calibrating ${position} position...`);
    
    try {
      await sendCommand(`calibrate accel position ${position}`);
    } catch (error) {
      console.error(`Position ${position} calibration failed:`, error);
      setCurrentAccelPosition(null);
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
    setAccelCalibrationActive(false);
    setCurrentAccelPosition(null);
  };

  const startMagnetometerCalibration = async () => {
    setMagCalibrationActive(true);
    setMagTimer(90); // 90 seconds
    setMagCalibrationInstructions('Starting magnetometer calibration...');
    
    try {
      await sendCommand('calibrate mag');
    } catch (error) {
      console.error('Magnetometer calibration failed:', error);
      setMagCalibrationActive(false);
      setMagTimer(0);
    }
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

  const getCompletedPositions = () => {
    return Object.values(accelPositions).filter(Boolean).length;
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

      {/* Accelerometer Calibration Visual Interface */}
      {showAccelPositions && (
        <div className="bg-white rounded-lg shadow-md mb-6">
          <div className="px-6 py-4 border-b border-gray-200">
            <div className="flex items-center justify-between">
              <h2 className="text-xl font-semibold">Accelerometer 6-Position Calibration</h2>
              <div className="text-sm text-gray-600">
                Progress: {getCompletedPositions()}/6 positions
              </div>
            </div>
          </div>
          <div className="p-6">
            <div className="mb-4 p-4 bg-blue-50 rounded-lg">
              <p className="text-blue-800 font-medium mb-2">📐 Instructions:</p>
              <p className="text-blue-700 text-sm">
                Position the drone in each orientation and click "Calibrate" for each position. 
                Hold the drone steady for 5-10 seconds when calibrating each position.
              </p>
            </div>
            
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
              {accelPositionData.map((position) => (
                <div 
                  key={position.key}
                  className={`border-2 rounded-lg p-4 transition-all ${
                    accelPositions[position.key] 
                      ? 'border-green-500 bg-green-50' 
                      : currentAccelPosition === position.key
                        ? 'border-yellow-500 bg-yellow-50'
                        : 'border-gray-300 bg-white'
                  }`}
                >
                  <div className="text-center mb-3">
                    <div className="text-3xl mb-2">{position.icon}</div>
                    <h3 className="font-semibold text-gray-900">{position.name}</h3>
                    <p className="text-sm text-gray-600 mb-2">{position.description}</p>
                  </div>
                  
                  <div className="text-xs text-gray-500 mb-3 text-center">
                    {position.instruction}
                  </div>
                  
                  <div className="flex flex-col items-center space-y-2">
                    {accelPositions[position.key] ? (
                      <div className="text-green-600 font-medium flex items-center">
                        <span className="mr-2">✅</span>
                        Complete
                      </div>
                    ) : currentAccelPosition === position.key ? (
                      <div className="text-yellow-600 font-medium flex items-center">
                        <div className="animate-spin rounded-full h-4 w-4 border-b-2 border-yellow-600 mr-2"></div>
                        Calibrating...
                      </div>
                    ) : (
                      <button
                        onClick={() => startAccelPositionCalibration(position.key)}
                        disabled={accelCalibrationActive && currentAccelPosition !== position.key}
                        className="bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white text-sm font-medium py-2 px-4 rounded"
                      >
                        Calibrate
                      </button>
                    )}
                  </div>
                </div>
              ))}
            </div>
            
            <div className="mt-6 flex justify-between">
              <button
                onClick={resetAccelCalibration}
                className="bg-red-600 hover:bg-red-700 text-white font-medium py-2 px-4 rounded"
              >
                Reset Calibration
              </button>
              <div className="text-sm text-gray-600">
                {getCompletedPositions() === 6 && (
                  <span className="text-green-600 font-medium">
                    ✅ All positions complete! Accelerometer calibrated.
                  </span>
                )}
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Magnetometer Calibration Timer */}
      {magCalibrationActive && (
        <div className="bg-white rounded-lg shadow-md mb-6">
          <div className="px-6 py-4 border-b border-gray-200">
            <h2 className="text-xl font-semibold">Magnetometer Calibration in Progress</h2>
          </div>
          <div className="p-6">
            <div className="text-center mb-6">
              <div className="text-6xl font-bold text-blue-600 mb-2">
                {Math.floor(magTimer / 60)}:{(magTimer % 60).toString().padStart(2, '0')}
              </div>
              <div className="text-lg text-gray-600">Time remaining</div>
            </div>
            
            <div className="bg-blue-50 rounded-lg p-4 mb-4">
              <div className="flex items-center mb-3">
                <span className="text-2xl mr-3">🧭</span>
                <h3 className="font-semibold text-blue-800">Calibration Instructions</h3>
              </div>
              <div className="text-blue-700 space-y-2">
                <p>• Hold the drone firmly and move it in slow figure-8 patterns</p>
                <p>• Rotate the drone in all axes (pitch, roll, yaw)</p>
                <p>• Keep moving throughout the entire calibration period</p>
                <p>• Stay away from metal objects and electronics</p>
                <p>• The more varied the motion, the better the calibration</p>
              </div>
            </div>
            
            <div className="w-full bg-gray-200 rounded-full h-2">
              <div 
                className="bg-blue-600 h-2 rounded-full transition-all duration-1000"
                style={{ width: `${((90 - magTimer) / 90) * 100}%` }}
              ></div>
            </div>
            
            <div className="text-center mt-4">
              <p className="text-sm text-gray-600">{magCalibrationInstructions}</p>
            </div>
          </div>
        </div>
      )}

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
                        onClick={() => setShowAccelPositions(true)}
                        disabled={calibrating}
                        className="bg-green-600 hover:bg-green-700 disabled:bg-gray-400 text-white font-medium py-2 px-4 rounded text-sm"
                      >
                        Show Positions
                      </button>
                    )}
                    {step.id === 'magnetometer' ? (
                      <button
                        onClick={startMagnetometerCalibration}
                        disabled={calibrating || magCalibrationActive}
                        className="bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white font-medium py-2 px-4 rounded"
                      >
                        {magCalibrationActive ? 'Calibrating...' : 'Start'}
                      </button>
                    ) : (
                      <button
                        onClick={() => startCalibrationStep(step.id)}
                        disabled={calibrating}
                        className="bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white font-medium py-2 px-4 rounded"
                      >
                        {calibrating ? 'Running...' : 'Start'}
                      </button>
                    )}
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>

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

export default SensorCalibration; 