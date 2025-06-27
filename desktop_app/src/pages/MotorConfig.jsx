import React, { useState, useEffect } from 'react';

const ESC_PROTOCOLS = [
  { value: 'PWM', label: 'PWM (Standard)', description: '1000-2000μs PWM signal' },
  { value: 'ONESHOT125', label: 'OneShot125', description: '125-250μs signal' },
  { value: 'ONESHOT42', label: 'OneShot42', description: '42-84μs signal' },
  { value: 'MULTISHOT', label: 'Multishot', description: '5-25μs signal' },
  { value: 'DSHOT150', label: 'DShot150', description: 'Digital 150kbps' },
  { value: 'DSHOT300', label: 'DShot300', description: 'Digital 300kbps' },
  { value: 'DSHOT600', label: 'DShot600', description: 'Digital 600kbps' },
  { value: 'DSHOT1200', label: 'DShot1200', description: 'Digital 1200kbps' }
];

const DSHOT_COMMANDS = [
  { value: 'BEEP1', label: 'Beep 1' },
  { value: 'BEEP2', label: 'Beep 2' },
  { value: 'BEEP3', label: 'Beep 3' },
  { value: 'LED_ON', label: 'LED On' },
  { value: 'LED_OFF', label: 'LED Off' },
  { value: 'SAVE', label: 'Save Settings' }
];

const MOTOR_POSITIONS = {
  1: { x: 20, y: 20, label: 'M1 (Front Right)', rotation: 'CW' },
  2: { x: 80, y: 20, label: 'M2 (Front Left)', rotation: 'CCW' },
  3: { x: 80, y: 80, label: 'M3 (Rear Left)', rotation: 'CW' },
  4: { x: 20, y: 80, label: 'M4 (Rear Right)', rotation: 'CCW' }
};

function MotorConfig({ config, setConfig, telemetry, sendCommand, connected }) {
  const [localConfig, setLocalConfig] = useState({
    protocol: config?.motors?.protocol || 'PWM',
    direction: config?.motors?.direction || [1, 1, 1, 1],
    enabled: config?.motors?.enabled || [true, true, true, true],
    telemetryEnabled: config?.motors?.telemetryEnabled || false,
    bidirectionalEnabled: config?.motors?.bidirectionalEnabled || false,
    calibrated: config?.motors?.calibrated || false
  });
  
  const [motorTesting, setMotorTesting] = useState({
    motor: 0,
    value: 1000,
    active: false
  });
  
  // ESC Calibration state
  const [escCalibration, setEscCalibration] = useState({
    active: false,
    step: 'idle', // idle, high_throttle, low_throttle, complete
    motorValues: [1000, 1000, 1000, 1000],
    masterValue: 1000,
    useIndividual: false
  });
  
  // Safety checks
  const [safetyChecks, setSafetyChecks] = useState({
    propellersRemoved: false,
    motorsDisarmed: true,
    stablePosition: false
  });
  
  const [calibrationStatus, setCalibrationStatus] = useState('idle'); // idle, running, complete
  const [telemetryData, setTelemetryData] = useState({
    rpm: [0, 0, 0, 0],
    temperature: [0, 0, 0, 0],
    voltage: [0, 0, 0, 0],
    current: [0, 0, 0, 0]
  });

  // Motor orientation validation
  const [motorOrientation, setMotorOrientation] = useState({
    valid: false,
    expectedRotations: ['CW', 'CCW', 'CW', 'CCW'], // Standard quadcopter X configuration
    actualRotations: ['Unknown', 'Unknown', 'Unknown', 'Unknown']
  });

  useEffect(() => {
    // Update telemetry data from props
    if (telemetry?.motors) {
      setTelemetryData(telemetry.motors);
    }
  }, [telemetry]);

  useEffect(() => {
    // Validate motor orientation
    const valid = motorOrientation.expectedRotations.every((expected, index) => {
      const actual = localConfig.direction[index] === 1 ? 
        MOTOR_POSITIONS[index + 1].rotation : 
        (MOTOR_POSITIONS[index + 1].rotation === 'CW' ? 'CCW' : 'CW');
      return expected === actual;
    });
    
    setMotorOrientation(prev => ({
      ...prev,
      valid,
      actualRotations: localConfig.direction.map((dir, index) => 
        dir === 1 ? MOTOR_POSITIONS[index + 1].rotation : 
        (MOTOR_POSITIONS[index + 1].rotation === 'CW' ? 'CCW' : 'CW')
      )
    }));
  }, [localConfig.direction]);

  const handleProtocolChange = async (protocol) => {
    if (!connected) {
      alert('Not connected to flight controller');
      return;
    }

    const result = await sendCommand(`SET ESC PROTOCOL ${protocol}`);
    if (result.success) {
      setLocalConfig(prev => ({ ...prev, protocol }));
      setConfig(prev => ({
        ...prev,
        motors: { ...prev.motors, protocol }
      }));
    } else {
      alert('Failed to set ESC protocol');
    }
  };

  const handleMotorDirectionToggle = async (motorIndex) => {
    if (!connected) {
      alert('Not connected to flight controller');
      return;
    }

    const newDirection = localConfig.direction[motorIndex] === 1 ? -1 : 1;
    const result = await sendCommand(`SET ESC DIRECTION ${motorIndex + 1} ${newDirection === 1 ? 'NORMAL' : 'REVERSE'}`);
    
    if (result.success) {
      const newDirections = [...localConfig.direction];
      newDirections[motorIndex] = newDirection;
      setLocalConfig(prev => ({ ...prev, direction: newDirections }));
      setConfig(prev => ({
        ...prev,
        motors: { ...prev.motors, direction: newDirections }
      }));
    }
  };

  const handleTelemetryToggle = async () => {
    if (!connected) {
      alert('Not connected to flight controller');
      return;
    }

    const newState = !localConfig.telemetryEnabled;
    const result = await sendCommand(`SET ESC TELEMETRY ${newState ? 'ON' : 'OFF'}`);
    
    if (result.success) {
      setLocalConfig(prev => ({ ...prev, telemetryEnabled: newState }));
    }
  };

  const handleBidirectionalToggle = async () => {
    if (!connected) {
      alert('Not connected to flight controller');
      return;
    }

    const newState = !localConfig.bidirectionalEnabled;
    const result = await sendCommand(`SET ESC BIDIRECTIONAL ${newState ? 'ON' : 'OFF'}`);
    
    if (result.success) {
      setLocalConfig(prev => ({ ...prev, bidirectionalEnabled: newState }));
    }
  };

  const handleMotorTest = async () => {
    if (!safetyChecks.propellersRemoved) {
      alert('Please confirm propellers are removed before testing motors!');
      return;
    }

    if (!connected) {
      alert('Not connected to flight controller');
      return;
    }

    setMotorTesting(prev => ({ ...prev, active: true }));
    const result = await sendCommand(`TEST MOTOR ${motorTesting.motor + 1} ${motorTesting.value}`);
    
    setTimeout(() => {
      setMotorTesting(prev => ({ ...prev, active: false }));
    }, 2000);
  };

  const startEscCalibration = async () => {
    if (!safetyChecks.propellersRemoved) {
      alert('ESC calibration requires propellers to be removed for safety!');
      return;
    }

    if (!connected) {
      alert('Not connected to flight controller');
      return;
    }

    setEscCalibration(prev => ({ 
      ...prev, 
      active: true, 
      step: 'high_throttle',
      masterValue: 2000,
      motorValues: [2000, 2000, 2000, 2000]
    }));
    
    // Send high throttle values to all motors
    for (let i = 0; i < 4; i++) {
      await sendCommand(`TEST MOTOR ${i + 1} 2000`);
    }
  };

  const proceedToLowThrottle = async () => {
    setEscCalibration(prev => ({ 
      ...prev, 
      step: 'low_throttle',
      masterValue: 1000,
      motorValues: [1000, 1000, 1000, 1000]
    }));
    
    // Send low throttle values to all motors
    for (let i = 0; i < 4; i++) {
      await sendCommand(`TEST MOTOR ${i + 1} 1000`);
    }
  };

  const completeEscCalibration = async () => {
    setEscCalibration(prev => ({ ...prev, step: 'complete' }));
    setLocalConfig(prev => ({ ...prev, calibrated: true }));
    
    // Stop all motors
    for (let i = 0; i < 4; i++) {
      await sendCommand(`TEST MOTOR ${i + 1} 1000`);
    }
    
    setTimeout(() => {
      setEscCalibration(prev => ({ ...prev, active: false, step: 'idle' }));
    }, 2000);
  };

  const handleIndividualMotorValue = async (motorIndex, value) => {
    if (!escCalibration.active) return;
    
    const newValues = [...escCalibration.motorValues];
    newValues[motorIndex] = value;
    setEscCalibration(prev => ({ ...prev, motorValues: newValues }));
    
    await sendCommand(`TEST MOTOR ${motorIndex + 1} ${value}`);
  };

  const handleMasterValue = async (value) => {
    if (!escCalibration.active) return;
    
    setEscCalibration(prev => ({ 
      ...prev, 
      masterValue: value,
      motorValues: [value, value, value, value]
    }));
    
    // Send to all motors
    for (let i = 0; i < 4; i++) {
      await sendCommand(`TEST MOTOR ${i + 1} ${value}`);
    }
  };

  const sendDShotCommand = async (command, motor = null) => {
    if (!connected) {
      alert('Not connected to flight controller');
      return;
    }

    const cmd = motor !== null ? 
      `DSHOT MOTOR ${motor + 1} ${command}` : 
      `DSHOT ${command}`;
    
    await sendCommand(cmd);
  };

  const isDShotProtocol = localConfig.protocol.startsWith('DSHOT');
  const supportsAdvancedFeatures = isDShotProtocol;
  const allSafetyChecksComplete = Object.values(safetyChecks).every(Boolean);

  return (
    <div className="max-w-6xl mx-auto space-y-6">
      <div className="bg-white dark:bg-gray-800 rounded-lg shadow p-6">
        <h2 className="text-2xl font-bold text-gray-900 dark:text-white mb-6">
          Motor & ESC Configuration
        </h2>

        {/* Connection Status */}
        <div className={`mb-4 p-3 rounded-lg ${connected ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'}`}>
          {connected ? '🟢 Connected to Flight Controller' : '🔴 Not Connected'}
        </div>

        {/* Safety Checks */}
        <div className="bg-red-50 border-l-4 border-red-400 p-4 mb-6">
          <h3 className="text-lg font-semibold text-red-800 mb-3">⚠️ SAFETY CHECKLIST</h3>
          <div className="space-y-2">
            {[
              { 
                key: 'propellersRemoved', 
                label: 'All propellers are REMOVED from motors',
                critical: true 
              },
              { 
                key: 'motorsDisarmed', 
                label: 'Flight controller is DISARMED',
                critical: true 
              },
              { 
                key: 'stablePosition', 
                label: 'Drone is in stable position on workbench',
                critical: false 
              }
            ].map((check) => (
              <label key={check.key} className="flex items-center space-x-3">
                <input
                  type="checkbox"
                  checked={safetyChecks[check.key]}
                  onChange={(e) => setSafetyChecks(prev => ({ 
                    ...prev, 
                    [check.key]: e.target.checked 
                  }))}
                  className="w-5 h-5 text-red-600"
                />
                <span className={`text-sm ${check.critical ? 'font-semibold text-red-800' : 'text-red-700'}`}>
                  {check.label}
                  {check.critical && <span className="text-red-600 ml-1">*</span>}
                </span>
              </label>
            ))}
          </div>
          {!allSafetyChecksComplete && (
            <p className="text-xs text-red-600 mt-2">
              * All safety checks must be completed before motor testing or calibration
            </p>
          )}
        </div>

        {/* Motor Orientation Diagram */}
        <div className="bg-white rounded-lg shadow-md mb-6">
          <div className="px-6 py-4 border-b border-gray-200">
            <h3 className="text-lg font-semibold">Motor Orientation & Direction</h3>
          </div>
          <div className="p-6">
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
              {/* Visual Diagram */}
              <div className="relative">
                <h4 className="font-medium mb-4 text-center">Quadcopter X Configuration</h4>
                <div className="relative w-64 h-64 mx-auto bg-gray-100 rounded-lg border-2 border-gray-300">
                  {/* Drone body */}
                  <div className="absolute inset-4 bg-gray-200 rounded-lg flex items-center justify-center">
                    <span className="text-gray-600 font-medium">BODY</span>
                  </div>
                  
                  {/* Motors */}
                  {Object.entries(MOTOR_POSITIONS).map(([motorNum, pos]) => {
                    const motorIndex = parseInt(motorNum) - 1;
                    const isCorrectDirection = motorOrientation.actualRotations[motorIndex] === pos.rotation;
                    
                    return (
                      <div
                        key={motorNum}
                        className={`absolute w-12 h-12 rounded-full border-2 flex items-center justify-center text-xs font-bold ${
                          isCorrectDirection 
                            ? 'bg-green-100 border-green-500 text-green-800' 
                            : 'bg-red-100 border-red-500 text-red-800'
                        }`}
                        style={{
                          left: `${pos.x}%`,
                          top: `${pos.y}%`,
                          transform: 'translate(-50%, -50%)'
                        }}
                      >
                        <div className="text-center">
                          <div>M{motorNum}</div>
                          <div className="text-xs">
                            {motorOrientation.actualRotations[motorIndex] === 'CW' ? '↻' : '↺'}
                          </div>
                        </div>
                      </div>
                    );
                  })}
                  
                  {/* Arms */}
                  <div className="absolute inset-0">
                    <div className="absolute w-1 bg-gray-400" style={{
                      left: '30%', top: '30%', width: '40%', height: '2px',
                      transform: 'rotate(45deg)', transformOrigin: 'left'
                    }}></div>
                    <div className="absolute w-1 bg-gray-400" style={{
                      right: '30%', top: '30%', width: '40%', height: '2px',
                      transform: 'rotate(-45deg)', transformOrigin: 'right'
                    }}></div>
                  </div>
                  
                  {/* Front indicator */}
                  <div className="absolute top-2 left-1/2 transform -translate-x-1/2 text-xs font-bold text-gray-600">
                    FRONT
                  </div>
                </div>
              </div>
              
              {/* Configuration Status */}
              <div className="space-y-4">
                <div className={`p-4 rounded-lg ${
                  motorOrientation.valid 
                    ? 'bg-green-50 border border-green-200' 
                    : 'bg-red-50 border border-red-200'
                }`}>
                  <h4 className={`font-semibold mb-2 ${
                    motorOrientation.valid ? 'text-green-800' : 'text-red-800'
                  }`}>
                    {motorOrientation.valid ? '✅ Configuration Valid' : '❌ Configuration Invalid'}
                  </h4>
                  <p className={`text-sm ${
                    motorOrientation.valid ? 'text-green-700' : 'text-red-700'
                  }`}>
                    {motorOrientation.valid 
                      ? 'Motor directions are correctly configured for stable flight.'
                      : 'Motor directions need adjustment. Check the configuration below.'
                    }
                  </p>
                </div>
                
                <div className="space-y-3">
                  {[1, 2, 3, 4].map(motorNum => {
                    const motorIndex = motorNum - 1;
                    const expected = motorOrientation.expectedRotations[motorIndex];
                    const actual = motorOrientation.actualRotations[motorIndex];
                    const isCorrect = expected === actual;
                    
                    return (
                      <div key={motorNum} className="flex items-center justify-between p-3 bg-gray-50 rounded">
                        <div className="flex items-center space-x-3">
                          <span className="font-medium">Motor {motorNum}:</span>
                          <span className="text-sm text-gray-600">
                            {MOTOR_POSITIONS[motorNum].label}
                          </span>
                        </div>
                        <div className="flex items-center space-x-2">
                          <span className={`text-sm ${isCorrect ? 'text-green-600' : 'text-red-600'}`}>
                            Expected: {expected} | Actual: {actual}
                          </span>
                          <button
                            onClick={() => handleMotorDirectionToggle(motorIndex)}
                            disabled={!connected || !safetyChecks.propellersRemoved}
                            className={`px-3 py-1 rounded text-xs font-medium ${
                              isCorrect 
                                ? 'bg-green-100 text-green-800' 
                                : 'bg-red-100 text-red-800'
                            } disabled:opacity-50`}
                          >
                            {isCorrect ? 'Correct' : 'Fix Direction'}
                          </button>
                        </div>
                      </div>
                    );
                  })}
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* ESC Protocol Selection */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
          <div className="space-y-4">
            <h3 className="text-lg font-semibold text-gray-900 dark:text-white">
              ESC Protocol
            </h3>
            
            <div className="space-y-3">
              {ESC_PROTOCOLS.map((protocol) => (
                <label key={protocol.value} className="flex items-center space-x-3 cursor-pointer">
                  <input
                    type="radio"
                    name="escProtocol"
                    value={protocol.value}
                    checked={localConfig.protocol === protocol.value}
                    onChange={() => handleProtocolChange(protocol.value)}
                    disabled={!connected}
                    className="w-4 h-4 text-blue-600"
                  />
                  <div className="flex-1">
                    <div className="font-medium text-gray-900 dark:text-white">
                      {protocol.label}
                    </div>
                    <div className="text-sm text-gray-500 dark:text-gray-400">
                      {protocol.description}
                    </div>
                  </div>
                </label>
              ))}
            </div>

            <div className={`p-3 rounded-lg ${isDShotProtocol ? 'bg-blue-100 text-blue-800' : 'bg-gray-100 text-gray-600'}`}>
              <strong>Current Protocol:</strong> {localConfig.protocol}
              {isDShotProtocol && (
                <div className="text-sm mt-1">
                  ✓ Supports telemetry and bidirectional control
                </div>
              )}
            </div>
          </div>

          {/* Advanced Features */}
          {supportsAdvancedFeatures && (
            <div className="space-y-4">
              <h3 className="text-lg font-semibold text-gray-900 dark:text-white">
                Advanced Features (DShot)
              </h3>
              
              <div className="space-y-3">
                <label className="flex items-center justify-between">
                  <span className="text-gray-700 dark:text-gray-300">Enable Telemetry</span>
                  <button
                    onClick={handleTelemetryToggle}
                    disabled={!connected}
                    className={`w-12 h-6 rounded-full transition-colors ${
                      localConfig.telemetryEnabled 
                        ? 'bg-blue-600' 
                        : 'bg-gray-300 dark:bg-gray-600'
                    }`}
                  >
                    <div className={`w-5 h-5 bg-white rounded-full transition-transform ${
                      localConfig.telemetryEnabled ? 'translate-x-6' : 'translate-x-0.5'
                    }`} />
                  </button>
                </label>

                <label className="flex items-center justify-between">
                  <span className="text-gray-700 dark:text-gray-300">Bidirectional Mode</span>
                  <button
                    onClick={handleBidirectionalToggle}
                    disabled={!connected}
                    className={`w-12 h-6 rounded-full transition-colors ${
                      localConfig.bidirectionalEnabled 
                        ? 'bg-blue-600' 
                        : 'bg-gray-300 dark:bg-gray-600'
                    }`}
                  >
                    <div className={`w-5 h-5 bg-white rounded-full transition-transform ${
                      localConfig.bidirectionalEnabled ? 'translate-x-6' : 'translate-x-0.5'
                    }`} />
                  </button>
                </label>
              </div>

              {/* DShot Commands */}
              <div>
                <h4 className="font-medium text-gray-900 dark:text-white mb-2">
                  DShot Commands
                </h4>
                <div className="grid grid-cols-2 gap-2">
                  {DSHOT_COMMANDS.map((cmd) => (
                    <button
                      key={cmd.value}
                      onClick={() => sendDShotCommand(cmd.value)}
                      disabled={!connected || !safetyChecks.propellersRemoved}
                      className="bg-gray-200 hover:bg-gray-300 disabled:bg-gray-100 disabled:text-gray-400 text-gray-800 font-medium py-2 px-3 rounded text-sm"
                    >
                      {cmd.label}
                    </button>
                  ))}
                </div>
              </div>
            </div>
          )}
        </div>

        {/* ESC Calibration Section */}
        <div className="bg-blue-50 dark:bg-blue-900/20 border border-blue-200 dark:border-blue-800 rounded-lg p-6 mb-6">
          <h3 className="text-lg font-semibold text-blue-800 dark:text-blue-200 mb-4">
            🔧 ESC Calibration
          </h3>
          
          {!escCalibration.active ? (
            <div>
              <p className="text-sm text-gray-600 dark:text-gray-400 mb-4">
                {isDShotProtocol 
                  ? 'DShot ESCs require minimal calibration. This will send configuration commands.'
                  : 'Traditional ESC calibration sequence. Follow the steps carefully!'
                }
              </p>
              
              <div className="flex items-center justify-between mb-4">
                <div>
                  <div className={`text-sm font-medium ${
                    localConfig.calibrated ? 'text-green-600' : 'text-orange-600'
                  }`}>
                    Status: {localConfig.calibrated ? 'Calibrated' : 'Not Calibrated'}
                  </div>
                </div>
                
                <button
                  onClick={startEscCalibration}
                  disabled={!connected || !safetyChecks.propellersRemoved || escCalibration.active}
                  className="bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white font-medium py-2 px-4 rounded"
                >
                  Start ESC Calibration
                </button>
              </div>
              
              {!safetyChecks.propellersRemoved && (
                <div className="text-sm text-red-600 bg-red-50 p-2 rounded">
                  ⚠️ Safety check required: Confirm propellers are removed
                </div>
              )}
            </div>
          ) : (
            <div className="space-y-4">
              {/* Calibration Steps */}
              <div className="bg-white rounded-lg p-4">
                <h4 className="font-semibold mb-3">ESC Calibration Progress</h4>
                <div className="space-y-2">
                  {[
                    { step: 'high_throttle', label: 'Step 1: High Throttle (Listen for ESC beeps)' },
                    { step: 'low_throttle', label: 'Step 2: Low Throttle (Complete calibration)' },
                    { step: 'complete', label: 'Step 3: Calibration Complete' }
                  ].map((stepInfo) => (
                    <div key={stepInfo.step} className={`flex items-center space-x-2 text-sm ${
                      escCalibration.step === stepInfo.step 
                        ? 'text-blue-600 font-medium' 
                        : escCalibration.step === 'complete' || 
                          (stepInfo.step === 'high_throttle' && escCalibration.step === 'low_throttle')
                          ? 'text-green-600'
                          : 'text-gray-500'
                    }`}>
                      <span>
                        {escCalibration.step === stepInfo.step ? '🔄' : 
                         escCalibration.step === 'complete' || 
                         (stepInfo.step === 'high_throttle' && escCalibration.step === 'low_throttle')
                         ? '✅' : '⏸️'}
                      </span>
                      <span>{stepInfo.label}</span>
                    </div>
                  ))}
                </div>
              </div>

              {/* Master or Individual Control */}
              <div className="space-y-4">
                <label className="flex items-center space-x-2">
                  <input
                    type="checkbox"
                    checked={escCalibration.useIndividual}
                    onChange={(e) => setEscCalibration(prev => ({ 
                      ...prev, 
                      useIndividual: e.target.checked 
                    }))}
                    className="w-4 h-4"
                  />
                  <span className="text-sm">Use individual motor control</span>
                </label>

                {!escCalibration.useIndividual ? (
                  // Master Control
                  <div className="bg-gray-50 rounded-lg p-4">
                    <h5 className="font-medium mb-3">Master Control - All Motors</h5>
                    <div className="space-y-2">
                      <label className="block text-sm font-medium">
                        Throttle Value: {escCalibration.masterValue}
                      </label>
                      <input
                        type="range"
                        min="1000"
                        max="2000"
                        value={escCalibration.masterValue}
                        onChange={(e) => handleMasterValue(parseInt(e.target.value))}
                        className="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer"
                      />
                      <div className="flex justify-between text-xs text-gray-500">
                        <span>1000 (Min)</span>
                        <span>1500 (Mid)</span>
                        <span>2000 (Max)</span>
                      </div>
                    </div>
                  </div>
                ) : (
                  // Individual Motor Controls
                  <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                    {[0, 1, 2, 3].map((motorIndex) => (
                      <div key={motorIndex} className="bg-gray-50 rounded-lg p-4">
                        <h5 className="font-medium mb-3">Motor {motorIndex + 1}</h5>
                        <div className="space-y-2">
                          <label className="block text-sm font-medium">
                            Value: {escCalibration.motorValues[motorIndex]}
                          </label>
                          <input
                            type="range"
                            min="1000"
                            max="2000"
                            value={escCalibration.motorValues[motorIndex]}
                            onChange={(e) => handleIndividualMotorValue(motorIndex, parseInt(e.target.value))}
                            className="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer"
                          />
                        </div>
                      </div>
                    ))}
                  </div>
                )}
              </div>

              {/* Calibration Step Controls */}
              <div className="flex space-x-3">
                {escCalibration.step === 'high_throttle' && (
                  <button
                    onClick={proceedToLowThrottle}
                    className="bg-yellow-600 hover:bg-yellow-700 text-white font-medium py-2 px-4 rounded"
                  >
                    Proceed to Low Throttle
                  </button>
                )}
                {escCalibration.step === 'low_throttle' && (
                  <button
                    onClick={completeEscCalibration}
                    className="bg-green-600 hover:bg-green-700 text-white font-medium py-2 px-4 rounded"
                  >
                    Complete Calibration
                  </button>
                )}
                {escCalibration.step === 'complete' && (
                  <div className="text-green-600 font-medium">
                    ✅ ESC Calibration Complete!
                  </div>
                )}
              </div>
            </div>
          )}
        </div>

        {/* Motor Testing */}
        <div className="bg-yellow-50 dark:bg-yellow-900/20 border border-yellow-200 dark:border-yellow-800 rounded-lg p-4 mb-6">
          <h3 className="text-lg font-semibold text-yellow-800 dark:text-yellow-200 mb-4">
            ⚠️ Motor Testing (Remove Propellers!)
          </h3>
          
          <div className="grid grid-cols-1 md:grid-cols-3 gap-4 items-end">
            <div>
              <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                Motor
              </label>
              <select
                value={motorTesting.motor}
                onChange={(e) => setMotorTesting(prev => ({ ...prev, motor: parseInt(e.target.value) }))}
                className="w-full p-2 border border-gray-300 rounded-md"
                disabled={!safetyChecks.propellersRemoved}
              >
                {[0, 1, 2, 3].map(motor => (
                  <option key={motor} value={motor}>Motor {motor + 1}</option>
                ))}
              </select>
            </div>
            
            <div>
              <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                Test Value
              </label>
              <input
                type="range"
                min="1000"
                max="2000"
                value={motorTesting.value}
                onChange={(e) => setMotorTesting(prev => ({ ...prev, value: parseInt(e.target.value) }))}
                disabled={!safetyChecks.propellersRemoved}
                className="w-full"
              />
              <div className="text-center text-sm text-gray-600 dark:text-gray-400">
                {motorTesting.value}
              </div>
            </div>
            
            <button
              onClick={handleMotorTest}
              disabled={!connected || motorTesting.active || !safetyChecks.propellersRemoved}
              className="bg-blue-600 hover:bg-blue-700 disabled:bg-gray-400 text-white font-medium py-2 px-4 rounded"
            >
              {motorTesting.active ? 'Testing...' : 'Test Motor'}
            </button>
          </div>
        </div>

        {/* Individual Motor Status Grid */}
        <div className="space-y-6">
          <h3 className="text-lg font-semibold text-gray-900 dark:text-white">
            Individual Motor Status
          </h3>

          <div className="grid grid-cols-1 md:grid-cols-2 xl:grid-cols-4 gap-4">
            {[0, 1, 2, 3].map((motorIndex) => (
              <div key={motorIndex} className="border border-gray-200 dark:border-gray-700 rounded-lg p-4">
                <div className="text-center mb-3">
                  <h4 className="font-semibold text-gray-900 dark:text-white">
                    Motor {motorIndex + 1}
                  </h4>
                  <div className="text-xs text-gray-500">
                    {MOTOR_POSITIONS[motorIndex + 1].label}
                  </div>
                  <div className={`text-sm ${localConfig.enabled[motorIndex] ? 'text-green-600' : 'text-red-600'}`}>
                    {localConfig.enabled[motorIndex] ? 'Enabled' : 'Disabled'}
                  </div>
                </div>

                {/* Motor Direction */}
                <div className="mb-3">
                  <label className="flex items-center justify-between text-sm">
                    <span>Direction:</span>
                    <button
                      onClick={() => handleMotorDirectionToggle(motorIndex)}
                      disabled={!connected || !safetyChecks.propellersRemoved}
                      className={`px-2 py-1 rounded text-xs ${
                        localConfig.direction[motorIndex] === 1
                          ? 'bg-green-100 text-green-800'
                          : 'bg-orange-100 text-orange-800'
                      } disabled:opacity-50`}
                    >
                      {localConfig.direction[motorIndex] === 1 ? 'Normal' : 'Reversed'}
                    </button>
                  </label>
                </div>

                {/* Telemetry Data */}
                {localConfig.telemetryEnabled && (
                  <div className="space-y-1 text-xs">
                    <div className="flex justify-between">
                      <span>RPM:</span>
                      <span className="font-mono">{telemetryData.rpm[motorIndex]}</span>
                    </div>
                    <div className="flex justify-between">
                      <span>Temp:</span>
                      <span className="font-mono">{telemetryData.temperature[motorIndex]}°C</span>
                    </div>
                    <div className="flex justify-between">
                      <span>Voltage:</span>
                      <span className="font-mono">{(telemetryData.voltage[motorIndex] / 100).toFixed(1)}V</span>
                    </div>
                    <div className="flex justify-between">
                      <span>Current:</span>
                      <span className="font-mono">{(telemetryData.current[motorIndex] / 100).toFixed(1)}A</span>
                    </div>
                  </div>
                )}

                {/* Individual Motor DShot Commands */}
                {supportsAdvancedFeatures && safetyChecks.propellersRemoved && (
                  <div className="mt-3 space-y-1">
                    <button
                      onClick={() => sendDShotCommand('BEEP1', motorIndex)}
                      disabled={!connected}
                      className="w-full bg-gray-200 hover:bg-gray-300 disabled:bg-gray-100 text-gray-800 font-medium py-1 px-2 rounded text-xs"
                    >
                      Beep
                    </button>
                  </div>
                )}
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
}

export default MotorConfig;
