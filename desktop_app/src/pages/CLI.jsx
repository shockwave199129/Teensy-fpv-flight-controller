import React, { useState, useEffect, useRef } from 'react';

const CLI = ({ sendCommand, connected }) => {
  const [commandHistory, setCommandHistory] = useState([]);
  const [currentCommand, setCurrentCommand] = useState('');
  const [historyIndex, setHistoryIndex] = useState(-1);
  const [output, setOutput] = useState([]);
  const [isWaitingForResponse, setIsWaitingForResponse] = useState(false);
  const [showHelp, setShowHelp] = useState(false);
  const [showTemplates, setShowTemplates] = useState(false);
  const terminalRef = useRef(null);
  const inputRef = useRef(null);

  // Available commands with categories for autocomplete
  const commands = {
    'Basic Commands': [
      'help', 'status', 'arm', 'disarm', 'reboot'
    ],
    'Safety & Status': [
      'safety check', 'gps status', 'battery status', 'calibration check'
    ],
    'Sensor Redundancy': [
      'redundancy status', 'sensor health', 'synthetic data', 
      'emergency mode', 'recovery mode'
    ],
    'Calibration': [
      'calibrate gyro', 'calibrate accel', 'calibrate mag', 'calibrate esc',
      'calibrate mag timer', 'calibration wizard', 'calibration status',
      'reset calibration', 'save calibration', 'load calibration'
    ],
    'Enhanced Calibration': [
      'calibrate accel position level', 'calibrate accel position upside_down',
      'calibrate accel position right_side', 'calibrate accel position left_side',
      'calibrate accel position nose_down', 'calibrate accel position nose_up',
      'esc calibration start', 'esc calibration high', 'esc calibration low',
      'esc calibration complete', 'check motor directions'
    ],
    'RC Configuration': [
      'set rc protocol', 'set channel', 'get_rc_data', 'channel_test',
      'reset_channel_mapping', 'get_channel_mapping'
    ],
    'ESC Configuration': [
      'set esc protocol', 'set esc direction', 'set esc telemetry',
      'set esc bidirectional', 'dshot beep1', 'dshot beep2', 'dshot save'
    ],
    'Motor Testing': [
      'test motor 1 1200', 'test motor 2 1200', 'test motor 3 1200', 'test motor 4 1200'
    ],
    'Sensor Detection': [
      'detect_sensors', 'sensor_status', 'sensor_json', 'i2c_scan',
      'detect imu', 'detect mag', 'detect baro', 'detect gps'
    ],
    'Safety Features': [
      'safety check propellers', 'confirm propellers removed',
      'confirm propellers installed', 'fix motor direction'
    ],
    'Configuration': [
      'save_config', 'load_config', 'reset pid'
    ],
    'LED Control': [
      'led red', 'led green', 'led blue', 'led rainbow', 'led pattern 1'
    ]
  };

  // Command templates with examples
  const commandTemplates = [
    {
      category: 'RC Channel Mapping',
      commands: [
        { cmd: 'set rc protocol 2', desc: 'Set SBUS protocol (0=PPM, 1=iBUS, 2=SBUS, 3=ELRS)' },
        { cmd: 'set channel 5 function 4 normal', desc: 'Map channel 5 to Arm/Disarm function' },
        { cmd: 'set channel 6 function 5 normal', desc: 'Map channel 6 to Flight Mode switch' },
        { cmd: 'channel_test 5', desc: 'Test channel 5 response' }
      ]
    },
    {
      category: 'ESC Configuration',
      commands: [
        { cmd: 'set esc protocol DSHOT600', desc: 'Set DShot600 protocol for ESCs' },
        { cmd: 'set esc telemetry on', desc: 'Enable DShot telemetry' },
        { cmd: 'set esc direction 1 reverse', desc: 'Reverse motor 1 direction' },
        { cmd: 'dshot beep1', desc: 'Send beep command to all ESCs' }
      ]
    },
    {
      category: 'Calibration Workflow',
      commands: [
        { cmd: 'calibration check', desc: 'Check which calibrations are needed' },
        { cmd: 'calibrate gyro', desc: 'Calibrate gyroscope (keep still)' },
        { cmd: 'calibrate accel', desc: 'Start 6-position accelerometer calibration' },
        { cmd: 'calibrate mag timer', desc: 'Start 90-second magnetometer calibration' },
        { cmd: 'save calibration', desc: 'Save all calibrations to EEPROM' }
      ]
    },
    {
      category: 'Motor Testing',
      commands: [
        { cmd: 'confirm propellers removed', desc: 'REQUIRED: Confirm propellers are OFF' },
        { cmd: 'test motor 1 1200', desc: 'Test motor 1 at 1200 PWM (1000-2000)' },
        { cmd: 'check motor directions', desc: 'Validate CW/CCW motor configuration' },
        { cmd: 'fix motor direction 1', desc: 'Auto-fix motor 1 direction if incorrect' }
      ]
    },
    {
      category: 'Safety Checks',
      commands: [
        { cmd: 'safety check', desc: 'Comprehensive pre-flight safety validation' },
        { cmd: 'gps status', desc: 'Check GPS fix and satellite count' },
        { cmd: 'battery status', desc: 'Check battery connection and voltage' },
        { cmd: 'redundancy status', desc: 'Check sensor redundancy and failover status' }
      ]
    }
  ];

  useEffect(() => {
    if (connected) {
      addOutput('🔌 Connected to flight controller', 'success');
      addOutput('💡 Type "help" to see all available commands', 'info');
      addOutput('💡 Use Tab for autocomplete, Up/Down arrows for history', 'info');
    }

    // Listen for serial data
    const handleSerialData = (event, data) => {
      if (data && data.trim()) {
        addOutput(data, 'response');
        setIsWaitingForResponse(false);
      }
    };

    window.electronAPI.onSerialData(handleSerialData);

    return () => {
      window.electronAPI.removeAllListeners('serial-data');
    };
  }, [connected]);

  useEffect(() => {
    // Auto-scroll to bottom when new output is added
    if (terminalRef.current) {
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
    }
  }, [output]);

  const addOutput = (text, type = 'normal') => {
    const timestamp = new Date().toLocaleTimeString();
    setOutput(prev => [...prev, { text, type, timestamp }]);
  };

  const executeCommand = async () => {
    if (!currentCommand.trim() || !connected) return;

    const cmd = currentCommand.trim();
    
    // Add to history
    setCommandHistory(prev => {
      const newHistory = [cmd, ...prev.filter(h => h !== cmd)].slice(0, 50);
      return newHistory;
    });
    
    // Add command to output
    addOutput(`$ ${cmd}`, 'command');
    
    // Clear input and reset history index
    setCurrentCommand('');
    setHistoryIndex(-1);
    
    // Handle special client-side commands
    if (cmd.toLowerCase() === 'clear') {
      setOutput([]);
      return;
    }
    
    if (cmd.toLowerCase() === 'history') {
      addOutput('Command History:', 'info');
      commandHistory.forEach((histCmd, index) => {
        addOutput(`${index + 1}. ${histCmd}`, 'normal');
      });
      return;
    }

    // Send command to firmware
    try {
      setIsWaitingForResponse(true);
      await sendCommand(cmd);
      
      // Set timeout for response
      setTimeout(() => {
        if (isWaitingForResponse) {
          addOutput('⚠️ No response received (timeout)', 'warning');
          setIsWaitingForResponse(false);
        }
      }, 5000);
      
    } catch (error) {
      addOutput(`❌ Error sending command: ${error.message}`, 'error');
      setIsWaitingForResponse(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter') {
      executeCommand();
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      if (historyIndex < commandHistory.length - 1) {
        const newIndex = historyIndex + 1;
        setHistoryIndex(newIndex);
        setCurrentCommand(commandHistory[newIndex]);
      }
    } else if (e.key === 'ArrowDown') {
      e.preventDefault();
      if (historyIndex > 0) {
        const newIndex = historyIndex - 1;
        setHistoryIndex(newIndex);
        setCurrentCommand(commandHistory[newIndex]);
      } else if (historyIndex === 0) {
        setHistoryIndex(-1);
        setCurrentCommand('');
      }
    } else if (e.key === 'Tab') {
      e.preventDefault();
      handleAutocomplete();
    }
  };

  const handleAutocomplete = () => {
    const input = currentCommand.toLowerCase();
    if (!input) return;

    // Find matching commands
    const allCommands = Object.values(commands).flat();
    const matches = allCommands.filter(cmd => 
      cmd.toLowerCase().startsWith(input)
    );

    if (matches.length === 1) {
      setCurrentCommand(matches[0]);
    } else if (matches.length > 1) {
      addOutput(`💡 Possible completions: ${matches.join(', ')}`, 'info');
    }
  };

  const insertCommand = (command) => {
    setCurrentCommand(command);
    inputRef.current?.focus();
  };

  const getOutputClass = (type) => {
    switch (type) {
      case 'command': return 'text-blue-400 font-semibold';
      case 'response': return 'text-green-300';
      case 'error': return 'text-red-400';
      case 'warning': return 'text-yellow-400';
      case 'info': return 'text-cyan-400';
      case 'success': return 'text-green-400';
      default: return 'text-gray-300';
    }
  };

  if (!connected) {
    return (
      <div className="p-6">
        <div className="bg-yellow-100 border border-yellow-400 text-yellow-700 px-4 py-3 rounded">
          <strong>Warning:</strong> Please connect to the flight controller to access CLI features.
        </div>
      </div>
    );
  }

  return (
    <div className="p-6 max-w-7xl mx-auto">
      <div className="mb-6">
        <h1 className="text-3xl font-bold text-gray-900 mb-2">Command Line Interface</h1>
        <p className="text-gray-600">
          Direct command access to the flight controller firmware. Type commands and see real-time responses.
        </p>
      </div>

      {/* Quick Actions */}
      <div className="grid grid-cols-1 md:grid-cols-4 gap-4 mb-6">
        <button
          onClick={() => insertCommand('help')}
          className="bg-blue-600 hover:bg-blue-700 text-white font-medium py-2 px-4 rounded-lg"
        >
          📋 Show Help
        </button>
        <button
          onClick={() => insertCommand('status')}
          className="bg-green-600 hover:bg-green-700 text-white font-medium py-2 px-4 rounded-lg"
        >
          📊 System Status
        </button>
        <button
          onClick={() => insertCommand('safety check')}
          className="bg-purple-600 hover:bg-purple-700 text-white font-medium py-2 px-4 rounded-lg"
        >
          🛡️ Safety Check
        </button>
        <button
          onClick={() => setCurrentCommand('clear')}
          className="bg-gray-600 hover:bg-gray-700 text-white font-medium py-2 px-4 rounded-lg"
        >
          🗑️ Clear Terminal
        </button>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        {/* Terminal */}
        <div className="lg:col-span-3">
          <div className="bg-black rounded-lg border border-gray-600 overflow-hidden">
            {/* Terminal Header */}
            <div className="bg-gray-800 px-4 py-2 flex items-center justify-between">
              <div className="flex items-center space-x-2">
                <div className="w-3 h-3 bg-red-500 rounded-full"></div>
                <div className="w-3 h-3 bg-yellow-500 rounded-full"></div>
                <div className="w-3 h-3 bg-green-500 rounded-full"></div>
                <span className="ml-4 text-gray-300 font-mono text-sm">
                  FPV Drone CLI - {connected ? 'Connected' : 'Disconnected'}
                </span>
              </div>
              <div className="flex space-x-2">
                <button
                  onClick={() => setShowHelp(!showHelp)}
                  className="text-gray-400 hover:text-gray-200 text-sm"
                >
                  {showHelp ? 'Hide' : 'Show'} Help
                </button>
                <button
                  onClick={() => setShowTemplates(!showTemplates)}
                  className="text-gray-400 hover:text-gray-200 text-sm"
                >
                  Templates
                </button>
              </div>
            </div>

            {/* Terminal Output */}
            <div 
              ref={terminalRef}
              className="p-4 h-96 overflow-y-auto font-mono text-sm bg-black"
            >
              {output.map((line, index) => (
                <div key={index} className={`${getOutputClass(line.type)} mb-1`}>
                  {line.type === 'command' || line.type === 'response' ? (
                    <span className="text-xs text-gray-500 mr-2">
                      [{line.timestamp}]
                    </span>
                  ) : null}
                  {line.text}
                </div>
              ))}
              {isWaitingForResponse && (
                <div className="text-yellow-400 mb-1">
                  <span className="animate-pulse">⏳ Waiting for response...</span>
                </div>
              )}
            </div>

            {/* Command Input */}
            <div className="bg-gray-900 px-4 py-2 flex items-center">
              <span className="text-green-400 mr-2 font-mono">$</span>
              <input
                ref={inputRef}
                type="text"
                value={currentCommand}
                onChange={(e) => setCurrentCommand(e.target.value)}
                onKeyDown={handleKeyPress}
                placeholder="Type command... (Tab for autocomplete, ↑↓ for history)"
                className="flex-1 bg-transparent text-white font-mono outline-none"
                disabled={!connected}
              />
              <button
                onClick={executeCommand}
                disabled={!connected || !currentCommand.trim()}
                className="ml-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 text-white px-3 py-1 rounded text-sm"
              >
                Send
              </button>
            </div>
          </div>

          {/* Command Help */}
          {showHelp && (
            <div className="mt-4 bg-gray-50 rounded-lg p-4">
              <h3 className="font-semibold text-gray-900 mb-3">Available Commands by Category</h3>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                {Object.entries(commands).map(([category, cmdList]) => (
                  <div key={category} className="bg-white rounded border p-3">
                    <h4 className="font-medium text-gray-800 mb-2">{category}</h4>
                    <div className="space-y-1">
                      {cmdList.map((cmd, index) => (
                        <button
                          key={index}
                          onClick={() => insertCommand(cmd)}
                          className="block w-full text-left text-sm text-blue-600 hover:text-blue-800 hover:bg-blue-50 px-2 py-1 rounded"
                        >
                          {cmd}
                        </button>
                      ))}
                    </div>
                  </div>
                ))}
              </div>
            </div>
          )}
        </div>

        {/* Side Panel */}
        <div className="space-y-4">
          {/* Connection Status */}
          <div className="bg-white rounded-lg shadow-md p-4">
            <h3 className="font-semibold text-gray-900 mb-3">Connection Status</h3>
            <div className={`p-3 rounded-lg ${
              connected 
                ? 'bg-green-100 text-green-800' 
                : 'bg-red-100 text-red-800'
            }`}>
              <div className="flex items-center">
                <div className={`w-3 h-3 rounded-full mr-2 ${
                  connected ? 'bg-green-500' : 'bg-red-500'
                }`}></div>
                {connected ? 'Connected' : 'Disconnected'}
              </div>
            </div>
          </div>

          {/* Command History */}
          <div className="bg-white rounded-lg shadow-md p-4">
            <h3 className="font-semibold text-gray-900 mb-3">Recent Commands</h3>
            <div className="space-y-1 max-h-32 overflow-y-auto">
              {commandHistory.slice(0, 10).map((cmd, index) => (
                <button
                  key={index}
                  onClick={() => insertCommand(cmd)}
                  className="block w-full text-left text-sm text-gray-600 hover:text-gray-900 hover:bg-gray-100 px-2 py-1 rounded truncate"
                >
                  {cmd}
                </button>
              ))}
              {commandHistory.length === 0 && (
                <div className="text-sm text-gray-500 italic">No commands yet</div>
              )}
            </div>
          </div>

          {/* Quick Commands */}
          <div className="bg-white rounded-lg shadow-md p-4">
            <h3 className="font-semibold text-gray-900 mb-3">Quick Commands</h3>
            <div className="space-y-2">
              {[
                'calibration check',
                'sensor_status',
                'get_rc_data',
                'battery status',
                'gps status'
              ].map((cmd, index) => (
                <button
                  key={index}
                  onClick={() => insertCommand(cmd)}
                  className="block w-full text-left text-sm bg-gray-100 hover:bg-gray-200 px-3 py-2 rounded"
                >
                  {cmd}
                </button>
              ))}
            </div>
          </div>
        </div>
      </div>

      {/* Command Templates Modal */}
      {showTemplates && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
          <div className="bg-white rounded-lg max-w-4xl w-full mx-4 max-h-[80vh] overflow-y-auto">
            <div className="p-6">
              <div className="flex items-center justify-between mb-4">
                <h2 className="text-2xl font-bold text-gray-900">Command Templates & Examples</h2>
                <button
                  onClick={() => setShowTemplates(false)}
                  className="text-gray-400 hover:text-gray-600"
                >
                  ✕
                </button>
              </div>
              
              <div className="space-y-6">
                {commandTemplates.map((section, index) => (
                  <div key={index} className="bg-gray-50 rounded-lg p-4">
                    <h3 className="text-lg font-semibold text-gray-800 mb-3">{section.category}</h3>
                    <div className="space-y-2">
                      {section.commands.map((item, cmdIndex) => (
                        <div key={cmdIndex} className="flex items-center justify-between bg-white rounded p-3">
                          <div className="flex-1">
                            <code className="text-blue-600 font-mono text-sm">{item.cmd}</code>
                            <p className="text-gray-600 text-sm mt-1">{item.desc}</p>
                          </div>
                          <button
                            onClick={() => {
                              insertCommand(item.cmd);
                              setShowTemplates(false);
                            }}
                            className="ml-4 bg-blue-600 hover:bg-blue-700 text-white px-3 py-1 rounded text-sm"
                          >
                            Use
                          </button>
                        </div>
                      ))}
                    </div>
                  </div>
                ))}
              </div>

              <div className="mt-6 p-4 bg-yellow-50 rounded-lg border border-yellow-200">
                <h4 className="font-semibold text-yellow-800 mb-2">⚠️ Safety Reminders</h4>
                <ul className="text-sm text-yellow-700 space-y-1">
                  <li>• Always confirm propellers are removed before motor testing</li>
                  <li>• Complete calibration before attempting to arm</li>
                  <li>• GPS-dependent functions require GPS fix before arming</li>
                  <li>• Monitor battery voltage during testing</li>
                  <li>• Use safety check command before first flight</li>
                </ul>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default CLI; 