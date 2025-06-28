import React, { useState, useEffect } from 'react';
import { BrowserRouter as Router, Routes, Route, Link, Navigate } from 'react-router-dom';
import SerialConnection from './components/SerialConnection';
import Dashboard from './pages/Dashboard';
import SafetyStatus from './pages/SafetyStatus';
import SensorCalibration from './pages/SensorCalibration';
import CalibrationWizard from './pages/CalibrationWizard';
import PIDTuning from './pages/PIDTuning';
import MotorConfig from './pages/MotorConfig';
import ReceiverConfig from './pages/ReceiverConfig';
import FlightModes from './pages/FlightModes';
import LEDConfig from './pages/LEDConfig';
import SystemInfo from './pages/SystemInfo';
import DualIMUMonitor from './pages/DualIMUMonitor';
import DynamicFiltering from './pages/DynamicFiltering';
import SensorRedundancy from './pages/SensorRedundancy';
import CLI from './pages/CLI';
import AdvancedPerformance from './pages/AdvancedPerformance';

function App() {
  const [isConnected, setIsConnected] = useState(false);
  const [activeSection, setActiveSection] = useState('setup');
  const [telemetry, setTelemetry] = useState({});

  useEffect(() => {
    // Listen for telemetry data
    const handleSerialData = (event, data) => {
      try {
        if (data.includes('"telemetry":')) {
          const telemetryData = JSON.parse(data);
          setTelemetry(telemetryData.telemetry);
        }
      } catch (error) {
        // Ignore parsing errors for non-telemetry data
      }
    };

    window.electronAPI.onSerialData(handleSerialData);

    return () => {
      window.electronAPI.removeAllListeners('serial-data');
    };
  }, []);

  const sendCommand = async (command) => {
    if (isConnected) {
      try {
        await window.electronAPI.sendCommand(command);
      } catch (error) {
        console.error('Error sending command:', error);
      }
    }
  };

  // Connection Guard Component
  const ConnectionGuard = ({ children }) => {
    if (!isConnected) {
      return <Navigate to="/connection" replace />;
    }
    return children;
  };

  // Dedicated Connection Page
  const ConnectionPage = () => (
    <div className="min-h-screen bg-gray-100 flex items-center justify-center">
      <div className="max-w-md w-full">
        <div className="bg-white rounded-lg shadow-lg p-8">
          <div className="text-center mb-6">
            <h1 className="text-3xl font-bold text-gray-900 mb-2">
              FPV Drone Controller
            </h1>
            <p className="text-gray-600">Enhanced Safety Edition</p>
          </div>

          {/* Connection Status */}
          <div className={`p-4 rounded-lg mb-6 text-center ${
            isConnected 
              ? 'bg-green-100 text-green-800 border border-green-200' 
              : 'bg-red-100 text-red-800 border border-red-200'
          }`}>
            <div className="flex items-center justify-center mb-2">
              <div className={`w-3 h-3 rounded-full mr-2 ${
                isConnected ? 'bg-green-500' : 'bg-red-500'
              }`}></div>
              <span className="font-medium">
                {isConnected ? 'Connected' : 'Not Connected'}
              </span>
            </div>
            {!isConnected && (
              <p className="text-sm">
                Please connect to your flight controller to access configuration features
              </p>
            )}
          </div>

          <SerialConnection 
            onConnectionChange={setIsConnected}
            connected={isConnected}
          />

          {!isConnected && (
            <div className="mt-6 p-4 bg-blue-50 rounded-lg border border-blue-200">
              <h3 className="text-sm font-semibold text-blue-800 mb-2">
                📋 Connection Requirements
              </h3>
              <ul className="text-sm text-blue-700 space-y-1">
                <li>• Connect Teensy 4.1 via USB</li>
                <li>• Select correct COM port</li>
                <li>• Ensure firmware is running</li>
                <li>• Check USB cable and drivers</li>
              </ul>
            </div>
          )}

          {isConnected && (
            <div className="mt-6 text-center">
              <p className="text-green-600 mb-4">✅ Connection established!</p>
              <p className="text-sm text-gray-600">
                You can now access all configuration pages.
              </p>
            </div>
          )}
        </div>
      </div>
    </div>
  );

  const navigationSections = {
    setup: {
      title: 'Setup & Calibration',
      icon: '🔧',
      items: [
        { path: '/safety', name: 'Safety Status', icon: '🛡️', priority: 'high' },
        { path: '/calibration-wizard', name: 'Calibration Wizard', icon: '🧙‍♂️', priority: 'high' },
        { path: '/calibration', name: 'Sensor Calibration', icon: '📏', priority: 'high' },
        { path: '/system', name: 'System Info', icon: 'ℹ️' }
      ]
    },
    basic: {
      title: 'Basic Configuration',
      icon: '⚙️',
      items: [
        { path: '/', name: 'Dashboard', icon: '📊' },
        { path: '/receiver', name: 'RC Receiver', icon: '📡' },
        { path: '/motors', name: 'Motor Config', icon: '🔧' },
        { path: '/leds', name: 'LED Config', icon: '💡' }
      ]
    },
    flight: {
      title: 'Flight Control',
      icon: '✈️',
      items: [
        { path: '/pid', name: 'PID Tuning', icon: '📈' },
        { path: '/flight-modes', name: 'Flight Modes', icon: '🛩️' }
      ]
    },
    sensors: {
      title: 'Sensor Systems',
      icon: '📡',
      items: [
        { path: '/sensor-redundancy', name: 'Sensor Redundancy', icon: '🔄', priority: 'high' },
        { path: '/dual-imu', name: 'Dual IMU Monitor', icon: '⚖️' },
        { path: '/filtering', name: 'Dynamic Filtering', icon: '📊' }
      ]
    },
    advanced: {
      title: 'Advanced Tools',
      icon: '🔧',
      items: [
        { path: '/cli', name: 'Command Line Interface', icon: '💻', priority: 'high' },
        { path: '/advanced-performance', name: 'Advanced Performance', icon: '📊', priority: 'high' }
      ]
    }
  };

  const NavSection = ({ section, items, isActive, onSelect }) => (
    <div className="mb-4">
      <button
        onClick={() => onSelect(section)}
        className={`w-full text-left p-3 rounded-lg font-medium transition-colors ${
          isActive 
            ? 'bg-blue-600 text-white' 
            : 'bg-gray-100 text-gray-800 hover:bg-gray-200'
        }`}
      >
        <span className="mr-2">{navigationSections[section].icon}</span>
        {navigationSections[section].title}
      </button>
      
      {isActive && (
        <div className="mt-2 ml-4 space-y-1">
          {items.map((item) => (
            <Link
              key={item.path}
              to={item.path}
              className={`flex items-center p-2 text-sm rounded hover:bg-gray-100 transition-colors ${
                item.priority === 'high' ? 'text-blue-700 font-medium' : 'text-gray-700'
              }`}
            >
              <span className="mr-2">{item.icon}</span>
              {item.name}
              {item.priority === 'high' && (
                <span className="ml-auto text-xs bg-blue-100 text-blue-600 px-2 py-1 rounded">
                  IMPORTANT
                </span>
              )}
            </Link>
          ))}
        </div>
      )}
    </div>
  );

  // If not connected, show only the connection page
  if (!isConnected) {
    return (
      <Router>
        <Routes>
          <Route path="*" element={<ConnectionPage />} />
        </Routes>
      </Router>
    );
  }

  return (
    <Router>
      <div className="flex h-screen bg-gray-100">
        {/* Sidebar - Only shown when connected */}
        <div className="w-80 bg-white shadow-lg overflow-y-auto">
          <div className="p-6">
            <h1 className="text-2xl font-bold text-gray-800 mb-2">
              FPV Drone Controller
            </h1>
            <div className="text-sm text-gray-600 mb-4">
              Enhanced Safety Edition
            </div>
            
            {/* Connection Status */}
            <div className="bg-green-100 text-green-800 border border-green-200 p-3 rounded-lg mb-6">
              <div className="flex items-center">
                <div className="w-3 h-3 rounded-full mr-2 bg-green-500"></div>
                <span className="font-medium">Connected</span>
                <button
                  onClick={() => window.electronAPI.disconnectSerial()}
                  className="ml-auto text-xs bg-red-100 text-red-600 px-2 py-1 rounded hover:bg-red-200"
                >
                  Disconnect
                </button>
              </div>
            </div>
          </div>

          {/* Navigation */}
          <div className="px-6 pb-6">
            <nav>
              {Object.entries(navigationSections).map(([section, config]) => (
                <NavSection
                  key={section}
                  section={section}
                  items={config.items}
                  isActive={activeSection === section}
                  onSelect={setActiveSection}
                />
              ))}
            </nav>

            {/* Safety Status Indicator */}
            <div className="mt-6 p-4 bg-blue-50 rounded-lg border border-blue-200">
              <h3 className="text-sm font-semibold text-blue-800 mb-2">
                🛡️ Safety Status
              </h3>
              <div className="space-y-1 text-xs">
                <div className="flex justify-between">
                  <span className="text-blue-700">Armed:</span>
                  <span className={telemetry?.armed ? 'text-red-600 font-bold' : 'text-green-600'}>
                    {telemetry?.armed ? 'ARMED' : 'DISARMED'}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-blue-700">Battery:</span>
                  <span className={`${
                    telemetry?.battery < 10.5 ? 'text-red-600' : 
                    telemetry?.battery < 11.5 ? 'text-yellow-600' : 'text-green-600'
                  }`}>
                    {telemetry?.battery?.toFixed(1) || '--'}V
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-blue-700">GPS:</span>
                  <span className={telemetry?.gps_fix ? 'text-green-600' : 'text-red-600'}>
                    {telemetry?.gps_fix ? 'LOCK' : 'NO FIX'}
                  </span>
                </div>
              </div>
            </div>

            {/* Feature Status */}
            <div className="mt-6 p-4 bg-green-50 rounded-lg border border-green-200">
              <h3 className="text-sm font-semibold text-green-800 mb-2">
                ✅ Enhanced Features
              </h3>
              <div className="space-y-1 text-xs text-green-700">
                <div>✓ GPS Safety Enforcement</div>
                <div>✓ Battery Safety System</div>
                <div>✓ Pre-Flight Safety Checks</div>
                <div>✓ ESC Voltage Telemetry</div>
                <div>✓ Enhanced Calibration</div>
                <div>✓ Real-time Warning System</div>
              </div>
            </div>
          </div>
        </div>

        {/* Main Content - Protected Routes */}
        <div className="flex-1 overflow-y-auto">
          <Routes>
            <Route path="/connection" element={<Navigate to="/" replace />} />
            <Route path="/" element={
              <ConnectionGuard>
                <Dashboard telemetry={telemetry} connected={isConnected} sendCommand={sendCommand} />
              </ConnectionGuard>
            } />
            <Route path="/safety" element={
              <ConnectionGuard>
                <SafetyStatus sendCommand={sendCommand} connected={isConnected} />
              </ConnectionGuard>
            } />
            <Route path="/calibration-wizard" element={
              <ConnectionGuard>
                <CalibrationWizard sendCommand={sendCommand} connected={isConnected} />
              </ConnectionGuard>
            } />
            <Route path="/calibration" element={
              <ConnectionGuard>
                <SensorCalibration sendCommand={sendCommand} connected={isConnected} />
              </ConnectionGuard>
            } />
            <Route path="/pid" element={
              <ConnectionGuard>
                <PIDTuning sendCommand={sendCommand} connected={isConnected} />
              </ConnectionGuard>
            } />
            <Route path="/motors" element={
              <ConnectionGuard>
                <MotorConfig sendCommand={sendCommand} connected={isConnected} telemetry={telemetry} />
              </ConnectionGuard>
            } />
            <Route path="/receiver" element={
              <ConnectionGuard>
                <ReceiverConfig sendCommand={sendCommand} connected={isConnected} />
              </ConnectionGuard>
            } />
            <Route path="/flight-modes" element={
              <ConnectionGuard>
                <FlightModes sendCommand={sendCommand} connected={isConnected} />
              </ConnectionGuard>
            } />
            <Route path="/leds" element={
              <ConnectionGuard>
                <LEDConfig sendCommand={sendCommand} connected={isConnected} />
              </ConnectionGuard>
            } />
            <Route path="/system" element={
              <ConnectionGuard>
                <SystemInfo sendCommand={sendCommand} connected={isConnected} />
              </ConnectionGuard>
            } />
            <Route path="/sensor-redundancy" element={
              <ConnectionGuard>
                <SensorRedundancy sendCommand={sendCommand} connected={isConnected} />
              </ConnectionGuard>
            } />
            <Route path="/dual-imu" element={
              <ConnectionGuard>
                <DualIMUMonitor sendCommand={sendCommand} connected={isConnected} />
              </ConnectionGuard>
            } />
            <Route path="/filtering" element={
              <ConnectionGuard>
                <DynamicFiltering sendCommand={sendCommand} connected={isConnected} />
              </ConnectionGuard>
            } />
            <Route path="/cli" element={
              <ConnectionGuard>
                <CLI sendCommand={sendCommand} connected={isConnected} />
              </ConnectionGuard>
            } />
            <Route path="/advanced-performance" element={
              <ConnectionGuard>
                <AdvancedPerformance sendCommand={sendCommand} connected={isConnected} />
              </ConnectionGuard>
            } />
            {/* Catch all route - redirect to dashboard if connected */}
            <Route path="*" element={<Navigate to="/" replace />} />
          </Routes>
        </div>
      </div>
    </Router>
  );
}

export default App; 