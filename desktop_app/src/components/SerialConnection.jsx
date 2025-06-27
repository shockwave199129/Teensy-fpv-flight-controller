import React, { useState, useEffect } from 'react';

function SerialConnection({ onConnectionChange, connected }) {
  const [ports, setPorts] = useState([]);
  const [selectedPort, setSelectedPort] = useState('');
  const [isConnecting, setIsConnecting] = useState(false);

  useEffect(() => {
    refreshPorts();
  }, []);

  const refreshPorts = async () => {
    if (window.electronAPI) {
      try {
        const availablePorts = await window.electronAPI.getSerialPorts();
        setPorts(availablePorts);
        if (availablePorts.length > 0 && !selectedPort) {
          setSelectedPort(availablePorts[0].path);
        }
      } catch (error) {
        console.error('Error getting serial ports:', error);
      }
    }
  };

  const handleConnect = async () => {
    if (!selectedPort) return;
    
    setIsConnecting(true);
    try {
      if (connected) {
        await window.electronAPI.disconnectSerial();
        onConnectionChange(false);
      } else {
        const result = await window.electronAPI.connectSerial(selectedPort, 115200);
        if (result.success) {
          onConnectionChange(true);
        }
      }
    } catch (error) {
      console.error('Serial connection error:', error);
    } finally {
      setIsConnecting(false);
    }
  };

  return (
    <div className="card">
      <h3 className="text-lg font-semibold mb-4">Serial Connection</h3>
      
      <div className="space-y-4">
        <div>
          <label className="block text-sm font-medium mb-2">Port:</label>
          <div className="flex space-x-2">
            <select
              value={selectedPort}
              onChange={(e) => setSelectedPort(e.target.value)}
              className="flex-1 px-3 py-2 border rounded-lg bg-white dark:bg-gray-700"
              disabled={connected}
            >
              <option value="">Select a port...</option>
              {ports.map((port) => (
                <option key={port.path} value={port.path}>
                  {port.path} {port.manufacturer && `(${port.manufacturer})`}
                </option>
              ))}
            </select>
            <button
              onClick={refreshPorts}
              className="btn-secondary"
              disabled={isConnecting}
            >
              🔄
            </button>
          </div>
        </div>

        <div>
          <button
            onClick={handleConnect}
            disabled={!selectedPort || isConnecting}
            className={connected ? 'btn-danger' : 'btn-primary'}
          >
            {isConnecting ? '⏳ Connecting...' : 
             connected ? '🔌 Disconnect' : '🔌 Connect'}
          </button>
        </div>

        <div className={`px-3 py-2 rounded-lg border text-sm ${
          connected ? 'status-connected' : 'status-disconnected'
        }`}>
          Status: {connected ? '✅ Connected' : '❌ Disconnected'}
          {connected && selectedPort && (
            <div className="text-xs mt-1">Port: {selectedPort}</div>
          )}
        </div>
      </div>
    </div>
  );
}

export default SerialConnection; 