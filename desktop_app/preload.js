const { contextBridge, ipcRenderer } = require('electron');

// Expose protected methods that allow the renderer process to use
// the ipcRenderer without exposing the entire object
contextBridge.exposeInMainWorld('electronAPI', {
  // Serial port operations
  getSerialPorts: () => ipcRenderer.invoke('get-serial-ports'),
  connectSerial: (port, baudRate) => ipcRenderer.invoke('connect-serial-port', port, baudRate),
  disconnectSerial: () => ipcRenderer.invoke('disconnect-serial-port'),
  sendCommand: (command) => ipcRenderer.invoke('send-serial-command', command),
  getSerialStatus: () => ipcRenderer.invoke('get-serial-status'),
  
  // Serial data listeners
  onSerialData: (callback) => ipcRenderer.on('serial-data', callback),
  onSerialError: (callback) => ipcRenderer.on('serial-error', callback),
  onSerialDisconnected: (callback) => ipcRenderer.on('serial-disconnected', callback),
  
  // Remove listeners
  removeAllListeners: (channel) => ipcRenderer.removeAllListeners(channel),
  
  // File operations
  saveConfig: (config) => ipcRenderer.invoke('save-config-file', config),
  loadConfig: () => ipcRenderer.invoke('load-config-file'),
  
  // App info
  getAppVersion: () => ipcRenderer.invoke('get-app-version')
}); 