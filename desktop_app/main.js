const { app, BrowserWindow, ipcMain, dialog } = require('electron');
const path = require('path');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

let mainWindow;
let serialPort = null;
let parser = null;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1200,
    height: 800,
    minWidth: 1000,
    minHeight: 600,
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      preload: path.join(__dirname, 'preload.js')
    },
    icon: path.join(__dirname, 'assets/icon.png'), // Add app icon
    titleBarStyle: 'default',
    show: false
  });

  // Load the app
  if (process.env.NODE_ENV === 'development') {
    mainWindow.loadURL('http://localhost:5173');
    mainWindow.webContents.openDevTools();
  } else {
    mainWindow.loadFile('dist/index.html');
  }

  mainWindow.once('ready-to-show', () => {
    mainWindow.show();
  });

  mainWindow.on('closed', () => {
    if (serialPort && serialPort.isOpen) {
      serialPort.close();
    }
    mainWindow = null;
  });
}

app.whenReady().then(createWindow);

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) {
    createWindow();
  }
});

// Serial Port Management
ipcMain.handle('get-serial-ports', async () => {
  try {
    const ports = await SerialPort.list();
    return ports.filter(port => port.path);
  } catch (error) {
    console.error('Error listing serial ports:', error);
    return [];
  }
});

ipcMain.handle('connect-serial-port', async (event, portPath, baudRate = 115200) => {
  try {
    if (serialPort && serialPort.isOpen) {
      await serialPort.close();
    }

    serialPort = new SerialPort({
      path: portPath,
      baudRate: baudRate,
      autoOpen: false
    });

    parser = serialPort.pipe(new ReadlineParser({ delimiter: '\n' }));

    return new Promise((resolve, reject) => {
      serialPort.open((err) => {
        if (err) {
          reject(err);
          return;
        }

        // Set up data listener
        parser.on('data', (data) => {
          mainWindow.webContents.send('serial-data', data.trim());
        });

        // Set up error listener
        serialPort.on('error', (err) => {
          console.error('Serial port error:', err);
          mainWindow.webContents.send('serial-error', err.message);
        });

        // Set up close listener
        serialPort.on('close', () => {
          mainWindow.webContents.send('serial-disconnected');
        });

        resolve({ success: true, port: portPath });
      });
    });
  } catch (error) {
    console.error('Error connecting to serial port:', error);
    throw error;
  }
});

ipcMain.handle('disconnect-serial-port', async () => {
  try {
    if (serialPort && serialPort.isOpen) {
      await serialPort.close();
      return { success: true };
    }
    return { success: false, message: 'Port not open' };
  } catch (error) {
    console.error('Error disconnecting serial port:', error);
    throw error;
  }
});

ipcMain.handle('send-serial-command', async (event, command) => {
  try {
    if (serialPort && serialPort.isOpen) {
      await serialPort.write(command + '\n');
      return { success: true };
    }
    return { success: false, message: 'Port not open' };
  } catch (error) {
    console.error('Error sending serial command:', error);
    throw error;
  }
});

ipcMain.handle('get-serial-status', async () => {
  return {
    connected: serialPort ? serialPort.isOpen : false,
    port: serialPort ? serialPort.path : null
  };
});

// File operations
ipcMain.handle('save-config-file', async (event, config) => {
  try {
    const result = await dialog.showSaveDialog(mainWindow, {
      title: 'Save Configuration',
      defaultPath: 'drone-config.json',
      filters: [
        { name: 'JSON Files', extensions: ['json'] },
        { name: 'All Files', extensions: ['*'] }
      ]
    });

    if (!result.canceled) {
      const fs = require('fs').promises;
      await fs.writeFile(result.filePath, JSON.stringify(config, null, 2));
      return { success: true, path: result.filePath };
    }
    return { success: false, message: 'Save canceled' };
  } catch (error) {
    console.error('Error saving config file:', error);
    throw error;
  }
});

ipcMain.handle('load-config-file', async () => {
  try {
    const result = await dialog.showOpenDialog(mainWindow, {
      title: 'Load Configuration',
      filters: [
        { name: 'JSON Files', extensions: ['json'] },
        { name: 'All Files', extensions: ['*'] }
      ],
      properties: ['openFile']
    });

    if (!result.canceled && result.filePaths.length > 0) {
      const fs = require('fs').promises;
      const data = await fs.readFile(result.filePaths[0], 'utf8');
      const config = JSON.parse(data);
      return { success: true, config, path: result.filePaths[0] };
    }
    return { success: false, message: 'Load canceled' };
  } catch (error) {
    console.error('Error loading config file:', error);
    throw error;
  }
});

// App info
ipcMain.handle('get-app-version', async () => {
  return app.getVersion();
}); 