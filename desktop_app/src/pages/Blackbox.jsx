import React, { useState, useEffect, useRef, useCallback, useMemo } from 'react';
import { Line } from 'react-chartjs-2';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  Filler
} from 'chart.js';

ChartJS.register(CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend, Filler);

const PLAYBACK_SPEEDS = [0.5, 1, 2, 4];
const VISIBLE_WINDOW_MS = 5000;

const DATASET_GROUPS = {
  gyro: { label: 'Gyro (°/s)', yAxisID: 'yGyro', items: ['gyro_x', 'gyro_y', 'gyro_z'] },
  setpoint: { label: 'Setpoint', yAxisID: 'yGyro', items: ['setpoint_roll', 'setpoint_pitch', 'setpoint_yaw'] },
  motors: { label: 'Motors', yAxisID: 'yMotor', items: ['motor1', 'motor2', 'motor3', 'motor4'] },
  pid: { label: 'PID Terms', yAxisID: 'yGyro', items: ['pid_p', 'pid_i', 'pid_d'] },
};

const DATASET_CONFIG = {
  gyro_x:         { label: 'Gyro X',       color: '#ef4444', group: 'gyro' },
  gyro_y:         { label: 'Gyro Y',       color: '#22c55e', group: 'gyro' },
  gyro_z:         { label: 'Gyro Z',       color: '#3b82f6', group: 'gyro' },
  setpoint_roll:  { label: 'SP Roll',      color: '#f97316', group: 'setpoint' },
  setpoint_pitch: { label: 'SP Pitch',     color: '#a855f7', group: 'setpoint' },
  setpoint_yaw:   { label: 'SP Yaw',       color: '#14b8a6', group: 'setpoint' },
  motor1:         { label: 'M1',           color: '#f59e0b', group: 'motors' },
  motor2:         { label: 'M2',           color: '#06b6d4', group: 'motors' },
  motor3:         { label: 'M3',           color: '#ec4899', group: 'motors' },
  motor4:         { label: 'M4',           color: '#84cc16', group: 'motors' },
  pid_p:          { label: 'PID P',        color: '#6366f1', group: 'pid' },
  pid_i:          { label: 'PID I',        color: '#d946ef', group: 'pid' },
  pid_d:          { label: 'PID D',        color: '#0ea5e9', group: 'pid' },
};

function formatBytes(bytes) {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1048576) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / 1048576).toFixed(1)} MB`;
}

function formatDuration(seconds) {
  const m = Math.floor(seconds / 60);
  const s = Math.floor(seconds % 60);
  return `${m}:${s.toString().padStart(2, '0')}`;
}

function computeStats(logData) {
  if (!logData.length) return null;
  const maxGyroX = Math.max(...logData.map(d => Math.abs(d.gyro_x)));
  const maxGyroY = Math.max(...logData.map(d => Math.abs(d.gyro_y)));
  const maxGyroZ = Math.max(...logData.map(d => Math.abs(d.gyro_z)));
  const avgMotor = logData.reduce((s, d) => s + (d.motor1 + d.motor2 + d.motor3 + d.motor4) / 4, 0) / logData.length;
  const duration = (logData[logData.length - 1].timestamp - logData[0].timestamp) / 1000;
  const voltages = logData.map(d => d.battery_voltage).filter(Boolean);
  return {
    maxGyroX, maxGyroY, maxGyroZ,
    avgMotor: avgMotor.toFixed(0),
    flightTime: formatDuration(duration),
    batteryMin: voltages.length ? Math.min(...voltages).toFixed(1) : '—',
    batteryMax: voltages.length ? Math.max(...voltages).toFixed(1) : '—',
    sampleCount: logData.length,
  };
}

const Blackbox = ({ sendCommand, connected }) => {
  const [logFiles, setLogFiles] = useState([]);
  const [logData, setLogData] = useState([]);
  const [selectedLog, setSelectedLog] = useState(null);
  const [isPlaying, setIsPlaying] = useState(false);
  const [playbackSpeed, setPlaybackSpeed] = useState(1);
  const [currentTime, setCurrentTime] = useState(0);
  const [visibleGroups, setVisibleGroups] = useState({
    gyro: true, setpoint: true, motors: true, pid: false,
  });
  const [downloading, setDownloading] = useState(null);
  const [loadingList, setLoadingList] = useState(false);

  const animFrameRef = useRef(null);
  const lastTickRef = useRef(null);
  const chartRef = useRef(null);

  // ── Data fetching ──────────────────────────────────────────────────
  const requestLogList = useCallback(() => {
    if (!connected) return;
    setLoadingList(true);
    sendCommand('blackbox list');
  }, [connected, sendCommand]);

  useEffect(() => {
    requestLogList();
  }, [requestLogList]);

  useEffect(() => {
    const handleLogList = (data) => {
      setLogFiles(data.files ?? []);
      setLoadingList(false);
    };

    const handleLogData = (data) => {
      if (data.points) {
        setLogData(data.points);
        setCurrentTime(data.points[0]?.timestamp ?? 0);
        setIsPlaying(false);
      }
      setDownloading(null);
    };

    const handleLogDeleted = (data) => {
      setLogFiles(prev => prev.filter(f => f.id !== data.id));
      if (selectedLog === data.id) {
        setSelectedLog(null);
        setLogData([]);
      }
    };

    window.electronAPI.on('blackbox-list', handleLogList);
    window.electronAPI.on('blackbox-data', handleLogData);
    window.electronAPI.on('blackbox-deleted', handleLogDeleted);

    return () => {
      window.electronAPI.off('blackbox-list', handleLogList);
      window.electronAPI.off('blackbox-data', handleLogData);
      window.electronAPI.off('blackbox-deleted', handleLogDeleted);
    };
  }, [selectedLog]);

  // ── Playback loop ──────────────────────────────────────────────────
  useEffect(() => {
    if (!isPlaying || !logData.length) return;
    const maxTime = logData[logData.length - 1].timestamp;

    const tick = (now) => {
      if (lastTickRef.current != null) {
        const dt = (now - lastTickRef.current) * playbackSpeed;
        setCurrentTime(prev => {
          const next = prev + dt;
          if (next >= maxTime) {
            setIsPlaying(false);
            return maxTime;
          }
          return next;
        });
      }
      lastTickRef.current = now;
      animFrameRef.current = requestAnimationFrame(tick);
    };

    lastTickRef.current = null;
    animFrameRef.current = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(animFrameRef.current);
  }, [isPlaying, playbackSpeed, logData]);

  // ── Actions ────────────────────────────────────────────────────────
  const handleDownload = (id) => {
    setDownloading(id);
    setSelectedLog(id);
    sendCommand(`blackbox download ${id}`);
  };

  const handleDownloadAll = () => {
    logFiles.forEach(f => sendCommand(`blackbox download ${f.id}`));
  };

  const handleDelete = (id) => {
    sendCommand(`blackbox delete ${id}`);
  };

  const handleExportCSV = () => {
    if (!logData.length) return;
    const keys = Object.keys(logData[0]);
    const header = keys.join(',');
    const rows = logData.map(d => keys.map(k => d[k] ?? '').join(','));
    const csv = [header, ...rows].join('\n');
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `blackbox_log_${selectedLog ?? 'export'}.csv`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const toggleGroup = (group) => {
    setVisibleGroups(prev => ({ ...prev, [group]: !prev[group] }));
  };

  // ── Chart data ─────────────────────────────────────────────────────
  const timeRange = useMemo(() => {
    if (!logData.length) return { start: 0, end: 0 };
    return { start: logData[0].timestamp, end: logData[logData.length - 1].timestamp };
  }, [logData]);

  const windowData = useMemo(() => {
    if (!logData.length) return [];
    const half = VISIBLE_WINDOW_MS / 2;
    const wStart = currentTime - half;
    const wEnd = currentTime + half;
    return logData.filter(d => d.timestamp >= wStart && d.timestamp <= wEnd);
  }, [logData, currentTime]);

  const chartData = useMemo(() => {
    const labels = windowData.map(d => ((d.timestamp - timeRange.start) / 1000).toFixed(2));
    const datasets = [];

    for (const [key, cfg] of Object.entries(DATASET_CONFIG)) {
      if (!visibleGroups[cfg.group]) continue;
      datasets.push({
        label: cfg.label,
        data: windowData.map(d => d[key]),
        borderColor: cfg.color,
        backgroundColor: cfg.color + '18',
        borderWidth: 1.5,
        pointRadius: 0,
        tension: 0.2,
        yAxisID: DATASET_GROUPS[cfg.group].yAxisID,
      });
    }
    return { labels, datasets };
  }, [windowData, visibleGroups, timeRange.start]);

  const chartOptions = useMemo(() => ({
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    interaction: { mode: 'index', intersect: false },
    plugins: {
      legend: { display: true, position: 'top', labels: { boxWidth: 12, padding: 8, font: { size: 11 } } },
      tooltip: { enabled: true, mode: 'index', intersect: false },
    },
    scales: {
      x: {
        title: { display: true, text: 'Time (s)', font: { size: 11 } },
        ticks: { maxTicksLimit: 12, font: { size: 10 } },
        grid: { color: '#e5e7eb44' },
      },
      yGyro: {
        type: 'linear',
        position: 'left',
        title: { display: true, text: 'Gyro / Setpoint (°/s)', font: { size: 11 } },
        ticks: { font: { size: 10 } },
        grid: { color: '#e5e7eb44' },
      },
      yMotor: {
        type: 'linear',
        position: 'right',
        title: { display: true, text: 'Motor Output', font: { size: 11 } },
        ticks: { font: { size: 10 } },
        grid: { drawOnChartArea: false },
        min: 1000,
        max: 2000,
      },
    },
  }), []);

  const stats = useMemo(() => computeStats(logData), [logData]);

  const scrubberPercent = useMemo(() => {
    if (!logData.length) return 0;
    return ((currentTime - timeRange.start) / (timeRange.end - timeRange.start)) * 100;
  }, [currentTime, timeRange, logData]);

  const handleScrub = (e) => {
    const rect = e.currentTarget.getBoundingClientRect();
    const pct = Math.max(0, Math.min(1, (e.clientX - rect.left) / rect.width));
    setCurrentTime(timeRange.start + pct * (timeRange.end - timeRange.start));
  };

  // ── Render ─────────────────────────────────────────────────────────
  return (
    <div className="space-y-6 p-4">
      <div className="flex items-center justify-between">
        <h2 className="text-2xl font-bold">Blackbox Logs</h2>
        <div className="flex gap-2">
          <button
            onClick={requestLogList}
            disabled={!connected}
            className="bg-gray-500 hover:bg-gray-700 disabled:opacity-40 text-white font-bold py-2 px-4 rounded"
          >
            Refresh
          </button>
          <button
            onClick={handleDownloadAll}
            disabled={!connected || !logFiles.length}
            className="bg-blue-500 hover:bg-blue-700 disabled:opacity-40 text-white font-bold py-2 px-4 rounded"
          >
            Download All
          </button>
          {logData.length > 0 && (
            <button
              onClick={handleExportCSV}
              className="bg-green-500 hover:bg-green-700 text-white font-bold py-2 px-4 rounded"
            >
              Export CSV
            </button>
          )}
        </div>
      </div>

      {!connected && (
        <div className="bg-yellow-100 border-l-4 border-yellow-500 text-yellow-700 p-4 rounded">
          Connect to the flight controller to access blackbox logs.
        </div>
      )}

      {/* ── Log File List ──────────────────────────────────────────── */}
      <div className="bg-white rounded-lg shadow-md overflow-hidden">
        <div className="px-4 py-3 bg-gray-50 border-b font-semibold text-sm text-gray-600 uppercase tracking-wide">
          Available Logs {logFiles.length > 0 && `(${logFiles.length})`}
        </div>
        {loadingList ? (
          <div className="p-8 text-center text-gray-400">Loading logs…</div>
        ) : logFiles.length === 0 ? (
          <div className="p-8 text-center text-gray-400">No log files found on the flight controller.</div>
        ) : (
          <div className="divide-y max-h-64 overflow-y-auto">
            {logFiles.map(file => (
              <div
                key={file.id}
                className={`flex items-center justify-between px-4 py-3 hover:bg-gray-50 transition-colors ${
                  selectedLog === file.id ? 'bg-blue-50 border-l-4 border-blue-500' : ''
                }`}
              >
                <div className="flex-1 min-w-0">
                  <div className="font-medium truncate">{file.name}</div>
                  <div className="text-xs text-gray-500 space-x-3">
                    <span>{file.date}</span>
                    <span>{formatBytes(file.size)}</span>
                    <span>{formatDuration(file.duration)}</span>
                  </div>
                </div>
                <div className="flex gap-2 ml-4 shrink-0">
                  <button
                    onClick={() => handleDownload(file.id)}
                    disabled={downloading === file.id}
                    className="bg-blue-500 hover:bg-blue-700 disabled:opacity-40 text-white text-sm py-1 px-3 rounded"
                  >
                    {downloading === file.id ? 'Loading…' : 'Download'}
                  </button>
                  <button
                    onClick={() => handleDelete(file.id)}
                    className="bg-red-500 hover:bg-red-700 text-white text-sm py-1 px-3 rounded"
                  >
                    Delete
                  </button>
                </div>
              </div>
            ))}
          </div>
        )}
      </div>

      {/* ── Replay Viewer ──────────────────────────────────────────── */}
      {logData.length > 0 && (
        <div className="bg-white rounded-lg shadow-md p-4 space-y-4">
          <div className="flex items-center justify-between">
            <h3 className="text-lg font-semibold">Replay Viewer</h3>
            <div className="flex gap-1">
              {Object.entries(DATASET_GROUPS).map(([key, group]) => (
                <button
                  key={key}
                  onClick={() => toggleGroup(key)}
                  className={`text-xs font-medium py-1 px-3 rounded-full border transition-colors ${
                    visibleGroups[key]
                      ? 'bg-gray-800 text-white border-gray-800'
                      : 'bg-white text-gray-500 border-gray-300 hover:border-gray-400'
                  }`}
                >
                  {group.label}
                </button>
              ))}
            </div>
          </div>

          <div className="h-80">
            <Line ref={chartRef} data={chartData} options={chartOptions} />
          </div>

          {/* Timeline scrubber */}
          <div className="space-y-1">
            <div
              className="relative h-4 bg-gray-200 rounded-full cursor-pointer group"
              onClick={handleScrub}
              onMouseDown={(e) => {
                handleScrub(e);
                const onMove = (ev) => handleScrub(ev);
                const onUp = () => {
                  window.removeEventListener('mousemove', onMove);
                  window.removeEventListener('mouseup', onUp);
                };
                window.addEventListener('mousemove', onMove);
                window.addEventListener('mouseup', onUp);
              }}
            >
              <div
                className="absolute top-0 left-0 h-full bg-blue-500 rounded-full transition-[width] duration-75"
                style={{ width: `${scrubberPercent}%` }}
              />
              <div
                className="absolute top-1/2 -translate-y-1/2 w-4 h-4 bg-white border-2 border-blue-500 rounded-full shadow group-hover:scale-125 transition-transform"
                style={{ left: `calc(${scrubberPercent}% - 8px)` }}
              />
            </div>
            <div className="flex justify-between text-xs text-gray-400">
              <span>{formatDuration((currentTime - timeRange.start) / 1000)}</span>
              <span>{formatDuration((timeRange.end - timeRange.start) / 1000)}</span>
            </div>
          </div>

          {/* Playback controls */}
          <div className="flex items-center justify-center gap-4">
            <button
              onClick={() => setCurrentTime(timeRange.start)}
              className="text-gray-500 hover:text-gray-800 transition-colors"
              title="Restart"
            >
              <svg className="w-5 h-5" viewBox="0 0 20 20" fill="currentColor">
                <path d="M4 4a1 1 0 011-1h2a1 1 0 010 2H6v10h1a1 1 0 110 2H5a1 1 0 01-1-1V4z" />
                <path d="M14.293 4.293a1 1 0 011.414 0l.01.01a1 1 0 010 1.414L11.414 10l4.303 4.283a1 1 0 01-1.414 1.434l-5.01-4.99a1 1 0 010-1.414l5.01-5.01-.01-.01z" />
              </svg>
            </button>
            <button
              onClick={() => setIsPlaying(p => !p)}
              className="w-10 h-10 flex items-center justify-center rounded-full bg-blue-500 hover:bg-blue-600 text-white shadow transition-colors"
            >
              {isPlaying ? (
                <svg className="w-5 h-5" viewBox="0 0 20 20" fill="currentColor">
                  <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zM7 8a1 1 0 012 0v4a1 1 0 11-2 0V8zm5-1a1 1 0 00-1 1v4a1 1 0 102 0V8a1 1 0 00-1-1z" clipRule="evenodd" />
                </svg>
              ) : (
                <svg className="w-5 h-5 ml-0.5" viewBox="0 0 20 20" fill="currentColor">
                  <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM9.555 7.168A1 1 0 008 8v4a1 1 0 001.555.832l3-2a1 1 0 000-1.664l-3-2z" clipRule="evenodd" />
                </svg>
              )}
            </button>
            <div className="flex gap-1">
              {PLAYBACK_SPEEDS.map(speed => (
                <button
                  key={speed}
                  onClick={() => setPlaybackSpeed(speed)}
                  className={`text-xs font-semibold py-1 px-2 rounded transition-colors ${
                    playbackSpeed === speed
                      ? 'bg-blue-500 text-white'
                      : 'bg-gray-100 text-gray-600 hover:bg-gray-200'
                  }`}
                >
                  {speed}x
                </button>
              ))}
            </div>
          </div>
        </div>
      )}

      {/* ── Statistics Panel ───────────────────────────────────────── */}
      {stats && (
        <div className="bg-white rounded-lg shadow-md p-4">
          <h3 className="text-lg font-semibold mb-3">Flight Statistics</h3>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
            <StatCard label="Flight Time" value={stats.flightTime} />
            <StatCard label="Samples" value={stats.sampleCount.toLocaleString()} />
            <StatCard label="Avg Motor" value={stats.avgMotor} />
            <StatCard
              label="Battery Range"
              value={stats.batteryMin === '—' ? '—' : `${stats.batteryMin}–${stats.batteryMax} V`}
            />
            <StatCard label="Max Gyro X" value={`${stats.maxGyroX.toFixed(1)} °/s`} />
            <StatCard label="Max Gyro Y" value={`${stats.maxGyroY.toFixed(1)} °/s`} />
            <StatCard label="Max Gyro Z" value={`${stats.maxGyroZ.toFixed(1)} °/s`} />
            <StatCard
              label="Peak Gyro"
              value={`${Math.max(stats.maxGyroX, stats.maxGyroY, stats.maxGyroZ).toFixed(1)} °/s`}
            />
          </div>
        </div>
      )}
    </div>
  );
};

function StatCard({ label, value }) {
  return (
    <div className="bg-gray-50 rounded-lg p-3">
      <div className="text-xs text-gray-500 mb-1">{label}</div>
      <div className="text-lg font-bold text-gray-800">{value}</div>
    </div>
  );
}

export default Blackbox;
