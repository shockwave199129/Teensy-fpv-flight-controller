import React, { useState, useEffect } from 'react';
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
} from 'chart.js';

ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
);

const DynamicFiltering = () => {
  const [spectralData, setSpectralData] = useState({
    numPeaks: 0,
    peakFrequencies: [],
    peakAmplitudes: [],
    analysisComplete: false,
    lastAnalysis: 0,
    noiseFloor: 0.1
  });

  const [filterStatus, setFilterStatus] = useState({
    autoTuneEnabled: true,
    adaptiveFiltersActive: 0,
    currentPhase: 'GROUND',
    gyroLpfCutoff: 100,
    accelLpfCutoff: 30,
    notchFiltersEnabled: true
  });

  const [frequencyData, setFrequencyData] = useState({
    labels: [],
    datasets: [
      {
        label: 'FFT Magnitude',
        data: [],
        borderColor: 'rgb(75, 192, 192)',
        backgroundColor: 'rgba(75, 192, 192, 0.2)',
        tension: 0.1
      }
    ]
  });

  const [gyroData, setGyroData] = useState({
    labels: [],
    datasets: [
      {
        label: 'Gyro X (Raw)',
        data: [],
        borderColor: 'rgb(255, 99, 132)',
        backgroundColor: 'rgba(255, 99, 132, 0.2)',
        tension: 0.1
      },
      {
        label: 'Gyro X (Filtered)',
        data: [],
        borderColor: 'rgb(54, 162, 235)',
        backgroundColor: 'rgba(54, 162, 235, 0.2)',
        tension: 0.1
      }
    ]
  });

  const [realtimeMetrics, setRealtimeMetrics] = useState({
    vibrationsDetected: false,
    avgNoiseLevel: 0,
    filterEfficiency: 0,
    loopFrequency: 0
  });

  const flightPhases = {
    'GROUND': { color: 'bg-gray-500', description: 'On Ground' },
    'TAKEOFF': { color: 'bg-yellow-500', description: 'Taking Off' },
    'HOVER': { color: 'bg-green-500', description: 'Hovering' },
    'FORWARD_FLIGHT': { color: 'bg-blue-500', description: 'Forward Flight' },
    'AGGRESSIVE_MANEUVERS': { color: 'bg-red-500', description: 'Aggressive Maneuvers' },
    'LANDING': { color: 'bg-orange-500', description: 'Landing' }
  };

  useEffect(() => {
    // Request filtering status every second
    const interval = setInterval(() => {
      window.electronAPI.sendSerial('filtering status');
      window.electronAPI.sendSerial('filtering analysis');
    }, 1000);

    // Listen for filtering data updates
    const handleFilteringData = (data) => {
      setSpectralData(data.spectral);
      setFilterStatus(data.status);
      setRealtimeMetrics(data.metrics);
      updateCharts(data);
    };

    window.electronAPI.on('filtering-data', handleFilteringData);

    return () => {
      clearInterval(interval);
      window.electronAPI.off('filtering-data', handleFilteringData);
    };
  }, []);

  const updateCharts = (data) => {
    // Update frequency spectrum chart
    if (data.spectral.analysisComplete) {
      const frequencies = Array.from({ length: 128 }, (_, i) => i * 2); // 0-256 Hz range
      setFrequencyData(prev => ({
        ...prev,
        labels: frequencies,
        datasets: [{
          ...prev.datasets[0],
          data: data.spectral.frequencyBins || Array(128).fill(0)
        }]
      }));
    }

    // Update gyro data chart (rolling window)
    const timestamp = Date.now();
    setGyroData(prev => {
      const newLabels = [...prev.labels, timestamp].slice(-50); // Keep last 50 points
      return {
        labels: newLabels,
        datasets: [
          {
            ...prev.datasets[0],
            data: [...prev.datasets[0].data, data.gyroRaw?.x || 0].slice(-50)
          },
          {
            ...prev.datasets[1],
            data: [...prev.datasets[1].data, data.gyroFiltered?.x || 0].slice(-50)
          }
        ]
      };
    });
  };

  const NoiseSourceCard = ({ frequency, amplitude, index }) => (
    <div className="bg-white p-4 rounded-lg shadow-md border">
      <h4 className="font-semibold text-lg mb-2">Noise Peak #{index + 1}</h4>
      <div className="space-y-2">
        <div className="flex justify-between">
          <span>Frequency:</span>
          <span className="font-mono">{frequency.toFixed(1)} Hz</span>
        </div>
        <div className="flex justify-between">
          <span>Amplitude:</span>
          <div className="flex items-center">
            <div className="w-20 bg-gray-200 rounded-full h-2 mr-2">
              <div 
                className="bg-red-500 h-2 rounded-full" 
                style={{ width: `${amplitude * 100}%` }}
              ></div>
            </div>
            <span className="font-mono">{(amplitude * 100).toFixed(1)}%</span>
          </div>
        </div>
        <div className="text-sm text-gray-600">
          {frequency < 50 ? 'Low frequency vibration' :
           frequency < 150 ? 'Motor fundamental' :
           frequency < 400 ? 'Motor harmonic' :
           'High frequency noise'}
        </div>
      </div>
    </div>
  );

  const FilterStatusCard = ({ title, value, unit, status, description }) => (
    <div className="bg-white p-4 rounded-lg shadow-md">
      <h4 className="font-semibold text-sm text-gray-600 mb-1">{title}</h4>
      <div className="flex items-center justify-between">
        <div className="text-2xl font-bold">{value}{unit}</div>
        <div className={`px-2 py-1 rounded text-xs font-medium ${
          status === 'good' ? 'bg-green-100 text-green-800' :
          status === 'warning' ? 'bg-yellow-100 text-yellow-800' :
          'bg-red-100 text-red-800'
        }`}>
          {status.toUpperCase()}
        </div>
      </div>
      {description && (
        <p className="text-sm text-gray-600 mt-2">{description}</p>
      )}
    </div>
  );

  const chartOptions = {
    responsive: true,
    plugins: {
      legend: {
        position: 'top',
      },
      title: {
        display: true,
        text: 'Frequency Analysis'
      }
    },
    scales: {
      y: {
        beginAtZero: true
      }
    }
  };

  return (
    <div className="p-6">
      <h1 className="text-3xl font-bold mb-6">Dynamic Filtering System</h1>

      {/* System Status Overview */}
      <div className="grid grid-cols-1 md:grid-cols-4 gap-4 mb-6">
        <FilterStatusCard
          title="Auto-Tune Status"
          value={filterStatus.autoTuneEnabled ? 'ON' : 'OFF'}
          unit=""
          status={filterStatus.autoTuneEnabled ? 'good' : 'warning'}
          description="Automatic filter adjustment"
        />
        <FilterStatusCard
          title="Active Filters"
          value={filterStatus.adaptiveFiltersActive}
          unit=""
          status={filterStatus.adaptiveFiltersActive > 0 ? 'good' : 'warning'}
          description="Number of adaptive notch filters"
        />
        <FilterStatusCard
          title="Loop Frequency"
          value={realtimeMetrics.loopFrequency}
          unit=" Hz"
          status={realtimeMetrics.loopFrequency > 1800 ? 'good' : 'warning'}
          description="Control loop update rate"
        />
        <FilterStatusCard
          title="Filter Efficiency"
          value={realtimeMetrics.filterEfficiency}
          unit="%"
          status={realtimeMetrics.filterEfficiency > 80 ? 'good' : 'warning'}
          description="Noise reduction effectiveness"
        />
      </div>

      {/* Flight Phase Indicator */}
      <div className="mb-6 p-4 bg-white rounded-lg shadow-md">
        <h3 className="text-lg font-semibold mb-3">Current Flight Phase</h3>
        <div className="flex items-center space-x-4">
          <div className={`px-4 py-2 rounded-full text-white font-medium ${
            flightPhases[filterStatus.currentPhase]?.color || 'bg-gray-500'
          }`}>
            {flightPhases[filterStatus.currentPhase]?.description || filterStatus.currentPhase}
          </div>
          <div className="text-sm text-gray-600">
            Filter settings automatically adjust based on flight phase
          </div>
        </div>
      </div>

      {/* Vibration Alert */}
      {realtimeMetrics.vibrationsDetected && (
        <div className="mb-6 p-4 bg-red-100 border border-red-400 text-red-700 rounded-lg">
          <div className="flex items-center">
            <div className="w-4 h-4 bg-red-500 rounded-full mr-3 animate-pulse"></div>
            <strong>High Vibrations Detected!</strong>
          </div>
          <p className="mt-2 text-sm">
            Excessive vibrations detected. Check propeller balance, motor mounting, or frame integrity.
          </p>
        </div>
      )}

      {/* Charts Section */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
        <div className="bg-white p-6 rounded-lg shadow-md">
          <h3 className="text-lg font-semibold mb-4">Frequency Spectrum Analysis</h3>
          <Line data={frequencyData} options={chartOptions} />
        </div>
        <div className="bg-white p-6 rounded-lg shadow-md">
          <h3 className="text-lg font-semibold mb-4">Gyro Data (Raw vs Filtered)</h3>
          <Line 
            data={gyroData} 
            options={{
              ...chartOptions,
              plugins: {
                ...chartOptions.plugins,
                title: { display: true, text: 'Gyro X-Axis Filtering' }
              }
            }} 
          />
        </div>
      </div>

      {/* Detected Noise Sources */}
      <div className="mb-6">
        <h3 className="text-lg font-semibold mb-4">Detected Noise Sources</h3>
        {spectralData.numPeaks > 0 ? (
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
            {spectralData.peakFrequencies.slice(0, spectralData.numPeaks).map((freq, index) => (
              <NoiseSourceCard
                key={index}
                frequency={freq}
                amplitude={spectralData.peakAmplitudes[index]}
                index={index}
              />
            ))}
          </div>
        ) : (
          <div className="bg-green-100 border border-green-400 text-green-700 p-4 rounded-lg">
            No significant noise sources detected. System operating cleanly.
          </div>
        )}
      </div>

      {/* Filter Configuration */}
      <div className="bg-white p-6 rounded-lg shadow-md">
        <h3 className="text-lg font-semibold mb-4">Filter Configuration</h3>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-2">
              Gyro LPF Cutoff: {filterStatus.gyroLpfCutoff} Hz
            </label>
            <input
              type="range"
              min="50"
              max="200"
              value={filterStatus.gyroLpfCutoff}
              onChange={(e) => {
                const value = e.target.value;
                setFilterStatus(prev => ({ ...prev, gyroLpfCutoff: parseInt(value) }));
                window.electronAPI.sendSerial(`set gyro_lpf ${value}`);
              }}
              className="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-2">
              Accel LPF Cutoff: {filterStatus.accelLpfCutoff} Hz
            </label>
            <input
              type="range"
              min="10"
              max="100"
              value={filterStatus.accelLpfCutoff}
              onChange={(e) => {
                const value = e.target.value;
                setFilterStatus(prev => ({ ...prev, accelLpfCutoff: parseInt(value) }));
                window.electronAPI.sendSerial(`set accel_lpf ${value}`);
              }}
              className="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer"
            />
          </div>
          <div className="flex flex-col space-y-3">
            <label className="flex items-center">
              <input
                type="checkbox"
                checked={filterStatus.autoTuneEnabled}
                onChange={(e) => {
                  setFilterStatus(prev => ({ ...prev, autoTuneEnabled: e.target.checked }));
                  window.electronAPI.sendSerial(`filtering auto ${e.target.checked ? 'on' : 'off'}`);
                }}
                className="mr-2"
              />
              <span className="text-sm">Auto-tune filters</span>
            </label>
            <label className="flex items-center">
              <input
                type="checkbox"
                checked={filterStatus.notchFiltersEnabled}
                onChange={(e) => {
                  setFilterStatus(prev => ({ ...prev, notchFiltersEnabled: e.target.checked }));
                  window.electronAPI.sendSerial(`set notch_filters ${e.target.checked ? 'on' : 'off'}`);
                }}
                className="mr-2"
              />
              <span className="text-sm">Enable notch filters</span>
            </label>
          </div>
        </div>
      </div>
    </div>
  );
};

export default DynamicFiltering; 