import React, { useState, useEffect, useRef, useCallback } from 'react';

const DEG_TO_RAD = Math.PI / 180;
const EARTH_RADIUS_M = 6371000;
const MAX_TRAIL_POINTS = 100;

const FIX_LABELS = ['No Fix', '2D Fix', '3D Fix'];

function latLonToMeters(lat1, lon1, lat2, lon2) {
  const dLat = (lat2 - lat1) * DEG_TO_RAD;
  const dLon = (lon2 - lon1) * DEG_TO_RAD;
  const avgLat = ((lat1 + lat2) / 2) * DEG_TO_RAD;
  return { dx: dLon * EARTH_RADIUS_M * Math.cos(avgLat), dy: dLat * EARTH_RADIUS_M };
}

function haversineDistance(lat1, lon1, lat2, lon2) {
  const dLat = (lat2 - lat1) * DEG_TO_RAD;
  const dLon = (lon2 - lon1) * DEG_TO_RAD;
  const a =
    Math.sin(dLat / 2) ** 2 +
    Math.cos(lat1 * DEG_TO_RAD) * Math.cos(lat2 * DEG_TO_RAD) * Math.sin(dLon / 2) ** 2;
  return EARTH_RADIUS_M * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

function bearingTo(lat1, lon1, lat2, lon2) {
  const dLon = (lon2 - lon1) * DEG_TO_RAD;
  const y = Math.sin(dLon) * Math.cos(lat2 * DEG_TO_RAD);
  const x =
    Math.cos(lat1 * DEG_TO_RAD) * Math.sin(lat2 * DEG_TO_RAD) -
    Math.sin(lat1 * DEG_TO_RAD) * Math.cos(lat2 * DEG_TO_RAD) * Math.cos(dLon);
  return ((Math.atan2(y, x) * 180) / Math.PI + 360) % 360;
}

const GPSMap = ({ sendCommand, connected }) => {
  const canvasRef = useRef(null);

  const [gpsData, setGpsData] = useState({
    lat: 0, lon: 0, alt: 0, speed: 0, heading: 0,
    satellites: 0, fix: 0, hdop: 99.9,
  });

  const [homePosition, setHomePosition] = useState({ lat: 0, lon: 0, alt: 0, set: false });
  const [flightPath, setFlightPath] = useState([]);
  const [syntheticGps, setSyntheticGps] = useState({ active: false, confidence: 0, source: 'none' });
  const [rthAltitude, setRthAltitude] = useState(30);
  const [rthConfirmOpen, setRthConfirmOpen] = useState(false);
  const [zoomLevel, setZoomLevel] = useState(1);

  useEffect(() => {
    if (!connected) return;

    const interval = setInterval(() => {
      sendCommand('gps status');
    }, 1000);

    const handleSerialData = (_event, data) => {
      try {
        if (data.startsWith('{')) {
          const json = JSON.parse(data);

          if (json.gps) {
            const g = json.gps;
            setGpsData({
              lat: g.lat ?? 0,
              lon: g.lon ?? 0,
              alt: g.alt ?? 0,
              speed: g.speed ?? 0,
              heading: g.heading ?? 0,
              satellites: g.satellites ?? 0,
              fix: g.fix ?? 0,
              hdop: g.hdop ?? 99.9,
            });

            if (g.lat !== 0 || g.lon !== 0) {
              setFlightPath((prev) => {
                const next = [...prev, { lat: g.lat, lon: g.lon }];
                return next.length > MAX_TRAIL_POINTS ? next.slice(-MAX_TRAIL_POINTS) : next;
              });
            }
          }

          if (json.synthetic_gps) {
            const s = json.synthetic_gps;
            setSyntheticGps({
              active: s.active ?? false,
              confidence: s.confidence ?? 0,
              source: s.source ?? 'none',
            });
          }
        }
      } catch {
        // non-JSON serial traffic – ignore
      }
    };

    if (window.electronAPI) {
      window.electronAPI.onSerialData(handleSerialData);
    }

    return () => {
      clearInterval(interval);
      if (window.electronAPI) {
        window.electronAPI.removeAllListeners('serial-data');
      }
    };
  }, [connected, sendCommand]);

  const handleSetHome = () => {
    if (gpsData.fix >= 2) {
      setHomePosition({ lat: gpsData.lat, lon: gpsData.lon, alt: gpsData.alt, set: true });
      sendCommand(`gps set_home ${gpsData.lat} ${gpsData.lon} ${gpsData.alt}`);
    }
  };

  const handleActivateRTH = () => {
    sendCommand(`rth activate ${rthAltitude}`);
    setRthConfirmOpen(false);
  };

  const distToHome = homePosition.set
    ? haversineDistance(gpsData.lat, gpsData.lon, homePosition.lat, homePosition.lon)
    : null;

  const bearingHome = homePosition.set
    ? bearingTo(gpsData.lat, gpsData.lon, homePosition.lat, homePosition.lon)
    : null;

  // ── Canvas drawing ────────────────────────────────────────────────
  const drawMap = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width;
    const H = canvas.height;
    const cx = W / 2;
    const cy = H / 2;

    ctx.clearRect(0, 0, W, H);

    // Background
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(0, 0, W, H);

    // Determine center reference (home if set, else current pos)
    const refLat = homePosition.set ? homePosition.lat : gpsData.lat;
    const refLon = homePosition.set ? homePosition.lon : gpsData.lon;

    // Auto-scale: find max distance from ref among trail points + current pos
    let maxDist = 20;
    const allPoints = [...flightPath, { lat: gpsData.lat, lon: gpsData.lon }];
    allPoints.forEach((p) => {
      const d = haversineDistance(refLat, refLon, p.lat, p.lon);
      if (d > maxDist) maxDist = d;
    });
    maxDist *= 1.3;

    const baseScale = Math.min(W, H) / 2 / maxDist;
    const scale = baseScale * zoomLevel;

    const toScreen = (lat, lon) => {
      const { dx, dy } = latLonToMeters(refLat, refLon, lat, lon);
      return { x: cx + dx * scale, y: cy - dy * scale };
    };

    // Grid
    const gridSpacingM = chooseGridSpacing(maxDist);
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 0.5;
    ctx.font = '10px monospace';
    ctx.fillStyle = '#475569';

    const halfRangeM = (W / 2) / scale;
    const startM = -Math.ceil(halfRangeM / gridSpacingM) * gridSpacingM;
    const endM = -startM;
    for (let m = startM; m <= endM; m += gridSpacingM) {
      const px = cx + m * scale;
      ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, H); ctx.stroke();
      if (m !== 0) ctx.fillText(`${m.toFixed(0)}m`, px + 2, H - 4);

      const py = cy - m * scale;
      ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(W, py); ctx.stroke();
      if (m !== 0) ctx.fillText(`${m.toFixed(0)}m`, 4, py - 2);
    }

    // Distance rings from home
    if (homePosition.set) {
      ctx.setLineDash([4, 4]);
      [10, 50, 100, 200].forEach((r) => {
        const rPx = r * scale;
        if (rPx < 8 || rPx > W) return;
        ctx.strokeStyle = '#334155';
        ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.arc(cx, cy, rPx, 0, Math.PI * 2);
        ctx.stroke();
        ctx.fillStyle = '#64748b';
        ctx.fillText(`${r}m`, cx + rPx * 0.707 + 3, cy - rPx * 0.707 - 3);
      });
      ctx.setLineDash([]);
    }

    // Flight path trail
    if (flightPath.length > 1) {
      ctx.lineWidth = 2;
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      for (let i = 1; i < flightPath.length; i++) {
        const alpha = 0.2 + 0.8 * (i / flightPath.length);
        ctx.strokeStyle = `rgba(56, 189, 248, ${alpha})`;
        const a = toScreen(flightPath[i - 1].lat, flightPath[i - 1].lon);
        const b = toScreen(flightPath[i].lat, flightPath[i].lon);
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      }
    }

    // Home marker
    if (homePosition.set) {
      ctx.fillStyle = '#ef4444';
      ctx.strokeStyle = '#fca5a5';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(cx, cy, 7, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 10px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('H', cx, cy + 3.5);
      ctx.textAlign = 'start';
    }

    // Current position
    const cur = toScreen(gpsData.lat, gpsData.lon);
    const headingRad = (gpsData.heading - 90) * DEG_TO_RAD;

    ctx.save();
    ctx.translate(cur.x, cur.y);
    ctx.rotate(headingRad + Math.PI / 2);

    // Heading cone
    ctx.fillStyle = 'rgba(56, 189, 248, 0.15)';
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.arc(0, 0, 30, -0.4, 0.4);
    ctx.closePath();
    ctx.fill();

    // Drone triangle
    ctx.fillStyle = '#38bdf8';
    ctx.strokeStyle = '#0ea5e9';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(0, -10);
    ctx.lineTo(-6, 7);
    ctx.lineTo(6, 7);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.restore();

    // North arrow (top-right)
    const naX = W - 30;
    const naY = 30;
    ctx.save();
    ctx.translate(naX, naY);
    ctx.fillStyle = '#ef4444';
    ctx.beginPath();
    ctx.moveTo(0, -16); ctx.lineTo(-6, 4); ctx.lineTo(0, -2); ctx.closePath();
    ctx.fill();
    ctx.fillStyle = '#94a3b8';
    ctx.beginPath();
    ctx.moveTo(0, -2); ctx.lineTo(6, 4); ctx.lineTo(0, 16); ctx.closePath();
    ctx.fill();
    ctx.fillStyle = '#ef4444';
    ctx.font = 'bold 11px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('N', 0, -20);
    ctx.restore();

    // Scale bar (bottom-left)
    const scaleBarM = chooseScaleBar(maxDist);
    const scaleBarPx = scaleBarM * scale;
    if (scaleBarPx > 10) {
      const sbX = 16;
      const sbY = H - 16;
      ctx.strokeStyle = '#94a3b8';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(sbX, sbY); ctx.lineTo(sbX + scaleBarPx, sbY); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(sbX, sbY - 4); ctx.lineTo(sbX, sbY + 4); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(sbX + scaleBarPx, sbY - 4); ctx.lineTo(sbX + scaleBarPx, sbY + 4); ctx.stroke();
      ctx.fillStyle = '#94a3b8';
      ctx.font = '10px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(scaleBarM >= 1000 ? `${(scaleBarM / 1000).toFixed(1)}km` : `${scaleBarM}m`, sbX + scaleBarPx / 2, sbY - 6);
    }

    // No-fix overlay
    if (gpsData.fix === 0 && !homePosition.set && flightPath.length === 0) {
      ctx.fillStyle = 'rgba(15, 23, 42, 0.7)';
      ctx.fillRect(0, 0, W, H);
      ctx.fillStyle = '#94a3b8';
      ctx.font = '16px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Waiting for GPS fix…', cx, cy);
      ctx.textAlign = 'start';
    }
  }, [gpsData, homePosition, flightPath, zoomLevel]);

  useEffect(() => {
    drawMap();
  }, [drawMap]);

  const fixLabel = FIX_LABELS[gpsData.fix] ?? `Fix ${gpsData.fix}`;
  const fixColor = gpsData.fix >= 3 ? 'text-green-400' : gpsData.fix === 2 ? 'text-yellow-400' : 'text-red-400';
  const hdopColor = gpsData.hdop <= 1.5 ? 'text-green-400' : gpsData.hdop <= 3 ? 'text-yellow-400' : 'text-red-400';

  return (
    <div className="p-6">
      <h1 className="text-3xl font-bold mb-6">GPS Map &amp; Navigation</h1>

      {!connected && (
        <div className="p-4 rounded-lg mb-6 bg-yellow-100 border border-yellow-400 text-yellow-700">
          Connect to the flight controller to receive GPS data.
        </div>
      )}

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* ── Left column: Data panels ──────────────────────────── */}
        <div className="space-y-6">
          {/* Position Display */}
          <div className="p-5 rounded-lg shadow-md bg-white border border-gray-200">
            <h3 className="text-lg font-semibold mb-3 flex items-center gap-2">
              <span className="text-blue-500">📡</span> GPS Status
            </h3>
            <div className="grid grid-cols-2 gap-x-6 gap-y-2 text-sm">
              <DataRow label="Latitude" value={gpsData.lat.toFixed(7) + '°'} />
              <DataRow label="Longitude" value={gpsData.lon.toFixed(7) + '°'} />
              <DataRow label="Altitude" value={gpsData.alt.toFixed(1) + ' m'} />
              <DataRow label="Speed" value={gpsData.speed.toFixed(1) + ' m/s'} />
              <DataRow label="Heading" value={gpsData.heading.toFixed(0) + '°'} />
              <DataRow label="Satellites" value={gpsData.satellites} />
              <div className="flex justify-between col-span-2 py-1 border-b border-gray-100">
                <span className="text-gray-500">Fix</span>
                <span className={`font-medium ${fixColor}`}>{fixLabel}</span>
              </div>
              <div className="flex justify-between col-span-2 py-1">
                <span className="text-gray-500">HDOP</span>
                <span className={`font-medium ${hdopColor}`}>{gpsData.hdop.toFixed(1)}</span>
              </div>
            </div>
          </div>

          {/* Home Position */}
          <div className="p-5 rounded-lg shadow-md bg-white border border-gray-200">
            <h3 className="text-lg font-semibold mb-3 flex items-center gap-2">
              <span className="text-red-500">🏠</span> Home Position
            </h3>
            {homePosition.set ? (
              <div className="text-sm space-y-1">
                <DataRow label="Home Lat" value={homePosition.lat.toFixed(7) + '°'} />
                <DataRow label="Home Lon" value={homePosition.lon.toFixed(7) + '°'} />
                <DataRow label="Home Alt" value={homePosition.alt.toFixed(1) + ' m'} />
                <div className="pt-2 border-t border-gray-100 mt-2 grid grid-cols-2 gap-2">
                  <div>
                    <span className="text-gray-500 text-xs block">Distance</span>
                    <span className="font-bold text-lg">{distToHome != null ? distToHome.toFixed(1) : '--'} m</span>
                  </div>
                  <div>
                    <span className="text-gray-500 text-xs block">Bearing</span>
                    <span className="font-bold text-lg">{bearingHome != null ? bearingHome.toFixed(0) : '--'}°</span>
                  </div>
                </div>
              </div>
            ) : (
              <p className="text-sm text-gray-400 italic">Home not set</p>
            )}
            <button
              onClick={handleSetHome}
              disabled={gpsData.fix < 2}
              className="mt-3 w-full bg-red-500 hover:bg-red-600 disabled:bg-gray-300 disabled:cursor-not-allowed text-white font-bold py-2 px-4 rounded transition"
            >
              Set Home Position
            </button>
          </div>

          {/* Synthetic GPS */}
          <div className="p-5 rounded-lg shadow-md bg-white border border-gray-200">
            <h3 className="text-lg font-semibold mb-3 flex items-center gap-2">
              <span className="text-purple-500">🔮</span> Synthetic GPS
            </h3>
            <div className="text-sm space-y-2">
              <div className="flex justify-between">
                <span className="text-gray-500">Status</span>
                <span className={`font-medium ${syntheticGps.active ? 'text-green-600' : 'text-gray-400'}`}>
                  {syntheticGps.active ? 'Active' : 'Inactive'}
                </span>
              </div>
              {syntheticGps.active && (
                <>
                  <div>
                    <div className="flex justify-between mb-1">
                      <span className="text-gray-500">Confidence</span>
                      <span className="font-medium">{syntheticGps.confidence.toFixed(0)}%</span>
                    </div>
                    <div className="w-full bg-gray-200 rounded-full h-2">
                      <div
                        className={`h-2 rounded-full transition-all ${
                          syntheticGps.confidence > 70 ? 'bg-green-500' :
                          syntheticGps.confidence > 40 ? 'bg-yellow-500' : 'bg-red-500'
                        }`}
                        style={{ width: `${Math.min(syntheticGps.confidence, 100)}%` }}
                      />
                    </div>
                  </div>
                  <DataRow label="Source" value={syntheticGps.source} />
                </>
              )}
            </div>
          </div>
        </div>

        {/* ── Center + Right: Map canvas ───────────────────────── */}
        <div className="lg:col-span-2 space-y-4">
          <div className="bg-slate-900 rounded-lg shadow-md overflow-hidden border border-slate-700">
            <canvas
              ref={canvasRef}
              width={720}
              height={480}
              className="w-full"
              style={{ imageRendering: 'auto' }}
            />
          </div>

          {/* Map controls */}
          <div className="flex items-center gap-3">
            <button
              onClick={() => setZoomLevel((z) => Math.min(z * 1.5, 20))}
              className="bg-slate-700 hover:bg-slate-600 text-white font-bold py-1.5 px-4 rounded transition"
            >
              +
            </button>
            <button
              onClick={() => setZoomLevel((z) => Math.max(z / 1.5, 0.1))}
              className="bg-slate-700 hover:bg-slate-600 text-white font-bold py-1.5 px-4 rounded transition"
            >
              −
            </button>
            <button
              onClick={() => setZoomLevel(1)}
              className="bg-slate-700 hover:bg-slate-600 text-white text-sm py-1.5 px-4 rounded transition"
            >
              Reset Zoom
            </button>
            <button
              onClick={() => setFlightPath([])}
              className="bg-slate-700 hover:bg-slate-600 text-white text-sm py-1.5 px-4 rounded transition"
            >
              Clear Trail
            </button>
            <span className="ml-auto text-sm text-gray-500">
              Trail: {flightPath.length} / {MAX_TRAIL_POINTS} pts
            </span>
          </div>

          {/* RTH Controls */}
          <div className="p-5 rounded-lg shadow-md bg-white border border-gray-200">
            <h3 className="text-lg font-semibold mb-3 flex items-center gap-2">
              <span className="text-orange-500">🔁</span> Return to Home
            </h3>
            <div className="flex flex-wrap items-end gap-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  RTH Altitude (m)
                </label>
                <input
                  type="number"
                  min={5}
                  max={120}
                  value={rthAltitude}
                  onChange={(e) => setRthAltitude(Number(e.target.value))}
                  className="w-24 border border-gray-300 rounded px-3 py-2 text-sm focus:ring-2 focus:ring-orange-400 focus:outline-none"
                />
              </div>

              {!rthConfirmOpen ? (
                <button
                  onClick={() => setRthConfirmOpen(true)}
                  disabled={!homePosition.set}
                  className="bg-orange-500 hover:bg-orange-600 disabled:bg-gray-300 disabled:cursor-not-allowed text-white font-bold py-2 px-6 rounded transition"
                >
                  Activate RTH
                </button>
              ) : (
                <div className="flex items-center gap-2 bg-orange-50 border border-orange-300 rounded p-2">
                  <span className="text-sm text-orange-800 font-medium">Confirm RTH?</span>
                  <button
                    onClick={handleActivateRTH}
                    className="bg-orange-600 hover:bg-orange-700 text-white font-bold py-1 px-4 rounded text-sm transition"
                  >
                    Yes, Activate
                  </button>
                  <button
                    onClick={() => setRthConfirmOpen(false)}
                    className="bg-gray-200 hover:bg-gray-300 text-gray-700 font-bold py-1 px-4 rounded text-sm transition"
                  >
                    Cancel
                  </button>
                </div>
              )}

              {homePosition.set && distToHome != null && (
                <span className="text-sm text-gray-500 ml-auto">
                  Distance to home: <strong>{distToHome.toFixed(1)} m</strong>
                  &nbsp;·&nbsp;Bearing: <strong>{bearingHome.toFixed(0)}°</strong>
                </span>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

const DataRow = ({ label, value }) => (
  <div className="flex justify-between py-1 border-b border-gray-100">
    <span className="text-gray-500">{label}</span>
    <span className="font-medium">{value}</span>
  </div>
);

function chooseGridSpacing(maxDistM) {
  const targets = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000];
  for (const t of targets) {
    if (maxDistM / t <= 8) return t;
  }
  return 5000;
}

function chooseScaleBar(maxDistM) {
  const targets = [1, 2, 5, 10, 25, 50, 100, 250, 500, 1000, 2500];
  for (const t of targets) {
    if (t >= maxDistM * 0.15 && t <= maxDistM * 0.4) return t;
  }
  return Math.round(maxDistM * 0.25);
}

export default GPSMap;
