import React, { useState, useEffect, useRef, useCallback } from 'react';

const GRID_COLS = 288;
const GRID_ROWS = 162;

const DEFAULT_ELEMENTS = [
  { id: 'altitude',        name: 'Altitude',           visible: true,  x: 1,   y: 1,   fontSize: 'Normal' },
  { id: 'speed',           name: 'Speed',              visible: true,  x: 1,   y: 3,   fontSize: 'Normal' },
  { id: 'bat_voltage',     name: 'Battery Voltage',    visible: true,  x: 240, y: 1,   fontSize: 'Normal' },
  { id: 'bat_percent',     name: 'Battery Percentage', visible: true,  x: 240, y: 3,   fontSize: 'Normal' },
  { id: 'rssi',            name: 'RSSI',               visible: true,  x: 240, y: 5,   fontSize: 'Normal' },
  { id: 'horizon',         name: 'Artificial Horizon',  visible: true,  x: 120, y: 60,  fontSize: 'Large' },
  { id: 'crosshair',       name: 'Crosshair',          visible: true,  x: 140, y: 78,  fontSize: 'Normal' },
  { id: 'gps_coords',      name: 'GPS Coordinates',    visible: false, x: 1,   y: 150, fontSize: 'Small' },
  { id: 'gps_sats',        name: 'GPS Satellites',     visible: true,  x: 1,   y: 152, fontSize: 'Normal' },
  { id: 'armed',           name: 'Armed Status',       visible: true,  x: 120, y: 155, fontSize: 'Large' },
  { id: 'flight_mode',     name: 'Flight Mode',        visible: true,  x: 120, y: 5,   fontSize: 'Normal' },
  { id: 'timer',           name: 'Timer',              visible: true,  x: 240, y: 155, fontSize: 'Normal' },
  { id: 'compass',         name: 'Compass',            visible: false, x: 130, y: 10,  fontSize: 'Normal' },
  { id: 'throttle',        name: 'Throttle Position',  visible: true,  x: 1,   y: 5,   fontSize: 'Normal' },
  { id: 'current',         name: 'Current Draw',       visible: true,  x: 240, y: 7,   fontSize: 'Normal' },
  { id: 'mah_used',        name: 'mAh Used',           visible: true,  x: 240, y: 9,   fontSize: 'Normal' },
  { id: 'warnings',        name: 'Warnings',           visible: true,  x: 100, y: 140, fontSize: 'Large' },
];

const FONT_SIZES = ['Small', 'Normal', 'Large'];
const FONT_SCALE = { Small: 0.75, Normal: 1, Large: 1.35 };

const OSDConfig = ({ sendCommand, connected }) => {
  const canvasRef = useRef(null);
  const containerRef = useRef(null);
  const [elements, setElements] = useState(() => DEFAULT_ELEMENTS.map(e => ({ ...e })));
  const [selectedElement, setSelectedElement] = useState(null);
  const [isDragging, setIsDragging] = useState(false);
  const [dragOffset, setDragOffset] = useState({ x: 0, y: 0 });
  const [canvasSize, setCanvasSize] = useState({ width: 0, height: 0 });
  const [statusMsg, setStatusMsg] = useState('');

  const cellW = canvasSize.width / GRID_COLS;
  const cellH = canvasSize.height / GRID_ROWS;

  useEffect(() => {
    const measure = () => {
      if (containerRef.current) {
        const rect = containerRef.current.getBoundingClientRect();
        const aspect = GRID_COLS / GRID_ROWS;
        let w = rect.width;
        let h = w / aspect;
        if (h > rect.height) {
          h = rect.height;
          w = h * aspect;
        }
        setCanvasSize({ width: Math.floor(w), height: Math.floor(h) });
      }
    };
    measure();
    window.addEventListener('resize', measure);
    return () => window.removeEventListener('resize', measure);
  }, []);

  const drawCanvas = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas || canvasSize.width === 0) return;
    const ctx = canvas.getContext('2d');
    const { width, height } = canvasSize;
    canvas.width = width;
    canvas.height = height;

    ctx.fillStyle = '#0a0a0a';
    ctx.fillRect(0, 0, width, height);

    ctx.strokeStyle = '#1a1a1a';
    ctx.lineWidth = 0.5;
    const stepX = Math.max(1, Math.round(GRID_COLS / 36));
    const stepY = Math.max(1, Math.round(GRID_ROWS / 20));
    for (let c = 0; c <= GRID_COLS; c += stepX) {
      const x = Math.round(c * cellW);
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, height);
      ctx.stroke();
    }
    for (let r = 0; r <= GRID_ROWS; r += stepY) {
      const y = Math.round(r * cellH);
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }

    ctx.strokeStyle = '#222';
    ctx.lineWidth = 1;
    ctx.strokeRect(0, 0, width, height);

    elements.forEach(el => {
      if (!el.visible) return;
      const px = el.x * cellW;
      const py = el.y * cellH;
      const scale = FONT_SCALE[el.fontSize] || 1;
      const baseFontSize = Math.max(9, cellH * 1.1);
      const fontSize = baseFontSize * scale;
      const isSelected = selectedElement === el.id;

      ctx.font = `${isSelected ? 'bold ' : ''}${fontSize}px "Courier New", monospace`;
      ctx.textBaseline = 'top';

      const metrics = ctx.measureText(el.name);
      const textW = metrics.width;
      const textH = fontSize * 1.2;

      if (isSelected) {
        ctx.fillStyle = 'rgba(59, 130, 246, 0.15)';
        ctx.fillRect(px - 2, py - 2, textW + 4, textH + 4);
        ctx.strokeStyle = '#3b82f6';
        ctx.lineWidth = 1;
        ctx.strokeRect(px - 2, py - 2, textW + 4, textH + 4);
      }

      ctx.fillStyle = isSelected ? '#60a5fa' : '#e0e0e0';
      ctx.fillText(el.name, px, py);
    });
  }, [elements, selectedElement, canvasSize, cellW, cellH]);

  useEffect(() => {
    drawCanvas();
  }, [drawCanvas]);

  const pixelToGrid = useCallback((px, py) => {
    if (cellW === 0 || cellH === 0) return { gx: 0, gy: 0 };
    const canvas = canvasRef.current;
    if (!canvas) return { gx: 0, gy: 0 };
    const rect = canvas.getBoundingClientRect();
    const localX = px - rect.left;
    const localY = py - rect.top;
    return {
      gx: Math.max(0, Math.min(GRID_COLS - 1, Math.round(localX / cellW))),
      gy: Math.max(0, Math.min(GRID_ROWS - 1, Math.round(localY / cellH))),
    };
  }, [cellW, cellH]);

  const hitTest = useCallback((px, py) => {
    const canvas = canvasRef.current;
    if (!canvas || cellW === 0) return null;
    const rect = canvas.getBoundingClientRect();
    const localX = px - rect.left;
    const localY = py - rect.top;
    const ctx = canvas.getContext('2d');

    for (let i = elements.length - 1; i >= 0; i--) {
      const el = elements[i];
      if (!el.visible) continue;
      const ex = el.x * cellW;
      const ey = el.y * cellH;
      const scale = FONT_SCALE[el.fontSize] || 1;
      const baseFontSize = Math.max(9, cellH * 1.1);
      const fontSize = baseFontSize * scale;
      ctx.font = `${fontSize}px "Courier New", monospace`;
      const textW = ctx.measureText(el.name).width;
      const textH = fontSize * 1.2;
      if (localX >= ex - 4 && localX <= ex + textW + 4 &&
          localY >= ey - 4 && localY <= ey + textH + 4) {
        return el.id;
      }
    }
    return null;
  }, [elements, cellW, cellH]);

  const handleCanvasMouseDown = useCallback((e) => {
    const id = hitTest(e.clientX, e.clientY);
    if (id) {
      setSelectedElement(id);
      const el = elements.find(el => el.id === id);
      if (el) {
        const { gx, gy } = pixelToGrid(e.clientX, e.clientY);
        setDragOffset({ x: gx - el.x, y: gy - el.y });
        setIsDragging(true);
      }
    } else {
      setSelectedElement(null);
    }
  }, [hitTest, elements, pixelToGrid]);

  const handleCanvasMouseMove = useCallback((e) => {
    if (!isDragging || !selectedElement) return;
    const { gx, gy } = pixelToGrid(e.clientX, e.clientY);
    setElements(prev => prev.map(el => {
      if (el.id !== selectedElement) return el;
      return {
        ...el,
        x: Math.max(0, Math.min(GRID_COLS - 1, gx - dragOffset.x)),
        y: Math.max(0, Math.min(GRID_ROWS - 1, gy - dragOffset.y)),
      };
    }));
  }, [isDragging, selectedElement, pixelToGrid, dragOffset]);

  const handleCanvasMouseUp = useCallback(() => {
    setIsDragging(false);
  }, []);

  useEffect(() => {
    if (isDragging) {
      window.addEventListener('mousemove', handleCanvasMouseMove);
      window.addEventListener('mouseup', handleCanvasMouseUp);
      return () => {
        window.removeEventListener('mousemove', handleCanvasMouseMove);
        window.removeEventListener('mouseup', handleCanvasMouseUp);
      };
    }
  }, [isDragging, handleCanvasMouseMove, handleCanvasMouseUp]);

  const flash = (msg) => {
    setStatusMsg(msg);
    setTimeout(() => setStatusMsg(''), 3000);
  };

  const handleSave = () => {
    if (!connected) return flash('Not connected');
    elements.forEach(el => {
      sendCommand(`osd set ${el.id} ${el.x} ${el.y} ${el.visible ? 1 : 0}`);
    });
    sendCommand('osd save');
    flash('Layout saved to FC');
  };

  const handleLoad = () => {
    if (!connected) return flash('Not connected');
    sendCommand('osd get');
    flash('Loading layout from FC...');
  };

  const handleReset = () => {
    setElements(DEFAULT_ELEMENTS.map(e => ({ ...e })));
    setSelectedElement(null);
    flash('Layout reset to defaults');
  };

  const updateElement = (id, changes) => {
    setElements(prev => prev.map(el => el.id === id ? { ...el, ...changes } : el));
  };

  const selected = elements.find(el => el.id === selectedElement);

  return (
    <div className="h-full flex flex-col gap-3 p-4 overflow-hidden">
      <div className="flex items-center justify-between">
        <h2 className="text-xl font-semibold text-gray-800">OSD Layout Editor</h2>
        <div className="flex items-center gap-2">
          {statusMsg && (
            <span className="text-sm text-blue-600 font-medium animate-pulse">{statusMsg}</span>
          )}
          <button
            onClick={handleLoad}
            disabled={!connected}
            className="px-3 py-1.5 text-sm font-medium rounded-lg bg-gray-100 hover:bg-gray-200 text-gray-700 disabled:opacity-40 transition-colors"
          >
            Load from FC
          </button>
          <button
            onClick={handleSave}
            disabled={!connected}
            className="px-3 py-1.5 text-sm font-medium rounded-lg bg-blue-600 hover:bg-blue-700 text-white disabled:opacity-40 transition-colors"
          >
            Save to FC
          </button>
          <button
            onClick={handleReset}
            className="px-3 py-1.5 text-sm font-medium rounded-lg bg-red-50 hover:bg-red-100 text-red-600 transition-colors"
          >
            Reset Layout
          </button>
        </div>
      </div>

      <div className="flex-1 flex gap-3 min-h-0">
        {/* OSD Canvas */}
        <div className="flex-1 flex flex-col min-w-0">
          <div className="text-xs text-gray-400 mb-1 flex justify-between">
            <span>HD OSD Preview ({GRID_COLS}×{GRID_ROWS})</span>
            <span>{isDragging ? 'Dragging...' : selected ? `Selected: ${selected.name}` : 'Click element to select'}</span>
          </div>
          <div
            ref={containerRef}
            className="flex-1 bg-black rounded-lg overflow-hidden flex items-center justify-center"
          >
            <canvas
              ref={canvasRef}
              style={{ width: canvasSize.width, height: canvasSize.height, cursor: isDragging ? 'grabbing' : 'default' }}
              onMouseDown={handleCanvasMouseDown}
            />
          </div>
        </div>

        {/* Sidebar */}
        <div className="w-72 flex flex-col gap-3 min-h-0">
          {/* Element Palette */}
          <div className="bg-white rounded-xl shadow-sm border border-gray-200 flex-1 flex flex-col min-h-0">
            <div className="px-3 py-2 border-b border-gray-100">
              <h3 className="text-sm font-semibold text-gray-700">OSD Elements</h3>
            </div>
            <div className="flex-1 overflow-y-auto p-1">
              {elements.map(el => (
                <div
                  key={el.id}
                  onClick={() => setSelectedElement(el.id)}
                  className={`flex items-center gap-2 px-2 py-1.5 rounded-lg cursor-pointer transition-colors text-sm ${
                    selectedElement === el.id
                      ? 'bg-blue-50 text-blue-700'
                      : 'hover:bg-gray-50 text-gray-700'
                  }`}
                >
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      updateElement(el.id, { visible: !el.visible });
                    }}
                    className={`w-5 h-5 rounded flex items-center justify-center text-xs border transition-colors ${
                      el.visible
                        ? 'bg-green-500 border-green-500 text-white'
                        : 'bg-gray-100 border-gray-300 text-gray-400'
                    }`}
                    title={el.visible ? 'Hide' : 'Show'}
                  >
                    {el.visible ? '✓' : ''}
                  </button>
                  <span className={`flex-1 truncate ${!el.visible ? 'opacity-40' : ''}`}>
                    {el.name}
                  </span>
                  <span className="text-[10px] text-gray-400 font-mono">
                    {el.x},{el.y}
                  </span>
                </div>
              ))}
            </div>
          </div>

          {/* Element Properties */}
          <div className="bg-white rounded-xl shadow-sm border border-gray-200">
            <div className="px-3 py-2 border-b border-gray-100">
              <h3 className="text-sm font-semibold text-gray-700">Properties</h3>
            </div>
            {selected ? (
              <div className="p-3 space-y-3">
                <div className="text-sm font-medium text-gray-800">{selected.name}</div>

                <div className="flex items-center justify-between">
                  <span className="text-xs text-gray-500">Visible</span>
                  <button
                    onClick={() => updateElement(selected.id, { visible: !selected.visible })}
                    className={`relative w-9 h-5 rounded-full transition-colors ${
                      selected.visible ? 'bg-green-500' : 'bg-gray-300'
                    }`}
                  >
                    <span
                      className={`absolute top-0.5 left-0.5 w-4 h-4 bg-white rounded-full shadow transition-transform ${
                        selected.visible ? 'translate-x-4' : ''
                      }`}
                    />
                  </button>
                </div>

                <div className="grid grid-cols-2 gap-2">
                  <div>
                    <label className="text-xs text-gray-500 block mb-0.5">X</label>
                    <input
                      type="number"
                      min={0}
                      max={GRID_COLS - 1}
                      value={selected.x}
                      onChange={(e) => updateElement(selected.id, { x: clamp(parseInt(e.target.value) || 0, 0, GRID_COLS - 1) })}
                      className="w-full px-2 py-1 text-sm border border-gray-200 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500 outline-none"
                    />
                  </div>
                  <div>
                    <label className="text-xs text-gray-500 block mb-0.5">Y</label>
                    <input
                      type="number"
                      min={0}
                      max={GRID_ROWS - 1}
                      value={selected.y}
                      onChange={(e) => updateElement(selected.id, { y: clamp(parseInt(e.target.value) || 0, 0, GRID_ROWS - 1) })}
                      className="w-full px-2 py-1 text-sm border border-gray-200 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500 outline-none"
                    />
                  </div>
                </div>

                <div>
                  <label className="text-xs text-gray-500 block mb-1">Font Size</label>
                  <div className="flex gap-1">
                    {FONT_SIZES.map(size => (
                      <button
                        key={size}
                        onClick={() => updateElement(selected.id, { fontSize: size })}
                        className={`flex-1 py-1 text-xs font-medium rounded-lg border transition-colors ${
                          selected.fontSize === size
                            ? 'bg-blue-600 text-white border-blue-600'
                            : 'bg-gray-50 text-gray-600 border-gray-200 hover:bg-gray-100'
                        }`}
                      >
                        {size}
                      </button>
                    ))}
                  </div>
                </div>

                <div className="pt-1 border-t border-gray-100 text-[10px] text-gray-400 font-mono">
                  ID: {selected.id} &middot; Grid: {selected.x},{selected.y}
                </div>
              </div>
            ) : (
              <div className="p-4 text-center text-sm text-gray-400">
                Select an element to edit properties
              </div>
            )}
          </div>

          {/* Quick info */}
          <div className="bg-gray-50 rounded-xl border border-gray-200 p-3 text-xs text-gray-500 space-y-1">
            <p><strong className="text-gray-600">Tip:</strong> Drag elements on the canvas to reposition them.</p>
            <p>Toggle visibility with the checkboxes. Use the property panel for precise placement.</p>
            <div className="flex justify-between pt-1 text-[10px] text-gray-400">
              <span>Visible: {elements.filter(e => e.visible).length}/{elements.length}</span>
              <span>Resolution: {GRID_COLS}×{GRID_ROWS}</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

function clamp(val, min, max) {
  return Math.max(min, Math.min(max, val));
}

export default OSDConfig;
