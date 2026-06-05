import React, { useState } from 'react';
import StatusBar from './components/StatusBar';
import Video from './components/Video';
import TelemetryOverlay from './components/TelemetryOverlay';
import ThrusterController from './components/ThrusterController'; 

function App() {
  const [isConnected] = useState(true);
  const [rovIp] = useState("192.168.1.83");
  const [isLoading] = useState(false);

  const streamUrl = `http://${rovIp}:8080/stream?topic=/image_raw&type=mjpeg`;

  return (
    <div className="flex flex-col h-screen bg-[#05080a] text-slate-200 overflow-hidden font-sans select-none relative">
      
      {/* 1. BARRE DE STATUT (Z-INDEX 50) */}
      <StatusBar rovIp={rovIp} isConnected={isConnected} />

      {/* 2. COCKPIT CENTRAL */}
      <main className="flex-1 relative bg-black flex items-center justify-center overflow-hidden">
        
        {/* Caméra en fond */}
        <Video 
          streamUrl={streamUrl} 
          isConnected={isConnected} 
          isLoading={isLoading} 
        />

        {/* HUD de Télémétrie par-dessus (Z-INDEX 20) */}
        <TelemetryOverlay 
          headingDefault={45.0} 
          pitchDefault={2.0} 
          rollDefault={-1.0} 
          depthDefault={1.20} 
        />

        {/* CONTRÔLE DES MOTEURS (Z-INDEX 30) */}
        <ThrusterController rovIp={rovIp} />
        
      </main>

      {/* 3. TERMINAL DE FOND DE PAGE (FOOTER) */}
      <footer className="h-7 bg-[#0a0f14] border-t border-white/5 px-6 flex items-center justify-between z-50 relative shadow-[0_-4px_20px_rgba(0,0,0,0.5)]">
        <div className="flex items-center gap-3">
          <div className={`w-1.5 h-1.5 rounded-full ${isConnected ? 'bg-green-500 animate-pulse' : 'bg-red-600'}`} />
          <p className="text-[10px] font-mono text-slate-500 uppercase tracking-widest">
            {isConnected 
              ? "Système Nautilus optimal. Liaison montante active." 
              : "Alerte : Liaison de surface interrompue."}
          </p>
        </div>
        
        <div className="flex items-center gap-6">
          <p className="text-[10px] font-mono text-slate-700 uppercase">
             Enc: MJPEG | ROS2 Bridge: OK
          </p>
          <p className="text-[10px] font-mono text-slate-700 uppercase">
             Nautilus OS v1.2 | Domain: 42
          </p>
        </div>
      </footer>
    </div>
  );
}

export default App;