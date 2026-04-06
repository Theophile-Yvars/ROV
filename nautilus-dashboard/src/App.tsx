// src/App.tsx
import React, { useState } from 'react';
import StatusBar from './components/StatusBar';
import Video from './components/Video';
import TelemetryOverlay from './components/TelemetryOverlay';

function App() {
  // --- CONFIGURATION DU ROV ---
  const [isConnected, setIsConnected] = useState(true);
  const [rovIp, setRovIp] = useState("192.168.1.83");
  const [isLoading, setIsLoading] = useState(false);

  // Construction dynamique de l'URL du flux
  // Note: On utilise l'IP d'état pour que le changement soit réactif
  const streamUrl = `http://${rovIp}:8080/stream?topic=/image_raw&type=mjpeg`;

  return (
    // Container principal : Fullscreen, sans scroll, look "Deep Sea"
    <div className="flex flex-col h-screen bg-[#05080a] text-slate-200 overflow-hidden font-sans select-none relative">
      
      {/* 1. Barre de Statut supérieure (Position fixe) */}
      <StatusBar rovIp={rovIp} isConnected={isConnected} />

      {/* 2. Zone centrale : Le Cockpit (Prend tout l'espace restant) */}
      <main className="flex-1 relative bg-black flex items-center justify-center overflow-hidden">
        
        {/* Le flux vidéo (Le fond de l'interface) */}
        <Video 
          streamUrl={streamUrl} 
          isConnected={isConnected} 
          isLoading={isLoading} 
        />

        {/* La couche de Télémétrie (HUD flottant par-dessus la vidéo) */}
        {/* On lui passe l'état de connexion pour adapter les jauges si besoin */}
        <TelemetryOverlay isConnected={isConnected} />
        
      </main>

      {/* 3. Barre de notifications inférieure (Logs système) */}
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
             Enc: H.264 | Latency: --ms
          </p>
          <p className="text-[10px] font-mono text-slate-700 uppercase">
            Dev Mode v1.0 | Surface Comms: OK
          </p>
        </div>
      </footer>
    </div>
  );
}

export default App;