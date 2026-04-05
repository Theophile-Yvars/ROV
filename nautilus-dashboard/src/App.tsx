// src/App.tsx
import React, { useState } from 'react';
import StatusBar from './components/StatusBar';
import Video from './components/Video';
import TelemetryOverlay from './components/TelemetryOverlay';

function App() {
  // Etats simulés pour le développement
  const [isConnected, setIsConnected] = useState(true);
  const [rovIp, setRovIp] = useState("192.168.1.83");
  
  // URL d'une image de fond marin pour le test visuel
  const testStreamUrl = "https://images.unsplash.com/photo-1583212292454-1fe6229603b7?auto=format&fit=crop&q=80&w=1920";

  return (
    // Container principal en Flexbox colonne
    <div className="flex flex-col h-screen bg-[#05080a] text-slate-200 overflow-hidden font-sans select-none relative">
      
      {/* 1. Barre de Statut supérieure (Fixe en haut) */}
      <StatusBar rovIp={rovIp} />

      {/* 2. Zone centrale : Le Cockpit (Prend tout l'espace restant) */}
      <main className="flex-1 relative bg-black flex items-center justify-center">
        
        {/* Le flux vidéo (Fond de l'écran) */}
        <Video 
          isConnected={isConnected} 
          streamUrl={testStreamUrl} // Remplace par ton vrai flux plus tard
          isLoading={false}
        />

        {/* La couche de Télémétrie (HUD flottant par-dessus) */}
        <TelemetryOverlay />
        
      </main>

      {/* 3. Barre de notifications inférieure (Logs rapides) */}
      <footer className="h-7 bg-[#0a0f14] border-t border-white/5 px-6 flex items-center justify-between z-50 relative">
        <div className="flex items-center gap-3">
          <div className="w-1.5 h-1.5 rounded-full bg-green-500 animate-pulse" />
          <p className="text-[10px] font-mono text-slate-500 uppercase tracking-widest">
            Système Nautilus optimal. En attente de données capteurs...
          </p>
        </div>
        <p className="text-[10px] font-mono text-slate-700 uppercase">
          Dev Mode v1.0 | Surface Comms: OK
        </p>
      </footer>
    </div>
  );
}

export default App;