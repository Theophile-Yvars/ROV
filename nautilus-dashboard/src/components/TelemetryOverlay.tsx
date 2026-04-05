import React from 'react';
import { Compass, ArrowDownToLine, Navigation } from 'lucide-react';

interface TelemetryOverlayProps {
  heading?: number; // 0 à 360°
  pitch?: number;   // Inclinaison avant/arrière
  roll?: number;    // Inclinaison gauche/droite
  depth?: number;
}

const TelemetryOverlay: React.FC<TelemetryOverlayProps> = ({ 
  heading = 45, 
  pitch = 2, 
  roll = -1, 
  depth = 1.2 
}) => {
  
  // Génération des graduations de la boussole
  const renderCompassScale = () => {
    const marks = [];
    for (let i = -100; i <= 460; i += 10) {
      const isMajor = i % 30 === 0;
      marks.push(
        <div key={i} className="flex flex-col items-center flex-shrink-0" style={{ width: '40px' }}>
          <div className={`w-px ${isMajor ? 'h-4 bg-cyan-400' : 'h-2 bg-cyan-800'}`} />
          {isMajor && <span className="text-[10px] mt-1 text-cyan-400/80">{(i + 360) % 360}</span>}
        </div>
      );
    }
    return marks;
  };

  return (
    <div className="absolute inset-0 z-20 pointer-events-none p-8 flex flex-col justify-between overflow-hidden">
      
      {/* 1. RUBAN DE BOUSSOLE (TOP) */}
      <div className="absolute top-4 left-1/2 -translate-x-1/2 w-80">
        <div className="relative h-12 flex flex-col items-center overflow-hidden border-b border-cyan-500/30">
          {/* Curseur central */}
          <div className="absolute top-0 z-10 text-cyan-400 flex flex-col items-center">
            <Navigation size={14} fill="currentColor" className="rotate-180" />
          </div>
          {/* Défilement du ruban */}
          <div 
            className="flex items-start transition-transform duration-200 ease-out mt-4"
            style={{ transform: `translateX(${-heading * 4 + 160}px)` }}
          >
            {renderCompassScale()}
          </div>
        </div>
        <div className="text-center font-mono text-xl font-bold text-white mt-1">
          {heading.toFixed(0)}°
        </div>
      </div>

      {/* 2. HORIZON ARTIFICIEL (CENTRE) */}
      <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 flex items-center gap-12">
        {/* Aile gauche du HUD */}
        <div className="w-24 h-[2px] bg-cyan-500/50 shadow-[0_0_10px_rgba(0,210,255,0.5)]" />
        
        {/* Cercle central */}
        <div 
          className="w-48 h-48 border-2 border-dashed border-cyan-500/20 rounded-full flex items-center justify-center transition-transform duration-150"
          style={{ transform: `rotate(${roll}deg) translateY(${pitch * 2}px)` }}
        >
           <div className="w-4 h-4 border border-cyan-400 rounded-full" />
           <div className="absolute w-32 h-px bg-cyan-400/30" />
        </div>

        {/* Aile droite du HUD */}
        <div className="w-24 h-[2px] bg-cyan-500/50 shadow-[0_0_10px_rgba(0,210,255,0.5)]" />
      </div>

      {/* 3. BLOCS DE DONNÉES (BOTTOM) */}
      <div className="flex justify-between items-end w-full">
        {/* Profondeur */}
        <div className="bg-black/60 backdrop-blur-md p-4 rounded-lg border border-cyan-500/20 flex flex-col items-start min-w-[120px]">
          <div className="flex items-center gap-2 text-cyan-400 text-[10px] uppercase tracking-widest mb-1">
            <ArrowDownToLine size={14} />
            <span>Profondeur</span>
          </div>
          <div className="flex items-baseline gap-1">
            <span className="text-4xl font-bold font-mono text-white tracking-tighter">{depth.toFixed(1)}</span>
            <span className="text-lg text-cyan-700 font-light font-mono">m</span>
          </div>
        </div>

        {/* Attitude (Pitch/Roll) */}
        <div className="flex gap-4">
            <div className="bg-black/60 backdrop-blur-md p-3 rounded-lg border border-white/5 font-mono text-[10px] text-slate-400">
                PITCH: <span className="text-white">{pitch > 0 ? '+' : ''}{pitch}°</span>
            </div>
            <div className="bg-black/60 backdrop-blur-md p-3 rounded-lg border border-white/5 font-mono text-[10px] text-slate-400">
                ROLL: <span className="text-white">{roll > 0 ? '+' : ''}{roll}°</span>
            </div>
        </div>
      </div>
    </div>
  );
};

export default TelemetryOverlay;