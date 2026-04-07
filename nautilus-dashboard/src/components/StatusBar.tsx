// src/components/StatusBar.tsx
import React, { useState, useEffect } from 'react';
import { Wifi, BatteryCharging, Clock, Zap, Target, Thermometer } from 'lucide-react';
import { useRosTopic } from '../hooks/useRosTopic'; // On réutilise le hook

interface StatusBarProps {
  rovIp: string;
  isConnected: boolean;
}

const StatusBar: React.FC<StatusBarProps> = ({ rovIp, isConnected }) => {
  const [missionTime, setMissionTime] = useState(0);
  
  // --- NOUVEAU : Récupération de la température réelle ---
  const temp = useRosTopic<number>('/rov/temperature', 'std_msgs/msg/Float32', 0);

  useEffect(() => {
    if (!isConnected) return;
    const timer = setInterval(() => setMissionTime(prev => prev + 1), 1000);
    return () => clearInterval(timer);
  }, [isConnected]);

  // Formattage du temps (Inchangé)
  const formatTime = (seconds: number) => {
    const h = Math.floor(seconds / 3600).toString().padStart(2, '0');
    const m = Math.floor((seconds % 3600) / 60).toString().padStart(2, '0');
    const s = (seconds % 60).toString().padStart(2, '0');
    return `${h}:${m}:${s}`;
  };

  return (
    <div className="h-12 bg-[#0a0f14]/80 backdrop-blur-lg border-b border-white/5 flex items-center justify-between px-6 text-white font-mono text-sm z-50 relative">
      
      {/* Gauche : Logo & Mode */}
      <div className="flex items-center gap-4">
        <div className="flex items-center gap-2">
          <div className="w-7 h-7 bg-cyan-950 rounded border border-cyan-500/30 flex items-center justify-center shadow-[0_0_10px_rgba(0,210,255,0.2)]">
              <Zap size={16} className="text-cyan-400" fill="currentColor" />
          </div>
          <span className="font-bold tracking-tighter text-lg">NAUTILUS <span className="text-cyan-400 font-light">CMD</span></span>
        </div>
        <div className="h-5 w-px bg-white/10" />
        <div className="flex items-center gap-2 text-cyan-400 bg-cyan-950/50 px-3 py-1 rounded-full text-xs border border-cyan-500/20">
          <Target size={14} />
          <span>MODE: STABILIZE</span>
        </div>
      </div>

      {/* Centre : Mission Timer */}
      <div className="flex items-center gap-3 bg-black/40 px-5 py-1.5 rounded-full border border-white/5 shadow-inner">
        <Clock size={16} className="text-slate-500" />
        <span className="text-lg font-bold tracking-tight text-white/90 tabular-nums">
          {formatTime(missionTime)}
        </span>
        <span className="text-xs text-slate-500 uppercase tracking-widest mt-0.5">T-Mission</span>
      </div>

      {/* Droite : Stats Réseau & Santé (Temp + Batterie) */}
      <div className="flex items-center gap-6 text-slate-300">
        
        {/* NOUVEAU : Affichage Température dans la barre */}
        <div className="flex items-center gap-2 px-3 py-1 rounded border border-white/5 bg-white/5">
          <Thermometer size={14} className={temp > 40 ? 'text-red-500 animate-pulse' : 'text-cyan-400'} />
          <span className={`font-bold ${temp > 40 ? 'text-red-500' : 'text-white/90'}`}>
            {temp.toFixed(1)}°C
          </span>
        </div>

        <div className="flex items-center gap-2.5">
          <span className="text-slate-500 text-xs">PI_SUB:</span>
          <span className="text-white/80 font-medium">{rovIp}</span>
          <div className="flex items-center gap-1.5">
            <span className="text-green-400 font-bold tabular-nums">12ms</span>
            <Wifi size={18} className="text-green-400" />
          </div>
        </div>

        <div className="h-5 w-px bg-white/10" />
        
        <div className="flex items-center gap-2.5">
          <span className="text-white font-bold tabular-nums text-base">14.8V</span>
          <span className="text-green-500 font-bold text-base">98%</span>
          <BatteryCharging size={22} className="text-green-500" />
        </div>
      </div>
    </div>
  );
};

export default StatusBar;