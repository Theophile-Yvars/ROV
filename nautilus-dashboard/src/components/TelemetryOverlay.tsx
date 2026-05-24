import React from 'react';
import { ArrowDownToLine, Navigation } from 'lucide-react';
import { useRosTopic } from '../hooks/useRosTopic';
import TemperatureGauge from './TemperatureGauge';

interface TelemetryOverlayProps {
  headingDefault?: number;
  pitchDefault?: number;
  rollDefault?: number;
  depthDefault?: number;
}

const TelemetryOverlay: React.FC<TelemetryOverlayProps> = ({ 
  headingDefault = 0.0, 
  pitchDefault = 0.0, 
  rollDefault = 0.0, 
  depthDefault = 0.0 
}) => {
  
  // --- ABONNEMENTS PHYSIQUES ROS 2 ---
  // 1. On s'abonne à l'IMU (Type Vector3 qui contient x, y, z)
  const orientationRaw = useRosTopic<any>('/rov/orientation', 'geometry_msgs/msg/Vector3', null);
  
  // 2. On conserve notre abonnement profondeur en Float64 validé juste avant
  const depthRaw = useRosTopic<any>('/rov/water_depth', 'std_msgs/msg/Float64', null);

  // --- EXTRACTEUR POUR LA PROFONDEUR ---
  const parseDepthValue = (raw: any): number | null => {
    if (raw === null || raw === undefined) return null;
    if (typeof raw === 'object' && 'data' in raw) return Number(raw.data);
    return typeof raw === 'number' ? raw : Number(raw);
  };

  // --- TRAITEMENT DES DONNÉES EN TEMPS RÉEL ---
  const currentDepth = parseDepthValue(depthRaw) ?? depthDefault;

  // Extraction des axes IMU (x = roll, y = pitch, z = heading)
  let currentHeading = headingDefault;
  let currentPitch = pitchDefault;
  let currentRoll = rollDefault;

  if (orientationRaw && typeof orientationRaw === 'object') {
    if ('z' in orientationRaw) currentHeading = Number(orientationRaw.z);
    if ('y' in orientationRaw) currentPitch = Number(orientationRaw.y);
    if ('x' in orientationRaw) currentRoll = Number(orientationRaw.x);
  }

  // Génération dynamique du ruban de boussole (10° = 40px, donc 1° = 4px)
  const renderCompassScale = () => {
    const marks = [];
    for (let i = -90; i <= 450; i += 10) {
      const isMajor = i % 30 === 0;
      const displayValue = (i + 360) % 360;
      let label = displayValue.toString();
      if (displayValue === 0) label = "N";
      if (displayValue === 90) label = "E";
      if (displayValue === 180) label = "S";
      if (displayValue === 270) label = "W";

      marks.push(
        <div key={i} className="flex flex-col items-center flex-shrink-0" style={{ width: '40px' }}>
          <div className={`w-px ${isMajor ? 'h-3 bg-cyan-400' : 'h-1.5 bg-cyan-400/30'}`} />
          {isMajor && <span className="text-[9px] font-mono mt-1 text-cyan-400/70 font-bold">{label}</span>}
        </div>
      );
    }
    return marks;
  };

  return (
    <div className="absolute inset-0 z-20 pointer-events-none p-8 flex flex-col justify-between overflow-hidden">
      
      {/* 1. RUBAN DE BOUSSOLE DYNAMIQUE */}
      <div className="absolute top-4 left-1/2 -translate-x-1/2 w-80">
        <div className="relative h-12 flex flex-col items-center overflow-hidden border-b border-cyan-500/30">
          <div className="absolute top-0 z-10 text-cyan-400 flex flex-col items-center">
            <Navigation size={14} fill="currentColor" className="rotate-180" />
          </div>
          <div 
            className="flex items-start transition-transform duration-150 ease-out mt-4"
            style={{ transform: `translateX(${-currentHeading * 4 + 160 + 360}px)` }}
          >
            {renderCompassScale()}
          </div>
        </div>
        <div className="text-center font-mono text-xl font-bold text-white mt-1 drop-shadow-md">
          {currentHeading.toFixed(1)}°
        </div>
      </div>

      {/* 2. HORIZON ARTIFICIEL ANIMÉ (PITCH ET ROLL) */}
      <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 flex items-center gap-12">
        <div className="w-24 h-[2px] bg-cyan-500/50 shadow-[0_0_10px_rgba(0,210,255,0.5)]" />
        <div 
          className="w-48 h-48 border-2 border-dashed border-cyan-500/20 rounded-full flex items-center justify-center transition-transform duration-150 ease-out"
          style={{ transform: `rotate(${-currentRoll}deg) translateY(${currentPitch * 4}px)` }}
        >
           <div className="w-4 h-4 border border-cyan-400 rounded-full" />
           <div className="absolute w-32 h-px bg-cyan-400/30" />
        </div>
        <div className="w-24 h-[2px] bg-cyan-500/50 shadow-[0_0_10px_rgba(0,210,255,0.5)]" />
      </div>

      {/* 3. FOOTER HUD DES DONNÉES COCKPIT */}
      <div className="flex justify-between items-end w-full mt-auto pointer-events-auto">
        
        {/* BAS GAUCHE : PROFONDEUR FLUIDE */}
        <div className="bg-black/75 backdrop-blur-md p-4 rounded-lg border border-cyan-500/20 flex flex-col items-start min-w-[140px] shadow-2xl">
          <div className="flex items-center gap-2 text-cyan-400 text-[10px] uppercase tracking-widest mb-1">
            <ArrowDownToLine size={14} />
            <span>Profondeur</span>
          </div>
          <div className="flex items-baseline gap-1">
            <span className="text-4xl font-bold font-mono text-white tracking-tighter">
              {currentDepth.toFixed(2)}
            </span>
            <span className="text-lg text-cyan-700 font-light font-mono">m</span>
          </div>
        </div>

        {/* BAS CENTRE : MODULE THERMIQUE TRIPARTITE */}
        <TemperatureGauge />

        {/* BAS DROITE : ASSIETTE NUMÉRIQUE PITCH / ROLL */}
        <div className="flex flex-col gap-2">
          <div className="bg-black/75 backdrop-blur-md p-3 rounded-lg border border-white/5 font-mono text-[10px] text-slate-400 w-28 text-right shadow-2xl">
            PITCH: <span className="text-white font-bold">{currentPitch > 0 ? '+' : ''}{currentPitch.toFixed(1)}°</span>
          </div>
          <div className="bg-black/75 backdrop-blur-md p-3 rounded-lg border border-white/5 font-mono text-[10px] text-slate-400 w-28 text-right shadow-2xl">
            ROLL: <span className="text-white font-bold">{currentRoll > 0 ? '+' : ''}{currentRoll.toFixed(1)}°</span>
          </div>
        </div>

      </div>
    </div>
  );
};

export default TelemetryOverlay;