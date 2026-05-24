import React from 'react';
import { Thermometer, Cpu, Waves } from 'lucide-react';
import { useRosTopic } from '../hooks/useRosTopic';

const TemperatureGauge: React.FC = () => {
  // 1. Abonnements ROS 2 avec les types exacts découverts sur ta Pi
  const tempEauRaw = useRosTopic<any>('/rov/water_temperature', 'sensor_msgs/msg/Temperature', null);
  const tempInterneRaw = useRosTopic<any>('/rov/temperature', 'std_msgs/msg/Float32', null);
  const tempCpuRaw = useRosTopic<any>('/rov/cpu_temperature', 'std_msgs/msg/Float32', null);

  // 2. Fonction d'extraction magique pour tolérer tous les formats d'objets ou de primitifs
  const extractValue = (raw: any): number | null => {
    if (raw === null || raw === undefined) return null;
    if (typeof raw === 'object') {
      if ('temperature' in raw) return raw.temperature as number; // Format sensor_msgs
      if ('data' in raw) return raw.data as number;               // Format std_msgs
      return null;
    }
    return Number(raw);
  };

  const tempEau = extractValue(tempEauRaw);
  const tempInterne = extractValue(tempInterneRaw);
  const tempCpu = extractValue(tempCpuRaw);

  return (
    <div className="bg-black/75 backdrop-blur-md p-4 rounded-lg border border-white/5 flex gap-6 font-mono shadow-2xl pointer-events-auto select-none">
      {/* EAU EXTÉRIEURE */}
      <div className="flex flex-col">
        <span className="text-[9px] text-slate-500 flex items-center gap-1 uppercase tracking-wider mb-1">
          <Waves size={10} className="text-blue-400" /> Eau Ext.
        </span>
        <span className="text-base font-bold text-blue-400">
          {tempEau !== null ? `${tempEau.toFixed(1)}°C` : '---'}
        </span>
      </div>

      <div className="w-px bg-white/10 h-8 self-center" />

      {/* CAISSON INTERNE */}
      <div className="flex flex-col">
        <span className="text-[9px] text-slate-500 flex items-center gap-1 uppercase tracking-wider mb-1">
          <Thermometer size={10} className="text-cyan-400" /> Caisson Int.
        </span>
        <span className={`text-base font-bold ${tempInterne && tempInterne > 40 ? 'text-red-500 animate-pulse' : 'text-cyan-400'}`}>
          {tempInterne !== null ? `${tempInterne.toFixed(1)}°C` : '---'}
        </span>
      </div>

      <div className="w-px bg-white/10 h-8 self-center" />

      {/* CPU RASPBERRY PI 5 */}
      <div className="flex flex-col">
        <span className="text-[9px] text-slate-500 flex items-center gap-1 uppercase tracking-wider mb-1">
          <Cpu size={10} className="text-emerald-400" /> CPU Pi 5
        </span>
        <span className={`text-base font-bold ${tempCpu && tempCpu > 70 ? 'text-red-500 animate-bounce' : tempCpu && tempCpu > 55 ? 'text-amber-400' : 'text-emerald-400'}`}>
          {tempCpu !== null ? `${tempCpu.toFixed(1)}°C` : '---'}
        </span>
      </div>
    </div>
  );
};

export default TemperatureGauge;