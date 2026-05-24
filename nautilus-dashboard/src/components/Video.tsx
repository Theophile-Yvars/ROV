import React, { useState, useEffect } from 'react';
import { CameraOff, Loader2, Activity } from 'lucide-react';

interface VideoProps {
  streamUrl?: string; 
  isConnected: boolean;
  isLoading?: boolean;
}

const Video: React.FC<VideoProps> = ({ streamUrl, isConnected, isLoading = false }) => {
  const [hasError, setHasError] = useState(false);

  useEffect(() => {
    if (isConnected) setHasError(false);
  }, [isConnected, streamUrl]);

  return (
    <div className="absolute inset-0 w-full h-full bg-[#020406] overflow-hidden flex items-center justify-center">
      
      {/* Texture scanlines */}
      <div className="absolute inset-0 pointer-events-none z-10 opacity-[0.03] bg-[linear-gradient(rgba(18,16,16,0)_50%,rgba(0,0,0,0.25)_50%),linear-gradient(90deg,rgba(255,0,0,0.06),rgba(0,255,0,0.02),rgba(0,0,118,0.06))] bg-[length:100%_2px,3px_100%]" />
      
      {/* Ombres de bords (Vignetage) */}
      <div className="absolute inset-0 shadow-[inset_0_0_100px_rgba(0,0,0,0.8)] z-10 pointer-events-none" />

      {isConnected && !hasError ? (
        isLoading ? (
          <div className="flex flex-col items-center text-cyan-500 z-20">
            <Loader2 size={40} className="animate-spin mb-4" />
            <p className="font-mono text-[10px] uppercase tracking-[0.4em] animate-pulse">Establishing Uplink...</p>
          </div>
        ) : (
          <img 
            src={streamUrl} 
            alt="ROV Mission Feed" 
            className="w-full h-full object-contain z-0 filter saturate-[1.2] contrast-[1.05]"
            onError={() => setHasError(true)}
          />
        )
      ) : (
        /* Écran d'erreur en cas de coupure */
        <div className="flex flex-col items-center z-20">
          <div className="bg-red-500/5 p-8 rounded-full border border-red-500/20 mb-6 relative">
            <CameraOff size={48} className="text-red-600 animate-pulse" />
            <div className="absolute inset-0 rounded-full bg-red-600/5 animate-ping" />
          </div>
          <h2 className="text-3xl font-black font-mono tracking-tighter text-red-700 mb-2">SIGNAL LOST</h2>
          <div className="flex items-center gap-2 px-4 py-1.5 bg-red-950/40 border border-red-600/30 rounded text-[10px] font-mono text-red-400 uppercase tracking-widest">
            <Activity size={12} className="animate-bounce" /> Check Tether & Pi 5 Status
          </div>
        </div>
      )}

      {/* Étiquette caméra (Haut Gauche) */}
      <div className="absolute top-20 left-8 z-20 flex flex-col gap-1 drop-shadow-md">
        <div className="flex items-center gap-2 bg-black/70 px-3 py-1 rounded-sm border-l-4 border-cyan-500">
          <span className="w-1.5 h-1.5 rounded-full bg-cyan-500 animate-pulse" />
          <span className="font-mono text-[10px] text-cyan-400 font-bold tracking-wider">CAM_IMX708_WIDE</span>
        </div>
      </div>
    </div>
  );
};

export default Video;