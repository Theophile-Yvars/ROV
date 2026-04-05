import React from 'react';
import { CameraOff, Loader2 } from 'lucide-react';

interface VideoProps {
  streamUrl?: string;
  isConnected: boolean;
  isLoading?: boolean;
}

const Video: React.FC<VideoProps> = ({ streamUrl, isConnected, isLoading = false }) => {
  return (
    <div className="relative w-full h-full bg-[#020406] overflow-hidden flex items-center justify-center">
      
      {/* 1. Overlay de scanlines (Effet cockpit pro) */}
      <div className="absolute inset-0 pointer-events-none z-10 opacity-[0.03] bg-[linear-gradient(rgba(18,16,16,0)_50%,rgba(0,0,0,0.25)_50%),linear-gradient(90deg,rgba(255,0,0,0.06),rgba(0,255,0,0.02),rgba(0,0,118,0.06))] bg-[length:100%_2px,3px_100%]" />
      
      {/* 2. Vignettage immersif */}
      <div className="absolute inset-0 shadow-[inset_0_0_120px_rgba(0,0,0,0.9)] z-10 pointer-events-none" />

      {isConnected ? (
        isLoading ? (
          <div className="flex flex-col items-center text-cyan-600 z-20">
            <Loader2 size={48} className="animate-spin mb-4 stroke-[1.5px]" />
            <p className="font-mono text-xs uppercase tracking-[0.3em] animate-pulse">Establishing Uplink...</p>
          </div>
        ) : (
          <img 
            src={streamUrl} 
            alt="ROV HD Feed" 
            // On ajoute "contrast-110" et "brightness-110" pour compenser l'obscurité sous-marine
            className="absolute inset-0 w-full h-full object-cover z-0 transition-opacity duration-700 filter contrast-[1.1] saturate-[1.2]"
            onLoad={() => console.log("Stream HD Connecté")}
            onError={(e) => (e.currentTarget.src = "")}
          />
        )
      ) : (
        <div className="flex flex-col items-center z-20">
          <div className="bg-red-500/10 p-10 rounded-full border border-red-500/20 mb-6 backdrop-blur-md">
            <CameraOff size={60} className="text-red-500 animate-pulse" />
          </div>
          <p className="text-3xl font-black font-mono tracking-tighter text-red-600">NO VIDEO SIGNAL</p>
          <div className="flex items-center gap-2 mt-4 px-4 py-1 bg-red-950/30 border border-red-500/30 rounded text-[10px] font-mono text-red-400 uppercase tracking-widest">
            <span className="w-2 h-2 rounded-full bg-red-600 animate-ping" />
            Check Tether & Power
          </div>
        </div>
      )}

      {/* Interface de données technique sur l'image */}
      <div className="absolute top-6 left-6 z-20 flex flex-col gap-1">
        <div className="bg-black/60 px-2 py-0.5 rounded border-l-2 border-cyan-500 font-mono text-[10px] text-cyan-400 uppercase">
          HD_STREAM_CAM_V3
        </div>
        <div className="font-mono text-[9px] text-white/30 ml-1">
          720p @ 30FPS | H.264 ENC
        </div>
      </div>

      <div className="absolute bottom-6 right-6 z-20 bg-black/60 px-3 py-1 rounded font-mono text-[10px] text-white/50 border border-white/5 uppercase tracking-widest backdrop-blur-sm">
        REC <span className="text-red-600 animate-pulse">●</span> 00:00:00
      </div>
    </div>
  );
};

export default Video;