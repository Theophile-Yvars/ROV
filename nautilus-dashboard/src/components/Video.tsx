import React, { useState, useEffect } from 'react';
import { CameraOff, Loader2, Activity } from 'lucide-react';

interface VideoProps {
  /** * L'URL doit pointer vers le web_video_server de ROS2
   * Format type: http://192.168.1.83:8080/stream?topic=/image_raw&type=mjpeg
   */
  streamUrl?: string; 
  isConnected: boolean;
  isLoading?: boolean;
}

const Video: React.FC<VideoProps> = ({ streamUrl, isConnected, isLoading = false }) => {
  const [hasError, setHasError] = useState(false);

  // On réinitialise l'erreur si on détecte une nouvelle tentative de connexion
  useEffect(() => {
    if (isConnected) setHasError(false);
  }, [isConnected, streamUrl]);

  return (
    <div className="relative w-full h-full bg-[#020406] overflow-hidden flex items-center justify-center border border-white/5 shadow-2xl">
      
      {/* 1. Overlay de scanlines (Effet cockpit analogique) */}
      <div className="absolute inset-0 pointer-events-none z-10 opacity-[0.03] bg-[linear-gradient(rgba(18,16,16,0)_50%,rgba(0,0,0,0.25)_50%),linear-gradient(90deg,rgba(255,0,0,0.06),rgba(0,255,0,0.02),rgba(0,0,118,0.06))] bg-[length:100%_2px,3px_100%]" />
      
      {/* 2. Vignettage immersif (Ombres sur les bords pour focus central) */}
      <div className="absolute inset-0 shadow-[inset_0_0_150px_rgba(0,0,0,0.9)] z-10 pointer-events-none" />

      {/* 3. Affichage du Flux ou des États d'erreur */}
      {isConnected && !hasError ? (
        isLoading ? (
          <div className="flex flex-col items-center text-cyan-500 z-20">
            <Loader2 size={40} className="animate-spin mb-4 stroke-[1.5px]" />
            <p className="font-mono text-[10px] uppercase tracking-[0.4em] animate-pulse">Establishing Uplink...</p>
          </div>
        ) : (
          <img 
            src={streamUrl} 
            alt="ROV Mission Feed" 
            // object-contain évite de cropper l'image (important pour la navigation)
            className="absolute inset-0 w-full h-full object-contain z-0 transition-opacity duration-1000 filter saturate-[1.3] contrast-[1.1]"
            onLoad={() => console.log("ROV Stream: HD Feed Active")}
            onError={() => setHasError(true)}
          />
        )
      ) : (
        /* État Hors-Ligne / Perte de Signal */
        <div className="flex flex-col items-center z-20 animate-in fade-in duration-500">
          <div className="bg-red-500/5 p-12 rounded-full border border-red-500/20 mb-8 backdrop-blur-xl relative">
            <CameraOff size={64} className="text-red-600 animate-pulse" />
            <div className="absolute inset-0 rounded-full bg-red-600/10 animate-ping" />
          </div>
          
          <h2 className="text-4xl font-black font-mono tracking-tighter text-red-700 mb-2">
            SIGNAL LOST
          </h2>
          
          <div className="flex flex-col items-center gap-3">
            <div className="flex items-center gap-2 px-4 py-1.5 bg-red-950/40 border border-red-600/30 rounded text-[11px] font-mono text-red-400 uppercase tracking-widest">
              <Activity size={12} className="animate-bounce" />
              Check Tether & Pi 5 Status
            </div>
            <p className="text-[9px] font-mono text-white/20 uppercase tracking-tighter">
              ERR_V4L2_LOOPBACK_NOT_FOUND
            </p>
          </div>
        </div>
      )}

      {/* 4. HUD - Informations Techniques (Haut Gauche) */}
      <div className="absolute top-8 left-8 z-20 flex flex-col gap-1.5 drop-shadow-md">
        <div className="flex items-center gap-2 bg-black/70 px-3 py-1 rounded-sm border-l-4 border-cyan-500">
          <span className="w-1.5 h-1.5 rounded-full bg-cyan-500 animate-pulse" />
          <span className="font-mono text-[11px] text-cyan-400 font-bold uppercase tracking-wider">
            CAM_IMX708_WIDE
          </span>
        </div>
        <div className="font-mono text-[10px] text-white/40 ml-1 flex gap-3">
          <span>720P // 24FPS</span>
          <span className="text-white/20">|</span>
          <span>MJPEG_STREAM</span>
        </div>
      </div>

      {/* 5. HUD - Status Enregistrement (Bas Droite) */}
      <div className="absolute bottom-8 right-8 z-20 flex items-center gap-4">
        <div className="bg-black/60 px-4 py-1.5 rounded-full font-mono text-[11px] text-white/60 border border-white/10 uppercase tracking-[0.2em] backdrop-blur-md flex items-center gap-2">
          <span className="text-red-600 animate-[pulse_1s_infinite] text-lg">●</span>
          REC 00:00:00
        </div>
        
        {/* Indicateur de qualité du signal (barres) */}
        <div className="flex gap-1 items-end h-3">
            {[0,1,2,3].map((i) => (
                <div key={i} className={`w-1 rounded-full ${isConnected ? 'bg-cyan-500' : 'bg-white/10'}`} style={{ height: `${(i+1)*25}%` }} />
            ))}
        </div>
      </div>

      {/* 6. Overlay Texture (Bruit ISO pour le look "pro") */}
      <div className="absolute inset-0 pointer-events-none z-30 opacity-[0.015] bg-[url('https://grainy-gradients.vercel.app/noise.svg')]" />
    </div>
  );
};

export default Video;