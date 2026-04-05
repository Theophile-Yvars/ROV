// src/components/Video.tsx
import React from 'react';
import { CameraOff, Loader2 } from 'lucide-react';

interface VideoProps {
  streamUrl?: string;
  isConnected: boolean;
  isLoading?: boolean; // Ajout d'un état de chargement
}

const Video: React.FC<VideoProps> = ({ streamUrl, isConnected, isLoading = false }) => {
  return (
    <div className="relative w-full h-full bg-[#05080a] overflow-hidden flex items-center justify-center">
      {/* Lueur interne (Vignettage) pour l'immersion */}
      <div className="absolute inset-0 shadow-[inset_0_0_100px_rgba(0,0,0,0.8)] z-10 pointer-events-none" />

      {isConnected ? (
        isLoading ? (
          <div className="flex flex-col items-center text-cyan-600 z-20">
            <Loader2 size={48} className="animate-spin mb-4" />
            <p className="font-mono text-sm uppercase tracking-widest">Chargement du flux...</p>
          </div>
        ) : (
          <img 
            src={streamUrl} 
            alt="ROV Feed" 
            className="absolute inset-0 w-full h-full object-cover z-0"
            onError={(e) => (e.currentTarget.src = "")} // Evite l'icône d'image cassée
          />
        )
      ) : (
        <div className="flex flex-col items-center text-red-500/80 z-20 bg-black/50 p-8 rounded-xl backdrop-blur-sm border border-red-500/20">
          <CameraOff size={64} className="mb-6 animate-pulse" />
          <p className="text-2xl font-bold font-mono tracking-tight">SIGNAL VIDÉO PERDU</p>
          <p className="text-sm font-mono text-red-300/60 mt-2">Vérifiez la connexion sous-marine (Sub)</p>
        </div>
      )}

      {/* Label Caméra discret */}
      <div className="absolute bottom-4 left-4 z-20 bg-black/60 px-3 py-1 rounded font-mono text-[10px] text-white/50 border border-white/5 uppercase tracking-widest">
        Cam_01_Front | Main Feed
      </div>
    </div>
  );
};

export default Video;