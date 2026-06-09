import React, { useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib';

const TILT_SPEED = 1.0; // Vitesse envoyée pendant l'appui (-1.0 à +1.0)

const CameraTiltControl = () => {
  const publisherRef = useRef(null);
  const keysPressed  = useRef({ up: false, down: false });
  const intervalRef  = useRef(null);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'ws://192.168.1.83:9090' });
    const topic = new ROSLIB.Topic({
      ros,
      name: '/rov/camera_tilt',
      messageType: 'std_msgs/msg/Float32',
    });
    topic.advertise();
    publisherRef.current = topic;

    return () => ros.close();
  }, []);

  const publish = (value) => {
    publisherRef.current?.publish({ data: value });
  };

  // Publie la vitesse courante à 20Hz tant qu'une touche est enfoncée
  const startLoop = () => {
    if (intervalRef.current) return; // déjà actif
    intervalRef.current = setInterval(() => {
      const { up, down } = keysPressed.current;
      if (up && !down)       publish(TILT_SPEED);
      else if (down && !up)  publish(-TILT_SPEED);
      else                   publish(0.0);
    }, 50); // 20Hz
  };

  const stopLoop = () => {
    if (!keysPressed.current.up && !keysPressed.current.down) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
      publish(0.0); // Arrêt propre
    }
  };

  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.repeat) return; // Ignore l'auto-repeat du navigateur
      if (e.key.toLowerCase() === 'a') { keysPressed.current.up   = true; startLoop(); }
      if (e.key.toLowerCase() === 'q') { keysPressed.current.down = true; startLoop(); }
    };

    const handleKeyUp = (e) => {
      if (e.key.toLowerCase() === 'a') keysPressed.current.up   = false;
      if (e.key.toLowerCase() === 'q') keysPressed.current.down = false;
      stopLoop();
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup',   handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup',   handleKeyUp);
      clearInterval(intervalRef.current);
    };
  }, []);

  return (
    <div className="absolute right-6 bottom-20 z-40 bg-black/70 p-4 rounded border border-white/10 text-center">
      <h3 className="text-[10px] text-slate-400 uppercase">Tilt Caméra</h3>
      <div className="text-[9px] text-white/50 mt-1">A: Haut | Q: Bas</div>
    </div>
  );
};

export default CameraTiltControl;