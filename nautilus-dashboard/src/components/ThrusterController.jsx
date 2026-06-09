import React, { useEffect, useState, useRef } from 'react';
import * as ROSLIB from 'roslib'; // Import direct

const ThrusterController = ({ rovIp }) => {
  const [connected, setConnected] = useState(false);
  const [debugKey, setDebugKey] = useState("Neutre");
  
  const rosRef = useRef(null);
  const cmdVelTopicRef = useRef(null);
  const activeKeysRef = useRef({});

  // Fonction de publication corrigée
  const publishMessage = (linearX, linearZ, angularZ) => {
    if (!cmdVelTopicRef.current) return;

    // On crée un objet JSON simple au lieu d'un constructeur
    const twistStampedMessage = {
      header: {
        stamp: { 
          sec: Math.floor(Date.now() / 1000), 
          nanosec: (Date.now() % 1000) * 1e6 
        },
        frame_id: "base_link"
      },
      twist: {
        linear: { x: linearX, y: 0.0, z: linearZ },
        angular: { x: 0.0, y: 0.0, z: angularZ }
      }
    };

    // NE PAS faire : new ROSLIB.Message(...)
    // ROSLIB.Topic.publish accepte directement l'objet JSON
    console.log("📤 Publication directe :", twistStampedMessage);
    cmdVelTopicRef.current.publish(twistStampedMessage);
  };

  // 1. Initialisation de la connexion ROS 2
  useEffect(() => {
    if (!rovIp) return;

    const ros = new ROSLIB.Ros({ url: `ws://${rovIp}:9090` });
    rosRef.current = ros;

    ros.on('connection', () => {
      console.log('✅ Connecté au Rosbridge');
      setConnected(true);
      
      // Initialisation du Topic
      cmdVelTopicRef.current = new ROSLIB.Topic({
        ros: ros,
        name: '/rov/cmd_vel_input',
        messageType: 'geometry_msgs/msg/TwistStamped'
      });
      // Annonce nécessaire pour ROS
      cmdVelTopicRef.current.advertise();
    });

    ros.on('error', (err) => { console.error('❌ Erreur ROS:', err); setConnected(false); });
    ros.on('close', () => { setConnected(false); cmdVelTopicRef.current = null; });

    return () => ros.close();
  }, [rovIp]);

  // 2. Gestion des touches
  useEffect(() => {
    const handleKeyDown = (e) => {
      activeKeysRef.current[e.code] = true;
      activeKeysRef.current[e.key] = true;
    };

    const handleKeyUp = (e) => {
      activeKeysRef.current[e.code] = false;
      activeKeysRef.current[e.key] = false;
      
      const keys = activeKeysRef.current;
      if (!keys['ArrowUp'] && !keys['ArrowDown'] && !keys['ArrowLeft'] && !keys['ArrowRight'] && 
          !keys['KeyW'] && !keys['KeyS'] && !keys['w'] && !keys['s']) {
        publishMessage(0.0, 0.0, 0.0);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  // 3. Boucle de rafraîchissement (10Hz)
  useEffect(() => {
    const interval = setInterval(() => {
      if (!connected || !cmdVelTopicRef.current) return;

      const keys = activeKeysRef.current;
      let linearX = 0.0, linearZ = 0.0, angularZ = 0.0;

      if (keys['ArrowUp'] || keys['w'] || keys['W']) linearX = 1.0;
      if (keys['ArrowDown'] || keys['s'] || keys['S']) linearX = -1.0;
      if (keys['ArrowLeft']) angularZ = -1.0;
      if (keys['ArrowRight']) angularZ = 1.0;
      if (keys['KeyW'] || keys['Up']) linearZ = 1.0;
      if (keys['KeyS'] || keys['Down']) linearZ = -1.0;

      if (linearX !== 0 || angularZ !== 0 || linearZ !== 0) {
        setDebugKey(`X:${linearX} Z:${linearZ} Y:${angularZ}`);
        publishMessage(linearX, linearZ, angularZ);
      } else {
        setDebugKey("Neutre");
      }
    }, 100); 

    return () => clearInterval(interval);
  }, [connected]);

  return (
    <div className="absolute top-16 right-6 bg-black/60 border border-emerald-500/20 px-3 py-1.5 rounded text-[10px] font-mono tracking-widest uppercase z-30">
      <div className="flex items-center gap-2">
        <div className={`w-1.5 h-1.5 rounded-full ${connected ? 'bg-emerald-400' : 'bg-red-500'}`} />
        <span>Input : {connected ? "Armé" : "Hors-ligne"}</span>
      </div>
      <div className="text-slate-400 pt-1 border-t border-white/5">
        Moteurs: {debugKey}
      </div>
    </div>
  );
};

export default ThrusterController;