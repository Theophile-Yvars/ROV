import React, { useEffect, useState, useRef } from 'react';
import * as ROSLIB from 'roslib';

const ThrusterController = ({ rovIp }) => {
  const [connected, setConnected] = useState(false);
  const [debugKey, setDebugKey] = useState("Neutre");
  
  const rosRef = useRef(null);
  const cmdVelTopicRef = useRef(null);
  const activeKeysRef = useRef({});

  // Fonction utilitaire pour publier de manière centralisée
  const publishMessage = (linearX, linearZ, angularZ) => {
    if (!cmdVelTopicRef.current) return;

    const twistMessage = {
      linear: { x: linearX, y: 0.0, z: linearZ },
      angular: { x: 0.0, y: 0.0, z: angularZ }
    };

    try {
      cmdVelTopicRef.current.publish(twistMessage);
    } catch (err) {
      console.error("❌ Erreur lors de la publication sur le topic :", err);
    }
  };

  // 1. Initialisation de la connexion ROS 2 via Rosbridge
  useEffect(() => {
    if (!rovIp) return;

    const rosUrl = `ws://${rovIp}:9090`;
    
    const RosClass = ROSLIB['Ros'] || window['ROSLIB']?.Ros;
    const TopicClass = ROSLIB['Topic'] || window['ROSLIB']?.Topic;

    if (!RosClass || !TopicClass) {
      console.error("❌ [ROSLIB] Impossible de charger les classes de base.");
      return;
    }

    const ros = new RosClass({ url: rosUrl });
    rosRef.current = ros;

    ros.on('connection', () => {
      setConnected(true);
      console.log('⚡ Moteurs : Connecté au Rosbridge du ROV');
    });

    ros.on('error', () => { setConnected(false); });
    ros.on('close', () => { setConnected(false); });

    cmdVelTopicRef.current = new TopicClass({
      ros: ros,
      name: '/rov/cmd_vel_input', 
      messageType: 'geometry_msgs/msg/Twist'
    });

    return () => {
      if (rosRef.current) rosRef.current.close();
    };
  }, [rovIp]);

  // 2. Écouteurs de touches avec Arrêt Instantané Synchrone
  useEffect(() => {
    if (typeof window !== 'undefined') {
      window.focus();
    }

    const handleKeyDown = (e) => {
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' ', 'Space', 'KeyW', 'KeyS'].includes(e.code) ||
          ['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' ', 'w', 's', 'W', 'S'].includes(e.key)) {
        e.preventDefault(); 
      }

      activeKeysRef.current[e.code] = true;
      activeKeysRef.current[e.key] = true;
    };

    const handleKeyUp = (e) => {
      activeKeysRef.current[e.code] = false;
      activeKeysRef.current[e.key] = false;

      // 🏎️ CORRECTION LATENCE : Si l'utilisateur relâche une commande, on envoie un STOP instantané
      // sans attendre le prochain intervalle de 100ms.
      const keys = activeKeysRef.current;
      const des_touches_restent_enfoncees = 
        keys['ArrowUp'] || keys['ArrowDown'] || keys['ArrowLeft'] || keys['ArrowRight'] || 
        keys['KeyW'] || keys['KeyS'] || keys['w'] || keys['s'] || keys['W'] || keys['S'];

      if (!des_touches_restent_enfoncees) {
        publishMessage(0.0, 0.0, 0.0);
        setDebugKey("Neutre");
      }
    };

    const handleBlur = () => { 
      activeKeysRef.current = {}; 
      // 🛡️ SÉCURITÉ : Si la fenêtre perd le focus, on coupe TOUT immédiatement au lieu de laisser
      // le ROV continuer sur son élan à l'infini
      publishMessage(0.0, 0.0, 0.0);
      setDebugKey("Neutre");
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    window.addEventListener('blur', handleBlur);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      window.removeEventListener('blur', handleBlur);
    };
  }, [connected]); // On ajoute `connected` pour s'assurer que l'état est à jour

  // 3. Boucle de rafraîchissement (Maintien du signal continu à 10Hz)
  useEffect(() => {
    const interval = setInterval(() => {
      if (!connected || !cmdVelTopicRef.current) return;

      const keys = activeKeysRef.current;

      let linearX = 0.0;   
      let angularZ = 0.0;  
      let linearZ = 0.0;   

      if (keys['ArrowUp'] || keys['Up'])        linearX = 1.0;
      if (keys['ArrowDown'] || keys['Down'])    linearX = -1.0;
      if (keys['ArrowLeft'] || keys['Left'])    angularZ = -1.0;
      if (keys['ArrowRight'] || keys['Right'])  angularZ = 1.0;
      if (keys['KeyW'] || keys['w'] || keys['W']) linearZ = 1.0;   
      if (keys['KeyS'] || keys['s'] || keys['S']) linearZ = -1.0;  

      if (keys['Space'] || keys[' ']) {
        linearX = 0.0;
        angularZ = 0.0;
        linearZ = 0.0;
      }

      // Mise à jour de l'affichage de débogage
      if (linearX !== 0 || angularZ !== 0 || linearZ !== 0) {
        setDebugKey(`X: ${linearX} | Z: ${linearZ} | Y: ${angularZ}`);
      } else {
        setDebugKey("Neutre");
      }

      // Publication périodique (permet d'alimenter le Watchdog 500ms du BrainNode)
      publishMessage(linearX, linearZ, angularZ);

    }, 100); 

    return () => clearInterval(interval);
  }, [connected]);

  return (
    <div className="absolute top-16 right-6 bg-black/60 border border-emerald-500/20 px-3 py-1.5 rounded text-[10px] font-mono tracking-widest uppercase z-30 flex flex-col gap-1 backdrop-blur-sm">
      <div className="flex items-center gap-2">
        <div className={`w-1.5 h-1.5 rounded-full ${connected ? 'bg-emerald-400 shadow-[0_0_8px_#34d399]' : 'bg-red-500'}`} />
        <span>Input Clavier : {connected ? "Armé" : "Hors-ligne"}</span>
      </div>
      <div className="text-[9px] text-slate-400 border-t border-white/5 pt-1 text-center">
        Moteurs : <span className={debugKey !== "Neutre" ? "text-amber-400 font-bold" : "text-slate-500"}  >{debugKey}</span>
      </div>
    </div>
  );
};

export default ThrusterController;