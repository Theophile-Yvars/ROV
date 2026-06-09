import React, { useState, useEffect, useRef } from 'react';

const CameraTiltControl = () => {
  const [tilt, setTilt] = useState(0);
  const publisherRef = useRef(null);

  useEffect(() => {
    // Vérification de sécurité pour le chargement du CDN
    if (!window.ROSLIB) return;

    // Initialisation du Topic
    publisherRef.current = new window.ROSLIB.Topic({
      ros: new window.ROSLIB.Ros({ url: 'ws://192.168.1.83:9090' }),
      name: '/rov/camera_tilt',
      messageType: 'std_msgs/Float32'
    });
  }, []);

  const handleTiltChange = (e) => {
    const value = parseFloat(e.target.value);
    setTilt(value);

    // Publication de la valeur au format ROS
    if (publisherRef.current) {
      const msg = new window.ROSLIB.Message({ data: value });
      publisherRef.current.publish(msg);
    }
  };

  return (
    <div className="absolute right-6 bottom-20 z-40 bg-black/70 p-4 rounded border border-white/10 text-center">
      <h3 className="text-[10px] text-slate-400 uppercase">Tilt Caméra</h3>
      <input
        type="range"
        min="-1.0"
        max="1.0"
        step="0.1"
        value={tilt}
        onChange={handleTiltChange}
        className="cursor-pointer"
      />
    </div>
  );
};

export default CameraTiltControl;