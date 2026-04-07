// src/hooks/useRosTopic.ts
import { useEffect, useState } from 'react';
import * as ROSLIB from 'roslib'; // <-- Change ceci

export const useRosTopic = <T>(topicName: string, messageType: string, defaultValue: T): T => {
  const [data, setData] = useState<T>(defaultValue);

  useEffect(() => {
    // Vérification de sécurité pour éviter les erreurs SSR ou pendant le build
    if (typeof window === 'undefined') return;

    const ros = new ROSLIB.Ros({
      url: `ws://192.168.1.83:9090`
    });

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: topicName,
      messageType: messageType
    });

    topic.subscribe((message: any) => {
      // Dans ROS, la donnée est souvent dans .data, mais parfois c'est l'objet complet
      // On cast ici selon ce que ton node C++ envoie
      setData(message.data !== undefined ? message.data : message);
    });

    // Gestion des erreurs de connexion pour ne pas polluer la console
    ros.on('error', () => {
      console.log(`[ROS] Erreur de connexion sur ${topicName}`);
    });

    return () => {
      topic.unsubscribe();
      ros.close();
    };
  }, [topicName, messageType]);

  return data;
};