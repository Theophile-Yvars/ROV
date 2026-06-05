import { useEffect, useState } from 'react';
import * as ROSLIB from 'roslib';

export const useRosTopic = <T>(topicName: string, messageType: string, defaultValue: T): T => {
  const [data, setData] = useState<T>(defaultValue);

  useEffect(() => {
    if (typeof window === 'undefined') return;

    // 1. Extraction compatible avec le bundle de Vite
    const RosClass = (ROSLIB as any).Ros || (window as any).ROSLIB?.Ros;
    const TopicClass = (ROSLIB as any).Topic || (window as any).ROSLIB?.Topic;

    if (!RosClass || !TopicClass) {
      console.error(`[ROS Hook] Échec de chargement des constructeurs pour : ${topicName}`);
      return;
    }

    // 2. Utilisation des classes extraites
    const ros = new RosClass({
      url: `ws://192.168.1.83:9090`
    });

    const topic = new TopicClass({
      ros: ros,
      name: topicName,
      messageType: messageType
    });

    topic.subscribe((message: any) => {
      if (message !== undefined && message !== null) {
        setData(message as T);
      }
    });

    ros.on('error', () => {
      console.log(`[ROS Bridge] Erreur de communication ou serveur déconnecté sur : ${topicName}`);
    });

    return () => {
      topic.unsubscribe();
      ros.close();
    };
  }, [topicName, messageType]);

  return data;
};