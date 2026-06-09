import { useEffect, useState } from 'react';
import * as ROSLIB from 'roslib';

export const useRosTopic = <T>(topicName: string, messageType: string, defaultValue: T): T => {
  const [data, setData] = useState<T>(defaultValue);

  useEffect(() => {
    // Connexion au ROS Bridge
    const ros = new ROSLIB.Ros({
      url: 'ws://192.168.1.83:9090'
    });

    const topic = new ROSLIB.Topic({
      ros: ros,
      name: topicName,
      messageType: messageType
    });

    topic.subscribe((message: any) => {
      // Si le message est un objet { data: ... } (cas std_msgs/Float32), on extrait le .data
      if (message && message.data !== undefined) {
        setData(message.data as T);
      } else {
        setData(message as T);
      }
    });

    ros.on('error', (error: any) => {
      console.error(`[ROS Bridge] Erreur sur ${topicName}:`, error);
    });

    // Nettoyage lors du démontage du composant
    return () => {
      topic.unsubscribe();
      ros.close();
    };
  }, [topicName, messageType]);

  return data;
};