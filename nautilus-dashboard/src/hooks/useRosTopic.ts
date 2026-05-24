import { useEffect, useState } from 'react';
import * as ROSLIB from 'roslib';

export const useRosTopic = <T>(topicName: string, messageType: string, defaultValue: T): T => {
  const [data, setData] = useState<T>(defaultValue);

  useEffect(() => {
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