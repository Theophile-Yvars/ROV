import React, { createContext, useContext, useEffect, useState } from 'react';
import * as ROSLIB from 'roslib';

const RosContext = createContext<ROSLIB.Ros | null>(null);

export const RosProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [ros] = useState(() => new ROSLIB.Ros({ url: 'ws://192.168.1.83:9090' }));
  return <RosContext.Provider value={ros}>{children}</RosContext.Provider>;
};

export const useRos = () => useContext(RosContext);