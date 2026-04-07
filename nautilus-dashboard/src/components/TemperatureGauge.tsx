import React from 'react';
import { useRosTopic } from '../hooks/useRosTopic';

const TemperatureGauge: React.FC = () => {
  // On écoute le topic publié par ton temp_node C++
  const temp = useRosTopic<number>('/rov/temperature', 'std_msgs/msg/Float32');

  const getStatus = (t: number) => {
    if (t >= 45) return { label: 'CRITIQUE', color: '#ff4444' };
    if (t >= 35) return { label: 'CHAUD', color: '#ffcc00' };
    return { label: 'NOMINAL', color: '#00ff00' };
  };

  const currentTemp = temp ?? 0;
  const status = getStatus(currentTemp);

  return (
    <div style={styles.card}>
      <h3 style={styles.title}>Température ROV</h3>
      
      <div style={styles.gaugeBg}>
        <div 
          style={{ 
            ...styles.gaugeFill, 
            width: `${Math.min((currentTemp / 60) * 100, 100)}%`,
            backgroundColor: status.color 
          }} 
        />
      </div>

      <div style={styles.info}>
        <span style={styles.value}>{temp !== null ? `${temp.toFixed(1)}°C` : '---'}</span>
        <span style={{ ...styles.status, color: status.color }}>{status.label}</span>
      </div>
    </div>
  );
};

// Styles inline pour l'exemple (à mettre dans ton CSS/Tailwind)
const styles: { [key: string]: React.CSSProperties } = {
  card: {
    background: '#121212',
    padding: '20px',
    borderRadius: '12px',
    border: '1px solid #333',
    width: '220px',
    fontFamily: 'sans-serif'
  },
  title: { fontSize: '0.9rem', color: '#aaa', margin: '0 0 10px 0' },
  gaugeBg: { background: '#333', height: '8px', borderRadius: '4px', overflow: 'hidden' },
  gaugeFill: { height: '100%', transition: 'all 0.5s ease' },
  info: { display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginTop: '10px' },
  value: { fontSize: '1.4rem', fontWeight: 'bold', color: '#fff' },
  status: { fontSize: '0.7rem', fontWeight: 'bold' }
};

export default TemperatureGauge;