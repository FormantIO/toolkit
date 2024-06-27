import { Fleet, IStream } from '@formant/data-sdk';
import { useEffect, useState } from 'react';

export const useStreams = (device: any) => {
  const [streams, setStreams] = useState<IStream[]>([]);

  useEffect(() => {
    if (!device) return;
    Fleet.getStreams().then((_) => {
      setStreams(_);
    });
  }, [device]);

  return streams;
};
