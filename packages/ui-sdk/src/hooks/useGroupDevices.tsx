import { Fleet } from "@formant/data-sdk";
import { useState, useEffect } from "react";

// IDevice from EmbeddedAppMessage (simplified device info: id, name, tags)
// This matches what Fleet.getGroupDevices() returns
type IDevice = {
  id: string;
  name: string;
  tags: { [key: string]: string };
};

/**
 * Hook for accessing devices in a group context (coherence group views).
 * Returns device information from the group context if available, otherwise returns undefined.
 * 
 * This hook works **without authentication** by using Fleet.getGroupDevices() which returns
 * IDevice[] (id, name, tags) from the overview_devices message sent by the host.
 * 
 * For full Device instances with API access, use Fleet.getGroupDevicesAsDeviceInstances() directly.
 * 
 * This hook is designed for modules that need to work with multiple devices
 * in coherence group views. For single-device contexts, use useDevice() instead.
 * 
 * @returns Array of IDevice objects if in group context, undefined otherwise
 * 
 * @example
 * ```tsx
 * function GroupAwareModule() {
 *   const devices = useGroupDevices();
 *   
 *   if (!devices || devices.length === 0) {
 *     return <div>Not in group context</div>;
 *   }
 *   
 *   return (
 *     <div>
 *       {devices.map(device => (
 *         <DeviceCard key={device.id} device={device} />
 *       ))}
 *     </div>
 *   );
 * }
 * ```
 * 
 * @example
 * ```tsx
 * // If you need full Device instances with API access:
 * import { Fleet, Authentication } from "@formant/data-sdk";
 * 
 * function MyModule() {
 *   const [devices, setDevices] = useState<Device[]>();
 *   
 *   useEffect(() => {
 *     const load = async () => {
 *       if (await Authentication.waitTilAuthenticated()) {
 *         const fullDevices = await Fleet.getGroupDevicesAsDeviceInstances();
 *         setDevices(fullDevices);
 *       }
 *     };
 *     load();
 *   }, []);
 *   
 *   // Use devices...
 * }
 * ```
 */
const useGroupDevices = (): IDevice[] | undefined => {
  const [devices, setDevices] = useState<IDevice[] | undefined>();

  useEffect(() => {
    // Check for group devices immediately (no auth required)
    const checkDevices = () => {
      const groupDevices = Fleet.getGroupDevices();
      if (groupDevices && groupDevices.length > 0) {
        setDevices(groupDevices);
        return true;
      }
      return false;
    };

    // Check immediately
    if (!checkDevices()) {
      // If not available yet, poll briefly for when overview_devices arrives
      // This handles the case where overview_devices arrives after component mounts
      const checkInterval = setInterval(() => {
        if (checkDevices()) {
          clearInterval(checkInterval);
        }
      }, 100);

      // Clean up interval after 5 seconds (devices should arrive quickly)
      const timeout = setTimeout(() => {
        clearInterval(checkInterval);
      }, 5000);

      return () => {
        clearInterval(checkInterval);
        clearTimeout(timeout);
      };
    }
  }, []);

  return devices;
};

export default useGroupDevices;
