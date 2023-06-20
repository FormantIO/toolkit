### Running the static webpage

1. Ensure that the `FORMANT_AGENT_IP` ENV on the agent is set to an IP reachable by this UI
   1. You can use "0.0.0.0" if you would like to listen on all interfaces (careful, this exposes the agent API)
2. Set the IP of the device in src/main.ts. Look for `Fleet.getPeerDevice(...)`
   1. Port 5502 is the default and you likely will not need to change this
3. Install `node` and `npm`, if they are not already installed.
4. Install `yarn` (`npm install -g yarn`) if it is not already installed.
5. Run `yarn` in this directory to install local packages.
6. Run `yarn dev` in this directory to run the local website.
