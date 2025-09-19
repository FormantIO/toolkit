## Realtime Video

A real-time, peer-to-peer module application that displays a video feed from a camera, allowing users to connect and view the feed simply by knowing the camera name.

## Features

- Real-time H.264 video streaming with bandwidth optimization
- WebRTC connection management

## Node.js Polyfills

This module uses Vite with Node.js polyfills to handle Formant SDK dependencies that contain Node.js-specific code. The polyfills are configured in:

- **vite.config.ts**: Uses `vite-plugin-node-polyfills` for automatic polyfills
- **src/polyfills.ts**: Manual polyfill for `setImmediate` (required for WebRTC)

## Local Development

```bash
# Install dependencies
make install

# Clean install (useful when updating SDK versions)
make clean-install

# Local development server (port 5173)
make local-dev

# Test built version locally (port 4173)
make local-preview
```

## Deployment

```bash
# Prepare deployment files
make deploy-prep

# Deploy to current branch
make deploy

# Deploy to new branch (sets upstream automatically)
make deploy-with-upstream
```

## Troubleshooting

If you encounter "process is not defined" or "setImmediate is not defined" errors:

1. Ensure `vite-plugin-node-polyfills` is installed
2. Verify `src/polyfills.ts` is imported first in `src/main.tsx`
3. Check that Vite config includes the node polyfills plugin

These polyfills are required because the Formant SDK contains webpack-bundled code with Node.js globals that Vite needs help handling.
