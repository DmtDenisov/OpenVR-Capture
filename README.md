# OpenVR Capture for OBS Studio

This plugin allows capturing directly from OpenVR/SteamVR in full resolution.

A fork of OBS-OpenVR-Input-Plugin, originally made by Keijo "Kegetys" Ruotsalainen

![obs64_4E9advFPF8](https://github.com/user-attachments/assets/98e52da2-f58d-4a63-a975-b07704e4a4e9)


### Q. What benefits does this have over the original?
A.
- Real-time image stabilization: head-rotation smoothing with presets and Roll Lock, done as a GPU reprojection pass (see below).
- Crop function replaced with realtime Aspect Ratio dropdown with Zoom and Offsets.
- Threaded initialization prevents stutter in OBS Studio.
- OpenVR SDK updated from v1.12.5 to v2.5.1
- Minor performance tweaks.

---------

### Image Stabilization

Smooths head rotation (yaw/pitch/roll) in real time by reprojecting the SteamVR mirror image against headset pose data — an async-timewarp-style GPU pass that replaces the plugin's texture copy, so it adds practically no cost.

- Tick **Stabilization** in the source properties, then pick a preset (Low / Medium / High) or Custom strength.
- The adaptive filter locks the image while your head is still and catches up smoothly during deliberate turns.
- **Roll Lock** keeps the horizon gravity-level instead of following head tilt.
- Corrections hide inside the **Zoom** margin — set Zoom to ~1.2 or higher. At Zoom 1.0 stabilization still works but reveals black edges while correcting (the properties dialog shows a warning).
- **Pose Delay** fine-tunes pose-to-frame pairing if fast head turns cause wobble (usually 1–2).

---------

Installation:
1. Download latest release .zip
2. Extract all files to the root of your OBS Studio installation.

---------

Compiling:
1. Pull OBS Studio source code recursively (`git clone https://github.com/obsproject/obs-studio.git --recursive`)
2. Pull this repo, copy "plugins" into the root of OBS Studio's source code (`git clone https://github.com/Pigney/OpenVR-Capture.git`)
3. Pull OpenVR SDK **v2.5.1** inside "deps" folder. (`git clone --branch v2.5.1 --depth 1 https://github.com/ValveSoftware/openvr.git`) — newer SDK headers request runtime interface versions that older SteamVR installs don't provide, which makes VR_Init fail with "Interface Not Found (105)".
4. Add `add_obs_plugin(win-openvr PLATFORMS WINDOWS)` to the end of obs-studio/plugins/CMakeLists.txt
5. Compile from root directory with `cmake --preset windows-x64 && cmake --build ./build_x64/plugins/win-openvr --config Release`
