# AirSim Camera Pipeline

This document explains the camera pipeline implemented in the `airsim_offboard_control.py` script, which handles real-time video feed from AirSim simulation.

## Overview

The camera pipeline uses a producer-consumer model with multi-threading to fetch frames from AirSim and display them in real-time. It includes FPS tracking and frame queuing to maintain smooth video playback.

The pipeline runs on a **dedicated AirSim client** (its own `MultirotorClient` RPC session) that is separate from the main control client, so video RPC calls never contend with the flight-control connection. Frames are requested **compressed** (JPEG/PNG), decoded with `cv2.imdecode`, and the fetch loop is **rate-limited** to `video_target_fps` to keep bandwidth, CPU, and GIL pressure low.

## Pipeline Architecture

```mermaid
graph TD
    A[Start Video Feed] --> A2[Create dedicated AirSim video client]
    A2 --> B[Create Threads]
    B --> C[video_fetch_thread]
    B --> D[video_display_thread]
    C --> E[Fetch compressed frames via video client]
    E --> E2[Decode with cv2.imdecode]
    E2 --> F[Convert to OpenCV format]
    F --> G[Calculate fetch FPS]
    G --> H[Push to frame_queue]
    H --> P2[Throttle loop to video_target_fps]
    P2 --> E
    I[Queue Full?] -->|Yes| J[Remove oldest frame]
    I -->|No| K[Add new frame]
    J --> L[Frame ready in queue]
    K --> L
    D --> M[Get frame from queue]
    M --> N[Update render FPS]
    N --> O[Add FPS overlay to frame]
    O --> P[Display in OpenCV window]
    P --> Q[Wait for next frame]
    Q --> M
    L --> M
```

## Detailed Flow

1. **Producer Thread (`video_fetch_loop`)**:
   - Uses the dedicated `video_client` (a separate `MultirotorClient` session) so its RPC loop is never shared with the control client
   - Fetches frames from AirSim using `video_client.simGetImages()` with `compress=True`
   - Decodes the compressed bytes into a BGR image via `cv2.imdecode`
   - Calculates and smooths fetch FPS
   - Pushes frames to a thread-safe queue
   - Throttles itself to `video_target_fps` so the loop stops only as fast as needed

2. **Consumer Thread (`video_display_loop`)**:
   - Retrieves frames from the queue
   - Calculates and smooths render FPS
   - Adds FPS overlay to the frame
   - Displays the frame in an OpenCV window
   - Handles window refresh

3. **Frame Management**:
   - Uses a bounded queue to limit memory usage
   - Implements frame dropping when queue is full
   - Maintains most recent frames for smooth playback

## Key Features

- **Isolated RPC Session**: Video uses its own AirSim client, avoiding thread-unsafe sharing of the main control client's msgpackrpc session
- **Compressed Transfer**: Frames requested with `compress=True`, cutting per-frame payload from ~1.2 MB (raw 848x480 RGB) to tens of KB
- **Rate Limiting**: Fetch loop paced to `video_target_fps` to bound bandwidth, CPU, and GIL contention
- **Thread Safety**: Uses Python's `queue.Queue` for thread-safe frame passing
- **Performance Monitoring**: Tracks both fetch and render FPS
- **Memory Management**: Bounded queue prevents memory overflow
- **Real-time Display**: Smooth frame rendering with FPS overlay
- **Error Handling**: Graceful degradation if AirSim connection fails