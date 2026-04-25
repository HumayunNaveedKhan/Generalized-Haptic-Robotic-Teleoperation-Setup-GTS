# Generalized Haptic Teleoperation Setup — GTS

> Intercontinental haptic teleoperation framework 
> featuring adaptive jitter buffering, delay compensation,
> and EBA+Projection force control — validated 
> over 7,000 km between HHRCM Lab-NCRA, NEDUET, Karachi, Pakistan and IGS Group, University of Innsbruck, Innsbruck, Austria.

---

## Overview

GTS is the software and hardware framework developed by Humayun Khan at HHRCM Lab, NCRA-NEDUET, under the 
mentorship of Prof. Dr. Riaz Uddin (Professor, Director NCRA & Director ORIC, NEDUET).

The system enables real-time bilateral force-feedback 
teleoperation over Wide Area Networks, where a human operator 
at the **master** site manipulates a haptic device and feels 
real-time force feedback from the **slave** environment — 
across any network distance.

**Date of Intercontinental Experiment:** 10 November 2022  
**Distance:** 7,000 km — Karachi, Pakistan ↔ Innsbruck, Austria  
**Master Lab:** HHRCM Lab, NCRA-NEDUET, Karachi  
**Slave Lab:** IGS Group, University of Innsbruck, Austria  
**Haptic Rate:** 1,000 Hz  
**Protocol:** UDP over WAN  

<img width="1386" height="935" alt="GTS-Overview" src="https://github.com/user-attachments/assets/9e00b176-e362-4b00-862e-28f8150c5c59" />
<img width="4032" height="1908" alt="GTS-server-site-KHI-PK" src="https://github.com/user-attachments/assets/483b5aaa-4903-4220-8833-96073cb1f8e9" />
<img width="1600" height="1201" alt="GTS-client-site-INBK-AUS" src="https://github.com/user-attachments/assets/8eb1084f-0451-40ba-9684-c0c371654f33" />



---

## Problem It Solves

Haptic teleoperation over networks faces three fundamental 
challenges:

- **Latency and Network jitter** — variable inter-packet delays corrupt 
  force feedback timing, causing instability
- **Propagation delay** — intercontinental latency 
  (processing + queuing + transmission + propagation) 
  degrades transparency and task performance
- **Packet loss** — dropped force packets cause force 
  discontinuities that can destabilise the control loop

GTS addresses all three through an adaptive buffer mechanism 
(RH-Buffer), delay compensation, and 
EBA+Projection force rendering — tested under real 
intercontinental WAN conditions (improveness is in progress).

---

## System Architecture

<img width="1536" height="1024" alt="Architecture-GTS" src="https://github.com/user-attachments/assets/b01ca78b-54ae-41ca-a2c8-8774eed32672" />


```
MASTER — HHRCM Lab, Karachi, Pakistan
┌─────────────────────────────────────────┐
│  Phantom TouchX / Desktop               │
│  CHAI3D Haptic Rendering Engine         │
│  GLFW Virtual Environment (3D scene)   │
│  Position Controller                   │
│  EBA + Projection Block                │
│  RH-Buffer (Adaptive Jitter Buffer)    │
│  Smith Predictor (Delay Compensation)  │
└──────────────┬──────────────────────────┘
               │ UDP — Position (x,y,z) @ 1000 Hz
               │ ←── Force (Fx,Fy,Fz) @ 1000 Hz
        ╔══════╧═══════╗
        ║  WAN — 7000 km ║
        ║  Latency · Jitter · Packet Loss ║
        ╚══════╤═══════╝
               │
SLAVE — IGS Lab, University of Innsbruck, Austria
┌─────────────────────────────────────────┐
│  Novint Falcon Haptic Device           │
│  Force Dimension SDK                   │
│  AdaptivePacketBuffer (RH-Buffer)      │
│  EBA + Projection Block                │
│  Haptic Control Loop @ 100 µs          │
│  Virtual Walls + Stiffness Adaptation  │
└─────────────────────────────────────────┘
```

---

## Key Technical Contributions

### 1. RH-Buffer — Adaptive Jitter Buffer
Dynamically regulates incoming haptic data packets to 
approximate a constant-delay model. The buffer release 
interval adapts to measured inter-packet jitter using a 
sliding window of 20 packets, preventing force 
discontinuities caused by network variability.

### 2. EBA + Projection Force Control
Energy Bounding Algorithm with Projection ensures haptic 
stability under time-varying network delays. Forces are 
projected into the passive subspace before rendering, 
guaranteeing energy boundedness of the bilateral loop.

### 3. Smith Predictor Delay Compensation
A G(z)-Smith Predictor architecture on the master side 
estimates plant dynamics and round-trip delay, allowing the 
controller to compensate for known propagation delay and 
improve transparency.

### 4. Adaptive Stiffness
Server-side stiffness (Kp) adapts in real-time based on 
measured network jitter — reducing stiffness when jitter 
exceeds threshold (10 ms) to maintain stability, increasing 
it as the network stabilises.

---

## Hardware Stack

| Component | Details |
|---|---|
| Master haptic device | 3D Systems Phantom TouchX / Phantom Desktop |
| Slave haptic device | Novint Falcon (3-DOF) |
| Master framework | CHAI3D 3.2.0 + GLFW + GLEW + freeglut |
| Slave framework | Force Dimension SDK 3.17.1 (dhdc / drdc) |
| Network | UDP over WAN (IPv4) |
| Haptic rate | 1,000 Hz (1 ms loop) |
| Haptic thread | Windows `THREAD_PRIORITY_TIME_CRITICAL` |
| Network thread | Windows `THREAD_PRIORITY_ABOVE_NORMAL` |
| OS | Windows 10/11 (both ends) |

---

## Software Stack

| Layer | Technology |
|---|---|
| Master firmware | C++ (CHAI3D, GLFW, Winsock2) |
| Slave firmware | C++ (Force Dimension SDK, Winsock2, OpenGL/GLUT) |
| Force rendering | CHAI3D haptic rendering pipeline |
| Slave haptic | DHD/DRD API (Force Dimension) |
| Network | UDP sockets — bidirectional position/force packets |
| Jitter buffer | Custom `AdaptivePacketBuffer` class |
| Visualisation | GLFW + GLUT dual-window dashboard |
| Logging | Timestamped console + file logging |

---

## Packet Protocol

### Master → Slave (Position Packet)
```
struct OutgoingPacket {
    int   packetNumber;
    float x, y, z;           // Haptic device position (m)
    float delay;             // Nominal interval (ms)
    char  timestamp[64];
    char  srcIP[16];
    char  dstIP[16];
    int   srcPort;
    int   dstPort;
};
```

### Slave → Master (Force Feedback Packet)
```
struct IncomingPacket {
    int    packetNumber;
    double force[3];         // Applied force (N)
    double position[3];      // Slave device position (m)
    double error[3];         // Position error (m)
    double stiffness;        // Adaptive stiffness (N/m)
    double damping;          // Adaptive damping (N·s/m)
    bool   isActive;
    char   timestamp[64];
};
```

---

## Source Code

| File | Description |
|---|---|
| [`src/GTS-Master-Client.cpp`](src/GTS-Master-Client.cpp) | Master (Karachi) — CHAI3D haptic rendering, GLFW virtual environment, adaptive damping, UDP position sender, force receiver, real-time dashboard |
| [`src/GTS-Slave-Server.cpp`](src/GTS-Slave-Server.cpp) | Slave (Innsbruck) — Force Dimension SDK control, AdaptivePacketBuffer (RH-Buffer), adaptive stiffness, jitter analysis, UDP force sender |

> ⚠️ **Configuration before use:**  
> Set `CLIENT_IP` and `SERVER_IP` in both files to your own machine IPs.  
> Update CHAI3D and Force Dimension SDK include/library paths.  
> Build with Teensyduino 1.59 + CHAI3D 3.2.0 + FD SDK 3.17.1.

---

## Intercontinental Experiment — 10 November 2022

The GTS was validated in a live intercontinental experiment 
spanning 7,000 km between two university research labs.

**Master site:** HHRCM Lab, NCRA, NED University of 
Engineering & Technology, Karachi, Pakistan  
**Slave site:** IGS Group, University of Innsbruck, 
Innsbruck, Austria  

Two test modes were conducted:
- **Free Motion Testing** — operator moves freely, 
  evaluating position tracking fidelity and force 
  feedback continuity under intercontinental delay
- **Contact Mode Testing** — operator makes contact with 
  virtual object at slave site, evaluating force fidelity, 
  stability, and transparency under WAN jitter

The experiment highlighted the need for adaptive buffering 
and Smith Predictor compensation, which form the core of 
the GTS framework.

---

## Thesis Abstract

> In the evolving field of haptic teleoperation, ensuring 
> robust and transparent performance over networked 
> environments remains a significant challenge due to 
> factors such as jitter, variable communication delays, 
> and packet loss — especially during real-time, complex 
> manipulation tasks. This thesis presents the design and 
> implementation of a resilient haptic teleoperation 
> framework featuring an adaptive buffer mechanism, 
> strategically developed to mitigate the adverse effects 
> of network disturbances. The buffer dynamically regulates 
> incoming haptic data packets to approximate a 
> constant-delay model, thereby enhancing control stability 
> and preserving consistent force feedback. In parallel, 
> a custom-optimised network protocol is employed to further 
> minimise jitter and improve overall transmission 
> efficiency. Extensive real-time experiments under varying 
> network conditions confirm substantial improvements in 
> reliability, interaction transparency, and force fidelity.

---

## Haptic Frame Rate Reference

| Rate (Hz) | Quality | Application |
|---|---|---|
| 500 | High-fidelity tactile | Advanced VR systems |
| **1000** | **Standard for realistic haptics** | **Teleoperation, immersive training** |
| 1500 | Advanced precision | Cutting-edge research devices |
| 2000 | Ultra-high-fidelity | Advanced haptics research |

GTS operates at 1,000 Hz — the standard for realistic 
haptic teleoperation.

---

## Network Delay Model

End-to-End delay in GTS over WAN:

```
E2E Delay = Processing Delay 
          + Queuing Delay 
          + Transmission Delay 
          + Propagation Delay
```

At 7,000 km, propagation delay alone is ~35 ms 
(at ⅔ speed of light over fibre). The RH-Buffer and 
Smith Predictor are designed to compensate for this 
while maintaining haptic stability.

---

## Publication

H. Khan and R. Uddin,  
"Haptic Teleoperation Framework with Jitter Compensation,"  
*Engineering Proceedings*, MDPI, 2023.  
DOI: [10.3390/engproc2023032009](https://doi.org/10.3390/engproc2023032009)

---

## Related Projects

- [NED-EIoT](https://github.com/HumayunNaveedKhan/Embedded-Industrial-Energy-Monitoring-Device-NED_EIoT) 
  — Industrial Energy & Equipment Health Monitor
- [NED-SILL](https://github.com/HumayunNaveedKhan/Smart-Industrial-Liquid-Level-Monitoring-Device-NED_SILL) 
  — Smart Industrial Liquid Level Monitor
- [Realtime Embedded Vision Pipeline](https://github.com/HumayunNaveedKhan/Realtime-Embedded-Vision-Pipeline-Teensy-4.x-MT9V034) 
  — Teensy 4.x + MT9V034 @ 60 FPS

---

## Author

**Humayun Khan**  
Team Lead, HHRCM Lab, NCRA-NEDUET  
M.Engg. Computer Networks & System Security, NEDUET (GPA 3.89/4.00)  
Co-Founder, Haptronica & RobAutoStem (NIC Karachi, Cohort 12)

📧 humayunnaveedkhan@gmail.com  
🔗 [LinkedIn](https://linkedin.com/in/humayunnaveedkhan)  
🌐 [Portfolio](https://humayunnaveedkhan.github.io/portfolio)

---

## License

© 2023 Humayun Khan, HHRCM Lab NCRA-NEDUET.  
All rights reserved.  
No part of this repository may be reproduced or used 
commercially without written permission.
