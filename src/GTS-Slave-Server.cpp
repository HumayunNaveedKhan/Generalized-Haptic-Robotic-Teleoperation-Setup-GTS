/**
 * Generalized Haptic Teleoperation Setup — GTS
 * Slave Server (Innsbruck, Austria)
 * Version: 1.0
 *
 * Force Dimension SDK-based slave server for bilateral
 * force-feedback teleoperation. Receives position packets
 * from master at 1000 Hz, processes them through the
 * AdaptivePacketBuffer (RH-Buffer), computes haptic forces,
 * and sends force feedback back to the master.
 *
 * Key innovations:
 *   RH-Buffer — Adaptive jitter buffer with sliding window
 *     statistics to approximate constant-delay model
 *   Adaptive Stiffness — Real-time Kp adaptation based on
 *     measured network jitter (threshold: 10 ms)
 *   EBA + Projection — Force rendering with position error
 *     bounded by adaptive stiffness and damping
 *   Virtual Walls — Configurable workspace boundaries
 *
 * CONFIGURATION:
 *   Replace SERVER_IP with your slave machine IP.
 *   Update Force Dimension SDK include/library paths.
 *   Build: Visual Studio + Force Dimension SDK 3.17.1
 *         + freeglut + OpenGL
 *
 * Intercontinental Experiment: 10 November 2022
 *   Master: HHRCM Lab, NCRA-NEDUET, Karachi, Pakistan
 *   Slave:  IGS Group, University of Innsbruck, Austria
 *   Distance: ~7,000 km over WAN
 *
 * Author:   Humayun Khan
 * Lab:      HHRCM Lab, NCRA-NEDUET, Karachi, Pakistan
 * Mentor:   Prof. Dr. Riaz Uddin (Director NCRA & ORIC, NEDUET)
 * Thesis:   Network Jitter Compensation in Haptic Teleoperation
 *
 * Publication:
 *   H. Khan and R. Uddin, "Haptic Teleoperation Framework
 *   with Jitter Compensation," Engineering Proceedings,
 *   MDPI, 2023. DOI: 10.3390/engproc2023032009
 *
 * © 2023 Humayun Khan, HHRCM Lab NCRA-NEDUET.
 * All rights reserved.
 */

#define WIN32_LEAN_AND_MEAN
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define NOMINMAX
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <GL/glut.h>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <map>
using namespace std;

// Update these paths to match your Force Dimension SDK installation:
#include "dhdc.h"
#include "drdc.h"

#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "dhdms64.lib")
#pragma comment(lib, "drdms64.lib")

// ============================================================
// NETWORK CONFIGURATION — replace before deployment
// ============================================================
#define UDP_PORT 4000
#define MAX_PACKET_SIZE 400
#define MAX_BUFFER_SIZE 500
const char* SERVER_IP       = "YOUR_SLAVE_IP";   // Slave machine IP
const int SOCKET_TIMEOUT_MS = 5000;

// ============================================================
// RH-BUFFER — Adaptive Jitter Buffer Configuration
// ============================================================
const int    SLIDING_WINDOW_SIZE = 20;
const double JITTER_THRESHOLD_MS = 10.0;  // ms — threshold for stiffness adaptation

// ============================================================
// HAPTIC CONFIGURATION
// ============================================================
double STIFFNESS_K         = 100.0;   // N/m — base stiffness
double MAX_FORCE           = 5.0;     // N
double FORCE_SCALE         = 1.0;
double POSITION_LIMIT_X    = 0.1;     // m
double POSITION_LIMIT_Y    = 0.1;
double POSITION_LIMIT_Z    = 0.1;
bool   ENABLE_VIRTUAL_WALLS = true;
double MIN_STIFFNESS_K     = 50.0;    // N/m
double MAX_STIFFNESS_K     = 150.0;   // N/m
double STIFFNESS_ADAPTATION_RATE = 0.1;

// Processing mode
bool  USE_ADAPTIVE_BUFFER       = true;
std::mutex adaptiveBufferMutex;
int   FIXED_PROCESSING_INTERVAL_MS = 10;
bool  USE_FIXED_PROCESSING_RATE    = false;

// Packet processing
int   FEEDBACK_ENABLED           = 1;
const int PACKET_HISTORY_SIZE    = 100;
float GLOBAL_RELEASE_WIDTH_FACTOR = 1.0f;
bool  SHOW_PACKET_NUMBERS        = true;
bool  SHOW_TIME_GAPS             = true;
bool  LOG_TO_CONSOLE             = true;
bool  LOG_TO_CSV                 = false;
int   MAX_GRAPH_POINTS           = 50;
bool  UNIFORM_DELAY_ENABLED      = false;
int   UNIFORM_DELAY_MS           = 0;
bool  RANDOM_DELAY_ENABLED       = false;
int   RANDOM_DELAY_MIN_MS        = 0;
int   RANDOM_DELAY_MAX_MS        = 50;

// Display
const int   MAX_DISPLAY_PACKETS        = 30;
const int   TIMESTAMP_DISPLAY_INTERVAL = 4;
const float TEXT_SPACING               = 0.04f;
const float BAR_WIDTH                  = 0.016f;
const float BAR_MAX_HEIGHT             = 2.0f;
const float TEXT_Y_OFFSET              = 0.030f;
const float LEGEND_WIDTH               = 0.30f;
const float LEGEND_HEIGHT              = 0.20f;
int   windowWidth  = 1280;
int   windowHeight = 800;
bool  fullscreen   = false;
float scaleFactor  = 1.0f;

// Thread priorities
const int THREAD_PRIORITY_PACKET_RECEPTION = THREAD_PRIORITY_HIGHEST;
const int THREAD_PRIORITY_PROCESSING       = THREAD_PRIORITY_ABOVE_NORMAL;
const int THREAD_PRIORITY_HAPTICS          = THREAD_PRIORITY_TIME_CRITICAL;
const int HAPTIC_THREAD_INTERVAL_US        = 100;

// UI
bool showHelp          = false;
bool showLegend        = true;
bool showStats         = true;
bool showGraphs        = true;
bool showVectorVis     = true;
bool showJitterAnalysis = true;
const void* FONT_SMALL  = GLUT_BITMAP_8_BY_13;
const void* FONT_MEDIUM = GLUT_BITMAP_9_BY_15;
const void* FONT_LARGE  = GLUT_BITMAP_HELVETICA_12;
const void* FONT_TITLE  = GLUT_BITMAP_HELVETICA_18;

// ============================================================
// DATA STRUCTURES
// ============================================================
struct HapticConfig {
    double Kp          = STIFFNESS_K;
    double Cd          = 0.5;           // Damping coefficient (N·s/m)
    double maxForce    = MAX_FORCE;
    double forceScale  = FORCE_SCALE;
    bool hapticEnabled = true;
    bool deviceConnected = false;
    int  deviceID      = -1;
    bool wallsEnabled  = ENABLE_VIRTUAL_WALLS;
    double posLimit[3] = { POSITION_LIMIT_X, POSITION_LIMIT_Y, POSITION_LIMIT_Z };
};
HapticConfig hapticConfig;
std::mutex hapticMutex;

struct DeviceState {
    double position[3]  = { 0.0, 0.0, 0.0 };
    double velocity[3]  = { 0.0, 0.0, 0.0 };
    double force[3]     = { 0.0, 0.0, 0.0 };
    double targetPos[3] = { 0.0, 0.0, 0.0 };
};
DeviceState deviceState;
std::mutex deviceMutex;

struct PacketStats {
    double avgReceiveRate    = 0.0;
    int totalPacketsReceived = 0;
    int totalPacketsProcessed = 0;
    int totalFeedbackSent    = 0;
    std::chrono::steady_clock::time_point firstPacketTime;
    std::chrono::steady_clock::time_point lastPacketTime;
    double avgInterPacketTime = 0.0;
    std::deque<double> packetTimings;
    std::deque<std::pair<int,float>> rawDelays;
    std::deque<std::pair<int,float>> processedDelays;
    double avgNetworkDelay  = 0.0;
    double networkJitter    = 0.0;
    double adaptedStiffness = STIFFNESS_K;
};
PacketStats globalPacketStats;
std::mutex statsMutex;

struct Packet {
    int   packetNumber;
    float x, y, z;
    float delay;
    char  timestamp[64];
    char  srcIP[INET_ADDRSTRLEN];
    char  dstIP[INET_ADDRSTRLEN];
    int   srcPort;
    int   dstPort;
    std::chrono::steady_clock::time_point receiveTime;
    float processingDelay;
    bool  isProcessed = false;
};

struct IncomingPacket {
    int    packetNumber;
    double force[3];
    double position[3];
    double error[3];
    double stiffness;
    double damping;
    bool   isActive;
    char   timestamp[64];
};

std::deque<IncomingPacket> feedbackHistory;
std::mutex feedbackMutex;
const int MAX_FEEDBACK_HISTORY = 50;

SOCKET recvSocket = INVALID_SOCKET;
std::mutex socketMutex;

struct ClientInfo {
    sockaddr_in addr;
    bool valid = false;
    char ipStr[INET_ADDRSTRLEN];
    int  port;
    std::chrono::steady_clock::time_point lastActivity;
};
ClientInfo lastClient;
std::mutex clientMutex;

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<> randomDelayDist(RANDOM_DELAY_MIN_MS, RANDOM_DELAY_MAX_MS);

// Forward declarations
void sendFeedback(const sockaddr_in& clientAddr, int packetNumber, const double force[3],
                  const double position[3], const double error[3], double stiffness);
void cleanup();
void adaptHapticParameters();
double calculateAdaptiveStiffness(double jitter, double avgDelay);
void calculateVirtualWallForces(double position[3], double velocity[3], double limits[3], double outForce[3]);
void display(); void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);
void specialKeyboard(int key, int x, int y);
void timerCallback(int value);

// ============================================================
// ADAPTIVE PACKET BUFFER (RH-BUFFER)
// ============================================================
class AdaptivePacketBuffer {
private:
    std::deque<Packet> packetBuffer;
    std::deque<Packet> processedPackets;
    std::deque<Packet> slidingWindow;
    std::mutex bufferMutex, processedMutex, windowMutex;
    std::chrono::steady_clock::time_point lastReleaseTime;
    std::map<int, double> packetTimeGaps;

public:
    AdaptivePacketBuffer() : lastReleaseTime(std::chrono::steady_clock::now()) {}

    void addPacket(const Packet& packet) {
        std::lock_guard<std::mutex> lock(bufferMutex);
        if (!packetBuffer.empty()) {
            double tg = std::chrono::duration_cast<std::chrono::duration<double,std::milli>>(
                packet.receiveTime - packetBuffer.back().receiveTime).count();
            packetTimeGaps[packet.packetNumber] = tg;
        }
        packetBuffer.push_back(packet);
        if (packetBuffer.size() > MAX_BUFFER_SIZE) packetBuffer.pop_front();
        {
            std::lock_guard<std::mutex> wl(windowMutex);
            slidingWindow.push_back(packet);
            while (slidingWindow.size() > SLIDING_WINDOW_SIZE) slidingWindow.pop_front();
            calculateDelayStatistics();
        }
    }

    void addProcessedPacket(const Packet& p) {
        std::lock_guard<std::mutex> lock(processedMutex);
        processedPackets.push_back(p);
        while (processedPackets.size() > MAX_DISPLAY_PACKETS) processedPackets.pop_front();
    }

    void calculateDelayStatistics() {
        if (slidingWindow.size() < 2) return;
        std::vector<double> delays;
        double sum = 0.0;
        for (size_t i=1; i<slidingWindow.size(); i++) {
            double d = std::chrono::duration_cast<std::chrono::duration<double,std::milli>>(
                slidingWindow[i].receiveTime - slidingWindow[i-1].receiveTime).count();
            sum += d; delays.push_back(d);
        }
        double avg = sum / delays.size();
        double ssq = 0.0;
        for (double d : delays) { double diff=d-avg; ssq+=diff*diff; }
        double jitter = sqrt(ssq / delays.size());
        std::lock_guard<std::mutex> sl(statsMutex);
        globalPacketStats.avgNetworkDelay  = avg;
        globalPacketStats.networkJitter    = jitter;
        globalPacketStats.adaptedStiffness = calculateAdaptiveStiffness(jitter, avg);
    }

    void processPackets() {
        auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> bl(bufferMutex);
        std::lock_guard<std::mutex> pl(processedMutex);
        if (packetBuffer.empty()) return;
        double intervalMs = USE_ADAPTIVE_BUFFER
            ? [&]{ double a; { std::lock_guard<std::mutex> sl(statsMutex); a=globalPacketStats.avgInterPacketTime; } return (a<=0?10.0:a)*GLOBAL_RELEASE_WIDTH_FACTOR; }()
            : (double)FIXED_PROCESSING_INTERVAL_MS;
        if (std::chrono::duration<double,std::milli>(now - lastReleaseTime).count() < intervalMs) return;
        auto oldest = std::min_element(packetBuffer.begin(), packetBuffer.end(),
            [](const Packet& a, const Packet& b){ return a.packetNumber < b.packetNumber; });
        if (UNIFORM_DELAY_ENABLED && UNIFORM_DELAY_MS > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(UNIFORM_DELAY_MS));
        if (RANDOM_DELAY_ENABLED)
            std::this_thread::sleep_for(std::chrono::milliseconds(randomDelayDist(gen)));
        Packet processed = *oldest;
        processed.processingDelay = std::chrono::duration<float,std::milli>(
            std::chrono::steady_clock::now() - processed.receiveTime).count();
        processed.isProcessed = true;
        { std::lock_guard<std::mutex> dl(deviceMutex);
          deviceState.targetPos[0]=processed.x; deviceState.targetPos[1]=processed.y; deviceState.targetPos[2]=processed.z; }
        { std::lock_guard<std::mutex> sl(statsMutex);
          globalPacketStats.rawDelays.push_back({processed.packetNumber, processed.delay});
          globalPacketStats.processedDelays.push_back({processed.packetNumber, processed.processingDelay});
          while (globalPacketStats.rawDelays.size()>MAX_GRAPH_POINTS) globalPacketStats.rawDelays.pop_front();
          while (globalPacketStats.processedDelays.size()>MAX_GRAPH_POINTS) globalPacketStats.processedDelays.pop_front();
          globalPacketStats.totalPacketsProcessed++; }
        processedPackets.push_back(processed);
        while (processedPackets.size() > MAX_DISPLAY_PACKETS) processedPackets.pop_front();
        packetBuffer.erase(oldest);
        lastReleaseTime = now;
        // Send force feedback
        ClientInfo client; bool hasClient=false;
        { std::lock_guard<std::mutex> cl(clientMutex); if (lastClient.valid) { client=lastClient; hasClient=true; } }
        if (hasClient) {
            double pos[3],frc[3],err[3],Kp;
            { std::lock_guard<std::mutex> dl(deviceMutex);
              memcpy(pos,deviceState.position,24); memcpy(frc,deviceState.force,24);
              for (int i=0;i<3;i++) err[i]=deviceState.targetPos[i]-pos[i]; }
            { std::lock_guard<std::mutex> hl(hapticMutex); Kp=hapticConfig.Kp; }
            sendFeedback(client.addr, processed.packetNumber, frc, pos, err, Kp);
        }
    }

    std::deque<Packet> getBufferedPackets()  { std::lock_guard<std::mutex> l(bufferMutex); return packetBuffer; }
    std::deque<Packet> getProcessedPackets() { std::lock_guard<std::mutex> l(processedMutex); return processedPackets; }
    std::deque<Packet> getSlidingWindow()    { std::lock_guard<std::mutex> l(windowMutex); return slidingWindow; }
    double getPacketTimeGap(int pn) { auto it=packetTimeGaps.find(pn); return (it!=packetTimeGaps.end())?it->second:0.0; }
    size_t getBufferSize()       { std::lock_guard<std::mutex> l(bufferMutex); return packetBuffer.size(); }
    size_t getSlidingWindowSize(){ std::lock_guard<std::mutex> l(windowMutex); return slidingWindow.size(); }
};
AdaptivePacketBuffer adaptiveBuffer;

double calculateAdaptiveStiffness(double jitter, double avgDelay) {
    double stiffness = STIFFNESS_K;
    if (jitter > JITTER_THRESHOLD_MS) {
        double jf = std::max(0.0, std::min(1.0, JITTER_THRESHOLD_MS / (jitter + JITTER_THRESHOLD_MS)));
        double target = MIN_STIFFNESS_K + jf * (MAX_STIFFNESS_K - MIN_STIFFNESS_K);
        stiffness = stiffness*(1.0-STIFFNESS_ADAPTATION_RATE) + target*STIFFNESS_ADAPTATION_RATE;
    } else {
        stiffness = stiffness*(1.0-STIFFNESS_ADAPTATION_RATE) + MAX_STIFFNESS_K*STIFFNESS_ADAPTATION_RATE;
    }
    return stiffness;
}

void adaptHapticParameters() {
    double s; { std::lock_guard<std::mutex> sl(statsMutex); s=globalPacketStats.adaptedStiffness; }
    std::lock_guard<std::mutex> hl(hapticMutex); hapticConfig.Kp=s;
}

std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss; struct tm ti; localtime_s(&ti,&t);
    ss << std::put_time(&ti, "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

void calculateVirtualWallForces(double pos[3], double vel[3], double lim[3], double out[3]) {
    const double k=500.0; out[0]=out[1]=out[2]=0.0;
    for (int i=0;i<3;i++) {
        if (pos[i] < -lim[i]) out[i] += k*(-lim[i]-pos[i]);
        else if (pos[i] > lim[i]) out[i] -= k*(pos[i]-lim[i]);
    }
}

void sendFeedback(const sockaddr_in& clientAddr, int packetNumber, const double force[3],
                  const double position[3], const double error[3], double stiffness) {
    if (!FEEDBACK_ENABLED) return;
    std::lock_guard<std::mutex> lock(socketMutex);
    IncomingPacket fb;
    fb.packetNumber = packetNumber;
    memcpy(fb.force,    force,    24);
    memcpy(fb.position, position, 24);
    memcpy(fb.error,    error,    24);
    fb.stiffness = stiffness;
    { std::lock_guard<std::mutex> hl(hapticMutex); fb.damping=hapticConfig.Cd; fb.isActive=hapticConfig.hapticEnabled&&hapticConfig.deviceConnected; }
    std::string ts = getCurrentTimestamp();
    strncpy_s(fb.timestamp, ts.c_str(), sizeof(fb.timestamp)-1);
    int result = sendto(recvSocket,(const char*)&fb,sizeof(fb),0,(const sockaddr*)&clientAddr,sizeof(clientAddr));
    if (result == SOCKET_ERROR) {
        std::cerr << "Feedback send failed: " << WSAGetLastError() << std::endl;
    } else {
        std::lock_guard<std::mutex> fl(feedbackMutex);
        feedbackHistory.push_back(fb);
        while (feedbackHistory.size()>MAX_FEEDBACK_HISTORY) feedbackHistory.pop_front();
        std::lock_guard<std::mutex> sl(statsMutex);
        globalPacketStats.totalFeedbackSent++;
    }
}

bool initNetwork() {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2,2),&wsaData) != 0) { std::cerr << "WSAStartup failed" << std::endl; return false; }
    std::lock_guard<std::mutex> lock(socketMutex);
    recvSocket = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    if (recvSocket==INVALID_SOCKET) { WSACleanup(); return false; }
    DWORD timeout = SOCKET_TIMEOUT_MS;
    setsockopt(recvSocket,SOL_SOCKET,SO_RCVTIMEO,(const char*)&timeout,sizeof(timeout));
    sockaddr_in serverAddr; memset(&serverAddr,0,sizeof(serverAddr));
    serverAddr.sin_family=AF_INET; serverAddr.sin_port=htons(UDP_PORT);
    if (inet_pton(AF_INET,SERVER_IP,&serverAddr.sin_addr)!=1) serverAddr.sin_addr.s_addr=INADDR_ANY;
    if (bind(recvSocket,(sockaddr*)&serverAddr,sizeof(serverAddr))==SOCKET_ERROR) {
        closesocket(recvSocket); WSACleanup(); return false;
    }
    std::cout << "Network ready — UDP port " << UDP_PORT << std::endl;
    return true;
}

void receivePackets() {
    SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_PACKET_RECEPTION);
    char buffer[MAX_PACKET_SIZE];
    while (true) {
        sockaddr_in clientAddr; int addrLen=sizeof(clientAddr);
        int bytes = recvfrom(recvSocket,buffer,MAX_PACKET_SIZE,0,(sockaddr*)&clientAddr,&addrLen);
        if (bytes <= 0) continue;
        { std::lock_guard<std::mutex> cl(clientMutex);
          lastClient.addr=clientAddr; lastClient.valid=true;
          lastClient.port=ntohs(clientAddr.sin_port);
          lastClient.lastActivity=std::chrono::steady_clock::now();
          inet_ntop(AF_INET,&clientAddr.sin_addr,lastClient.ipStr,INET_ADDRSTRLEN); }
        Packet packet; memcpy(&packet,buffer,min(bytes,(int)sizeof(Packet)));
        packet.receiveTime = std::chrono::steady_clock::now();
        bool useAdaptive;
        { std::lock_guard<std::mutex> al(adaptiveBufferMutex); useAdaptive=USE_ADAPTIVE_BUFFER; }
        if (useAdaptive) {
            adaptiveBuffer.addPacket(packet);
        } else {
            if (UNIFORM_DELAY_ENABLED && UNIFORM_DELAY_MS>0)
                std::this_thread::sleep_for(std::chrono::milliseconds(UNIFORM_DELAY_MS));
            if (RANDOM_DELAY_ENABLED)
                std::this_thread::sleep_for(std::chrono::milliseconds(randomDelayDist(gen)));
            std::this_thread::sleep_for(std::chrono::milliseconds(FIXED_PROCESSING_INTERVAL_MS));
            packet.processingDelay = std::chrono::duration<float,std::milli>(
                std::chrono::steady_clock::now()-packet.receiveTime).count();
            packet.isProcessed=true;
            { std::lock_guard<std::mutex> dl(deviceMutex);
              deviceState.targetPos[0]=packet.x; deviceState.targetPos[1]=packet.y; deviceState.targetPos[2]=packet.z; }
            adaptiveBuffer.addProcessedPacket(packet);
            ClientInfo client; bool hasClient=false;
            { std::lock_guard<std::mutex> cl(clientMutex); if (lastClient.valid) { client=lastClient; hasClient=true; } }
            if (hasClient) {
                double pos[3],frc[3],err[3],Kp;
                { std::lock_guard<std::mutex> dl(deviceMutex);
                  memcpy(pos,deviceState.position,24); memcpy(frc,deviceState.force,24);
                  for (int i=0;i<3;i++) err[i]=deviceState.targetPos[i]-pos[i]; }
                { std::lock_guard<std::mutex> hl(hapticMutex); Kp=hapticConfig.Kp; }
                sendFeedback(client.addr,packet.packetNumber,frc,pos,err,Kp);
            }
        }
        { std::lock_guard<std::mutex> sl(statsMutex);
          if (globalPacketStats.totalPacketsReceived==0) globalPacketStats.firstPacketTime=packet.receiveTime;
          if (globalPacketStats.totalPacketsReceived>0) {
              double gap=std::chrono::duration<double,std::milli>(packet.receiveTime-globalPacketStats.lastPacketTime).count();
              globalPacketStats.packetTimings.push_back(gap);
              while (globalPacketStats.packetTimings.size()>PACKET_HISTORY_SIZE) globalPacketStats.packetTimings.pop_front();
              double sum=0.0; for (auto& t:globalPacketStats.packetTimings) sum+=t;
              globalPacketStats.avgInterPacketTime=sum/globalPacketStats.packetTimings.size();
          }
          globalPacketStats.totalPacketsReceived++;
          globalPacketStats.lastPacketTime=packet.receiveTime;
          double elapsed=std::chrono::duration<double>(globalPacketStats.lastPacketTime-globalPacketStats.firstPacketTime).count();
          if (elapsed>0) globalPacketStats.avgReceiveRate=globalPacketStats.totalPacketsReceived/elapsed; }
    }
}

void processPacketsThread() {
    SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_PROCESSING);
    while (true) {
        bool useAdaptive;
        { std::lock_guard<std::mutex> al(adaptiveBufferMutex); useAdaptive=USE_ADAPTIVE_BUFFER; }
        if (useAdaptive) { adaptiveBuffer.processPackets(); adaptHapticParameters(); }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool initHapticDevice() {
    int id = dhdOpen();
    if (id < 0) {
        std::cerr << "Haptic device not found: " << dhdErrorGetLastStr() << std::endl;
        std::cout << "Continuing without haptic device (simulation mode)" << std::endl;
        std::lock_guard<std::mutex> hl(hapticMutex);
        hapticConfig.deviceConnected=false; hapticConfig.deviceID=-1;
        return false;
    }
    { std::lock_guard<std::mutex> hl(hapticMutex); hapticConfig.deviceConnected=true; hapticConfig.deviceID=id; }
    dhdEnableForce(DHD_ON);
    std::cout << "Haptic device: " << dhdGetSystemName() << std::endl;
    return true;
}

void updateHapticState() {
    SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_HAPTICS);
    while (true) {
        bool hapticEnabled; double Kp,Cd,maxForce,forceScale,posLimit[3]; bool wallsEnabled;
        { std::lock_guard<std::mutex> hl(hapticMutex);
          hapticEnabled=hapticConfig.hapticEnabled&&hapticConfig.deviceConnected;
          Kp=hapticConfig.Kp; Cd=hapticConfig.Cd; maxForce=hapticConfig.maxForce;
          forceScale=hapticConfig.forceScale; wallsEnabled=hapticConfig.wallsEnabled;
          memcpy(posLimit,hapticConfig.posLimit,24); }
        if (hapticEnabled) {
            double pos[3],target[3],vel[3];
            dhdGetPosition(&pos[0],&pos[1],&pos[2]);
            dhdGetLinearVelocity(&vel[0],&vel[1],&vel[2]);
            { std::lock_guard<std::mutex> dl(deviceMutex);
              memcpy(target,deviceState.targetPos,24);
              memcpy(deviceState.position,pos,24); memcpy(deviceState.velocity,vel,24); }
            double ctrl[3],err[3];
            for (int i=0;i<3;i++) { err[i]=target[i]-pos[i]; ctrl[i]=(Kp*err[i]-Cd*vel[i])*forceScale; }
            double wall[3]={0,0,0};
            if (wallsEnabled) calculateVirtualWallForces(pos,vel,posLimit,wall);
            double force[3]; for (int i=0;i<3;i++) force[i]=ctrl[i]+wall[i];
            double mag=sqrt(force[0]*force[0]+force[1]*force[1]+force[2]*force[2]);
            if (mag>maxForce) { double s=maxForce/mag; force[0]*=s; force[1]*=s; force[2]*=s; }
            dhdSetForce(force[0],force[1],force[2]);
            { std::lock_guard<std::mutex> dl(deviceMutex); memcpy(deviceState.force,force,24); }
        } else {
            dhdSetForce(0.0,0.0,0.0);
            std::lock_guard<std::mutex> dl(deviceMutex);
            deviceState.force[0]=deviceState.force[1]=deviceState.force[2]=0.0;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(HAPTIC_THREAD_INTERVAL_US));
    }
}

void cleanup() {
    if (dhdGetDeviceCount()>0) dhdStop();
    { std::lock_guard<std::mutex> sl(socketMutex);
      if (recvSocket!=INVALID_SOCKET) { closesocket(recvSocket); recvSocket=INVALID_SOCKET; } }
    WSACleanup();
    std::cout << "Cleanup done" << std::endl;
}

void renderText(float x, float y, const void* font, const char* str, float r=0.7f, float g=0.7f, float b=0.7f) {
    glColor3f(r,g,b); glRasterPos2f(x,y);
    for (const char* c=str; *c; c++) glutBitmapCharacter((void*)font,*c);
}

void drawRect(float x, float y, float w, float h, float r, float g, float b, float a=0.7f) {
    glColor4f(r,g,b,a); glBegin(GL_QUADS);
    glVertex2f(x,y); glVertex2f(x+w,y); glVertex2f(x+w,y+h); glVertex2f(x,y+h); glEnd();
}

void display() {
    glClearColor(0.05f,0.05f,0.1f,1.0f); glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); gluOrtho2D(-1,1,-1,1);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity(); glScalef(scaleFactor,scaleFactor,1.0f);

    int totalRx,totalTx,totalFb; double avgRate,avgIPT,jitter,adaptStiff;
    { std::lock_guard<std::mutex> sl(statsMutex);
      totalRx=globalPacketStats.totalPacketsReceived; totalTx=globalPacketStats.totalPacketsProcessed;
      totalFb=globalPacketStats.totalFeedbackSent; avgRate=globalPacketStats.avgReceiveRate;
      avgIPT=globalPacketStats.avgInterPacketTime; jitter=globalPacketStats.networkJitter;
      adaptStiff=globalPacketStats.adaptedStiffness; }

    // Title
    renderText(-0.9f, 0.92f, FONT_TITLE, "GTS Slave Server — Jitter Analysis Dashboard", 0.9f,0.9f,1.0f);
    renderText(-0.9f, 0.82f, FONT_LARGE, "IGS Group, University of Innsbruck, Austria", 0.7f,0.7f,0.9f);

    // Stats
    float y=0.72f;
    char s[128];
    snprintf(s,128,"Packets Received: %d | Processed: %d | Feedback Sent: %d",totalRx,totalTx,totalFb);
    renderText(-0.9f,y,FONT_MEDIUM,s); y-=0.06f;
    snprintf(s,128,"Receive Rate: %.1f pkt/s | Avg Inter-Packet: %.2f ms",avgRate,avgIPT);
    renderText(-0.9f,y,FONT_MEDIUM,s); y-=0.06f;
    snprintf(s,128,"Network Jitter: %.2f ms (Threshold: %.1f ms)",jitter,JITTER_THRESHOLD_MS);
    renderText(-0.9f,y,FONT_MEDIUM,s,jitter>JITTER_THRESHOLD_MS?0.9f:0.2f,jitter>JITTER_THRESHOLD_MS?0.2f:0.9f,0.2f); y-=0.06f;
    snprintf(s,128,"Adaptive Stiffness: %.2f N/m (Range: %.0f–%.0f)",adaptStiff,MIN_STIFFNESS_K,MAX_STIFFNESS_K);
    renderText(-0.9f,y,FONT_MEDIUM,s); y-=0.06f;

    // Jitter bar
    float bx=-0.9f, by=y-0.04f, bw=1.2f, bh=0.04f;
    drawRect(bx,by,bw,bh,0.2f,0.2f,0.2f);
    float fill=std::min((float)(jitter/50.0)*bw,bw);
    drawRect(bx,by,fill,bh,jitter>JITTER_THRESHOLD_MS?0.9f:0.2f,jitter>JITTER_THRESHOLD_MS?0.2f:0.9f,0.2f);
    float tx=bx+(JITTER_THRESHOLD_MS/50.0)*bw;
    glColor3f(1,1,1); glLineWidth(2); glBegin(GL_LINES); glVertex2f(tx,by-0.01f); glVertex2f(tx,by+bh+0.01f); glEnd();
    renderText(bx,by-0.06f,FONT_SMALL,"0 ms"); renderText(bx+bw-0.08f,by-0.06f,FONT_SMALL,"50+ ms");

    // Haptic status
    bool hconn,henbl,walls; double Kp,Cd;
    { std::lock_guard<std::mutex> hl(hapticMutex); hconn=hapticConfig.deviceConnected; henbl=hapticConfig.hapticEnabled; walls=hapticConfig.wallsEnabled; Kp=hapticConfig.Kp; Cd=hapticConfig.Cd; }
    renderText(0.3f,0.72f,FONT_LARGE,"Haptic Device Status",0.9f,0.9f,1.0f);
    renderText(0.3f,0.64f,FONT_MEDIUM,hconn?"Device: Connected":"Device: Not Connected",hconn?0.2f:0.9f,hconn?0.9f:0.2f,0.2f);
    snprintf(s,128,"Stiffness: %.2f N/m | Damping: %.2f N·s/m",Kp,Cd);
    renderText(0.3f,0.56f,FONT_MEDIUM,s);
    renderText(0.3f,0.48f,FONT_MEDIUM,walls?"Virtual Walls: ON":"Virtual Walls: OFF",walls?0.2f:0.9f,walls?0.9f:0.2f,0.2f);

    // Buffer info
    bool useAdaptive; { std::lock_guard<std::mutex> al(adaptiveBufferMutex); useAdaptive=USE_ADAPTIVE_BUFFER; }
    snprintf(s,128,"Buffer: %s | Buffer Size: %zu | Window: %zu",
        useAdaptive?"Adaptive (RH-Buffer)":"Fixed Delay",adaptiveBuffer.getBufferSize(),adaptiveBuffer.getSlidingWindowSize());
    renderText(-0.9f,-0.85f,FONT_MEDIUM,s);
    renderText(-0.9f,-0.92f,FONT_SMALL,"Keys: T=AdaptiveBuffer | H=HapticFeedback | W=VirtualWalls | K/I=Stiffness | D/E=Damping | F=Fullscreen | Esc=Exit");

    glutSwapBuffers();
}

void reshape(int w, int h) { windowWidth=w; windowHeight=h; glViewport(0,0,w,h); }
void timerCallback(int v) { glutPostRedisplay(); glutTimerFunc(16,timerCallback,0); }

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case 27: cleanup(); exit(0); break;
    case 'p': case 'P': USE_FIXED_PROCESSING_RATE=!USE_FIXED_PROCESSING_RATE; break;
    case 'o': case 'O': FIXED_PROCESSING_INTERVAL_MS++; break;
    case 'q': case 'Q': if (FIXED_PROCESSING_INTERVAL_MS>1) FIXED_PROCESSING_INTERVAL_MS--; break;
    case 'f': case 'F': fullscreen=!fullscreen; if (fullscreen) glutFullScreen(); else glutReshapeWindow(1280,800); break;
    case 'h': case 'H': { std::lock_guard<std::mutex> hl(hapticMutex); hapticConfig.hapticEnabled=!hapticConfig.hapticEnabled; } break;
    case 'w': case 'W': { std::lock_guard<std::mutex> hl(hapticMutex); hapticConfig.wallsEnabled=!hapticConfig.wallsEnabled; } break;
    case 's': case 'S': showStats=!showStats; break;
    case 'g': case 'G': showGraphs=!showGraphs; break;
    case 'j': case 'J': showJitterAnalysis=!showJitterAnalysis; break;
    case 't': case 'T': { std::lock_guard<std::mutex> al(adaptiveBufferMutex); USE_ADAPTIVE_BUFFER=!USE_ADAPTIVE_BUFFER; } break;
    case 'u': case 'U': UNIFORM_DELAY_ENABLED=!UNIFORM_DELAY_ENABLED; break;
    case 'r': case 'R': RANDOM_DELAY_ENABLED=!RANDOM_DELAY_ENABLED; break;
    case 'k': case 'K': { std::lock_guard<std::mutex> hl(hapticMutex); hapticConfig.Kp*=1.1; } break;
    case 'i': case 'I': { std::lock_guard<std::mutex> hl(hapticMutex); hapticConfig.Kp*=0.9; } break;
    case 'd': case 'D': { std::lock_guard<std::mutex> hl(hapticMutex); hapticConfig.Cd*=1.1; } break;
    case 'e': case 'E': { std::lock_guard<std::mutex> hl(hapticMutex); hapticConfig.Cd*=0.9; } break;
    case '1':case '2':case '3':case '4':case '5':case '6':case '7':case '8':case '9':
        GLOBAL_RELEASE_WIDTH_FACTOR=(float)(key-'0'); break;
    }
    glutPostRedisplay();
}

void specialKeyboard(int key, int x, int y) {
    if (key==GLUT_KEY_F1) showHelp=!showHelp;
    glutPostRedisplay();
}

int main(int argc, char** argv) {
    std::cout << "=================================================" << std::endl;
    std::cout << "GTS Slave Server — IGS Group, Innsbruck, Austria" << std::endl;
    std::cout << "Intercontinental Haptic Teleoperation Framework"  << std::endl;
    std::cout << "M.Engg. Thesis: Humayun Khan, HHRCM NCRA-NEDUET" << std::endl;
    std::cout << "UDP Port: " << UDP_PORT << " | RH-Buffer Window: " << SLIDING_WINDOW_SIZE << std::endl;
    std::cout << "Jitter Threshold: " << JITTER_THRESHOLD_MS << " ms | Press F1 for help" << std::endl;
    std::cout << "=================================================" << std::endl;

    glutInit(&argc,argv); glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE);
    glutInitWindowSize(windowWidth,windowHeight);
    glutCreateWindow("GTS Slave Server — Jitter Analysis Dashboard");
    glutDisplayFunc(display); glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard); glutSpecialFunc(specialKeyboard);
    glutTimerFunc(16,timerCallback,0);

    if (!initNetwork()) { std::cerr << "Network init failed" << std::endl; return 1; }
    initHapticDevice();

    std::thread recvThread(receivePackets);       recvThread.detach();
    std::thread procThread(processPacketsThread); procThread.detach();
    std::thread hapticThread(updateHapticState);  hapticThread.detach();

    glutMainLoop();
    return 0;
}
