/**
 * Generalized Haptic Teleoperation Setup — GTS
 * Master Client (Karachi, Pakistan)
 * Version: 1.0
 *
 * CHAI3D-based master haptic client for bilateral force-feedback
 * teleoperation over WAN. Sends 6-DOF position packets to slave
 * at 1000 Hz, receives force feedback, and renders it via
 * Phantom TouchX / Desktop haptic device.
 *
 * Features:
 *   - 1000 Hz haptic control loop (TIME_CRITICAL priority)
 *   - Adaptive damping based on force error variance
 *   - GLFW 3D virtual environment (CHAI3D scene)
 *   - GLUT real-time stats dashboard
 *   - Dual UDP socket with fallback port
 *   - Smith Predictor compatible force rendering
 *   - Gripper button toggles Network / Virtual Environment mode
 *
 * CONFIGURATION:
 *   Replace CLIENT_IP and SERVER_IP with your machine IPs.
 *   Update CHAI3D include/library paths for your installation.
 *   Build: Visual Studio + CHAI3D 3.2.0 + GLFW + GLEW + freeglut
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
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mmsystem.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
// Update this path to match your CHAI3D installation:
#include <GL/glut.h>
#include "chai3d.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <mutex>
#include <chrono>
#include <string>
#include <vector>
#include <numeric>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <string.h>
#include <random>

#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "winmm.lib")
#pragma comment(lib, "chai3d.lib")
#pragma comment(lib, "freeglut.lib")

using namespace chai3d;
using namespace std;

template<typename T>
string to_string_with_precision(const T a_value, const int n) {
    ostringstream out;
    out << fixed << setprecision(n) << a_value;
    return out.str();
}

int windowedWidth  = 1280;
int windowedHeight = 720;
int windowedPosX   = 100;
int windowedPosY   = 100;

double maxAdaptiveDamping    = 0.0;
double maxAdaptiveStiffness  = 0.0;
const int ADAPTIVE_LIMIT_PACKET_COUNT = 100;
bool adaptiveLimited = false;

// ============================================================
// NETWORK CONFIGURATION — replace before deployment
// ============================================================
const char CLIENT_IP[] = "YOUR_MASTER_IP";   // Master machine IP
const char SERVER_IP[] = "YOUR_SLAVE_IP";    // Slave machine IP
const int SEND_PORT    = 4000;
const int RECV_PORT    = 4001;
const int FALLBACK_PORT = 4000;
const int BUFFER_SIZE  = 5000;
const int SOCKET_TIMEOUT_MS   = 1;
const int SOCKET_RCVBUF_SIZE  = 65536;

// ============================================================
// HAPTIC CONFIGURATION
// ============================================================
const int    HAPTIC_RATE_HZ   = 1000;
const double MAX_FORCE        = 5.0;
int          FORCE_PACKET_WINDOW = 10;
const int    ADAPTIVE_UPDATE_MS  = 100;
const bool   USE_ADAPTIVE_DAMPING = true;
const double ERROR_VARIANCE_SCALE = 0.5;
const double MAX_DAMPING      = 10.0;
const double MAX_STIFFNESS    = 1000.0;

// ============================================================
// VIRTUAL ENVIRONMENT
// ============================================================
const int    NUM_SPHERES      = 2;
const double SPHERE_RADIUS    = 0.007;
const double SPHERES_RADIUS   = 0.007;
const double SPHERE_STIFFNESS = 1000.0;
const double SPHERE_MASS      = 0.04;
const double K_DAMPING        = 0.996;
const double K_MAGNET         = 500.0;
const double HAPTIC_STIFFNESS = 500.0;

cStereoMode stereoMode   = C_STEREO_DISABLED;
bool fullscreen          = false;
bool mirroredDisplay     = false;
string resourceRoot      = "../../../bin/";

// Control flags
bool sendPacketsEnabled    = true;
bool receiveFeedbackEnabled = true;
bool applyHapticFeedback   = true;
double packetDelayMin      = 0.0;
double packetDelayMax      = 0.0;
int    maxPacketsToSend    = 0;
double packetIntervalMs    = 1000.0 / HAPTIC_RATE_HZ;
bool rawModeEnabled        = false;
mutex controlMutex;

// Graph / stats
const int GRAPH_WINDOW = 60;
mutex forceFeedbackMutex;
cVector3d forceFeedback(0.0, 0.0, 0.0);
bool newFeedback = false;
vector<int>    sentPacketCounts(GRAPH_WINDOW, 0);
vector<int>    receivedPacketCounts(GRAPH_WINDOW, 0);
vector<double> sendDelays(GRAPH_WINDOW, 0.0);
vector<double> receiveDelays(GRAPH_WINDOW, 0.0);

struct PacketData {
    double magnitude;
    chrono::steady_clock::time_point timestamp;
    double delay;
    int packetNumber;
};
const int MAX_PACKETS_DISPLAY = 50;
vector<PacketData> sentPackets;
vector<PacketData> receivedPackets;
mutex packetHistoryMutex;
mutex graphMutex;
chrono::steady_clock::time_point graphStartTime;

int networkThreadPriority = THREAD_PRIORITY_ABOVE_NORMAL;
int hapticThreadPriority  = THREAD_PRIORITY_ABOVE_NORMAL;

bool newForceFeedback = false;
int  packetCount      = 0;
int  successfulSends  = 0;
int  successfulReceives = 0;
int  failedSends      = 0;
int  failedReceives   = 0;
int  timeoutReceives  = 0;
double lastStiffness  = 0.0;
double lastDamping    = 0.0;
double adaptiveStiffness = 0.0;
double adaptiveDamping   = 0.0;
cVector3d lastError(0.0, 0.0, 0.0);
double lastPacketTime = 0.0;
double avgLatency     = 0.0;
int latencyCount      = 0;

volatile bool running          = true;
volatile bool hapticRunning    = true;
bool useVirtualEnvironment     = false;
bool simulationRunning         = false;
bool simulationFinished        = true;

vector<cVector3d> forceBuffer;
vector<double>    stiffnessBuffer;
vector<double>    dampingBuffer;
vector<cVector3d> errorBuffer;
mutex bufferMutex;

SOCKET sock         = INVALID_SOCKET;
SOCKET fallbackSock = INVALID_SOCKET;

cHapticDeviceHandler*    handler     = nullptr;
cGenericHapticDevicePtr  hapticDevice = nullptr;
cFrequencyCounter freqCounterHaptics;
double hapticDeviceMaxStiffness = 0.0;

cWorld*       world      = nullptr;
cCamera*      camera     = nullptr;
cSpotLight*   light      = nullptr;
cShapeSphere* spheres[NUM_SPHERES];
cMesh*        plane      = nullptr;
cVector3d     sphereVel[NUM_SPHERES];
cBackground*  background = nullptr;
cFontPtr      font       = nullptr;
cLabel*       labelRates   = nullptr;
cLabel*       labelMessage = nullptr;
GLFWwindow*   window     = nullptr;
int width = 0, height = 0;
int swapInterval = 1;
cFrequencyCounter freqCounterGraphics;
int glutWindow  = 0;
int glutWidth   = 1960;
int glutHeight  = 1200;
ofstream logFile("GTS-Master.log");

// Colour palette
const float BG_COLOR[4]    = { 0.10f, 0.11f, 0.15f, 1.0f };
const float PANEL_COLOR[4] = { 0.16f, 0.17f, 0.21f, 0.97f };
const float PANEL_SUBTLE[4]= { 0.12f, 0.13f, 0.18f, 0.87f };
const float GRID_COLOR[3]  = { 0.23f, 0.27f, 0.34f };
const float TEXT_TITLE[3]  = { 0.99f, 0.99f, 0.99f };
const float TEXT_BODY[3]   = { 0.82f, 0.86f, 0.95f };
const float STATUS_ON[3]   = { 0.14f, 0.97f, 0.31f };
const float STATUS_OFF[3]  = { 0.94f, 0.21f, 0.26f };
const float BAR_SENT[3]    = { 0.24f, 0.52f, 0.98f };
const float BAR_RECV[3]    = { 0.96f, 0.34f, 0.34f };

const float MARGIN         = 10.0f;
const float TEXT_HEIGHT    = 15.0f;
const float GRAPH_HEIGHT   = 150.0f;
const float BAR_WIDTH      = 10.0f;
const float STATUS_RADIUS  = 8.0f;
void* LABEL_FONT = GLUT_BITMAP_HELVETICA_12;
void* TITLE_FONT = GLUT_BITMAP_HELVETICA_18;

struct OutgoingPacket {
    int   packetNumber;
    float x, y, z;
    float delay;
    char  timestamp[64];
    char  srcIP[INET_ADDRSTRLEN];
    char  dstIP[INET_ADDRSTRLEN];
    int   srcPort;
    int   dstPort;
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

// Function prototypes
void initializeWinsock();
void setupSocket(SOCKET& s, int port, int maxRetries = 3);
void networkCommunication();
void hapticLoop();
void glutGUI();
void glutDisplay();
void glutReshape(int w, int h);
void glutKeyboard(unsigned char key, int x, int y);
void cleanupSocket();
void getCurrentTimestamp(char* buffer, size_t size);
void handleError(const string& message);
void logRawPacket(const char* buffer, int size);
void updateAdaptiveParameters();
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);
void errorCallback(int a_error, const char* a_description);
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);
void updateGraphics();
void close();
void logMessage(const string& message);
void drawTextModern(float x, float y, const std::string& text, void* font, const float col[3], float angle = 0.0f);
void drawRoundedRect(float x, float y, float w, float h, float r, const float col[4]);
void drawBar(float x, float y, float w, float h, const float color[3], float alpha = 1.0f);
void drawCircle(float x, float y, float radius, const float col[3]);
void drawGrid(float x, float y, float width, float height, int numHLines, const float color[3]);
void timerCallback(int value);

#define RESOURCE_PATH(p) (char*)((resourceRoot+string(p)).c_str())

void logMessage(const string& message) {
    auto now  = chrono::system_clock::now();
    auto time = chrono::system_clock::to_time_t(now);
    struct tm ts;
    localtime_s(&ts, &time);
    char timeStr[64];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &ts);
    string entry = "[" + string(timeStr) + "] " + message + "\n";
    cout << entry;
    if (logFile.is_open()) { logFile << entry; logFile.flush(); }
}

void handleError(const string& message) { logMessage("Error: " + message); }

void getCurrentTimestamp(char* buffer, size_t size) {
    auto now  = chrono::system_clock::now();
    auto time = chrono::system_clock::to_time_t(now);
    struct tm ts;
    localtime_s(&ts, &time);
    strftime(buffer, size, "%Y-%m-%d %H:%M:%S", &ts);
}

void initializeWinsock() {
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0) { handleError("WSAStartup failed: " + to_string(result)); return; }
    logMessage("Winsock initialised");
}

void setupSocket(SOCKET& s, int port, int maxRetries) {
    for (int attempt = 1; attempt <= maxRetries; ++attempt) {
        s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s == INVALID_SOCKET) {
            if (attempt == maxRetries) handleError("Socket creation failed");
            continue;
        }
        sockaddr_in addr; memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);
        if (inet_pton(AF_INET, CLIENT_IP, &addr.sin_addr) <= 0) {
            closesocket(s);
            if (attempt == maxRetries) handleError("Invalid client IP");
            continue;
        }
        if (::bind(s, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
            closesocket(s);
            if (attempt == maxRetries) handleError("Bind failed on port " + to_string(port));
            continue;
        }
        int rcvbuf = SOCKET_RCVBUF_SIZE;
        setsockopt(s, SOL_SOCKET, SO_RCVBUF, (char*)&rcvbuf, sizeof(rcvbuf));
        logMessage("Socket bound to " + string(CLIENT_IP) + ":" + to_string(port));
        return;
    }
}

void cleanupSocket() {
    if (sock != INVALID_SOCKET)         { closesocket(sock);         sock = INVALID_SOCKET; }
    if (fallbackSock != INVALID_SOCKET) { closesocket(fallbackSock); fallbackSock = INVALID_SOCKET; }
    WSACleanup();
}

void logRawPacket(const char* buffer, int size) {
    stringstream ss;
    ss << "Raw (" << size << " bytes): ";
    for (int i = 0; i < min(size, 32); i++)
        ss << hex << setfill('0') << setw(2) << (unsigned char)buffer[i] << " ";
    if (size > 32) ss << "...";
    logMessage(ss.str());
}

void updateAdaptiveParameters() {
    lock_guard<mutex> lock(bufferMutex);
    if (errorBuffer.empty()) { adaptiveDamping = 0.0; adaptiveStiffness = 0.0; return; }
    if (adaptiveLimited) {
        adaptiveDamping   = maxAdaptiveDamping;
        adaptiveStiffness = maxAdaptiveStiffness;
        return;
    }
    cVector3d meanError(0.0, 0.0, 0.0);
    for (const auto& err : errorBuffer) meanError += err;
    meanError /= static_cast<double>(errorBuffer.size());
    cVector3d variance(0.0, 0.0, 0.0);
    for (const auto& err : errorBuffer) {
        cVector3d diff = err - meanError;
        variance += cVector3d(diff.x()*diff.x(), diff.y()*diff.y(), diff.z()*diff.z());
    }
    variance /= static_cast<double>(errorBuffer.size());
    double totalVariance = variance.length();
    double avgDamping   = dampingBuffer.empty()   ? 0.0 : accumulate(dampingBuffer.begin(),   dampingBuffer.end(),   0.0) / dampingBuffer.size();
    double avgStiffness = stiffnessBuffer.empty() ? 0.0 : accumulate(stiffnessBuffer.begin(), stiffnessBuffer.end(), 0.0) / stiffnessBuffer.size();
    adaptiveDamping   = std::max(0.0, std::min(avgDamping   * (1.0 - ERROR_VARIANCE_SCALE * totalVariance), MAX_DAMPING));
    adaptiveStiffness = std::max(0.0, std::min(avgStiffness * (1.0 - ERROR_VARIANCE_SCALE * totalVariance), MAX_STIFFNESS));
    maxAdaptiveDamping   = std::max(maxAdaptiveDamping,   adaptiveDamping);
    maxAdaptiveStiffness = std::max(maxAdaptiveStiffness, adaptiveStiffness);
    if (packetCount >= ADAPTIVE_LIMIT_PACKET_COUNT) adaptiveLimited = true;
}

void networkCommunication() {
    logMessage("Network thread started");
    sockaddr_in serverAddr; memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port   = htons(SEND_PORT);
    if (inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr) <= 0) {
        handleError("Invalid server IP"); running = false; return;
    }
    char recvBuffer[BUFFER_SIZE];
    sockaddr_in senderAddr; int senderAddrSize = sizeof(senderAddr);
    SetThreadPriority(GetCurrentThread(), networkThreadPriority);
    fd_set readSet; timeval timeout; timeout.tv_sec = 0; timeout.tv_usec = SOCKET_TIMEOUT_MS * 1000;
    auto lastAdaptiveUpdate = chrono::steady_clock::now();
    auto lastSendTime       = chrono::steady_clock::now();
    auto lastGraphUpdate    = chrono::steady_clock::now();
    auto prevSendTime       = lastSendTime;
    random_device rd; mt19937 gen(rd());
    vector<int>    tempSentCounts(GRAPH_WINDOW, 0);
    vector<int>    tempReceivedCounts(GRAPH_WINDOW, 0);
    vector<double> tempSendDelays(GRAPH_WINDOW, 0.0);
    vector<double> tempReceiveDelays(GRAPH_WINDOW, 0.0);

    while (running) {
        auto loopStart = chrono::high_resolution_clock::now();
        bool shouldSend = false; double currentInterval = packetIntervalMs; bool currentRawMode = false;
        {
            lock_guard<mutex> lock(controlMutex);
            shouldSend = sendPacketsEnabled && (maxPacketsToSend == 0 || packetCount < maxPacketsToSend);
            currentInterval = rawModeEnabled ? 0.0 : packetIntervalMs;
            currentRawMode = rawModeEnabled;
        }
        if (shouldSend || currentRawMode) {
            auto now = chrono::steady_clock::now();
            auto elapsedMs = chrono::duration_cast<chrono::milliseconds>(now - lastSendTime).count();
            if (elapsedMs >= currentInterval) {
                cVector3d position;
                bool validPosition = hapticDevice ? hapticDevice->getPosition(position) : false;
                if (validPosition) {
                    OutgoingPacket packet; memset(&packet, 0, sizeof(packet));
                    packet.packetNumber = ++packetCount;
                    packet.x = static_cast<float>(position.x());
                    packet.y = static_cast<float>(position.y());
                    packet.z = static_cast<float>(position.z());
                    packet.delay = currentRawMode ? 0.0f : 1000.0f / HAPTIC_RATE_HZ;
                    getCurrentTimestamp(packet.timestamp, sizeof(packet.timestamp));
                    strcpy_s(packet.srcIP, CLIENT_IP);
                    strcpy_s(packet.dstIP, SERVER_IP);
                    packet.srcPort = SEND_PORT; packet.dstPort = SEND_PORT;
                    int sendResult = sendto(sock, (char*)&packet, sizeof(packet), 0, (sockaddr*)&serverAddr, sizeof(serverAddr));
                    if (sendResult == SOCKET_ERROR) {
                        failedSends++;
                        int error = WSAGetLastError();
                        if (error == WSAECONNRESET || error == WSAENOTCONN) { closesocket(sock); setupSocket(sock, RECV_PORT, 1); }
                    } else {
                        successfulSends++;
                        lastPacketTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
                        double magnitude = position.length();
                        double sendDelay = chrono::duration_cast<chrono::microseconds>(now - prevSendTime).count() / 1e3;
                        { lock_guard<mutex> lock(packetHistoryMutex);
                          sentPackets.push_back({ magnitude, now, sendDelay, packet.packetNumber });
                          if (sentPackets.size() > MAX_PACKETS_DISPLAY) sentPackets.erase(sentPackets.begin()); }
                        prevSendTime = now;
                    }
                    lastSendTime = now;
                    if (!currentRawMode && packetDelayMax > packetDelayMin) {
                        uniform_real_distribution<double> dist(packetDelayMin, packetDelayMax);
                        this_thread::sleep_for(chrono::microseconds(static_cast<int>(dist(gen) * 1000)));
                    }
                } else { failedSends++; }
            }
        }
        // Receive force feedback
        bool shouldReceive = false;
        { lock_guard<mutex> lock(controlMutex); shouldReceive = receiveFeedbackEnabled || rawModeEnabled; }
        if (shouldReceive) {
            auto receiveFromSocket = [&](SOCKET& s, int port) {
                FD_ZERO(&readSet); FD_SET(s, &readSet);
                if (select(0, &readSet, nullptr, nullptr, &timeout) > 0 && FD_ISSET(s, &readSet)) {
                    int recvResult = recvfrom(s, recvBuffer, BUFFER_SIZE, 0, (sockaddr*)&senderAddr, &senderAddrSize);
                    if (recvResult >= (int)sizeof(IncomingPacket)) {
                        auto recvTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
                        auto now = chrono::steady_clock::now();
                        IncomingPacket* packet = reinterpret_cast<IncomingPacket*>(recvBuffer);
                        if (packet->isActive) {
                            cVector3d force(packet->force[0], packet->force[1], packet->force[2]);
                            if (force.length() > MAX_FORCE) force = force * (MAX_FORCE / force.length());
                            { lock_guard<mutex> lock(forceFeedbackMutex);
                              lock_guard<mutex> bufferLock(bufferMutex);
                              forceBuffer.push_back(force);
                              stiffnessBuffer.push_back(packet->stiffness);
                              dampingBuffer.push_back(packet->damping);
                              errorBuffer.push_back(cVector3d(packet->error[0], packet->error[1], packet->error[2]));
                              if (forceBuffer.size() > FORCE_PACKET_WINDOW) {
                                  forceBuffer.erase(forceBuffer.begin());
                                  stiffnessBuffer.erase(stiffnessBuffer.begin());
                                  dampingBuffer.erase(dampingBuffer.begin());
                                  errorBuffer.erase(errorBuffer.begin());
                              }
                              cVector3d avg(0,0,0);
                              for (const auto& f : forceBuffer) avg += f;
                              forceFeedback = avg / static_cast<double>(forceBuffer.size());
                            }
                            lastStiffness = packet->stiffness;
                            lastDamping   = packet->damping;
                            lastError.set(packet->error[0], packet->error[1], packet->error[2]);
                            double latency = recvTime - lastPacketTime;
                            { lock_guard<mutex> lock(packetHistoryMutex);
                              receivedPackets.push_back({ force.length(), now, latency >= 0 ? latency : 0.0, packet->packetNumber });
                              if (receivedPackets.size() > MAX_PACKETS_DISPLAY) receivedPackets.erase(receivedPackets.begin()); }
                            if (packet->packetNumber == packetCount) {
                                avgLatency = latencyCount == 0 ? latency : (avgLatency * latencyCount + latency) / (latencyCount + 1);
                                latencyCount++;
                            }
                            successfulReceives++;
                            newForceFeedback = true;
                        }
                    }
                }
            };
            receiveFromSocket(sock, RECV_PORT);
            receiveFromSocket(fallbackSock, FALLBACK_PORT);
        }

        // Update graph data
        auto now = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(now - lastGraphUpdate).count() >= 100) {
            lock_guard<mutex> lock(graphMutex);
            sentPacketCounts     = tempSentCounts;
            receivedPacketCounts = tempReceivedCounts;
            sendDelays           = tempSendDelays;
            receiveDelays        = tempReceiveDelays;
            lastGraphUpdate = now;
        }
        if (chrono::duration_cast<chrono::seconds>(now - graphStartTime).count() >= GRAPH_WINDOW) {
            lock_guard<mutex> lock(graphMutex);
            fill(tempSentCounts.begin(),     tempSentCounts.end(),     0);
            fill(tempReceivedCounts.begin(), tempReceivedCounts.end(), 0);
            fill(tempSendDelays.begin(),     tempSendDelays.end(),     0.0);
            fill(tempReceiveDelays.begin(),  tempReceiveDelays.end(),  0.0);
            fill(sentPacketCounts.begin(),     sentPacketCounts.end(),     0);
            fill(receivedPacketCounts.begin(), receivedPacketCounts.end(), 0);
            fill(sendDelays.begin(),           sendDelays.end(),           0.0);
            fill(receiveDelays.begin(),        receiveDelays.end(),        0.0);
            graphStartTime = now; lastGraphUpdate = now;
        }
        if (chrono::duration_cast<chrono::milliseconds>(now - lastAdaptiveUpdate).count() >= ADAPTIVE_UPDATE_MS && USE_ADAPTIVE_DAMPING) {
            updateAdaptiveParameters();
            lastAdaptiveUpdate = now;
        }
        auto elapsed = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - loopStart);
        auto sleepTime = chrono::microseconds(1000000 / HAPTIC_RATE_HZ) - elapsed;
        if (sleepTime.count() > 0 && !currentRawMode) this_thread::sleep_for(sleepTime);
    }
    logMessage("Network thread exiting");
}

void hapticLoop() {
    logMessage("Haptic thread started");
    SetThreadPriority(GetCurrentThread(), hapticThreadPriority);
    cVector3d lastVelocity(0.0, 0.0, 0.0);
    bool flagHapticsEnabled = false;
    cPrecisionClock clock; clock.reset();
    simulationRunning = true; simulationFinished = false;
    if (hapticDevice) { hapticDevice->open(); hapticDevice->calibrate(); }
    double envStiffness = SPHERE_STIFFNESS;
    double envDamping   = K_DAMPING;

    while (hapticRunning) {
        auto loopStart = chrono::high_resolution_clock::now();
        clock.stop();
        double timeInterval = cMin(0.001, clock.getCurrentTimeSeconds());
        clock.reset(); clock.start();
        cVector3d force(0.0, 0.0, 0.0);
        double damping = 0.0;
        bool button = false;
        if (hapticDevice) hapticDevice->getUserSwitch(0, button);
        if (button) {
            useVirtualEnvironment = !useVirtualEnvironment;
            while (hapticDevice && hapticDevice->getUserSwitch(0, button) && button) cSleepMs(10);
        }
        if (useVirtualEnvironment) {
            cVector3d position;
            if (hapticDevice && hapticDevice->getPosition(position)) {
                { lock_guard<mutex> lock(forceFeedbackMutex);
                  if (newForceFeedback) { envStiffness = lastStiffness; envDamping = lastDamping; } }
                cVector3d sphereFce[NUM_SPHERES];
                for (int i = 0; i < NUM_SPHERES; i++) sphereFce[i].zero();
                for (int i = 0; i < NUM_SPHERES; i++) {
                    cVector3d frc, pos0 = spheres[i]->getLocalPos();
                    for (int j = i+1; j < NUM_SPHERES; j++) {
                        frc.zero();
                        cVector3d pos1 = spheres[j]->getLocalPos();
                        cVector3d dir01 = pos1 - pos0;
                        double distance = dir01.length();
                        if (distance > 1e-8) dir01 /= distance; else dir01.zero();
                        if (!button && distance < 2.5*SPHERE_RADIUS && distance > 2.0*SPHERE_RADIUS)
                            frc += (-K_MAGNET * (distance - 2.5*SPHERE_RADIUS) * dir01);
                        if (distance < 2.0*SPHERES_RADIUS)
                            frc += (envStiffness * (distance - 2.0*SPHERE_RADIUS) * dir01);
                        sphereFce[i] += frc; sphereFce[j] += (-frc);
                    }
                    const double WG=0.0+SPHERE_RADIUS, WL=-0.1, WR=0.2, WF=0.08, WB=-0.08;
                    if (pos0.z()<WG) sphereFce[i] += cVector3d(0,0,envStiffness*(WG-pos0.z()));
                    if (pos0.y()<WL) sphereFce[i] += cVector3d(0,envStiffness*(WL-pos0.y()),0);
                    if (pos0.y()>WR) sphereFce[i] += cVector3d(0,envStiffness*(WR-pos0.y()),0);
                    if (pos0.x()<WB) sphereFce[i] += cVector3d(envStiffness*(WB-pos0.x()),0,0);
                    if (pos0.x()>WF) sphereFce[i] += cVector3d(envStiffness*(WF-pos0.x()),0,0);
                }
                cVector3d dirHS = spheres[0]->getLocalPos() - position;
                force = HAPTIC_STIFFNESS * dirHS;
                sphereFce[0] += (-envStiffness * dirHS);
                for (int i = 0; i < NUM_SPHERES; i++) {
                    cVector3d acc = (sphereFce[i] / SPHERE_MASS) + cVector3d(0,0,-9.81);
                    sphereVel[i] = envDamping * (sphereVel[i] + timeInterval * acc);
                    spheres[i]->setLocalPos(spheres[i]->getLocalPos() + timeInterval*sphereVel[i] + timeInterval*timeInterval*acc);
                }
                if (!flagHapticsEnabled) { if (force.length() < 1.0) flagHapticsEnabled = true; else force.zero(); }
                if (hapticDeviceMaxStiffness < HAPTIC_STIFFNESS) force *= hapticDeviceMaxStiffness / HAPTIC_STIFFNESS;
            }
        } else {
            bool shouldApply = false;
            { lock_guard<mutex> lock(controlMutex); shouldApply = applyHapticFeedback || rawModeEnabled; }
            if (shouldApply && forceFeedbackMutex.try_lock()) {
                cVector3d tempForce = forceFeedback;
                bool fb = newForceFeedback;
                damping = USE_ADAPTIVE_DAMPING ? adaptiveDamping : lastDamping;
                forceFeedbackMutex.unlock();
                if (fb) force = tempForce;
            }
        }
        if (hapticDevice) {
            cVector3d velocity;
            if (hapticDevice->getLinearVelocity(velocity)) {
                if (!useVirtualEnvironment && newForceFeedback && (applyHapticFeedback || rawModeEnabled))
                    force -= damping * velocity;
                lastVelocity = velocity;
            }
            hapticDevice->setForce(force.length() > 0 ? force : cVector3d(0,0,0));
        }
        freqCounterHaptics.signal(1);
        auto elapsed = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - loopStart);
        auto sleepTime = chrono::microseconds(1000000 / HAPTIC_RATE_HZ) - elapsed;
        if (sleepTime.count() > 0 && !rawModeEnabled) this_thread::sleep_for(sleepTime);
    }
    if (hapticDevice) hapticDevice->close();
    simulationFinished = true;
    logMessage("Haptic thread exiting");
}

void drawTextModern(float x, float y, const std::string& text, void* font, const float col[3], float angle) {
    glPushMatrix(); glColor3fv(col);
    glTranslatef(x, y, 0.0f); glRotatef(angle, 0.0f, 0.0f, 1.0f);
    glRasterPos2f(0.0f, 0.0f);
    for (char c : text) glutBitmapCharacter(font, c);
    glPopMatrix();
}
void drawRoundedRect(float x, float y, float w, float h, float r, const float col[4]) {
    glColor4fv(col); glBegin(GL_POLYGON);
    for (int i=0;i<=16;i++) glVertex2f(x+r+cosf(i*3.14159f/32.0f)*r, y+h-r+sinf(i*3.14159f/32.0f)*r);
    for (int i=16;i<=32;i++) glVertex2f(x+w-r+cosf(i*3.14159f/32.0f)*r, y+h-r+sinf(i*3.14159f/32.0f)*r);
    for (int i=32;i<=48;i++) glVertex2f(x+w-r+cosf(i*3.14159f/32.0f)*r, y+r+sinf(i*3.14159f/32.0f)*r);
    for (int i=48;i<=64;i++) glVertex2f(x+r+cosf(i*3.14159f/32.0f)*r, y+r+sinf(i*3.14159f/32.0f)*r);
    glEnd();
}
void drawBar(float x, float y, float w, float h, const float color[3], float alpha) {
    glColor4f(color[0],color[1],color[2],alpha);
    glBegin(GL_QUADS); glVertex2f(x,y); glVertex2f(x+w,y); glVertex2f(x+w,y+h); glVertex2f(x,y+h); glEnd();
}
void drawCircle(float x, float y, float radius, const float col[3]) {
    glColor3fv(col); glBegin(GL_TRIANGLE_FAN);
    for (int i=0;i<360;i+=15) { float rad=i*3.1415926f/180.0f; glVertex2f(x+cos(rad)*radius, y+sin(rad)*radius); }
    glEnd();
}
void drawGrid(float x, float y, float width, float height, int n, const float color[3]) {
    glColor3fv(color); glBegin(GL_LINES);
    for (int i=1;i<n;i++) { float ly=y+(i*height/n); glVertex2f(x,ly); glVertex2f(x+width,ly); }
    glEnd();
}

void glutDisplay() {
    glClearColor(BG_COLOR[0],BG_COLOR[1],BG_COLOR[2],BG_COLOR[3]);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0,glutWidth,0,glutHeight,-1,1);
    glMatrixMode(GL_MODELVIEW);  glLoadIdentity();
    drawRoundedRect(24,24,glutWidth-48,glutHeight-48,22,PANEL_COLOR);
    drawTextModern(54,glutHeight-50,"GTS — Haptic Teleoperation Dashboard",TITLE_FONT,TEXT_TITLE);
    float y=glutHeight-100, left=64;
    drawTextModern(left,y,"Mode: "+string(useVirtualEnvironment?"Virtual Environment":"Network"),LABEL_FONT,TEXT_BODY); y-=28;
    drawTextModern(left,y,"Sent: "+to_string(successfulSends)+" (Failed: "+to_string(failedSends)+")",LABEL_FONT,TEXT_BODY); y-=22;
    drawTextModern(left,y,"Received: "+to_string(successfulReceives)+" (Failed: "+to_string(failedReceives)+")",LABEL_FONT,TEXT_BODY); y-=22;
    drawTextModern(left,y,"Avg Latency: "+to_string_with_precision(avgLatency,1)+" ms",LABEL_FONT,TEXT_BODY); y-=22;
    drawTextModern(left,y,"Adaptive Damping: "+to_string_with_precision(adaptiveDamping,2)+" N·s/m  Stiffness: "+to_string_with_precision(adaptiveStiffness,2)+" N/m",LABEL_FONT,TEXT_BODY);
    glutSwapBuffers();
}
void glutReshape(int w, int h) { glutWidth=w; glutHeight=max(h,1); glViewport(0,0,w,h); glutPostRedisplay(); }
void timerCallback(int value) { glutPostRedisplay(); glutTimerFunc(33,timerCallback,0); }
void glutKeyboard(unsigned char key, int x, int y) {
    lock_guard<mutex> lock(controlMutex);
    switch (key) {
    case '1': sendPacketsEnabled    = !sendPacketsEnabled;    break;
    case '2': receiveFeedbackEnabled = !receiveFeedbackEnabled; break;
    case '3': applyHapticFeedback   = !applyHapticFeedback;  break;
    case '4': rawModeEnabled        = !rawModeEnabled;        break;
    case '7': packetDelayMin += 1.0; if (packetDelayMin > packetDelayMax) packetDelayMin = packetDelayMax; break;
    case '8': packetDelayMax += 1.0; break;
    case '9': maxPacketsToSend += 100; break;
    case '0': packetIntervalMs += 0.1; break;
    case 'w': FORCE_PACKET_WINDOW++; break;
    case 27: case 'q': running = false; glutDestroyWindow(glutWindow); break;
    }
    glutPostRedisplay();
}
void glutGUI() {
    int argc=1; char* argv[]={(char*)"GTS-Master",nullptr};
    glutInit(&argc,argv); glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);
    glutInitWindowSize(glutWidth,glutHeight); glutInitWindowPosition(100,100);
    glutWindow = glutCreateWindow("GTS Haptic Teleoperation Dashboard");
    glutDisplayFunc(glutDisplay); glutReshapeFunc(glutReshape); glutKeyboardFunc(glutKeyboard);
    glClearColor(BG_COLOR[0],BG_COLOR[1],BG_COLOR[2],BG_COLOR[3]);
    glutTimerFunc(33,timerCallback,0); glutMainLoop();
}
void updateGraphics() {
    if (!camera) return;
    labelRates->setText("Graphics: "+to_string((int)freqCounterGraphics.getFrequency())+" Hz  |  Haptics: "+to_string((int)freqCounterHaptics.getFrequency())+" Hz");
    labelRates->setLocalPos(10,height-30,0); labelMessage->setLocalPos(10,height-60,0);
    camera->renderView(width,height);
}
void windowSizeCallback(GLFWwindow* w, int a, int b) { width=a; height=b; }
void errorCallback(int a, const char* d) { logMessage("GLFW Error "+to_string(a)+": "+string(d)); }
void keyCallback(GLFWwindow* w, int key, int sc, int action, int mods) {
    if (action != GLFW_PRESS) return;
    if (key == GLFW_KEY_F) {
        fullscreen = !fullscreen;
        if (fullscreen) { const GLFWvidmode* m=glfwGetVideoMode(glfwGetPrimaryMonitor()); glfwSetWindowMonitor(w,glfwGetPrimaryMonitor(),0,0,m->width,m->height,m->refreshRate); }
        else glfwSetWindowMonitor(w,nullptr,100,100,1280,720,0);
    } else if (key==GLFW_KEY_M && camera) { mirroredDisplay=!mirroredDisplay; camera->setMirrorVertical(mirroredDisplay); }
    else if (key==GLFW_KEY_Q||key==GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(w,GLFW_TRUE);
}
void close() { running=false; hapticRunning=false; }

int main() {
    timeBeginPeriod(1);
    logMessage("===== GTS Master Client — HHRCM Lab, NCRA-NEDUET, Karachi =====");
    logMessage("M.Engg. Thesis: Network Jitter Compensation in Haptic Teleoperation");
    logMessage("Intercontinental Experiment: Nov 10, 2022 | Karachi ↔ Innsbruck | 7000 km");
    logMessage("Client IP: "+string(CLIENT_IP)+" | Server IP: "+string(SERVER_IP));
    logMessage("Haptic Rate: "+to_string(HAPTIC_RATE_HZ)+" Hz | Max Force: "+to_string(MAX_FORCE)+" N");

    if (!glfwInit()) { handleError("GLFW init failed"); return 1; }
    glfwSetErrorCallback(errorCallback);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_STEREO, stereoMode==C_STEREO_ACTIVE ? GL_TRUE : GL_FALSE);
    const GLFWvidmode* mode = glfwGetPrimaryMonitor() ? glfwGetVideoMode(glfwGetPrimaryMonitor()) : nullptr;
    if (!mode) { handleError("Failed to get video mode"); glfwTerminate(); return 1; }
    int w=0.8*mode->height, h=0.5*mode->height, x=0.5*(mode->width-w), y=0.5*(mode->height-h);
    window = glfwCreateWindow(w,h,"GTS — Haptic Teleoperation with Virtual Environment",nullptr,nullptr);
    if (!window) { handleError("Failed to create window"); glfwTerminate(); return 1; }
    glfwGetWindowSize(window,&width,&height);
    glfwSetWindowPos(window,x,y);
    glfwSetKeyCallback(window,keyCallback);
    glfwSetWindowSizeCallback(window,windowSizeCallback);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(swapInterval);
    glewInit();
    initializeWinsock();
    setupSocket(sock,RECV_PORT,3);
    setupSocket(fallbackSock,FALLBACK_PORT,3);

    handler = new cHapticDeviceHandler();
    handler->getDevice(hapticDevice,0);
    if (hapticDevice && hapticDevice->open()) {
        cHapticDeviceInfo info = hapticDevice->getSpecifications();
        logMessage("Haptic device: "+info.m_modelName);
        hapticDevice->calibrate();
        hapticDeviceMaxStiffness = info.m_maxLinearStiffness;
    } else { logMessage("No haptic device — VE mode only"); useVirtualEnvironment=true; }

    world = new cWorld(); world->m_backgroundColor.setWhite();
    camera = new cCamera(world); world->addChild(camera);
    camera->set(cVector3d(0.20,0.00,0.10),cVector3d(0.00,0.00,0.05),cVector3d(0.00,0.00,1.00));
    camera->setClippingPlanes(0.01,10.0);
    camera->setStereoMode(stereoMode); camera->setMirrorVertical(mirroredDisplay);
    light = new cSpotLight(world); world->addChild(light);
    light->setEnabled(true); light->setLocalPos(0.0,0.3,0.4); light->setDir(0.0,-0.25,-0.4);
    light->setShadowMapEnabled(true); light->m_shadowMap->setQualityHigh();
    world->setShadowIntensity(0.3); light->setCutOffAngleDeg(30);
    plane = new cMesh(); world->addChild(plane);
    cCreateMap(plane,0.2,1.0,20,20); plane->setUseDisplayList(true); plane->m_material->setWhite();
    for (int i=0;i<NUM_SPHERES;i++) {
        spheres[i] = new cShapeSphere(SPHERE_RADIUS); world->addChild(spheres[i]);
        spheres[i]->setLocalPos(0.8*SPHERE_RADIUS*(i+2)*cos(1.0*i),0.8*SPHERE_RADIUS*(i+2)*sin(1.0*i),SPHERE_RADIUS);
        spheres[i]->m_material->setWhite(); sphereVel[i].zero();
    }
    font = NEW_CFONTCALIBRI20();
    labelRates = new cLabel(font); labelRates->m_fontColor.setBlack(); camera->m_frontLayer->addChild(labelRates);
    labelMessage = new cLabel(font); labelMessage->m_fontColor.setBlack();
    labelMessage->setText("Press gripper to toggle Network / Virtual Environment mode");
    camera->m_frontLayer->addChild(labelMessage);
    background = new cBackground(); camera->m_backLayer->addChild(background);
    background->setFixedAspectRatio(true);
    if (!background->loadFromFile(RESOURCE_PATH("resources/images/HHRCM background.png"))) {
        camera->m_backLayer->removeChild(background); background=nullptr;
    }
    forceBuffer.reserve(FORCE_PACKET_WINDOW);
    stiffnessBuffer.reserve(FORCE_PACKET_WINDOW);
    dampingBuffer.reserve(FORCE_PACKET_WINDOW);
    errorBuffer.reserve(FORCE_PACKET_WINDOW);
    graphStartTime = chrono::steady_clock::now();

    thread networkThread(networkCommunication);
    thread hapticThread(hapticLoop);
    thread glutThread(glutGUI);
    atexit(close);
    windowSizeCallback(window,width,height);

    while (running && !glfwWindowShouldClose(window)) {
        glfwMakeContextCurrent(window);
        glfwGetWindowSize(window,&width,&height);
        updateGraphics();
        glfwSwapBuffers(window);
        glfwPollEvents();
        freqCounterGraphics.signal(1);
    }

    running=false; hapticRunning=false; simulationRunning=false;
    while (!simulationFinished) cSleepMs(100);
    networkThread.join(); hapticThread.join(); glutThread.join();
    if (hapticDevice) hapticDevice->close();
    delete handler; delete world;
    cleanupSocket();
    glfwDestroyWindow(window); glfwTerminate();
    timeEndPeriod(1); logFile.close();
    return 0;
}
