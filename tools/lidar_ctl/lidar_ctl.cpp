// lidar_ctl: put a Livox Avia (Livox SDK 1.x protocol) into Normal / Power-Saving / Standby mode over Ethernet and report its state.
// The LiDAR serves ONE host at a time, so the pipeline's ROS driver must not be running (use the `lidar` wrapper script).
// Usage: lidar_ctl <status|normal|low|standby|sampling-on|sampling-off|watch> [--code BROADCASTCODE] [--timeout SECONDS] [--watch SECONDS]
// Exit code: 0 = requested state confirmed by the LiDAR itself, 2 = timeout, 3 = LiDAR not found, 4 = SDK failed to start.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <cstdarg>
#include "livox_sdk.h"

static std::string g_code;                    // broadcast code to connect to ("" = first device heard)
static std::atomic<bool> g_added(false), g_connected(false);
static std::atomic<int> g_state(-1), g_handle(-1);
static std::mutex g_log_mu;
static auto g_t0 = std::chrono::steady_clock::now();

static double now_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now() - g_t0).count(); }
static const char* state_name(int s) {
  switch (s) { case kLidarStateInit: return "Initializing"; case kLidarStateNormal: return "Normal"; case kLidarStatePowerSaving: return "PowerSaving";
    case kLidarStateStandBy: return "Standby"; case kLidarStateError: return "Error"; default: return "Unknown"; }
}
static void logf(const char* fmt, ...) {
  std::lock_guard<std::mutex> l(g_log_mu); va_list a; va_start(a, fmt); fprintf(stderr, "[%7.2fs] ", now_s()); vfprintf(stderr, fmt, a); fprintf(stderr, "\n"); va_end(a);
}
static void OnBroadcast(const BroadcastDeviceInfo* info) {
  if (!info || g_added) return;
  if (!g_code.empty() && g_code != info->broadcast_code) return;
  uint8_t handle = 0;
  livox_status r = AddLidarToConnect(info->broadcast_code, &handle);
  logf("broadcast from %s (%s) -> connect request: %s", info->broadcast_code, info->ip, r == kStatusSuccess ? "ok" : "FAILED");
  if (r == kStatusSuccess) { g_added = true; g_handle = handle; }
}
static void OnState(const DeviceInfo* d, DeviceEvent ev) {
  if (!d) return;
  g_handle = d->handle; g_state = d->state;
  if (ev == kEventConnect) g_connected = true;
  if (ev == kEventDisconnect) g_connected = false;
  logf("event=%s state=%s(%d) ip=%s fw=%d.%d.%d.%d", ev == kEventConnect ? "connect" : ev == kEventDisconnect ? "disconnect" : ev == kEventStateChange ? "state-change" : "other",
       state_name(d->state), (int)d->state, d->ip, d->firmware_version[0], d->firmware_version[1], d->firmware_version[2], d->firmware_version[3]);
}
static std::atomic<int> g_cmd_status(-99), g_cmd_resp(-1);
static void OnCmd(livox_status st, uint8_t, uint8_t resp, void*) { g_cmd_status = st; g_cmd_resp = resp; logf("command callback: status=%d response=%d", (int)st, (int)resp); }

int main(int argc, char** argv) {
  if (argc < 2) { fprintf(stderr, "usage: %s <status|normal|low|standby|sampling-on|sampling-off|watch> [--code C] [--timeout S] [--watch S]\n", argv[0]); return 1; }
  std::string cmd = argv[1]; double timeout = 45, watch = 0;
  for (int i = 2; i + 1 < argc; i += 2) { std::string k = argv[i]; if (k == "--code") g_code = argv[i + 1]; else if (k == "--timeout") timeout = atof(argv[i + 1]); else if (k == "--watch") watch = atof(argv[i + 1]); }
  int target = -1;
  if (cmd == "normal") target = kLidarStateNormal; else if (cmd == "low" || cmd == "power-saving") target = kLidarStatePowerSaving; else if (cmd == "standby") target = kLidarStateStandBy;
  if (!Init()) { printf("{\"ok\":false,\"error\":\"SDK Init failed (is the pipeline's livox driver still running?)\"}\n"); return 4; }
  SetBroadcastCallback(OnBroadcast); SetDeviceStateUpdateCallback(OnState);
  if (!Start()) { printf("{\"ok\":false,\"error\":\"SDK Start failed\"}\n"); Uninit(); return 4; }
  logf("SDK started, waiting for the LiDAR to broadcast / connect (code filter: %s)", g_code.empty() ? "none" : g_code.c_str());
  auto wait_for = [&](auto pred, double secs) { double t = now_s() + secs; while (now_s() < t) { if (pred()) return true; std::this_thread::sleep_for(std::chrono::milliseconds(50)); } return pred(); };
  if (!wait_for([] { return g_connected.load() && g_state.load() >= 0; }, timeout)) {
    printf("{\"ok\":false,\"error\":\"LiDAR not found/connected within %.0fs\",\"added\":%s}\n", timeout, g_added ? "true" : "false"); Uninit(); return 3;
  }
  // The state in the connect event is not yet valid: wait for a heartbeat that reports a real state (Normal / PowerSaving / Standby / Error).
  auto valid = [] { int s = g_state.load(); return s == kLidarStateNormal || s == kLidarStatePowerSaving || s == kLidarStateStandBy || s == kLidarStateError; };
  if (!wait_for(valid, 25)) logf("no valid state heartbeat within 25 s (state=%s)", state_name(g_state.load()));
  int initial = g_state.load(); int rc = 0; double t_cmd = now_s();
  if (cmd == "status") { /* nothing to send */ }
  else if (cmd == "watch") { /* only observe */ }
  else if (target > 0) {
    if (initial == target) logf("already in %s", state_name(target));
    else {
      livox_status r = LidarSetMode(g_handle, (LidarMode)target, OnCmd, nullptr); logf("LidarSetMode(%s) sent: %d", state_name(target), (int)r);
      if (r != kStatusSuccess) { printf("{\"ok\":false,\"error\":\"LidarSetMode send failed\",\"code\":%d}\n", (int)r); Uninit(); return 2; }
      if (!wait_for([&] { return g_state.load() == target; }, timeout)) rc = 2;
    }
  } else if (cmd == "sampling-on") { LidarStartSampling(g_handle, OnCmd, nullptr); wait_for([] { return g_cmd_status.load() != -99; }, 10); }
  else if (cmd == "sampling-off") { LidarStopSampling(g_handle, OnCmd, nullptr); wait_for([] { return g_cmd_status.load() != -99; }, 10); }
  else { printf("{\"ok\":false,\"error\":\"unknown command\"}\n"); Uninit(); return 1; }
  if (watch > 0) { logf("watching state for %.0f s (heartbeats continue)", watch); std::this_thread::sleep_for(std::chrono::duration<double>(watch)); }
  int fin = g_state.load();
  printf("{\"ok\":%s,\"command\":\"%s\",\"initial_state\":\"%s\",\"final_state\":\"%s\",\"switch_seconds\":%.1f,\"broadcast_code\":\"%s\"}\n", rc == 0 ? "true" : "false", cmd.c_str(), state_name(initial), state_name(fin), now_s() - t_cmd, g_code.c_str());
  Uninit(); return rc;
}
