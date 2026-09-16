// 실제 SDK(dds_middleware + CycloneDDS)로 rt/lower/state 를 내고 rt/lower/cmd 를 받는 가상 로봇.
//
// 목적: 실기 없이 "통신 프로토콜" 을 검증한다 — 토픽·QoS 매칭, LowerCmd/LowerState 직렬화, 16 슬롯 규약,
//       영점 오프셋, 500Hz 상태 발행, Python 클라이언트(run.py --backend dds) 와의 왕복.
// 물리: 관절별 1차 모델 J*ddq = tau - B*dq (중력 없음). 역학 검증은 MuJoCo 쪽(backend_mujoco, fake_dds) 이 맡는다.
// 규약: 하드웨어 슬롯 16개, 관절 12개는 ABS2HW 슬롯, q_hw = q + MOTOR_OFFSET[hw],
//       모터 드라이버 토크 = kp*(q_cmd - q_hw) + kd*(dq_cmd - dq) + tau (SDK 공식), 슬롯 토크 한계로 자름.
//
// 사용: virtual_robot [seconds=60] [report.json]     (SIGINT/SIGTERM 으로도 종료, 보고서 저장)
#include "dds_middleware.hpp"
#include "lower_cmd.hpp"
#include "lower_state.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>

using namespace dobotmh4::msg::dds_;

static const int ABS2HW[12] = {0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14};
static const double MOTOR_OFFSET[16] = {-0.05, -0.5, 1.17, 0.0, 0.05, -0.5, 1.17, 0.0,
                                        -0.05, 0.5, -1.17, 0.0, 0.05, 0.5, -1.17, 0.0};
static const double TAU_MAX[12] = {23, 23, 55, 23, 23, 55, 23, 23, 55, 23, 23, 55};
static const double Q_MIN[12] = {-0.6632, -2.618, -2.53, -0.6632, -2.618, -2.53,
                                 -0.6632, -2.618, -2.53, -0.6632, -2.618, -2.53};
static const double Q_MAX[12] = {0.6632, 2.618, 2.53, 0.6632, 2.618, 2.53,
                                 0.6632, 2.618, 2.53, 0.6632, 2.618, 2.53};
static const double Q_INIT[12] = {0.0, 1.5, -2.53, 0.0, 1.5, -2.53, 0.0, 1.5, -2.53, 0.0, 1.5, -2.53};  // 엎드림
static const double J_INERTIA = 0.05;   // kg·m²  (kp 60/kd 1.8 에서 감쇠비 0.6)
static const double B_VISCOUS = 0.3;    // N·m·s/rad
static const double PHYSICS_DT = 0.001;
static const int PUB_EVERY = 2;         // 2ms = 500Hz

struct Cmd {
    std::array<double, 16> q{}, dq{}, tau{}, kp{}, kd{};
    std::array<int, 16> mode{};
    bool valid = false;
};

static std::mutex g_mutex;
static Cmd g_cmd;
static long g_cmds = 0;
static double g_cmd_gap_max_ms = 0.0, g_kp_max = 0.0;
static std::chrono::steady_clock::time_point g_last_cmd;
static std::atomic<bool> g_stop{false};

static void onCmd(const LowerCmd_& msg)
{
    Cmd c;
    for (int hw = 0; hw < 16; ++hw) {
        const auto& m = msg.motor_cmd()[hw];
        c.q[hw] = m.q();
        c.dq[hw] = m.dq();
        c.tau[hw] = m.tau();
        c.kp[hw] = m.kp();
        c.kd[hw] = m.kd();
        c.mode[hw] = m.mode();
    }
    c.valid = true;
    auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_cmds > 0) {
        double gap = std::chrono::duration<double, std::milli>(now - g_last_cmd).count();
        if (gap > g_cmd_gap_max_ms) g_cmd_gap_max_ms = gap;
    }
    g_last_cmd = now;
    ++g_cmds;
    for (int hw = 0; hw < 16; ++hw) if (c.kp[hw] > g_kp_max) g_kp_max = c.kp[hw];
    g_cmd = c;
}

static void onSignal(int) { g_stop = true; }

int main(int argc, char** argv)
{
    double seconds = argc > 1 ? std::atof(argv[1]) : 60.0;
    const char* report = argc > 2 ? argv[2] : nullptr;
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    dds_middleware::DDSMiddleware mw(0);
    dds_middleware::QoSProfile state_qos;              // 실기 센서 데이터처럼 best_effort
    state_qos.reliability = dds_middleware::ReliabilityPolicy::BEST_EFFORT;
    state_qos.durability = dds_middleware::DurabilityPolicy::VOLATILE;
    state_qos.history = dds_middleware::HistoryPolicy::KEEP_LAST;
    state_qos.history_depth = 1;
    auto pub = mw.create_publisher<LowerState_>("rt/lower/state", state_qos);
    auto sub = mw.create_subscription<LowerCmd_>("rt/lower/cmd", onCmd,
                                                 dds_middleware::QoSProfile::SensorData());
    std::printf("[virtual_robot] rt/lower/state 발행(500Hz), rt/lower/cmd 구독. %.0fs 동안 실행\n", seconds);

    std::array<double, 12> q, dq, tau_applied;
    for (int i = 0; i < 12; ++i) { q[i] = Q_INIT[i]; dq[i] = 0.0; tau_applied[i] = 0.0; }
    double tau_max_seen = 0.0;
    long states = 0, steps = 0;
    auto t0 = std::chrono::steady_clock::now();
    auto next = t0;
    auto last_print = t0;
    LowerState_ state;

    while (!g_stop) {
        Cmd c;
        { std::lock_guard<std::mutex> lock(g_mutex); c = g_cmd; }
        for (int i = 0; i < 12; ++i) {
            double tau = 0.0;
            if (c.valid) {
                int hw = ABS2HW[i];
                double q_hw = q[i] + MOTOR_OFFSET[hw];
                tau = c.kp[hw] * (c.q[hw] - q_hw) + c.kd[hw] * (c.dq[hw] - dq[i]) + c.tau[hw];
                if (tau > TAU_MAX[i]) tau = TAU_MAX[i];
                if (tau < -TAU_MAX[i]) tau = -TAU_MAX[i];
            }
            double ddq = (tau - B_VISCOUS * dq[i]) / J_INERTIA;
            dq[i] += ddq * PHYSICS_DT;
            q[i] += dq[i] * PHYSICS_DT;
            if (q[i] > Q_MAX[i]) { q[i] = Q_MAX[i]; dq[i] = 0.0; }
            if (q[i] < Q_MIN[i]) { q[i] = Q_MIN[i]; dq[i] = 0.0; }
            tau_applied[i] = tau;
            if (std::fabs(tau) > tau_max_seen) tau_max_seen = std::fabs(tau);
        }
        ++steps;
        if (steps % PUB_EVERY == 0) {
            auto& motors = state.motor_state();
            for (int i = 0; i < 12; ++i) {
                int hw = ABS2HW[i];
                motors[hw].mode(c.valid ? 4 : 3);
                motors[hw].q(static_cast<float>(q[i] + MOTOR_OFFSET[hw]));
                motors[hw].dq(static_cast<float>(dq[i]));
                motors[hw].tau_est(static_cast<float>(tau_applied[i]));
                motors[hw].q_raw(static_cast<float>(q[i] + MOTOR_OFFSET[hw]));
                motors[hw].motor_temp(35);
            }
            auto& imu = state.imu_state();
            imu.quaternion({1.0f, 0.0f, 0.0f, 0.0f});      // 평평하게 놓인 몸체
            imu.gyroscope({0.0f, 0.0f, 0.0f});
            imu.accelerometer({0.0f, 0.0f, 9.81f});
            imu.rpy({0.0f, 0.0f, 0.0f});
            imu.temperature(30);
            imu.timestamp(static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count()));
            state.bms_state().battery_level(88);
            pub->publish(state);
            ++states;
        }
        next += std::chrono::microseconds(static_cast<long>(PHYSICS_DT * 1e6));
        std::this_thread::sleep_until(next);
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_print).count() >= 1.0) {
            last_print = now;
            long cmds; double kp; { std::lock_guard<std::mutex> lock(g_mutex); cmds = g_cmds; kp = g_kp_max; }
            std::printf("[virtual_robot] t=%5.1fs states=%ld cmds=%ld kp_max=%.0f FL(abad,thigh,calf)=(%+.3f %+.3f %+.3f)\n",
                        std::chrono::duration<double>(now - t0).count(), states, cmds, kp, q[0], q[1], q[2]);
            std::fflush(stdout);
        }
        if (std::chrono::duration<double>(now - t0).count() >= seconds) break;
    }

    long cmds; double gap, kp;
    { std::lock_guard<std::mutex> lock(g_mutex); cmds = g_cmds; gap = g_cmd_gap_max_ms; kp = g_kp_max; }
    std::printf("[virtual_robot] 종료: states=%ld cmds=%ld cmd_gap_max=%.1fms kp_max=%.0f tau_max=%.1f\n",
                states, cmds, gap, kp, tau_max_seen);
    if (report) {
        FILE* f = std::fopen(report, "w");
        if (f) {
            std::fprintf(f, "{\n  \"kind\": \"virtual_robot_dds_cpp\",\n  \"states\": %ld,\n  \"cmds\": %ld,\n"
                            "  \"cmd_gap_max_ms\": %.3f,\n  \"kp_max_seen\": %.3f,\n  \"tau_max_seen\": %.3f,\n"
                            "  \"seconds\": %.3f,\n  \"final_q\": [",
                         states, cmds, gap, kp, tau_max_seen,
                         std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
            for (int i = 0; i < 12; ++i) std::fprintf(f, "%s%.4f", i ? ", " : "", q[i]);
            std::fprintf(f, "]\n}\n");
            std::fclose(f);
        }
    }
    return 0;
}
