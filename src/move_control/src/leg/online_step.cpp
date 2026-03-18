#include "leg/online_step.hpp"

OnlineFootTrajectory::OnlineFootTrajectory() {}

void OnlineFootTrajectory::setStartEnd(const Vector3D& p0, const Vector3D& pf, double stepHeight) {
    startPos_   = p0;
    endPos_     = pf;
    stepHeight_ = stepHeight;
}

// 在线计算当前 phase 足端位置、速度、加速度
// phase ∈ [0,1]，1 表示摆动结束
void OnlineFootTrajectory::compute(double phase, Vector3D& pos, Vector3D& vel, Vector3D& acc){
    phase    = std::clamp(phase, 0.0, 1.0);
    double T = 1.0; // phase-based normalized duration (速度和加速度可按实际 swing_time 缩放)

    // ---------- XY 方向 cubic Bezier ----------
    pos[0] = cubic(startPos_[0], endPos_[0], phase);
    pos[1] = cubic(startPos_[1], endPos_[1], phase);

    vel[0] = cubicVel(startPos_[0], endPos_[0], phase, T);
    vel[1] = cubicVel(startPos_[1], endPos_[1], phase, T);

    acc[0] = cubicAcc(startPos_[0], endPos_[0], phase, T);
    acc[1] = cubicAcc(startPos_[1], endPos_[1], phase, T);

    // ---------- Z 方向抬脚轨迹 ----------
    if (phase < 0.5) {
        double s = phase * 2.0;
        pos[2]   = startPos_[2] + stepHeight_ * (2.0 * s - s * s);
        vel[2]   = (4.0 * stepHeight_ / T) * (1.0 - s);
        acc[2]   = -4.0 * stepHeight_ / (T * T);
    } else {
        double s = (phase - 0.5) * 2.0;
        pos[2]   = endPos_[2] + stepHeight_ * (1.0 - s * s);
        vel[2]   = -(2.0 * stepHeight_ / T) * s;
        acc[2]   = -2.0 * stepHeight_ / (T * T);
    }
}

// 在线更新目标点 pf（支持中途 replanning）
void OnlineFootTrajectory::updateTarget(const Vector3D& new_pf) { endPos_ = new_pf; }