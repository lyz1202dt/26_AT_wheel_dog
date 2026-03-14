#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>


class OnlineFootTrajectory{
    using Vector3D = Eigen::Vector3d;
public:
    OnlineFootTrajectory();
    ~OnlineFootTrajectory();

    void setStartEnd(const Vector3D& p0, const Vector3D& pf, double stepHeight);
    void compute(double phase, Vector3D& pos, Vector3D& vel, Vector3D& acc);
    void updateTarget(const Vector3D& new_pf);
private:
    Vector3D startPos_;
    Vector3D endPos_;
    double stepHeight_;

    static double cubic(double p0, double pf, double phase) {
        return p0 + (pf - p0) * (3.0 * phase * phase - 2.0 * phase * phase * phase);
    }

    static double cubicVel(double p0, double pf, double phase, double T) {
        return (pf - p0) * (6.0 * phase - 6.0 * phase * phase) / T;
    }

    static double cubicAcc(double p0, double pf, double phase, double T) {
        return (pf - p0) * (6.0 - 12.0 * phase) / (T * T);
    }
};