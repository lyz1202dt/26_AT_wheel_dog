#pragma once

#include <Eigen/Dense>

class Robot;

class Estimater{
public:
    Estimater();
    ~Estimater();
    void set_x();
    bool update();

private:
    void build_state_space_matrix(Robot *robot);
};
