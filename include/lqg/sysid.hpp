#include "odom.hpp"
#include "chassis.hpp"

class SysID {
public:
    explicit SysID(Chassis& chassis);
    void moveRamp(double dV, double time);
    void moveStep(double targetMV, double time);
    void turnRamp(double dV, double time);
    void turnStep(double targetMV, double time);
private:
    Chassis& chassis;
};