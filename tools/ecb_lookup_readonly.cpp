// 只调用 SDK 发现接口，不使能、不运动、不改写工作区 YAML。
#include "actuatorcontroller.h"
#include <iostream>

int main()
{
    auto *controller = ActuatorController::initController();
    if (!controller) return 1;
    Actuator::ErrorsDefine error;
    const auto devices = controller->lookupActuators(error);
    for (const auto &device : devices) {
        std::cout << "MOTOR " << static_cast<unsigned>(device.actuatorID)
                  << ' ' << device.ipAddress << '\n';
    }
    return devices.empty() ? 1 : 0;
}
