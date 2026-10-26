#pragma once
#include <array>
#include <vector>
namespace robot_hardware {
// Simulation-only measured plant bridge. Does not expose or grant write permits.
class ExternalPlant {
public:
 virtual ~ExternalPlant()=default;
 virtual void measuredPlant(const std::vector<std::array<double,3>> & measured)=0;
};
}
