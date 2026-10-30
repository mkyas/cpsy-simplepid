
#include <cstdio>
#include "simplepid.hpp"

class waterboiler {
public:
    waterboiler(void) : water_temp{20.0} {};
    float update(float power, float dt) noexcept {
        if (power > 0) {
            water_temp += power * dt;
        }
        water_temp -= 0.02 * dt;
        return water_temp;
    }
    float water_temp;
};

int main() {
    waterboiler boiler{};
    float water_temp = boiler.water_temp;
    simplepid<float> pid{5, 0.01, 0.1, water_temp};
    pid.set_setpoint(20);
    pid.set_output_limits(0, 100);

    const float dt = 0.01;
    for (float time = 0; time <= 5; time += dt) {
        float power = pid(water_temp, dt);
        water_temp = boiler.update(power, dt);
        printf("%f,%f,%f\n", time, water_temp, pid.get_setpoint());
        if (time > 1.0)
            pid.set_setpoint(100.0);
    }
    return 0;
}
