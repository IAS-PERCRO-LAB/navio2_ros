
#include <unistd.h>
#include "Navio2/PWM.h"
#include "Navio2/RCOutput_Navio2.h"
#include <memory>

#define SERVO_MIN 1250 /*mS*/
#define SERVO_MAX 1750 /*mS*/

#define PWM_OUTPUT 0

std::unique_ptr<RCOutput> get_rcout() {
        auto ptr = std::unique_ptr<RCOutput>{new RCOutput_Navio2()};
        return ptr;
}

int main(int argc, char *argv[]) {
    auto pwm = get_rcout();

    if (getuid()) {
        fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
    }

    if (!(pwm->initialize(PWM_OUTPUT))) {
        fprintf(stderr, "Failed to initialise output");
        return 1;
    }

    pwm->set_frequency(PWM_OUTPUT, 50);

    if (!(pwm->enable(PWM_OUTPUT))) {
        fprintf(stderr, "Failed to enable output");
        return 1;
    }

    while (true) {
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_MIN);
        sleep(1);
        pwm->set_duty_cycle(PWM_OUTPUT, SERVO_MAX);
        sleep(1);
    }
}
