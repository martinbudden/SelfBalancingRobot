![license](https://img.shields.io/badge/license-MIT-green) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

# PID Library

This library contains a [PID controller](https://en.wikipedia.org/wiki/Proportional-integral-derivative_controller) with
additional [feed forward](https://en.wikipedia.org/wiki/Feed_forward_(control)).

The PID controller has the following features:

1. Addition of optional feed forward component. The PID calculation pseudocode is:<br>
    `output = kp*error + Ki*errorIntegral + kd*_errorDerivative + kf*setpoint`<br>
    Setting `kf` to zero gives a traditional PID controller.
2. Calculation of derivative on measurement, avoiding "derivative kick" when the setpoint changes.
3. _delta-t_ input parameter to PID `update` function. This allows for jitter in the timing of the call to the `update` function.
4. A choice of two methods of controlling integral windup. Either the integral term can be limited to a maximum value,
    or it can be set to zero when the output saturates. Both methods can be used together, if desired.
5. Functions to return the current error terms. These can be used for PID tuning, telemetry, and test code.

The PID controller deliberately does not implement these features:

1. Output clipping. Usually it is desirable to clip the PID output before it is fed to the device being controlled.
    However often several PID outputs are combined, when this is the case it is the combined output that needs to be clipped.
2. PID "negation". Some PID controllers provide a function to negate all the PID constants. If this is required, just subtract the PID output, rather than add it.