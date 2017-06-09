# imx28-gpio-pwm

i.MX28 GPIO pulse width modulator. Generates given number of impulses on specified GPIO.

## Cross-compile build
```sh
export PATH=<path to toolchain executables>
export KERNEL_DIR=<path to kernel build directory>
export CROSS_COMPILE=<cross compilation prefix>
export ARCH=<architecture eg.: arm>
make
```

## Device tree bindings

Required properties:
* gpios: a spec for a GPIO to be used as impulse output

Optional properties:
* pulse-delay: delay between impulses.The value is given in timer ticks. Default is 60000
* pulse-width: width of an impulse. The value is given in timer ticks. Default is 60000
* timrot-number: used timer/rotary encoder of i.MX28 procesor. Defaut is 2. It can be also 3. Timers 0 and 1 are already used by Linux kernel.

Example:
```c
        pulse-generator {
                compatible = "gpio-pulse-generator";
                gpios = <&gpio1 14 GPIO_ACTIVE_HIGH>;
                pulse-width = <60000>;
                pulse-delay = <60000>;
                timrot-number = <2>;
        };
```
