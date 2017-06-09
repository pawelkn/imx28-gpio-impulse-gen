# imx28-gpio-pwm

i.MX28 GPIO pulse width modulator. It generates impulses on specified GPIO.

## Cross-compile build
```sh
export PATH=<path to toolchain executables>
export KERNEL_DIR=<path to kernel build directory>
export CROSS_COMPILE=<cross compilation prefix>
export ARCH=<architecture eg.: arm>
make
```
### Usage

To generate impulses write impulses count to a driver's file. 
The following command would generate two impulses:
```sh
echo 2 > /dev/pwm
```

The name of the driver's file depends on a device's name, that has been declared in a platform device tree.
Here it is the `pwm`.

## Device tree bindings

Required properties:
* gpios: a spec for a GPIO to be used as impulse output

Optional properties:
* pulse-delay: delay between impulses. The value is given in timer ticks. A default is 60000
* pulse-width: width of an impulse. The value is given in timer ticks. A default is 60000
* timrot-number: used timer/rotary encoder of i.MX28 processor. A default is 2. It can be also 3. Timers 0 and 1 are already used by Linux kernel.

Example:
```c
        pwm {
                compatible = "imx28-gpio-pwm";
                gpios = <&gpio1 14 GPIO_ACTIVE_HIGH>;
                pulse-width = <60000>;
                pulse-delay = <60000>;
                timrot-number = <2>;
        };
```
