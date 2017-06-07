# gpio-pulse-generator

Generic GPIO impulse generator. Generates given number of pulses on specified GPIO.

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
* gpios: a spec for a GPIO to be used

Optional properties:
* delay-ms: pulse duration

Example:
```c
        pulse-generator {
                compatible = "gpio-pulse-generator";
                gpios = <&gpio1 14 GPIO_ACTIVE_HIGH>;
                delay-ms = <5>;
        };
```
