# i.MX28 GPIO impulse generator

It generates impulses on specified GPIO.

## Build

This driver does not work as a module. It must be embedded into kernel due to FIQ usage. 

## Usage

To generate impulses write impulses count to a driver's file. 
The following command would generate two impulses:
```sh
echo 2 > /dev/impulse-gen
```

The name of the driver's file depends on a device's name, that has been declared in a platform device tree.
Here it is the `impulse-gen`.

## Device tree bindings

Required properties:
* gpios: a spec for a GPIO to be used as impulse output

Optional properties:
* timrot: used timer of i.MX28 processor. By default it is 2. It can be also 3. Timers 0 and 1 are already used by the Linux kernel.
* timrot-fixed-count: contains the fixed timer counter value. It defines width of an impulse and delay between the next one. By default it is 60000

Example:
```c
        impulse-gen {
                compatible = "imx28-gpio-impulse-gen";
                gpios = <&gpio1 14 0>;                
                timrot = <2>;
                timrot-fixed-count = <60000>;
        };
```
