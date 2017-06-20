# i.MX28 GPIO impulse generator

The driver generates impulses on specified GPIO. It uses a hardware timer of the i.MX28 processor and FIQ interrupts.

## Build

This driver can not be built as a module. It must be embedded into kernel due to FIQ interrupts usage.

## Usage

To generate impulses write impulses count to a driver's file. A width of impulses can be set by an appropriate ioctl call.
The following code would generate two impulses:
```c
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define SET_FIXED_COUNT 0
#define GET_FIXED_COUNT 1

int main(int, char*)
{
    int fp = open("/dev/imp0", O_RDWR);
    int fixed_count = 60000;
    char impulses[] = "2";

    ioctl(fp, SET_FIXED_COUNT, &fixed_count);
    write(fp, impulses, sizeof(impulses));
    close(fp);

    return 0;
}
```

The name of the driver's file depends on a device's name, that has been declared in a platform device tree.
Here it is the `imp0`.

## Device tree bindings

Required properties:
* gpios: a spec for a GPIO to be used as impulse output

Optional properties:
* timrot: used timer of i.MX28 processor. By default, it is 2. It can be also 3. Timers 0 and 1 are already used by the Linux kernel.

Example:
```c
imp0 {
	compatible = "imx28-gpio-impulse-gen";
	gpios = <&gpio1 14 0>;
	timrot = <2>;
};
```
