# AUTOSAR Can Driver Implementation for STM32

This is an implementation of AUTOSAR Can and CanIf module, working on STM32 controllers.

This implementation uses AUTOSAR Standard Classic Release 4.3.1.

The CAN driver uses the HAL library, and can be integrated in your project using CMake. You just
need to provide the hardware headers for your controller.

It so far only supports STM32F3 controllers.

The implementation is copied together from existing open-source AUTOSAR implementations, and a lot of customization to get it running. The driver implementation is still incomplete, and full of bugs. Usage in professional systems is not recommended. There are lots of race conditions, and the driver is not thread safe.

I basically wanted to use CAN in my projects, and searched for a standard to base my implementation on. AUTOSAR is quite boring, but at least it's a standard, so I decided to go that route.

# What is working?
- Controller wakeup by bus
- Rx / Tx

If you consider this code helpful, and manage to improve things a little bit, please feel free to fix stuff via a PR.