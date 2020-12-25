# AUTOSAR Can Driver Implementation for STM32

This is an implementation of AUTOSAR Can and CanIf module, working on STM32 controllers.

This implementation uses AUTOSAR Standard Classic Release 4.3.1.

The CAN driver uses the HAL library, and can be integrated in your project using CMake. You just need to provide the hardware headers for your controller.

It so far only supports STM32F3 controllers.

The implementation is copied together from existing open-source AUTOSAR implementations, and a lot of customization to get it running. The driver implementation is still incomplete, and probably still has a bug here and there. Usage in professional systems is not recommended. There are probably also race conditions, and the driver is not thread safe.

I basically wanted to use CAN in my projects, and searched for a standard to base my implementation on. I realized that except AUTOSAR, there is not really a standard for CAN driver implementations, so I went that route. AUTOSAR is sometimes boring, but at least it's a standard, and it just works.

# What is working?
- Controller wakeup by bus
- Rx / Tx
- Bus off detection
- all the basic

If you consider this code helpful, and manage to improve things a little bit, please feel free to fix stuff via a PR.

# How to use it?
You  need to integrate the CAN stack to your project via CMake. This is how I use it:

    target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
        -DUSE_CAN
        -DHAL_CAN_MODULE_ENABLED
    )
    add_subdirectory(can_stack)
    list(APPEND EXTRA_LIBS can_stack)
    

An example project using this CAN stack can be found here: https://github.com/Danaozhong/sw.tool.sunny_rz1_fuel_gauge_converter