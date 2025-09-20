# STM32-DS18B20

## C driver to interface DS18B20 temperature sensor with an STM32 microcontroller

The driver itself is made of the files DS18B20.c and DS18B20.h.

It requires the files errors.h, console.h and console.c, which are common files for all my drivers. They are used to set up the error type (in errors.h) returned by some of the functions of the driver and to display data with the microcontroller on a terminal (in console.h and console.c). These files can be found here https://github.com/astarus-pyxis/stm32-common.

The file main.c is an example of main that uses the driver.

## How to use this driver in a project

No need to use CubeMX as the whole configuration of the sensor is done in the driver.

To use this driver in an STM32 CMake project, the C files  DS18B20.c and console.c shall be placed in the Core > Src folder of the project, and DS18B20.h, errors.h and console.h in the Core > Inc folder.

It also requires to add the sources to executable in the CMakeLists.txt file at the root of the project. To do this, the following at line 48 of this file.


```
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

```

shall be changed to


```
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    "Core/Src/console.c"
    "Core/Src/DS18B20.c"
)
```

## Licence & Warranty

This driver is licensed under GNU V3.0. It comes with no warranty.
