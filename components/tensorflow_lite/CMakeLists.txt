file(GLOB_RECURSE srcs *.c
                    *.cc
                    )

idf_component_register(
    SRCS  ${srcs}
    INCLUDE_DIRS    "."
    REQUIRES    espressif__esp-tflite-micro
                bsp
)
