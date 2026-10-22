set(srcs
        "main.c"
        "main.cpp")

set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
