# make the module do the build thing
# copied from https://github.com/PX4/PX4-Autopilot/blob/master/src/examples/work_item/CMakeLists.txt

px4_add_module(
MODULE examples__GIRBAL_Position_Calc
MAIN GIRBAL_Position_Calc
COMPILE_FLAGS
    #-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
    #-O0             # uncomment when debugging
SRCS
    GIRBAL_Position_Calc.cpp
    GIRBAL_Position_Calc.hpp
DEPENDS
    px4_work_queue
)
