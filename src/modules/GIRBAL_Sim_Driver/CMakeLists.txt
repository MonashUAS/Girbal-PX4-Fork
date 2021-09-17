# make the module do the build thing
# copied from https://github.com/PX4/PX4-Autopilot/blob/master/src/examples/work_item/CMakeLists.txt

px4_add_module(
MODULE modules__GIRBAL_Sim_Driver
MAIN GIRBAL_Sim_Driver
COMPILE_FLAGS
    #-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
    #-O0             # uncomment when debugging
SRCS
    GIRBAL_Sim_Driver.cpp
    GIRBAL_Sim_Driver.hpp
DEPENDS
    px4_work_queue
)
