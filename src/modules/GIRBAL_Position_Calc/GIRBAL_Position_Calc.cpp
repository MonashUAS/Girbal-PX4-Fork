#include "GIRBAL_Position_Calc.hpp"

GIRBAL_Position_Calc::GIRBAL_Position_Calc() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{

}

bool GIRBAL_Position_Calc::init()
{
    // execute Run() on every gps publication
    if (!sensor_gps_s.registerCallback()) {
        PX4_ERR("GPS data callback registration failed");
        return false;
    }

    // alternatively, Run on fixed interval
    // ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

    return true;
}

void GIRBAL_Position_Calc::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    // "work" happens here on distances callback
    if (GIRBAL_anchor_distances_s.updated()) {
		GIRBAL_anchor_distances_s distPtr;

		if (GIRBAL_anchor_distances_s.copy(&distPtr)) {


		}
	}
}

extern "C" __EXPORT int work_item_example_main(int argc, char *argv[]) // not really sure what this func does tbh
{
    return GIRBAL_Position_Calc::main(argc, argv);
}
