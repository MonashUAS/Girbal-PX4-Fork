# Project Girbal PX4 Autopilot Fork

UAS Fork of the PX4 autopilot, with custom messages and modules for Project Girbal. Original repo can be found [here](https://github.com/PX4/PX4-Autopilot); official PX4 docs can be found [here](https://docs.px4.io/master/en/). Good luck xx

## Setting up the dev environment
1. Firstly, you install the [Jmavsim simulator](https://docs.px4.io/master/en/dev_setup/dev_env_windows_cygwin.html) that we are using. Run the batch file `run-console.bat` after install. 

2. From the console that appears when you run the batch file, you want to clone [our fork of the PX4 software](https://github.com/MonashUAS/Girbal-PX4-Fork) (you're currently reading its readme). Make sure that you run a recursive clone as there are a lot of submodules in it: `git submodule update --recursive`

3. In the batch console, run `make px4_sitl jmavsim` inside the Girbal-PX4-Fork directory. 

3. Download [qgroundcontrol](https://docs.qgroundcontrol.com/master/en/releases/daily_builds.html) in Windows land, which allows you to control the drone/send commands to the sim.

## Developing for PX4
The Girbal modules can be located in "/src/modules/GIRBAL_{module_name}", with the message definitions located in "/msg/GIRBAL_{msg_name}". Typically, each module will have a main .cpp file, a header file with definitions, and a CMakeLists.txt that tells the compiler about it. Message file are added as needed, and are used to communicate between modules. There are a plethora of predefined message files (read more about messaging [here](https://docs.px4.io/master/en/middleware/uorb.html)). 

## Fixing common issues
If you're having random build issues not related to the code (can't help you with that, get gud :p), the following may help:

`git submodule update --recursive` - Runs a recursive update on any submodules in the repo

`make distclean` - Cleans out any files associated with building the software
