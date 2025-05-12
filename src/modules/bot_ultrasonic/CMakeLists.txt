px4_add_module(
	MODULE modules__bot_ultrasonic
	MAIN bot_ultrasonic
	COMPILE_FLAGS
		#-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
		#-O0             # uncomment when debugging
	SRCS
		bot_ultrasonic.cpp
		bot_ultrasonic.hpp
	DEPENDS
		px4_work_queue
	)
