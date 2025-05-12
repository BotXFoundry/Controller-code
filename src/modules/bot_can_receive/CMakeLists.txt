px4_add_module(
	MODULE modules__bot_can_receive
	MAIN bot_can_receive
	COMPILE_FLAGS
		#-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
		#-O0             # uncomment when debugging
	SRCS
		BotCanReceive.cpp
		BotCanReceive.hpp
	DEPENDS
		px4_work_queue
	)
