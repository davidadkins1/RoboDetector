/* behaviors.h		*/
#ifndef BEHAVIORS_H
#define BEHAVIORS_H

enum BEHAVIORS
{
	NO_CONTROL_REQUEST,
	CRUISE,
	ESCAPE,
	REMOTE_CONTROL,
	MAX_BEHAVIOR
};

// Escape states
enum ESCAPE_STATES
{
	NORMAL_CRUISE,
	STALL,
	STALL_STOP,
	MOVE_AWAY,
	TURN_FORWARD,
	BACKUP,
	TURN_AWAY
};
#endif
