# Execution result of a behavior

Header header                    # Message header
string name                      # Behavior name
int8 termination_cause         # Code of the behavior activation result
string error_message             # Explicative message in case of error

# Possible values for termination_cause

int8 GOAL_ACHIEVED        = 1
int8 TIME_OUT             = 2
int8 WRONG_PROGRESS       = 3
int8 PROCESS_FAILURE      = 4
int8 INTERRUPTED          = 5
