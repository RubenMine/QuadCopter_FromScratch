typedef struct {
    int Kp, Ki, Kd;
    float Integrator_STATE, Filter_STATE;
    float setpoint, output;    
    float UpperLimit, LowerLimit;
    bool anti_windup;
} PID;

static PID roll_pid;
static PID pitch_pid;
static PID yaw_pid;
static PID altitude_pid;
