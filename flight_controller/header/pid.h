typedef struct {
    float kp, ki, kd;
    float setpoint;
    float integral, previous_error;
} PIDController;


static PIDController roll_pid;
static PIDController pitch_pid;
static PIDController yaw_pid;
static PIDController altitude_pid;