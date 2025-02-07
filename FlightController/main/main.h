struct DroneSetting{
    int pwMin, int pwMax;
    int maxPWM, int minPWM;
    
    typedef struct {
    	float old_setpoint;
    	float new_setpoint;
    } altitude_interpol altitude_setpoints;
    
    typedef struct {
    	float K_ROLL,
    	float K_PITCH,
    	float K_YAW,
    } mixing mix;
} DroneSetting;

extern DroneSetting settings;


