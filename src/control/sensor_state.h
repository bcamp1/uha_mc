#ifndef SENSOR_STATE_H
#define SENSOR_STATE_H


/*
 * Partial state array looks like:
    0 -> theta1;
    1 -> theta2;
    2 -> tape_position;
    3 -> tape_speed;
    4 -> tension1;
    5 -> tension2;
 */
#define SENSOR_STATE_LEN (6)
typedef struct { 
    float state[SENSOR_STATE_LEN];
} SensorState;


#define SENSOR_STATE_IND_THETA1         (0)
#define SENSOR_STATE_IND_THETA2         (1)
#define SENSOR_STATE_IND_TAPE_POSITION  (2)
#define SENSOR_STATE_IND_TAPE_SPEED     (3)
#define SENSOR_STATE_IND_TENSION1       (4) 
#define SENSOR_STATE_IND_TENSION2       (5)

SensorState sensor_state_add(SensorState a, SensorState b);
SensorState sensor_state_sub(SensorState a, SensorState b);
SensorState sensor_state_scale(SensorState a, float c);
float sensor_state_dot(SensorState a, float* k);

#endif
