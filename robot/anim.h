#ifndef ROBOT_H
#define ROBOT_H 1

#define ANIM_LEN 3
#define SERVO_LEN 3

struct AnimationFrame
{
	// Servo degrees
	float servos[SERVO_LEN];
};

// Transition rate between frames in degrees / ms
struct AnimationTransition
{
	float servos[SERVO_LEN];
};

struct Animation
{
	struct AnimationFrame frames[ANIM_LEN];
	int transitionTimes[ANIM_LEN];
	struct AnimationTransition transitions[ANIM_LEN];
};

struct Leg
{
	// servo indices
	int servos[SERVO_LEN];

	struct Animation animation;

	int state;
	int counter;
};

void computeTransitions(struct Animation *animation);
struct AnimationFrame computeState(struct Animation animation, int *ctr);
void doLegs(void (*legFn)(struct Leg*));

struct Leg leg1 = {
    // Servo indices
    {16, 17, 18},
    // Animation
    {
        // Frames
        {
            {20, 45, 55},
            {30, 55, 55},
            {40, 45, 55},
        },
        // Timing
        {1000000, 500000, 1000000},
    },
    // state
    0,
    // counter
    0,
};

struct Leg leg5 = {
    // Servo indices
    {19, 20, 21},
    // Animation
    {

        // Frames
        {
            {15, 35, 80},
            {25, 45, 80},
            {35, 35, 80},
        },
        // Timing
        {1000000, 500000, 1000000},
    },
    // state
    0,
    // counter
    0,
};

#define NUM_LEGS 2

struct Leg* legs[NUM_LEGS] = {
	&leg1,
	&leg5,
};


#endif