#ifndef ROBOT_H
#define ROBOT_H

#if defined(__cplusplus)
extern "C"
{
#endif

#define ANIM_LEN 3
#define SERVO_LEN 3
#define NUM_LEGS 2

struct Leg *legs[NUM_LEGS];


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
void doLegs(void (*legFn)(struct Leg *));

#if defined(__cplusplus)
}
#endif

#endif