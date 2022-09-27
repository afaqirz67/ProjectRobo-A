#ifndef ROBOT_H
#define ROBOT_H

#if defined(__cplusplus)
extern "C"
{
#endif

#define ANIM_LEN 3
#define SERVO_LEN 3
#define NUM_LEGS 2

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
		float init[SERVO_LEN];
		int reflect;

		struct Animation animation;

		int state;
		int counter;
	};

	void initLeg(int i, struct Leg *leg);
	struct AnimationFrame computeState(struct Animation animation, int *ctr);
	void doLegs(void (*legFn)(int, struct Leg *));

#if defined(__cplusplus)
}
#endif

#endif
