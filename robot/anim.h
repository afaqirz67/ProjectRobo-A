#ifndef ROBOT_H
#define ROBOT_H

#if defined(__cplusplus)
extern "C"
{
#endif

#define ANIM_LEN 3
#define SERVO_LEN 3
#define NUM_LEGS 6

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
		// "Keyframes," or states that the animation will move between.
		struct AnimationFrame frames[ANIM_LEN];
		// The time it should take to move from one keyframe to the next.
		int transitionTimes[ANIM_LEN];
		// The transition rate needed to go from one keyframe to the next over
		// the time period.
		// Calculated at runtime, not user-defined.
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
