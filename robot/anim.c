#include "anim.h"

void computeTransitions(struct Animation *animation)
{
  for (int current = 0; current < ANIM_LEN; ++current)
  {
    int next = (current + 1) % ANIM_LEN;

    struct AnimationFrame currentFrame = animation->frames[current];
    struct AnimationFrame nextFrame = animation->frames[next];
    int transitionMs = animation->transitionTimes[current];

    struct AnimationTransition transition = {0, 0, 0};

    for (int servo = 0; servo < SERVO_LEN; ++servo)
    {
      transition.servos[servo] = ((float)nextFrame.servos[servo] - (float)currentFrame.servos[servo]) / (float)transitionMs;
    }

    animation->transitions[current] = transition;
  }
}

struct AnimationFrame computeState(struct Animation animation, int *ctr)
{
  int total = 0;
  for (int i = 0; i < ANIM_LEN; i++)
  {
    total += animation.transitionTimes[i];
  }

  *ctr %= total;

  int tmpCtr = *ctr;

  // Find the frame that we should be in given the current counter
  int frame;
  for (frame = 0; frame < ANIM_LEN; frame++)  
  {
    int thisTime = animation.transitionTimes[frame];
    if (tmpCtr < thisTime)
    {
      break;
    }
    tmpCtr -= thisTime;
  }
  // state -> last keyframe/transition
  struct AnimationFrame lastKey = animation.frames[frame];
  struct AnimationTransition trans = animation.transitions[frame];

  // tmpCtr -> offset from the start of the state
  struct AnimationFrame outState;

  for (int s = 0; s < SERVO_LEN; s++)
  {
    outState.servos[s] = lastKey.servos[s] + trans.servos[s] * tmpCtr;
  }

  return outState;
}

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


struct Leg leg3 = {
    // Servo indices
    {19, 20, 21},
    // Animation
    {

        // Frames
        {
            {15, 25, 80},
            {25, 35, 80},
            {35, 25, 80},
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
    {22, 23, 24},
    // Animation
    {

        // Frames
        {
            {25, 35, 70},
            {35, 45, 70},
            {45, 35, 70},
        },
        // Timing
        {1000000, 500000, 1000000},
    },
    // state
    0,
    // counter
    0,
};


struct Leg *legs[NUM_LEGS] = {
    &leg1,
    &leg3,
    &leg5,
};

void doLegs(void (*legFn)(struct Leg *))
{
  for (int i = 0; i < NUM_LEGS; ++i)
  {
    (*legFn)(legs[i]);
  }
}
