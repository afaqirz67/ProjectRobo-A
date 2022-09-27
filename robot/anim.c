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

float offsets[ANIM_LEN][SERVO_LEN] = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    //{10, 30, 20},
    //{20, 0, 0},
};

struct Animation calcAnimation(float inits[SERVO_LEN])
{
  struct Animation ret = {
      {},
      {1000000, 500000, 1000000},
      {},
  };

  for (int i = 0; i < ANIM_LEN; ++i)
  {
    for (int j = 0; j < SERVO_LEN; ++j)
    {
      ret.frames[i].servos[j] = inits[j] + offsets[i][j];
    }
  }

  computeTransitions(&ret);

  return ret;
}

void initLeg(int i, struct Leg *leg)

{
  leg->animation = calcAnimation(leg->init);
  if (i % 2 == 0)
  {
    leg->counter = 0;
  }
  else
  {
    leg->counter = 1000000;
  }
}

struct Leg leg1 = {
    // Servo indices
    {16, 17, 18},
    {90, 45, 105},
    1,
};

struct Leg leg3 = {
    {19, 20, 21},
    {90, 57, 100},
    1,
};

struct Leg leg5 = {
    {22, 23, 24},
    {90, 60, 90},
    1,
};

struct Leg leg2 = {
    {25, 26, 27},
    {90, 35, 10},
    0,
};

struct Leg leg4 = {
    {28, 29, 30},
    {50, 20, 20},
    0,
};

struct Leg leg6 = {
    {0, 1, 2},
    {60, 62, 30},
    0,
};

struct Leg *legs[NUM_LEGS] = {
    //&leg1,
    //&leg3,
    //&leg5,

    &leg2,
    &leg4,
    //&leg6,
};

void doLegs(void (*legFn)(int, struct Leg *))
{

  for (int i = 0; i < NUM_LEGS; ++i)

  {

    (*legFn)(i, legs[i]);
  }
}
