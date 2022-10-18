#include "anim.h"

// From a list of animation "frames" and timings, calculate the delta needed for
// each ms increment.
void computeTransitions(struct Animation *animation)
{
  for (int current = 0; current < ANIM_LEN; ++current)
  {
    int next = (current + 1) % ANIM_LEN;

    // The current keyframe
    struct AnimationFrame currentFrame = animation->frames[current];
    // The next keyframe
    struct AnimationFrame nextFrame = animation->frames[next];

    // The amount of time it should take to go from current -> next
    int transitionMs = animation->transitionTimes[current];

    struct AnimationTransition transition = {0, 0, 0};

    for (int servo = 0; servo < SERVO_LEN; ++servo)
    {
      // The degrees per ms needed to go from current -> next for this servo
      transition.servos[servo] = ((float)nextFrame.servos[servo] - (float)currentFrame.servos[servo]) / (float)transitionMs;
    }
    animation->transitions[current] = transition;
  }
}

// Calculate the servo angles at the given counter.
// Modifies the counter to keep it in the proper range.
// The counter should be incremented by the number of ms since the last
// animation update.
struct AnimationFrame computeState(struct Animation animation, int *ctr)
{
  // Calculate the overall time for the animation
  int total = 0;
  for (int i = 0; i < ANIM_LEN; i++)
  {
    total += animation.transitionTimes[i];
  }

  // Clamp the counter to the total animation time.
  *ctr %= total;

  int tmpCtr = *ctr;

  // Find the frame that we should be in given the current counter
  // After this, tmpCtr should be the offset *within* the found frame.
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

  // The current keyframe and its deltas to get to the next one
  struct AnimationFrame lastKey = animation.frames[frame];
  struct AnimationTransition trans = animation.transitions[frame];

  // Calculate the new angles from the base keyframe + (delta * time)
  struct AnimationFrame outState;
  for (int s = 0; s < SERVO_LEN; s++)
  {
    outState.servos[s] = lastKey.servos[s] + trans.servos[s] * tmpCtr;
  }

  return outState;
}

// Calculate the animation for the base servo values for a let.
// Uses static offsets and timings for the actual animations.
struct Animation calcAnimation(float inits[SERVO_LEN])
{
  // Keyframes a leg should go through.
  float offsets[ANIM_LEN][SERVO_LEN] = {
      {0, 0, 0},
      {10, 30, 20},
      {20, 0, 0},
  };

  // "blank" animation with only timing information.
  struct Animation ret = {
      {},
      {1000000, 500000, 1000000},
      {},
  };

  // Compute the angles for each servo for each frame
  for (int i = 0; i < ANIM_LEN; ++i)
  {
    for (int j = 0; j < SERVO_LEN; ++j)
    {
      ret.frames[i].servos[j] = inits[j] + offsets[i][j];
    }
  }

  // calculate the deltas needed to go from each frame to the next.
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
    {13, 14, 15},
    {90, 45, 105},
    0,
};

struct Leg leg3 = {
    {19, 20, 21},
    {90, 55, 95},
    0,
};

struct Leg leg5 = {
    {22, 23, 24},
    {90, 43, 110},
    0,
};

struct Leg leg2 = {
    {25, 26, 27},
    {90, 140, 20},
    1,
};

struct Leg leg4 = {
    {28, 29, 30},
    {90, 140, 10},
    1,
};

struct Leg leg6 = {
    {0, 1, 2},
    {90, 140, 80},
    1,
};

struct Leg *legs[NUM_LEGS] = {
    // Right legs
    &leg1,
    &leg3,
    &leg5,

    // Left legs
    &leg2,
    &leg4,
    &leg6,
};

void doLegs(void (*legFn)(int, struct Leg *))
{
  for (int i = 0; i < NUM_LEGS; ++i)
  {
    (*legFn)(i, legs[i]);
  }
}
