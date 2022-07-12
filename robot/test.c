#include "anim.h"
#include <stdio.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

void computeLegAnimations(struct Leg* leg) {
  computeTransitions(&leg->animation);
}

void setup() {
	doLegs(computeLegAnimations);
}

void loop() {

}

int now = 0;
int last = 0;

int main(void) {
	setup();

	for(int i = 0; i < 1000; i++) {
		for(int l = 0; l < NUM_LEGS; l++) {
			legs[l]->counter += 1000;
			struct Animation anim = legs[l]->animation;
			struct AnimationFrame frame = computeState(anim, &legs[l]->counter);
			printf("leg %d\n", l);
			for(int s = 0; s < SERVO_LEN; s++) {
				printf("\tservo %d: %g\n", s, frame.servos[s]);
			}
		}
	}
}