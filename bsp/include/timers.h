#include <stdint.h>
#include <bsp.h>

typedef enum timerIdentifier {TIMER0, TIMER1} timerIdentifier_t;

void initTimer(timerIdentifier_t timer, pVoidFunc_t handler, uint32_t ticksPerSec);
void startTimer(timerIdentifier_t timer);
void stopTimer(timerIdentifier_t timer);


/* Use TIMER1 as an interval timer */

/* Initialises TIMER1 for use as an interval timer */
void initWatch(void);

/* Starts the interval timer running */
void startWatch(void);

/* Stops the interval timer, captures its result,
 * resets the timer and returns the result 
 */
uint32_t stopWatch(void);
