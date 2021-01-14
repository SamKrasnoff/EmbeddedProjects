#include <stdio.h>
#include <stdlib.h>

#define idle 0 
#define up 1
#define down 2
#define pointinc 3

int timer = 0;
int timer_max = 60;
time_t t;
srand((unsigned)time(&t));

int timer_rand = rand() % 10;
int timer_since_reset = 0;
int points = 0;
typedef enum
{
    hit,
    start
} event_e;


int main()
{
    int state = S0;
    while (1)
    {
        switch (state)
        {
        idle:
            if (event_e == start)
            {
                state = down;
                timer_since_reset = 0;
            }
            break;
        down:
            if (timer >= timer_max)
            {
                state = idle;
            }
            else
            {
                if (timer_since_reset >= timer_rand)
                {
                    state = up;
                    timer_since_reset = 0;
                }
            }

            break;
        up:
            if (timer >= timer_max)
            {
                state = idle;
            }
            else
            {
                if (event_e == hit)
                {
                    state = pointinc;
                    timer_since_reset = 0;
                }
                else if (timer_since_reset >= timer_rand)
                {
                    state = down;
                    timer_since_reset = 0;
                }
            }

            break;

        pointinc:
            points++;
            state = down;
            break;
        }
        delay(1000);
        timer_since_reset++;
        timer += 1;
    }
}