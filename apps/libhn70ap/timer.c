/****************************************************************************
 * hn70ap/apps/libhn70ap/timer.c
 *
 *   Copyright (C) 2018 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <semaphore.h>
#include <time.h>

#include <hn70ap/timer.h>

struct timer_s
{
  struct timer_s *next; /* List link */
  int             id; /* Timer identifier, for deletion */
  struct timespec expiration; /* Expiration date, checked in timer thread */
  uint32_t        repeat_delay; /* If zero, one shoot, else, reload delay */
  void           *arg; /* Callback argument */
  void           (*callback)(void *arg); /* Timer callback */
};

struct timer_s *g_timers_head;
struct timer_s *g_timers_tail;

void *timer_thread(void *arg)
{
  sem_t sem;
  struct timespec tout;

  sem_init(&sem, 0, 0);

  printf("Starting timer thread\n");

  while(true)
    {
      clock_gettime(CLOCK_REALTIME, &tout);
      tout.tv_nsec += 100000000;
      if(tout.tv_nsec > 1000000000)
        {
          tout.tv_nsec -= 1000000000;
          tout.tv_sec  += 1;
        }
      sem_timedwait(&sem, &tout);
      printf("tick\n");
    }
}

int timer_init(void)
{
  g_timers_head = NULL;
  g_timers_tail = NULL;
  return 0;
}

int timer_add(void (*callback)(void*arg), void *arg, uint32_t delay, bool repeat)
{
}

