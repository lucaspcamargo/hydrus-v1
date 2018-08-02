// periodic thread utility for linux - uclibc version
// adapted from an example by Chris Simmonds 

#pragma once

#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>

struct periodic_info {
  sigset_t alarm_sig;
};

static int make_periodic(unsigned int period, struct periodic_info *info)
{
  int ret;
  struct itimerval value;

  /* Block SIGALRM in this thread */
  sigemptyset(&(info->alarm_sig));
  sigaddset(&(info->alarm_sig), SIGALRM);
  pthread_sigmask(SIG_BLOCK, &(info->alarm_sig), NULL);

  /* Set the timer to go off after the first period and then
     repetitively */
  value.it_value.tv_sec = period / 1000000;
  value.it_value.tv_usec = period % 1000000;
  value.it_interval.tv_sec = period / 1000000;
  value.it_interval.tv_usec = period % 1000000;
  ret = setitimer(ITIMER_REAL, &value, NULL);
  if (ret != 0)
    perror("Failed to set timer");
  return ret;
}

static void wait_period(struct periodic_info *info)
{
  int sig;

  /* Wait for the next SIGALRM */
  sigwait(&(info->alarm_sig), &sig);
}
