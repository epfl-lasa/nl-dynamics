#include <iostream>
#include <ctime>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif


class Timer
{
public:
  Timer() {
    current_utc_time(&beg_);
  }

  // Helper for OSX's lack of clock_gettime, see:
  // https://gist.github.com/jbenet/1087739
  void current_utc_time(struct timespec *ts) {

#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
    clock_serv_t cclock;
    mach_timespec_t mts;
    host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    ts->tv_sec = mts.tv_sec;
    ts->tv_nsec = mts.tv_nsec;
#else
    clock_gettime(CLOCK_REALTIME, ts);
#endif

  }

  double elapsed() {
    current_utc_time(&end_);
    return end_.tv_sec - beg_.tv_sec +
      (end_.tv_nsec - beg_.tv_nsec) / 1000000000.;
  }

  void reset() {
    current_utc_time(&beg_);
  }

private:
  timespec beg_, end_;
};
