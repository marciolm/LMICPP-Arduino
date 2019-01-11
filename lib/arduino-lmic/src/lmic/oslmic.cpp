/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *    Nicolas Graziano - cpp style.
 *******************************************************************************/

#include "../hal/print_debug.h"
#include "lmic.h"
#include <stdbool.h>

OsJobBase::OsJobBase(OsScheduler &scheduler) : scheduler(scheduler) {}

OsJob::OsJob(OsScheduler &scheduler) : OsJobBase(scheduler){};

void OsJob::setCallbackRunnable(osjobcb_t cb) {
  setCallbackFuture(cb);
  setRunnable();
}

// schedule immediately runnable job
void OsJobBase::setRunnable() {
  // remove if job was already queued
  scheduler.unlinkScheduledJobs(this);
  scheduler.unlinkRunableJobs(this);

  scheduler.linkRunableJob(this);
}

void OsScheduler::unlinkjob(OsJobBase **pnext, OsJobBase *job) {
  for (; *pnext; pnext = &((*pnext)->next)) {
    if (*pnext == job) { // unlink
      *pnext = job->next;
    }
  }
}

void OsScheduler::linkScheduledJob(OsJobBase *job) {
  const OsTime time = job->deadline;
  job->next = nullptr;
  OsJobBase **pnext;
  // insert into schedule
  for (pnext = &scheduledjobs; *pnext; pnext = &((*pnext)->next)) {
    if ((*pnext)->deadline > time) {
      // enqueue before next element and stop
      job->next = *pnext;
      break;
    }
  }
  *pnext = job;
}

void OsScheduler::linkRunableJob(OsJobBase *job) {
  // add to end of run queue
  job->next = nullptr;
  OsJobBase **pnext;
  for (pnext = &runnablejobs; *pnext; pnext = &((*pnext)->next))
    ;
  *pnext = job;
}

void OsScheduler::unlinkScheduledJobs(OsJobBase *job) {
  unlinkjob(&scheduledjobs, job);
}

void OsScheduler::unlinkRunableJobs(OsJobBase *job) {
  unlinkjob(&runnablejobs, job);
}

void OsScheduler::allowSleep() { is_sleep_allow = true; }

void OsScheduler::forbidSleep() { is_sleep_allow = false; }

bool OsScheduler::isSleepAllow() const { return is_sleep_allow; }

// clear scheduled job
void OsJobBase::clearCallback() {
  scheduler.allowSleep();
  scheduler.unlinkScheduledJobs(this);
  scheduler.unlinkRunableJobs(this);
}

void OsJobBase::forbidSleep() { scheduler.forbidSleep(); }

void OsJobBase::allowSleep() { scheduler.allowSleep(); }

void OsJob::setTimedCallback(OsTime time, osjobcb_t cb) {
  setCallbackFuture(cb);
  setTimed(time);
}

// schedule timed job
void OsJobBase::setTimed(OsTime time) {
  // remove if job was already queued
  scheduler.unlinkScheduledJobs(this);
  scheduler.unlinkRunableJobs(this);
  // fill-in job
  deadline = time;
  scheduler.linkScheduledJob(this);
  PRINT_DEBUG(2, F("Scheduled job %p, atRun %lu\n"), this, time);
}

void OsJob::call() const { func(); }

OsDeltaTime OsScheduler::runloopOnce() {

  bool has_deadline = false;

  OsJobBase const *j = nullptr;
  
  if (runnablejobs) {
    // Runnable jobs immediatly
    j = runnablejobs;
    runnablejobs = j->next;
  } else if (scheduledjobs && hal_checkTimer(scheduledjobs->deadline)) {
    // timed jobs runnable
    j = scheduledjobs;
    scheduledjobs = j->next;
    has_deadline = true;
  }

  if (j) { // run job callback
    PRINT_DEBUG(2, F("Running job %p, deadline %lu\n"), j,
                has_deadline ? j->deadline.tick() : 0);
    j->call();
  }

  if (!runnablejobs && scheduledjobs && isSleepAllow()) {
    // return the number of milisecond to wait ()
    return scheduledjobs->deadline - hal_ticks();
  }
  // need to run now or nothing to do
  return OsDeltaTime(0);
}

void os_init() { hal_init(); }

OsTime os_getTime() { return hal_ticks(); }
