/*
This JobServer class is modified from the ThreadPool class, as written by Jakob
Progsch and Václav Zeman. Their original copyright is contained below, and
their original implementation can be found at
https://github.com/progschj/ThreadPool

Copyright (c) 2012 Jakob Progsch, Václav Zeman

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

   1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.

   2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.

   3. This notice may not be removed or altered from any source distribution.
*/


#ifndef JOBSERVER_H
#define JOBSERVER_H

#include <algorithm>
#include <future>
#include <functional>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>

#include "task.h"

class JobServer {
  public:
    JobServer(size_t threads);
    ~JobServer();

    std::future<Status> q(Task * t);

  private:
    std::queue<std::function<void()> > ready;
    std::vector<std::pair<std::function<bool()>, std::function<void()> > > waiting;
    std::vector<std::thread> workers;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;

};

inline JobServer::JobServer(size_t threads) : stop(false) {
  for(size_t t = 0; t < threads; ++t) {
    workers.emplace_back(
        [this] {
          for (;;) {
            std::function<void()> task;
            {
              std::unique_lock<std::mutex> lock(this->queue_mutex);
              this->condition.wait(lock,
                  [this]{ return this->stop || !this->ready.empty(); });
              if (this->stop && this->ready.empty())
                return;
              task = std::move(this->ready.front());
              this->ready.pop();
            }
            task();
            // Something finished running, which means any number of tasks may
            // now be ready.
            int numNewTasks = 0;
            {
              std::unique_lock<std::mutex> lock(this->queue_mutex);
              auto readyJobs = std::remove_if(waiting.begin(), waiting.end(),
                  [this, &numNewTasks]
                  (std::pair<std::function<bool()>, std::function<void()>> f)
                  -> bool {
                    if (f.first()) {
                      this->ready.push(f.second);
                      numNewTasks++;
                      return true;
                    }
                  return false;
                  });
              waiting.erase(readyJobs, waiting.end());
            }
            // Wake up all waiting threads, so they look for new
            // tasks.
            while (numNewTasks > 0) {
              this->condition.notify_one();
              numNewTasks--;
            }
          }
        }
        );
  }
}

inline JobServer::~JobServer() {
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    stop = true;
  }
  condition.notify_all();
  for(std::thread &worker: workers) {
    worker.join();
  }
}

inline auto JobServer::q(Task * t)
    -> std::future<Status> {
  auto task = std::make_shared< std::packaged_task<Status()> >(
          std::bind(&Task::operator(), t)
      );
  std::future<Status> res = task->get_future();
  bool added = false;
  {
    std::unique_lock<std::mutex> lock(queue_mutex);

    // don't allow enqueueing after stopping the pool
    if(stop)
        throw std::runtime_error("enqueue on stopped ThreadPool");
    if (t->isReady()) {
      this->ready.emplace([task](){ (*task)(); });
      added = true;
    } else {
      auto ready = std::make_shared< std::function<bool()> >(
            std::bind(&Task::isReady, t) );
      this->waiting.emplace_back([ready]() -> bool { return (*ready)(); }, [task](){ (*task)(); });
    }
  }
  if (added)
    condition.notify_one();
  return res;
}

#endif /* JOBSERVER_H */
