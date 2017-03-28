/*

k-PPM - An implementation of the k-PPM method for multi-objective optimisation
Copyright (C) 2017 William Pettersson <william.pettersson@gmail.com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/


#include <iostream>
#include <iomanip>
#include <fstream>
#include <queue>
#include <vector>

#include <ilcplex/cplex.h>

#include <boost/program_options.hpp>

#include "hash.h"
#include "gather.h"
#include "jobserver.h"
#include "p1task.h"
#include "p2task.h"
#include "problem.h"
#include "env.h"



namespace po = boost::program_options;

#ifdef DEBUG
std::mutex debug_mutex;
#endif


std::atomic<int> ipcount;

int main(int argc, char* argv[]) {

  int status = 0; /* Operation status */
  ipcount = 0;
  Env e;

  std::string pFilename, outputFilename;

  int numSteps;

  /* Timing */
  clock_t starttime, endtime;
  double cpu_time_used, elapsedtime, startelapsed;
  int num_threads;

  po::variables_map v;
  po::options_description opt("Options for aira");
  opt.add_options()
    ("help,h", "Show this help.")
    ("lp,p",
      po::value<std::string>(&pFilename),
     "The LP file to solve. Required.")
    ("output,o",
      po::value<std::string>(&outputFilename),
     "The output file. Required.")
    ("threads,t",
      po::value<int>(&num_threads)->default_value(1),
     "Number of threads to use internally. Optional, default to 1.")
    ("steps,s",
      po::value<int>(&numSteps)->default_value(1),
     "Number of steps to take along each objective function when splitting up the search space. Optional, default to 1.")
  ;

  po::store(po::parse_command_line(argc, argv, opt), v);
  po::notify(v);

  if (v.count("help")) {
    // usage();
    std::cout << opt << std::endl;
    return(1);
  }

  if (v.count("lp") == 0) {
    std::cerr << "Error: You must pass in a problem file." << std::endl;
    std::cerr << opt << std::endl;
    return(1);
  }

  std::ofstream outFile;

  if (v.count("output") == 0) {
    std::cerr << "Error: You must pass in an output file." << std::endl;
    std::cerr << opt << std::endl;
    return(1);
  }


  /* Start the timer */
  starttime = clock();
  timespec start;
  clock_gettime(CLOCK_MONOTONIC, &start);
  startelapsed = start.tv_sec + start.tv_nsec/1e9;

  /* Initialize the CPLEX environment */
  e.env = CPXopenCPLEX (&status);

  Problem p(pFilename.c_str(), e);

  int objCount = p.objcnt;
  JobServer server(num_threads);
  int * objectives = new int[objCount];
  std::vector<std::vector<P1Task *> *> allTasks;
  for(int i = 0; i < objCount; ++i) {
    allTasks.push_back(new std::vector<P1Task *>);
  }
  std::queue<P1Task *> addPreReqs;
  for(int i = 1; i < (1 << objCount); ++i) {
    int numAdded = 0;
    for(int j = 0; j < objCount; ++j) {
      int isInSet = (int) i & 1 << j;
      if (isInSet > 0) {
        objectives[numAdded] = j;
        numAdded++;
      }
    }
    P1Task *t = new P1Task(pFilename, numAdded, p.objcnt, p.objsen, objectives, numSteps, &server);
    allTasks[numAdded-1]->push_back(t);
    addPreReqs.push(t);
  }

  while (! addPreReqs.empty()) {
    P1Task *here = addPreReqs.front();
    addPreReqs.pop();
    int k = here->objCount();
    if (k == 1)
      continue;
    for( int i = 0; i < k; ++i) {
      int obj = here->objective(i);
      // Check to see if other is a pre-req of here.
      // other is a pre-req if all it's objectives are objectives of here as
      // well, and if obj is not an objective of other.
      for( P1Task *other: *allTasks[k-2]) {
        bool doesntHaveObj = true;
        bool hasAllOthers = true;
        for( int j = 0; j < k-1; ++j) {
          int inOther = other->objective(j);
          if (obj == inOther) {
            doesntHaveObj = false;
            break;
          } else {
            // Check to see that here has objective inOther
            bool found = false;
            for(int l = 0; l < k; ++l) {
              if (here->objective(l) == inOther) {
                found = true;
                break;
              }
            }
            // If it doesn't, then other has an objective that here doesn't
            // so other can't be a pre-requisite.
            if (! found) {
              hasAllOthers = false;
              break;
            }
          }
        }
        if (hasAllOthers && doesntHaveObj) {
          here->addPreReq(other);
          other->addNextLevel(here);
        }
      }
    }
  }

  // Create final grouping task
  Task * g = new Gather(pFilename, objCount, objectives, p.objsen);
  for(auto t: *allTasks[objCount-1]) {
    t->addNextLevel(g);
    g->addPreReq(t);
  }
  std::vector<std::future<Status> > results;
  for(auto l: allTasks) {
    for(Task *t: *l) {
      results.push_back(server.q(t));
    }
  }

  results.push_back(server.q(g));

  delete[] objectives;
  for(auto & jobs: results) {
    jobs.wait();
  }

  for(auto l: allTasks) {
    for(Task *t: *l) {
      delete t;
    }
    delete l;
  }
  /* Stop the clock. Sort and print results.*/
  endtime = clock();
  cpu_time_used=((double) (endtime - starttime)) / CLOCKS_PER_SEC;
  clock_gettime(CLOCK_MONOTONIC, &start);
  elapsedtime = (start.tv_sec + start.tv_nsec/1e9 - startelapsed);

  constexpr int width = 8;
  constexpr int precision = 3;
  outFile.open(outputFilename);
  outFile << std::endl << "Using k-PPM at " << HASH << std::endl;
  g->cleanSolutions();
  g->dumpSolutions(outFile);
  outFile << std::endl << "---" << std::endl;
  int solCount = g->solutions().size();
  outFile << cpu_time_used << " CPU seconds" << std::endl;
  outFile << std::setw(width) << std::setprecision(precision) << std::fixed;
  outFile << elapsedtime << " elapsed seconds" << std::endl;
  outFile << std::setw(width) << std::setprecision(precision) << std::fixed;
  outFile << ipcount << " IPs solved" << std::endl;
  outFile << std::setw(width) << std::setprecision(precision) << std::fixed;
  outFile << solCount << " Solutions found" << std::endl;
  return 0;
}
