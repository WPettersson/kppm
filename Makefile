SRC = ./src
TARGETDIR = ./build
#TARGETDIR = ./debug
#DEBUG_FLAGS = -g -DDEBUG

# Extra debugging flags.
# -DDEBUG_SOLUTION_SEARCH
# -DDEBUG_SYNC
CPLEXDIR=/home/enigma/opt/ibm/ILOG/CPLEX_Studio127
CXX = g++
CFLAGS = $(DEBUG_FLAGS) -Wextra -std=c++11 -pedantic -I$(CPLEXDIR)/cplex/include/
CLNFLAGS = -L$(CPLEXDIR)/cplex/lib/x86-64_linux/static_pic/
LIBS=-pthread -lcplex -lboost_program_options

HASH=$(shell git rev-parse HEAD)

all: executable

executable: update-hash $(TARGETDIR)/kppm

OBJS = $(TARGETDIR)/main.o $(TARGETDIR)/p1task.o $(TARGETDIR)/p2task.o $(TARGETDIR)/solutions.o $(TARGETDIR)/result.o $(TARGETDIR)/problem.o $(TARGETDIR)/p3task.o $(TARGETDIR)/p3creator.o $(TARGETDIR)/box.o

$(TARGETDIR):
	mkdir -p $(TARGETDIR)

clean:
	rm -Rf ./build ./debug

test: executable
	@TARGETDIR=.$(TARGETDIR) make -C tests

update-hash:
	@echo "const std::string HASH =\"$(HASH)\";" > $(SRC)/hash.h

$(TARGETDIR)/kppm: $(TARGETDIR) $(OBJS)
	$(CXX) $(OBJS) $(LIBS) -o $(TARGETDIR)/kppm $(CLNFLAGS)


$(TARGETDIR)/main.o: $(SRC)/p1task.h $(SRC)/p2task.h $(SRC)/main.cpp $(SRC)/jobserver.h $(SRC)/task.h $(SRC)/gather.h
	$(CXX) -c $(CFLAGS) -o $@ $(SRC)/main.cpp

$(TARGETDIR)/solutions.o: $(SRC)/solutions.h $(SRC)/solutions.cpp $(SRC)/sense.h $(SRC)/result.h
	$(CXX) -c $(CFLAGS) -o $@ $(SRC)/solutions.cpp

$(TARGETDIR)/result.o: $(SRC)/result.h $(SRC)/result.cpp
	$(CXX) -c $(CFLAGS) -o $@ $(SRC)/result.cpp

$(TARGETDIR)/problem.o: $(SRC)/problem.h $(SRC)/problem.cpp
	$(CXX) -c $(CFLAGS) -o $@ $(SRC)/problem.cpp

$(TARGETDIR)/p1task.o: $(SRC)/p1task.h $(SRC)/p1task.cpp $(SRC)/task.h $(SRC)/jobserver.h
	$(CXX) -c $(CFLAGS) -o $@ $(SRC)/p1task.cpp

$(TARGETDIR)/p2task.o: $(SRC)/p2task.h $(SRC)/p2task.cpp $(SRC)/task.h $(SRC)/jobserver.h
	$(CXX) -c $(CFLAGS) -o $@ $(SRC)/p2task.cpp

$(TARGETDIR)/p3task.o: $(SRC)/p3task.h $(SRC)/p3task.cpp $(SRC)/task.h $(SRC)/jobserver.h
	$(CXX) -c $(CFLAGS) -o $@ $(SRC)/p3task.cpp

$(TARGETDIR)/p3creator.o: $(SRC)/p3creator.h $(SRC)/p3creator.cpp $(SRC)/task.h $(SRC)/jobserver.h
	$(CXX) -c $(CFLAGS) -o $@ $(SRC)/p3creator.cpp

$(TARGETDIR)/box.o: $(SRC)/box.h $(SRC)/box.cpp $(SRC)/boxstore.h
	$(CXX) -c $(CFLAGS) -o $@ $(SRC)/box.cpp
