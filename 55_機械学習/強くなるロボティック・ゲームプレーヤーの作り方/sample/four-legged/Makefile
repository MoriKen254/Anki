##------------------------------------------------------------------
EXEC = four-legged.exe
EXEC2 = client.exe
EXEC3 = client.oct
SLINKS= ./startSimulation.oct \
	./stepSimulation.oct \
	./getBaseState.oct \
	./getJointState.oct \
	./drawWorld.oct \
	./stopSimulation.oct \
	./resetSimulation.oct
OBJS = four-legged.o
OBJS2 = client.o
OBJS3 = client-oct.o
#OBJS4 = resources.o	#for Cygwin
LIBOBJS =
CXX = g++
CXXFLAGS = -g -Wall -O2 -march=i686
CXXFLAGS += -DdDOUBLE	#for ODE-0.10.x
LDFLAGS  = $(CXXFLAGS)
RMFLAGS  =  -vf

# for ODE
LIBS    = -I/usr/local/include/ode
LDLIBS  = /usr/local/lib/libode.a -lm
LDLIBS +=  /usr/local/lib/libdrawstuff.a
CXXFLAGS += -DODE_MINOR_VERSION=10	#for ODE-0.10.x
#CXXFLAGS += -DODE_MINOR_VERSION=9	#for ODE-0.9.x

# for OpenGL
LDLIBS += -lglut

# for Octave
LIBS   += -I/usr/local/include/octave-`octave-config -v`
LDLIBS += -L/usr/local/lib/octave-`octave-config -v` -loctave -lcruft

# for Cygwin
#LDLIBS += -lopengl32 -lglu32 -lwinmm -lgdi32

##------------------------------------------------------------------

all : a b c
a : $(EXEC)
b : $(EXEC2)
c : $(EXEC3)

clean   :
	@rm $(RMFLAGS) *.o *.a *.il $(EXEC) $(EXEC2) $(EXEC3) $(SLINKS) $(EXTRA_CLEAN)
	@find -name '*.d' -type f | xargs rm $(RMFLAGS)

$(EXEC) : $(OBJS) $(LIBOBJS)
	$(CXX) -o $@ $(OBJS) $(OBJS4) $(LDFLAGS) $(LIBOBJS) $(LDLIBS)
	@(echo "$(EXEC) is generated")

$(EXEC2) : $(OBJS2) $(LIBOBJS)
	$(CXX) -o $@ $(OBJS2) $(LDFLAGS) $(LIBOBJS) $(LDLIBS)
	@(echo "$(EXEC2) is generated")

$(EXEC3) : $(OBJS3) $(LIBOBJS)
	mkoctfile -o $@ $(OBJS3)
	@(for i in $(SLINKS); do \
	if [ -f $$i ]; then rm $$i; fi; \
	ln -vs $(EXEC3) $$i; \
	done)
	@(echo "$(EXEC3) is generated and $(SLINKS) are linked to it")

$(OBJS3) : $(OBJS2:.o=.cpp)
	$(CXX) $(LIBS) $(CXXFLAGS) -c $< -DOUTPUT_OCT -o $@

%.o : %.cpp
	$(CXX) $(LIBS) $(CXXFLAGS) -c $<

DEPOPT  ?= -MM

$(OBJS3:.o=.d) : $(OBJS2:.o=.cpp)
	@(rm -f $@; \
	$(CXX) $(DEPOPT) $(LIBS) $(CXXFLAGS) $< -DOUTPUT_OCT > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1-oct.o $@ : ,g' < $@.$$$$ > $@; \
	[ -s $@ ] || rm -f $@; \
	rm -f $@.$$$$; \
	)

%.d: %.cpp
	@(rm -f $@; \
	$(CXX) $(DEPOPT) $(LIBS) $(CXXFLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	[ -s $@ ] || rm -f $@; \
	rm -f $@.$$$$; \
	)

include $(OBJ:.o=.d) $(OBJS:.o=.d) $(OBJS2:.o=.d) $(OBJS3:.o=.d) $(objects:.o=.d) $(addsuffix .d, $(BINARY))

