
include Makefile.sources
include Makefile.objects


HAPI_HOME          = ../..
H3DUTIL_HOME          = ../../../H3DUtil

#
#
###############################################################

CXX        = g++ -Wall
SHARED     = -shared -export-dynamic
INCLUDES   = -I$(HAPI_HOME)/src
CXXFLAGS   = -g -Wall $(INCLUDES)
LDFLAGS    = 

###############################################################

#H3DUtil
CXXLIBS   += -lH3DUtil
INCLUDES  += -I$(H3DUTIL_HOME)/src

# OpenGL etc
CXXLIBS   += -lGL -lGLU

all:	libHAPI.so

HAPITest:	$(HAPI_HOME)/src/HAPIMain.o libHAPI.so
	$(CXX) $(LDFLAGS) -o $@ $^ $(CXXLIBS) 

libHAPI.so:	$(OBJECTS)
	$(CXX) $(LDFLAGS) $(CXXLIBS) $(SHARED) -o $@ $(OBJECTS) 

clean:	
	$(RM) $(HAPI_HOME)/src/*~ $(HAPI_HOME)/src/*.o 

install: libHAPI.so
	cp libHAPI.so /usr/lib

update:	
	$(MAKE) sources
	$(MAKE) objects
	$(MAKE) depend

sources:
	ls $(HAPI_HOME)/src/*.cpp > Makefile.sources
	echo -n "SOURCES=" > Makefile.sources2
	cat Makefile.sources \
	  | sed 's/.cpp/.cpp \\/g' \
	  >> Makefile.sources2
	mv Makefile.sources2 Makefile.sources

objects:
	ls $(HAPI_HOME)/src/*.cpp  > Makefile.objects
	echo -n "OBJECTS=" > Makefile.objects2
	cat Makefile.objects \
	  | grep -v "HAPIMain.cpp" \
	  | sed 's/.cpp/.o \\/g' \
	  >> Makefile.objects2
	mv Makefile.objects2 Makefile.objects

depend:
	touch Makefile.depend
	makedepend -fMakefile.depend $(INCLUDES) $(SOURCES)


include Makefile.depend
