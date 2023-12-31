CFLAGS=-Wall -O3 -g -Wextra -Wno-unused-parameter -Wno-psabi
CXXFLAGS=$(CFLAGS)
OBJECTS=attractor-main.o
BINARIES=attractor plasma-clock

# Where our library resides. You mostly only need to change the
# RGB_LIB_DISTRIBUTION, this is where the library is checked out.
RGB_LIB_DISTRIBUTION=..
RGB_INCDIR=$(RGB_LIB_DISTRIBUTION)/include
RGB_LIBDIR=$(RGB_LIB_DISTRIBUTION)/lib
RGB_LIBRARY_NAME=rgbmatrix
RGB_LIBRARY=$(RGB_LIBDIR)/lib$(RGB_LIBRARY_NAME).a
LDFLAGS+=-L/usr/local/lib -l$(RGB_LIBRARY_NAME) -lrt -lm -lpthread

all : $(BINARIES)

# $(RGB_LIBRARY): FORCE
# 	$(MAKE) -C $(RGB_LIBDIR)

attractor : attractor-main.o # $(RGB_LIBRARY)
	$(CXX) $< -o $@ $(LDFLAGS)

plasma : plasma-clock.o # $(RGB_LIBRARY)
	$(CXX) $< -o $@ $(LDFLAGS)

# All the binaries that have the same name as the object file.q
% : %.o # $(RGB_LIBRARY)
	$(CXX) $< -o $@ $(LDFLAGS)

%.o : %.cc
	$(CXX) -I$(RGB_INCDIR) $(CXXFLAGS) -c -o $@ $<

%.o : %.c
	$(CC) -I$(RGB_INCDIR) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJECTS) $(BINARIES)

FORCE:
.PHONY: FORCE
