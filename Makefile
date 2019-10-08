CC = gcc
cc = g++

definc = -I./Detour/Include -I./DetourTileCache/Include -I./DetourCrowd/Include -I./Recast/Include -I./Include -I./fastlz

incs += $(definc)

csrcs = $(wildcard ./fastlz/*.c)
cppsrcs = $(wildcard ./Detour/Source/*.cpp ./DetourTileCache/Source/*.cpp ./DetourCrowd/Source/*.cpp ./Recast/Source/*.cpp ./Source/*.cpp)

cobjects := $(csrcs:.c=.o)

objects := $(cppsrcs:.cpp=.o)

c_flags += $(incs)
c_flags += -O2 -Wall -g -fPIC
cc_flags += $(incs)
cc_flags += $(CPP_STD_VER) -ggdb -Wall -Werror $(optimization_flag)
#cc_flags += -O2 -ggdb -Wall -Werror -fPIC -std=c++11

target = libnavigation

.PHONY:
all: $(target).a 

$(target).a : $(objects) $(cobjects)
	ar rs $(target).a $(objects) $(cobjects)

$(target).so : $(objects) $(cobjects)
	$(cc) -shared $(objects) $(cobjects) -o $(target).so

%.o: %.c
	$(CC) $(c_flags) -c $< -o $@

%.o: %.cpp
	$(cc) $(cc_flags) -c $< -o $@

%.d: %.cpp
#此行用于调试	@set -e -x; \ #此行用于调试
	@rm -f $@; \
	$(cc) $(cc_flags) -MM $< > $@.$$$$;  \
	sed -r 's,^(.*)\.o:,$@ $*\.o:,' $@.$$$$ > $@;  \
	rm -f $@.$$$$

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(MAKECMDGOALS),distclean)
-include $(objects:.o=.d)
endif
endif

.PHONY: clean distclean
clean:
	rm -f $(objects) $(objects:.o=.d) *.d *.d.* */*.d
	rm -f $(target).a

distclean: clean

