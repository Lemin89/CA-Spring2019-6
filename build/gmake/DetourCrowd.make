# GNU Make project makefile autogenerated by Premake
ifndef config
  config=debug
endif

ifndef verbose
  SILENT = @
endif

ifndef CC
  CC = gcc
endif

ifndef CXX
  CXX = g++
endif

ifndef AR
  AR = ar
endif

ifeq ($(config),debug)
  OBJDIR     = obj/Debug/DetourCrowd
  TARGETDIR  = ../lib
  TARGET     = $(TARGETDIR)/libDetourCrowd.a
  DEFINES   += -DENABLE_GUI -DENABLE_GLFW -DDEBUG
  INCLUDES  += -I../../navmeshBuilder/include -I../../external/recastnavigation/DebugUtils/Include -I../../external/recastnavigation/Detour/Include -I../../external/recastnavigation/Recast/Include -I../../external/recastnavigation/DetourTileCache/Include -I../../external/recastnavigation/DetourCrowd/Include -I../../util/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -Wall -g -std=c++0x -ggdb -fPIC
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -Wl,-rpath,/home/jetmir/Classes/Spring2019/CharacterAnimation/steersuite-rutgers/build/lib -L../lib
  LIBS      += -lRecast -lDetour
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LDDEPS    += ../lib/libRecast.a ../lib/libDetour.a
  LINKCMD    = $(AR) -rcs $(TARGET) $(OBJECTS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),release)
  OBJDIR     = obj/Release/DetourCrowd
  TARGETDIR  = ../lib
  TARGET     = $(TARGETDIR)/libDetourCrowd.a
  DEFINES   += -DENABLE_GUI -DENABLE_GLFW -DNDEBUG
  INCLUDES  += -I../../navmeshBuilder/include -I../../external/recastnavigation/DebugUtils/Include -I../../external/recastnavigation/Detour/Include -I../../external/recastnavigation/Recast/Include -I../../external/recastnavigation/DetourTileCache/Include -I../../external/recastnavigation/DetourCrowd/Include -I../../util/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -Wall -g -O2 -std=c++0x -ggdb -fPIC
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -Wl,-rpath,/home/jetmir/Classes/Spring2019/CharacterAnimation/steersuite-rutgers/build/lib -L../lib
  LIBS      += -lRecast -lDetour
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LDDEPS    += ../lib/libRecast.a ../lib/libDetour.a
  LINKCMD    = $(AR) -rcs $(TARGET) $(OBJECTS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

OBJECTS := \
	$(OBJDIR)/DetourPathQueue.o \
	$(OBJDIR)/DetourPathCorridor.o \
	$(OBJDIR)/DetourObstacleAvoidance.o \
	$(OBJDIR)/DetourLocalBoundary.o \
	$(OBJDIR)/DetourCrowd.o \
	$(OBJDIR)/DetourProximityGrid.o \

RESOURCES := \

SHELLTYPE := msdos
ifeq (,$(ComSpec)$(COMSPEC))
  SHELLTYPE := posix
endif
ifeq (/bin,$(findstring /bin,$(SHELL)))
  SHELLTYPE := posix
endif

.PHONY: clean prebuild prelink

all: $(TARGETDIR) $(OBJDIR) prebuild prelink $(TARGET)
	@:

$(TARGET): $(GCH) $(OBJECTS) $(LDDEPS) $(RESOURCES)
	@echo Linking DetourCrowd
	$(SILENT) $(LINKCMD)
	$(POSTBUILDCMDS)

$(TARGETDIR):
	@echo Creating $(TARGETDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(TARGETDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(TARGETDIR))
endif

$(OBJDIR):
	@echo Creating $(OBJDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(OBJDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(OBJDIR))
endif

clean:
	@echo Cleaning DetourCrowd
ifeq (posix,$(SHELLTYPE))
	$(SILENT) rm -f  $(TARGET)
	$(SILENT) rm -rf $(OBJDIR)
else
	$(SILENT) if exist $(subst /,\\,$(TARGET)) del $(subst /,\\,$(TARGET))
	$(SILENT) if exist $(subst /,\\,$(OBJDIR)) rmdir /s /q $(subst /,\\,$(OBJDIR))
endif

prebuild:
	$(PREBUILDCMDS)

prelink:
	$(PRELINKCMDS)

ifneq (,$(PCH))
$(GCH): $(PCH)
	@echo $(notdir $<)
	-$(SILENT) cp $< $(OBJDIR)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
endif

$(OBJDIR)/DetourPathQueue.o: ../../external/recastnavigation/DetourCrowd/Source/DetourPathQueue.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/DetourPathCorridor.o: ../../external/recastnavigation/DetourCrowd/Source/DetourPathCorridor.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/DetourObstacleAvoidance.o: ../../external/recastnavigation/DetourCrowd/Source/DetourObstacleAvoidance.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/DetourLocalBoundary.o: ../../external/recastnavigation/DetourCrowd/Source/DetourLocalBoundary.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/DetourCrowd.o: ../../external/recastnavigation/DetourCrowd/Source/DetourCrowd.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/DetourProximityGrid.o: ../../external/recastnavigation/DetourCrowd/Source/DetourProximityGrid.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"

-include $(OBJECTS:%.o=%.d)
