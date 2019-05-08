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
  OBJDIR     = obj/Debug/DebugUtils
  TARGETDIR  = ../lib
  TARGET     = $(TARGETDIR)/libDebugUtils.a
  DEFINES   += -DENABLE_GUI -DENABLE_GLFW -DDEBUG
  INCLUDES  += -I../../navmeshBuilder/include -I../../external/recastnavigation/DebugUtils/Include -I../../external/recastnavigation/Detour/Include -I../../external/recastnavigation/Recast/Include -I../../external/recastnavigation/DetourTileCache/Include -I../../util/include
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
  OBJDIR     = obj/Release/DebugUtils
  TARGETDIR  = ../lib
  TARGET     = $(TARGETDIR)/libDebugUtils.a
  DEFINES   += -DENABLE_GUI -DENABLE_GLFW -DNDEBUG
  INCLUDES  += -I../../navmeshBuilder/include -I../../external/recastnavigation/DebugUtils/Include -I../../external/recastnavigation/Detour/Include -I../../external/recastnavigation/Recast/Include -I../../external/recastnavigation/DetourTileCache/Include -I../../util/include
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
	$(OBJDIR)/DetourDebugDraw.o \
	$(OBJDIR)/RecastDebugDraw.o \
	$(OBJDIR)/RecastDump.o \
	$(OBJDIR)/DebugDraw.o \

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
	@echo Linking DebugUtils
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
	@echo Cleaning DebugUtils
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

$(OBJDIR)/DetourDebugDraw.o: ../../external/recastnavigation/DebugUtils/Source/DetourDebugDraw.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/RecastDebugDraw.o: ../../external/recastnavigation/DebugUtils/Source/RecastDebugDraw.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/RecastDump.o: ../../external/recastnavigation/DebugUtils/Source/RecastDump.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"
$(OBJDIR)/DebugDraw.o: ../../external/recastnavigation/DebugUtils/Source/DebugDraw.cpp
	@echo $(notdir $<)
	$(SILENT) $(CXX) $(CXXFLAGS) -o "$@" -c "$<"

-include $(OBJECTS:%.o=%.d)