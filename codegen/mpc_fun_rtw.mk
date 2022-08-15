###########################################################################
## Makefile generated for component 'mpc_fun'. 
## 
## Makefile     : mpc_fun_rtw.mk
## Generated on : Tue Aug 09 13:32:29 2022
## Final product: ./mpc_fun.a
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# MODELLIB                Static library target

PRODUCT_NAME              = mpc_fun
MAKEFILE                  = mpc_fun_rtw.mk
MATLAB_ROOT               = /home/ffn/Software/MATLAB
MATLAB_BIN                = /home/ffn/Software/MATLAB/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/glnxa64
START_DIR                 = /home/ffn/Nextcloud/CableLandingControl/Work/ICRA2023/codegen/lib/mpc_fun
TGT_FCN_LIB               = ISO_C++11
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = .
C_STANDARD_OPTS           = -fwrapv
CPP_STANDARD_OPTS         = -fwrapv
MODELLIB                  = mpc_fun.a

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          GNU gcc/g++ | gmake (64-bit Linux)
# Supported Version(s):    
# ToolchainInfo Version:   2021a
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# C_STANDARD_OPTS
# CPP_STANDARD_OPTS

#-----------
# MACROS
#-----------

WARN_FLAGS         = -Wall -W -Wwrite-strings -Winline -Wstrict-prototypes -Wnested-externs -Wpointer-arith -Wcast-align
WARN_FLAGS_MAX     = $(WARN_FLAGS) -Wcast-qual -Wshadow
CPP_WARN_FLAGS     = -Wall -W -Wwrite-strings -Winline -Wpointer-arith -Wcast-align
CPP_WARN_FLAGS_MAX = $(CPP_WARN_FLAGS) -Wcast-qual -Wshadow

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = 

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: GNU C Compiler
CC = gcc

# Linker: GNU Linker
LD = g++

# C++ Compiler: GNU C++ Compiler
CPP = g++

# C++ Linker: GNU C++ Linker
CPP_LD = g++

# Archiver: GNU Archiver
AR = ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE_PATH = %MATLAB%/bin/glnxa64
MAKE = "$(MAKE_PATH)/gmake"


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @rm -f
ECHO                = @echo
MV                  = @mv
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = ruvs
CFLAGS               = -c $(C_STANDARD_OPTS) -fPIC \
                       -O3 -fno-loop-optimize -fno-aggressive-loop-optimizations
CPPFLAGS             = -c $(CPP_STANDARD_OPTS) -fPIC \
                       -O3 -fno-loop-optimize -fno-aggressive-loop-optimizations
CPP_LDFLAGS          =
CPP_SHAREDLIB_LDFLAGS  = -shared -Wl,--no-undefined
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              =
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -shared -Wl,--no-undefined



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = ./mpc_fun.a
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I/home/ffn/Nextcloud/CableLandingControl/Work/ICRA2023 -I$(MATLAB_ROOT)/extern/include

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_CUSTOM = 
DEFINES_STANDARD = -DMODEL=mpc_fun

DEFINES = $(DEFINES_CUSTOM) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/mpc_fun_data.cpp $(START_DIR)/rt_nonfinite.cpp $(START_DIR)/rtGetNaN.cpp $(START_DIR)/rtGetInf.cpp $(START_DIR)/mpc_fun_initialize.cpp $(START_DIR)/mpc_fun_terminate.cpp $(START_DIR)/eye.cpp $(START_DIR)/quadprog.cpp $(START_DIR)/xnrm2.cpp $(START_DIR)/mpc_fun.cpp $(START_DIR)/compressBounds.cpp $(START_DIR)/loadProblem.cpp $(START_DIR)/setProblemType.cpp $(START_DIR)/modifyOverheadPhaseOne_.cpp $(START_DIR)/driver.cpp $(START_DIR)/PresolveWorkingSet.cpp $(START_DIR)/factorQRE.cpp $(START_DIR)/xzgeqp3.cpp $(START_DIR)/xzlarfg.cpp $(START_DIR)/xzlarf.cpp $(START_DIR)/xgerc.cpp $(START_DIR)/computeQ_.cpp $(START_DIR)/countsort.cpp $(START_DIR)/removeConstr.cpp $(START_DIR)/feasibleX0ForWorkingSet.cpp $(START_DIR)/factorQR.cpp $(START_DIR)/xgemv.cpp $(START_DIR)/maxConstraintViolation.cpp $(START_DIR)/phaseone.cpp $(START_DIR)/computeFval.cpp $(START_DIR)/linearForm_.cpp $(START_DIR)/iterate.cpp $(START_DIR)/computeGrad_StoreHx.cpp $(START_DIR)/computeFval_ReuseHx.cpp $(START_DIR)/xrotg.cpp $(START_DIR)/deleteColMoveEnd.cpp $(START_DIR)/compute_deltax.cpp $(START_DIR)/partialColLDL3_.cpp $(START_DIR)/fullColLDL2_.cpp $(START_DIR)/feasibleratiotest.cpp $(START_DIR)/ratiotest.cpp $(START_DIR)/addBoundToActiveSetMatrix_.cpp $(START_DIR)/computeFirstOrderOpt.cpp

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = mpc_fun_data.o rt_nonfinite.o rtGetNaN.o rtGetInf.o mpc_fun_initialize.o mpc_fun_terminate.o eye.o quadprog.o xnrm2.o mpc_fun.o compressBounds.o loadProblem.o setProblemType.o modifyOverheadPhaseOne_.o driver.o PresolveWorkingSet.o factorQRE.o xzgeqp3.o xzlarfg.o xzlarf.o xgerc.o computeQ_.o countsort.o removeConstr.o feasibleX0ForWorkingSet.o factorQR.o xgemv.o maxConstraintViolation.o phaseone.o computeFval.o linearForm_.o iterate.o computeGrad_StoreHx.o computeFval_ReuseHx.o xrotg.o deleteColMoveEnd.o compute_deltax.o partialColLDL3_.o fullColLDL2_.o feasibleratiotest.o ratiotest.o addBoundToActiveSetMatrix_.o computeFirstOrderOpt.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS =  -lm -lstdc++

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################

###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	@echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) $(OBJS)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/ffn/Nextcloud/CableLandingControl/Work/ICRA2023/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : /home/ffn/Nextcloud/CableLandingControl/Work/ICRA2023/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


mpc_fun_data.o : $(START_DIR)/mpc_fun_data.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rt_nonfinite.o : $(START_DIR)/rt_nonfinite.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rtGetNaN.o : $(START_DIR)/rtGetNaN.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rtGetInf.o : $(START_DIR)/rtGetInf.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


mpc_fun_initialize.o : $(START_DIR)/mpc_fun_initialize.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


mpc_fun_terminate.o : $(START_DIR)/mpc_fun_terminate.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


eye.o : $(START_DIR)/eye.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


quadprog.o : $(START_DIR)/quadprog.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xnrm2.o : $(START_DIR)/xnrm2.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


mpc_fun.o : $(START_DIR)/mpc_fun.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


compressBounds.o : $(START_DIR)/compressBounds.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


loadProblem.o : $(START_DIR)/loadProblem.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


setProblemType.o : $(START_DIR)/setProblemType.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


modifyOverheadPhaseOne_.o : $(START_DIR)/modifyOverheadPhaseOne_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


driver.o : $(START_DIR)/driver.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


PresolveWorkingSet.o : $(START_DIR)/PresolveWorkingSet.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


factorQRE.o : $(START_DIR)/factorQRE.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xzgeqp3.o : $(START_DIR)/xzgeqp3.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xzlarfg.o : $(START_DIR)/xzlarfg.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xzlarf.o : $(START_DIR)/xzlarf.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xgerc.o : $(START_DIR)/xgerc.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeQ_.o : $(START_DIR)/computeQ_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


countsort.o : $(START_DIR)/countsort.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


removeConstr.o : $(START_DIR)/removeConstr.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


feasibleX0ForWorkingSet.o : $(START_DIR)/feasibleX0ForWorkingSet.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


factorQR.o : $(START_DIR)/factorQR.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xgemv.o : $(START_DIR)/xgemv.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


maxConstraintViolation.o : $(START_DIR)/maxConstraintViolation.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


phaseone.o : $(START_DIR)/phaseone.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeFval.o : $(START_DIR)/computeFval.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


linearForm_.o : $(START_DIR)/linearForm_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


iterate.o : $(START_DIR)/iterate.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeGrad_StoreHx.o : $(START_DIR)/computeGrad_StoreHx.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeFval_ReuseHx.o : $(START_DIR)/computeFval_ReuseHx.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xrotg.o : $(START_DIR)/xrotg.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


deleteColMoveEnd.o : $(START_DIR)/deleteColMoveEnd.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


compute_deltax.o : $(START_DIR)/compute_deltax.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


partialColLDL3_.o : $(START_DIR)/partialColLDL3_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


fullColLDL2_.o : $(START_DIR)/fullColLDL2_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


feasibleratiotest.o : $(START_DIR)/feasibleratiotest.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


ratiotest.o : $(START_DIR)/ratiotest.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


addBoundToActiveSetMatrix_.o : $(START_DIR)/addBoundToActiveSetMatrix_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeFirstOrderOpt.o : $(START_DIR)/computeFirstOrderOpt.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(ECHO) "### Deleted all derived files."


