#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-Simulator.mk)" "nbproject/Makefile-local-Simulator.mk"
include nbproject/Makefile-local-Simulator.mk
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=Simulator
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/dsPID4W.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/dsPID4W.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/src/com.o ${OBJECTDIR}/src/traps.o ${OBJECTDIR}/src/dsPID4W.o ${OBJECTDIR}/src/dsPid4W_settings.o
POSSIBLE_DEPFILES=${OBJECTDIR}/src/com.o.d ${OBJECTDIR}/src/traps.o.d ${OBJECTDIR}/src/dsPID4W.o.d ${OBJECTDIR}/src/dsPid4W_settings.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/src/com.o ${OBJECTDIR}/src/traps.o ${OBJECTDIR}/src/dsPID4W.o ${OBJECTDIR}/src/dsPid4W_settings.o


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-Simulator.mk dist/${CND_CONF}/${IMAGE_TYPE}/dsPID4W.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33FJ128MC802
MP_LINKER_FILE_OPTION=,--script=p33FJ128MC802.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/src/com.o: src/com.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/com.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/com.c  -o ${OBJECTDIR}/src/com.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/com.o.d"        -g -D__DEBUG   -omf=elf -legacy-libc -fast-math -fno-short-double -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/com.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/traps.o: src/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/traps.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/traps.c  -o ${OBJECTDIR}/src/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/traps.o.d"        -g -D__DEBUG   -omf=elf -legacy-libc -fast-math -fno-short-double -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/dsPID4W.o: src/dsPID4W.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/dsPID4W.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/dsPID4W.c  -o ${OBJECTDIR}/src/dsPID4W.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/dsPID4W.o.d"        -g -D__DEBUG   -omf=elf -legacy-libc -fast-math -fno-short-double -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/dsPID4W.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/dsPid4W_settings.o: src/dsPid4W_settings.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/dsPid4W_settings.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/dsPid4W_settings.c  -o ${OBJECTDIR}/src/dsPid4W_settings.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/dsPid4W_settings.o.d"        -g -D__DEBUG   -omf=elf -legacy-libc -fast-math -fno-short-double -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/dsPid4W_settings.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/src/com.o: src/com.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/com.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/com.c  -o ${OBJECTDIR}/src/com.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/com.o.d"        -g -omf=elf -legacy-libc -fast-math -fno-short-double -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/com.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/traps.o: src/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/traps.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/traps.c  -o ${OBJECTDIR}/src/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/traps.o.d"        -g -omf=elf -legacy-libc -fast-math -fno-short-double -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/dsPID4W.o: src/dsPID4W.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/dsPID4W.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/dsPID4W.c  -o ${OBJECTDIR}/src/dsPID4W.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/dsPID4W.o.d"        -g -omf=elf -legacy-libc -fast-math -fno-short-double -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/dsPID4W.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/dsPid4W_settings.o: src/dsPid4W_settings.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/dsPid4W_settings.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/dsPid4W_settings.c  -o ${OBJECTDIR}/src/dsPid4W_settings.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/dsPid4W_settings.o.d"        -g -omf=elf -legacy-libc -fast-math -fno-short-double -mlarge-data -mlarge-scalar -O0 -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/dsPid4W_settings.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/dsPID4W.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  /Applications/microchip/xc16/v1.10/lib/libdsp-elf.a /Applications/microchip/xc16/v1.10/lib/libfastm-elf.a /Applications/microchip/xc16/v1.10/lib/libq-elf.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/dsPID4W.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    /Applications/microchip/xc16/v1.10/lib/libdsp-elf.a /Applications/microchip/xc16/v1.10/lib/libfastm-elf.a /Applications/microchip/xc16/v1.10/lib/libq-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG   -omf=elf -legacy-libc -fast-math -Wl,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__ICD2RAM=1,--defsym=__DEBUG=1,,$(MP_LINKER_FILE_OPTION),--heap=512,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/dsPID4W.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  /Applications/microchip/xc16/v1.10/lib/libdsp-elf.a /Applications/microchip/xc16/v1.10/lib/libfastm-elf.a /Applications/microchip/xc16/v1.10/lib/libq-elf.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/dsPID4W.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    /Applications/microchip/xc16/v1.10/lib/libdsp-elf.a /Applications/microchip/xc16/v1.10/lib/libfastm-elf.a /Applications/microchip/xc16/v1.10/lib/libq-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -legacy-libc -fast-math -Wl,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--heap=512,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}/xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/dsPID4W.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/Simulator
	${RM} -r dist/Simulator

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
