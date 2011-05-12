#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/FigureFrame.o \
	${OBJECTDIR}/_ext/2139650202/PhidgetHelper.o \
	${OBJECTDIR}/_ext/2139650202/MotorHelper.o \
	${OBJECTDIR}/cvtest.o \
	${OBJECTDIR}/_ext/2139650202/KinectHelper.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=-std=c++0x
CXXFLAGS=-std=c++0x

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-L/usr/local/lib `pkg-config --libs opencv` -lfreenect /usr/local/lib/libcvblob.so -lboost_regex -lphidget21 -lusb  

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cvtest

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cvtest: /usr/local/lib/libcvblob.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cvtest: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cvtest ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/FigureFrame.o: FigureFrame.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DDEBUG -I/usr/local/include/opencv -I/usr/include/libusb-1.0 -I/usr/local/include/libfreenect -I/usr/local/include/eigen3 -I/usr/local/include/pcl-1.0 `pkg-config --cflags opencv`    -MMD -MP -MF $@.d -o ${OBJECTDIR}/FigureFrame.o FigureFrame.cpp

${OBJECTDIR}/_ext/2139650202/PhidgetHelper.o: /home/ros/NetBeansProjects/CvTest/PhidgetHelper.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2139650202
	${RM} $@.d
	$(COMPILE.cc) -g -DDEBUG -I/usr/local/include/opencv -I/usr/include/libusb-1.0 -I/usr/local/include/libfreenect -I/usr/local/include/eigen3 -I/usr/local/include/pcl-1.0 `pkg-config --cflags opencv`    -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2139650202/PhidgetHelper.o /home/ros/NetBeansProjects/CvTest/PhidgetHelper.cpp

${OBJECTDIR}/_ext/2139650202/MotorHelper.o: /home/ros/NetBeansProjects/CvTest/MotorHelper.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2139650202
	${RM} $@.d
	$(COMPILE.cc) -g -DDEBUG -I/usr/local/include/opencv -I/usr/include/libusb-1.0 -I/usr/local/include/libfreenect -I/usr/local/include/eigen3 -I/usr/local/include/pcl-1.0 `pkg-config --cflags opencv`    -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2139650202/MotorHelper.o /home/ros/NetBeansProjects/CvTest/MotorHelper.cpp

${OBJECTDIR}/cvtest.o: cvtest.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -DDEBUG -I/usr/local/include/opencv -I/usr/include/libusb-1.0 -I/usr/local/include/libfreenect -I/usr/local/include/eigen3 -I/usr/local/include/pcl-1.0 `pkg-config --cflags opencv`    -MMD -MP -MF $@.d -o ${OBJECTDIR}/cvtest.o cvtest.cpp

${OBJECTDIR}/_ext/2139650202/KinectHelper.o: /home/ros/NetBeansProjects/CvTest/KinectHelper.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/2139650202
	${RM} $@.d
	$(COMPILE.cc) -g -DDEBUG -I/usr/local/include/opencv -I/usr/include/libusb-1.0 -I/usr/local/include/libfreenect -I/usr/local/include/eigen3 -I/usr/local/include/pcl-1.0 `pkg-config --cflags opencv`    -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/2139650202/KinectHelper.o /home/ros/NetBeansProjects/CvTest/KinectHelper.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/cvtest

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
