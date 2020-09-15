#---RepRapFirmware---
RRF_SRC_BASE  = $(REPRAPFIRMWARE_DIR)/src


RRF_SRC_DIRS = FilamentMonitors GCodes GCodes/GCodeBuffer Heating 
RRF_SRC_DIRS += Movement Movement/BedProbing Movement/Kinematics 
RRF_SRC_DIRS += Storage Libraries/sha1 Comms
RRF_SRC_DIRS += Heating/Sensors Fans ObjectModel Endstops Hardware Hardware/SharedSpi Tools
RRF_SRC_DIRS += Display Display/ST7920 GPIO

#LPC RRF Addons
RRF_SRC_DIRS += STM32 STM32/FatFS

#networking support?
ifeq ($(NETWORK), ETHERNET)
	RRF_SRC_DIRS += Networking STM32/LPCNetworking STM32/Networking/RTOSPlusTCPEthernet
else ifeq ($(NETWORK), ESP8266WIFI) 
	RRF_SRC_DIRS += Networking Networking/ESP8266WiFi STM32/Networking/ESP8266WiFi
else ifeq ($(NETWORK), SBC)
	RRF_SRC_DIRS += Linux
	RRF_SRC_DIRS += STM32/NoNetwork
else
	RRF_SRC_DIRS += STM32/NoNetwork
endif

#TMC22XX support?
ifeq ($(TMC22XX), true)
	RRF_SRC_DIRS += STM32/Movement/StepperDrivers Movement/StepperDrivers
endif

#Find the c and cpp source files
RRF_SRC = $(RRF_SRC_BASE) $(addprefix $(RRF_SRC_BASE)/, $(RRF_SRC_DIRS))
RRF_OBJ_SRC_C	   += $(foreach src, $(RRF_SRC), $(wildcard $(src)/*.c) ) 
RRF_OBJ_SRC_CXX   += $(foreach src, $(RRF_SRC), $(wildcard $(src)/*.cpp) )

RRF_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(RRF_OBJ_SRC_C)) $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(RRF_OBJ_SRC_CXX))


RRF_INCLUDES = $(addprefix -I, $(RRF_SRC))
RRF_INCLUDES += -I$(RRF_SRC_BASE)/Libraries/

#If building ESP8266 WIFI we only need to add the include from DuetWifiSocketServer as it has a file needed to compile RRF 
ifeq ($(NETWORK), ESP8266WIFI)
	RRF_INCLUDES += -IDuetWiFiSocketServer/src/include
endif

#end RRF

#Libc and libcpp in RRF
RRFLIBC_SRC_DIRS = libcpp libc
RRFLIBC_SRC = $(addprefix $(RRF_SRC_BASE)/, $(RRFLIBC_SRC_DIRS))
RRFLIBC_OBJ_SRC_C	  += $(foreach src, $(RRFLIBC_SRC), $(wildcard $(src)/*.c) ) 
RRFLIBC_OBJ_SRC_CXX   += $(foreach src, $(RRFLIBC_SRC), $(wildcard $(src)/*.cpp) )
RRFLIBC_OBJ_SRC_CC    += $(foreach src, $(RRFLIBC_SRC), $(wildcard $(src)/*.cc) )

RRFLIBC_INCLUDES = $(addprefix -I, $(RRF_SRC))

RRFLIBC_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(RRFLIBC_OBJ_SRC_C)) $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(RRFLIBC_OBJ_SRC_CXX))
RRFLIBC_OBJS += $(patsubst %.cc,$(BUILD_DIR)/%.o,$(RRFLIBC_OBJ_SRC_CC))


#


