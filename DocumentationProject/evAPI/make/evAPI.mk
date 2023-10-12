# SRC made data for compiling the evAPI

SRC_C += $(wildcard evAPI/Common/src/*.cpp)
SRC_C += $(wildcard evAPI/Common/src/*.c)
SRC_C += $(wildcard evAPI/Common/src/*/*.cpp)
SRC_C += $(wildcard evAPI/Common/src/*/*.c)

SRC_C += $(wildcard evAPI/VisionTracker/src/*.cpp)
SRC_C += $(wildcard evAPI/VisionTracker/src/*.c)
SRC_C += $(wildcard evAPI/VisionTracker/src/*/*.cpp)
SRC_C += $(wildcard evAPI/VisionTracker/src/*/*.c)

# Include made data for compiling the evAPI

SRC_H += $(wildcard evAPI/*.h)
SRC_H += $(wildcard evAPI/Common/include/*.h)
SRC_H += $(wildcard evAPI/VisionTracker/include/*.h)