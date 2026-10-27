# Native fallback for the installed MaixCAM2 runtime. Use -j1 on 512MiB boards.
ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
SDK_INCLUDE ?= /root/rmic26_mount180/sdk_include
MAIX_RUNTIME ?= /usr/local/lib/python3.13/site-packages/maix
LEGACY_LIB_DIR ?= /root/dist/RMIC26_release/dl_lib
BUILD := $(ROOT)/build/native
CXX := g++
CXXFLAGS := -std=c++17 -O1 -Wall
CPPFLAGS := -I$(SDK_INCLUDE) -I$(ROOT)/main/include -I/usr/include/opencv4
SOURCES := $(wildcard $(ROOT)/main/src/*.cpp)
OBJECTS := $(patsubst $(ROOT)/main/src/%.cpp,$(BUILD)/%.o,$(SOURCES))
HEADERS := $(wildcard $(ROOT)/main/include/*.hpp $(SDK_INCLUDE)/*.hpp $(SDK_INCLUDE)/*.h)
LDLIBS := -L$(BUILD) -L$(LEGACY_LIB_DIR) -Wl,-rpath,'$$ORIGIN' -Wl,-rpath,$(MAIX_RUNTIME)/dl_lib -Wl,-rpath-link,/soc/lib -Wl,-rpath-link,$(MAIX_RUNTIME)/dl_lib -lmaix -L/usr/local/lib -lpython3.13 -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -l:libopencv_calib3d.so.411 -l:libopencv_video.so.411 -lpthread -ldl
TEST_LIBS := -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_video
CALIBRATION ?= /root/camera_intrinsics.yml
.PHONY: all test
all: $(BUILD)/RMIC26
$(BUILD):
	mkdir -p $@
$(BUILD)/libmaix.so: | $(BUILD)
	ln -sf $(MAIX_RUNTIME)/_maix.so $@
$(BUILD)/%.o: $(ROOT)/main/src/%.cpp $(HEADERS) | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@
$(BUILD)/RMIC26: $(OBJECTS) $(BUILD)/libmaix.so
	$(CXX) $(OBJECTS) $(LDLIBS) -o $@
$(BUILD)/mount180_test: $(ROOT)/tests/mount180_test.cpp $(BUILD)/CameraModel.o $(BUILD)/AimUart.o $(HEADERS)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< $(BUILD)/CameraModel.o $(BUILD)/AimUart.o $(TEST_LIBS) -o $@
$(BUILD)/tracking_test: $(ROOT)/tests/tracking_test.cpp $(BUILD)/Armorboard_Detector.o $(HEADERS)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< $(BUILD)/Armorboard_Detector.o $(TEST_LIBS) -o $@
test: $(BUILD)/mount180_test $(BUILD)/tracking_test
	$(BUILD)/mount180_test $(CALIBRATION)
	$(BUILD)/tracking_test
