ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
BUILD := $(ROOT)/build/native
SDK_INCLUDE := $(ROOT)/vendor/maix_sdk
MAIX_RUNTIME ?= /usr/local/lib/python3.13/site-packages/maix
LEGACY_LIB_DIR ?= /root/dist/RMIC26_release/dl_lib
export LD_LIBRARY_PATH := $(LEGACY_LIB_DIR):$(MAIX_RUNTIME)/dl_lib:/opt/lib:/soc/lib:/opt/usr/lib:$(LD_LIBRARY_PATH)
CXX := g++
CC := gcc
CPPFLAGS := -I$(SDK_INCLUDE) -I$(ROOT)/main/include -I/usr/include/opencv4
CXXFLAGS := -std=c++17 -O1 -Wall -Wextra -pthread
SOURCES := $(wildcard $(ROOT)/main/src/*.cpp)
OBJECTS := $(patsubst $(ROOT)/main/src/%.cpp,$(BUILD)/%.o,$(SOURCES))
HEADERS := $(wildcard $(ROOT)/main/include/*.hpp)
LINK := -L$(BUILD) -L$(LEGACY_LIB_DIR) -Wl,-rpath,'$$ORIGIN' -Wl,-rpath,$(MAIX_RUNTIME)/dl_lib -Wl,-rpath-link,/soc/lib -Wl,-rpath-link,/opt/lib -Wl,-rpath-link,/opt/usr/lib -Wl,-rpath-link,$(MAIX_RUNTIME)/dl_lib
CVLIBS := -lopencv_core -lopencv_imgproc -l:libopencv_video.so.411
.PHONY: all test
all: $(BUILD)/VirtualVision $(BUILD)/libfixed_ae.so
$(BUILD):
	mkdir -p $@
$(BUILD)/libmaix.so: | $(BUILD)
	ln -sf $(MAIX_RUNTIME)/_maix.so $@
$(BUILD)/%.o: $(ROOT)/main/src/%.cpp $(HEADERS) | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@
$(BUILD)/ColorContourDetector.o: CXXFLAGS += -O3
$(BUILD)/VirtualVision: $(OBJECTS) $(BUILD)/libmaix.so
	$(CXX) $(OBJECTS) $(LINK) -lmaix -L/usr/local/lib -lpython3.13 $(CVLIBS) -lpthread -ldl -o $@
$(BUILD)/libfixed_ae.so: $(ROOT)/native/fixed_ae.c $(wildcard $(ROOT)/vendor/ax/*.h) | $(BUILD)
	$(CC) -shared -fPIC -O2 -I$(ROOT)/vendor/ax $< -L/opt/lib -lax_ae -Wl,-rpath,/opt/lib -o $@
TESTS := pixel_motion_test color_contour_review_test pixel_pipeline_test fixed_camera_settings_test latest_preview_test
$(BUILD)/pixel_motion_test: $(ROOT)/tests/pixel_motion_test.cpp $(HEADERS) $(ROOT)/tests/LegacyPixelTracker.hpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -UNDEBUG $< $(LINK) $(CVLIBS) -o $@
$(BUILD)/color_contour_review_test: $(ROOT)/tests/color_contour_review_test.cpp $(BUILD)/ColorContourDetector.o $(HEADERS)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -UNDEBUG $< $(BUILD)/ColorContourDetector.o $(LINK) $(CVLIBS) -o $@
$(BUILD)/pixel_pipeline_test: $(ROOT)/tests/pixel_pipeline_test.cpp $(BUILD)/ColorContourDetector.o $(HEADERS)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -UNDEBUG $< $(BUILD)/ColorContourDetector.o $(LINK) $(CVLIBS) -o $@
$(BUILD)/fixed_camera_settings_test: $(ROOT)/tests/fixed_camera_settings_test.cpp $(ROOT)/main/include/FixedCameraSettings.hpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -UNDEBUG $< -o $@
$(BUILD)/latest_preview_test: $(ROOT)/tests/latest_preview_test.cpp $(ROOT)/main/include/LatestPreview.hpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -UNDEBUG $< -o $@
test: $(addprefix $(BUILD)/,$(TESTS))
	$(BUILD)/pixel_motion_test
	$(BUILD)/color_contour_review_test
	$(BUILD)/pixel_pipeline_test
	$(BUILD)/fixed_camera_settings_test
	$(BUILD)/latest_preview_test
