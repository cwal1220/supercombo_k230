CXX ?= g++
CC ?= gcc
AR ?= ar

CXXFLAGS ?= -std=c++17 -O3 -DNDEBUG -Wall -Wextra
CFLAGS ?= -O2 -DNDEBUG -Wall -Wextra

OPENCV_CFLAGS ?= $(shell pkg-config --cflags opencv4 2>/dev/null || echo -I/usr/include/opencv4)
OPENCV_LIBS ?= $(shell pkg-config --libs-only-L --libs-only-other opencv4 2>/dev/null) -lopencv_imgproc -lopencv_core
LIBUSB_CFLAGS ?= $(shell pkg-config --cflags libusb-1.0 2>/dev/null || echo -I/usr/include/libusb-1.0)
LIBUSB_LIBS ?= $(shell pkg-config --libs libusb-1.0 2>/dev/null || echo -lusb-1.0)

INCLUDES := -Isrc -Iinclude -Ideps/include -I/usr/include/libdrm $(OPENCV_CFLAGS)
PANDA_INCLUDES := $(INCLUDES) $(LIBUSB_CFLAGS)
LIBDIRS := -L/usr/lib/riscv64-linux-gnu
STATIC_LIBS := deps/lib/libNncase.Runtime.Native.a deps/lib/libnncase.rt_modules.k230.a deps/lib/libfunctional_k230.a build/libmmz.a
SHARED_LIBS := /usr/lib/riscv64-linux-gnu/libv4l2-drm.so /usr/lib/riscv64-linux-gnu/libdisplay.so /usr/lib/riscv64-linux-gnu/libdrm.so.2
LDLIBS := -pthread -latomic -ldl
BINDIR := build/bin

CORE_OBJS := build/app_config.o \
	build/common_utils.o \
	build/model_output.o \
	build/projection.o \
	build/calibration_service.o \
	build/input_source.o \
	build/json_utils.o \
	build/lateral_control.o \
	build/k230_ipc.o \
	build/model_input_transform.o \
	build/supercombo_model.o \
	build/online_calibrator.o \
	build/ai_base.o

MONOLITH_OBJS := build/main.o \
	build/app_config.o \
	build/common_utils.o \
	build/model_output.o \
	build/projection.o \
	build/overlay_renderer.o \
	build/calibration_service.o \
	build/input_source.o \
	build/json_utils.o \
	build/lateral_control.o \
	build/supercombo_runtime.o \
	build/model_input_transform.o \
	build/supercombo_model.o \
	build/online_calibrator.o \
	build/ai_base.o

CAMERAD_OBJS := build/k230_camerad.o \
	build/app_config.o \
	build/common_utils.o \
	build/input_source.o \
	build/model_output.o \
	build/projection.o \
	build/k230_ipc.o

MODELD_OBJS := build/k230_modeld.o \
	build/app_config.o \
	build/common_utils.o \
	build/model_output.o \
	build/projection.o \
	build/calibration_service.o \
	build/input_source.o \
	build/json_utils.o \
	build/lateral_control.o \
	build/k230_ipc.o \
	build/model_input_transform.o \
	build/supercombo_model.o \
	build/online_calibrator.o \
	build/ai_base.o

OVERLAY_OBJS := build/k230_overlay.o \
	build/app_config.o \
	build/common_utils.o \
	build/model_output.o \
	build/projection.o \
	build/overlay_renderer.o \
	build/k230_ipc.o

PANDAD_OBJS := build/k230_pandad.o \
	build/panda_client.o \
	build/common_utils.o \
	build/k230_ipc.o \
	build/model_output.o \
	build/projection.o \
	build/lateral_control.o

.PHONY: all clean supercombo.elf k230_camerad k230_modeld k230_overlay k230_pandad

all: $(BINDIR)/supercombo.elf $(BINDIR)/k230_camerad $(BINDIR)/k230_modeld $(BINDIR)/k230_overlay

supercombo.elf: $(BINDIR)/supercombo.elf

k230_camerad: $(BINDIR)/k230_camerad

k230_modeld: $(BINDIR)/k230_modeld

k230_overlay: $(BINDIR)/k230_overlay

k230_pandad: $(BINDIR)/k230_pandad

check-parser: build/check_model_output_parser
	./build/check_model_output_parser $(RAW_DUMP)

build:
	mkdir -p build

$(BINDIR): | build
	mkdir -p $(BINDIR)

build/mmz.o: src/mmz.c include/mmz.h | build
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/%.o: src/%.cc | build
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

build/panda_client.o: src/panda_client.cc src/panda_client.h | build
	$(CXX) $(CXXFLAGS) $(PANDA_INCLUDES) -c $< -o $@

build/k230_pandad.o: src/k230_pandad.cc src/panda_client.h | build
	$(CXX) $(CXXFLAGS) $(PANDA_INCLUDES) -c $< -o $@

build/check_model_output_parser.o: benchmarks/check_model_output_parser.cc | build
	$(CXX) $(CXXFLAGS) -Isrc -c $< -o $@

build/check_model_output_parser: build/check_model_output_parser.o build/model_output.o
	$(CXX) $(CXXFLAGS) $^ -o $@

build/libmmz.a: build/mmz.o
	$(AR) rcs $@ $<

$(BINDIR)/supercombo.elf: $(MONOLITH_OBJS) build/libmmz.a | $(BINDIR)
	$(CXX) $(CXXFLAGS) $(MONOLITH_OBJS) -o $@ $(LIBDIRS) -Wl,--start-group $(STATIC_LIBS) $(SHARED_LIBS) $(OPENCV_LIBS) $(LDLIBS) -Wl,--end-group

$(BINDIR)/k230_camerad: $(CAMERAD_OBJS) build/libmmz.a | $(BINDIR)
	$(CXX) $(CXXFLAGS) $(CAMERAD_OBJS) -o $@ $(LIBDIRS) -Wl,--start-group build/libmmz.a $(SHARED_LIBS) $(LDLIBS) -Wl,--end-group

$(BINDIR)/k230_modeld: $(MODELD_OBJS) build/libmmz.a | $(BINDIR)
	$(CXX) $(CXXFLAGS) $(MODELD_OBJS) -o $@ $(LIBDIRS) -Wl,--start-group $(STATIC_LIBS) $(SHARED_LIBS) $(LDLIBS) -Wl,--end-group

$(BINDIR)/k230_overlay: $(OVERLAY_OBJS) build/libmmz.a | $(BINDIR)
	$(CXX) $(CXXFLAGS) $(OVERLAY_OBJS) -o $@ $(LIBDIRS) -Wl,--start-group build/libmmz.a $(SHARED_LIBS) $(OPENCV_LIBS) $(LDLIBS) -Wl,--end-group

$(BINDIR)/k230_pandad: $(PANDAD_OBJS) | $(BINDIR)
	$(CXX) $(CXXFLAGS) $(PANDAD_OBJS) -o $@ $(LIBDIRS) -Wl,--start-group $(LIBUSB_LIBS) $(LDLIBS) -Wl,--end-group

clean:
	rm -rf build
