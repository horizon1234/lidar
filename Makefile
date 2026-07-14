# YLJ5 Linux CMake 便捷入口。

BUILD_DIR ?= build
BIN_DIR := bin
JOBS ?= $(shell nproc 2>/dev/null || echo 4)
CMAKE_BUILD := cmake --build $(BUILD_DIR) -j$(JOBS)

.PHONY: config all server client protocol core examples tests test run-server run-client clean pristine list

config:
	cmake -S . -B $(BUILD_DIR)

all: config
	$(CMAKE_BUILD)

server: config
	$(CMAKE_BUILD) --target lidar_sim_server
	@echo "服务端已编译：$(BIN_DIR)/lidar_sim_server"

client: config
	$(CMAKE_BUILD) --target lidar_gui
	@echo "客户端已编译：$(BIN_DIR)/lidar_gui"

protocol: config
	$(CMAKE_BUILD) --target lidar_protocol

core: config
	$(CMAKE_BUILD) --target lidar_core

examples: config
	$(CMAKE_BUILD) --target lidar_example_1d_ray lidar_example_2d_ppi lidar_example_3d_rhi

tests test: all
	ctest --test-dir $(BUILD_DIR) --output-on-failure

run-server: server
	./$(BIN_DIR)/lidar_sim_server

run-client: client
	./$(BIN_DIR)/lidar_gui

clean:
	cmake -E remove_directory $(BUILD_DIR)

pristine: clean
	cmake -E remove_directory $(BIN_DIR)

list:
	@echo "all         编译 YLJ5 运行时、测试和教学算例"
	@echo "server      编译 YLJ5 仿真服务端"
	@echo "client      编译 Linux Qt 客户端"
	@echo "test        编译并运行四项设备测试"
	@echo "examples    编译三个知识学习算例"
	@echo "run-server  启动服务端"
	@echo "run-client  启动客户端"
