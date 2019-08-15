BUILD_DIR = ./build
BIN_DIR = ./bin

.PHONY:build

build:
	mkdir $(BUILD_DIR); cd $(BUILD_DIR); cmake ..; make;

clean:
	rm -rf build/*;

clean_result:
	rm -rf result/result*

run:
	cd $(BIN_DIR); ./pc_pipeline