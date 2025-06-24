SCRIPT_EXT := .sh
PATHSEP := /

BIN := build-conan/build/RelWithDebInfo/tflite_cpu$(BIN_EXT)

.PHONY: orbbec 
orbbec: $(BIN)

$(BIN): conanfile.py src/*
	bin$(PATHSEP)build$(SCRIPT_EXT)

.PHONY: setup
setup:
	bin$(PATHSEP)setup$(SCRIPT_EXT)

module.tar.gz: $(BIN) meta.json
	bin$(PATHSEP)package$(SCRIPT_EXT) $^

.PHONY: lint
lint:
	./bin/run-clang-format.sh
