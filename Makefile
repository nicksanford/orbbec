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

TAG_VERSION?=latest
# Define a function for building AppImages
define BUILD_APPIMAGE
    export TAG_NAME=$(TAG_VERSION); \
    cd packaging/appimages && \
    mkdir -p deploy && \
    rm -f deploy/$(1)* && \
    appimage-builder --recipe $(1)-$(2).yml
endef

appimage: export OUTPUT_NAME = viam-camera-orbbec
appimage: export ARCH = x86_64
# appimage: orbbec
 appimage: 
	$(call BUILD_APPIMAGE,$(OUTPUT_NAME),$(ARCH))
	cp ./packaging/appimages/$(OUTPUT_NAME)-*-$(ARCH).AppImage ./packaging/appimages/deploy/
