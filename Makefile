BIN := build-conan/build/RelWithDebInfo/orbbec-module

.PHONY: orbbec lint setup appimage
orbbec: $(BIN)

$(BIN): conanfile.py src/* bin/*
	bin/build.sh

clean:
	rm -rf build build-conan

setup:
	bin/setup.sh

module.tar.gz: $(BIN) meta.json
	bin/package.sh $^

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
appimage: orbbec
	$(call BUILD_APPIMAGE,$(OUTPUT_NAME),$(ARCH))
	cp ./packaging/appimages/$(OUTPUT_NAME)-*-$(ARCH).AppImage ./packaging/appimages/deploy/
