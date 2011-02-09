
INSTALL_DIR = octomap

all: installed

SVN_DIR = build/octomap-svn
SVN_URL = https://octomap.svn.sourceforge.net/svnroot/octomap/trunk
# latest release (testing 0.9)
SVN_REVISION = -r176
SVN_PATCH = 
include $(shell rospack find mk)/svn_checkout.mk

CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=`rospack find octomap`/$(INSTALL_DIR) 



installed: wiped $(SVN_DIR)
	mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	cp $(SVN_DIR)/doxygen.h $(INSTALL_DIR)/include/doxygen.h
	touch installed

clean:
	-cd $(SVN_DIR)/build && make clean
	rm -rf $(INSTALL_DIR) installed

wiped: Makefile
	rm -rf build patched
	touch wiped

wipe: clean
	rm -rf build patched

.PHONY : clean download
