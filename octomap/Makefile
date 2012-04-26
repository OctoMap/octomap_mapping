all: installed

#
# Download, extract and compile from a released tarball:
#
TARBALL = build/octomap.tar.gz
TARBALL_URL = https://code.ros.org/svn/release/download/thirdparty/octomap/octomap-1.4.2.tar.gz
TARBALL_PATCH = 
INITIAL_DIR = build/octomap
SOURCE_DIR = build/octomap-tar
include $(shell rospack find mk)/download_unpack_build.mk


#
# Use the following commented block (instead of the one above)
# to work on a SVN checkout of OctoMap. This will not work with
# ROS stack releases.
#
#SVN_DIR = build/octomap-svn
#SVN_URL = https://octomap.svn.sourceforge.net/svnroot/octomap/trunk/octomap
# latest code from trunk: empty revision
#SVN_REVISION = 
#SVN_PATCH = 
#include $(shell rospack find mk)/svn_checkout.mk
#SOURCE_DIR=$(SVN_DIR)
#

INSTALL_DIR = octomap
CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE="Release" \
	     -D CMAKE_INSTALL_PREFIX=`rospack find octomap`/$(INSTALL_DIR) \
	     -D WITH_ROS=ON



installed: wiped $(SOURCE_DIR)/unpacked
	mkdir -p $(SOURCE_DIR)/build
	cd $(SOURCE_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SOURCE_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	cp $(SOURCE_DIR)/doxygen.h $(INSTALL_DIR)/include/doxygen.h
	touch installed

clean:
	rm -rf build
	rm -rf $(INSTALL_DIR) installed

wiped: Makefile
	make wipe
	touch wiped

wipe: clean
	rm -rf build patched
