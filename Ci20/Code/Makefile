TOOLS_APPS_PREFIX := mips-mti-linux-gnu
CC			:= g++
CFLAGS		:= -EL -fpermissive -std=c++11
OBJECTS		:= SDOArray.o SDOQueue.o Support.o VirtualData.o WinSockDebug.o cvImageProc.o
LIBRARIES	:= 
DIRECTIVES	:= -DLINUX_SAFE -DWIRINGX_SUPPORT
FILES		:= main.o

.PHONY: tvs clean

#opencv: OPENCV_INSTALL_PREFIX := /home/ci20/opencv-3.1.0/
tvs: OPENCV_INSTALL_PREFIX := /usr/local/include/
tvs: CFLAGS     += -I${OPENCV_INSTALL_PREFIX}opencv -I${OPENCV_INSTALL_PREFIX}opencv2/core/ -I${OPENCV_INSTALL_PREFIX}opencv2/imgproc/ -I${OPENCV_INSTALL_PREFIX}opencv2/photo/ -I${OPENCV_INSTALL_PREFIX}opencv2/video/ -I${OPENCV_INSTALL_PREFIX}opencv2/objdetect/ -I${OPENCV_INSTALL_PREFIX}opencv2/highgui/ -I${OPENCV_INSTALL_PREFIX}opencv2/imgcodecs/ -I${OPENCV_INSTALL_PREFIX}opencv2/videoio/ -I${OPENCV_INSTALL_PREFIX}opencv2/ml/ -I${OPENCV_INSTALL_PREFIX}opencv2/features2d/ -I${OPENCV_INSTALL_PREFIX}opencv2/flann/ -I${OPENCV_INSTALL_PREFIX}opencv2/calib3d/ -I${OPENCV_INSTALL_PREFIX}opencv2/face/ -Lusr/local/lib/
tvs: DIRECTIVES += -DDISABLE_INPUT -DOPENCV
tvs: LIBRARIES  += -lopencv_objdetect -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_imgcodecs -lopencv_flann -lopencv_photo  -lopencv_shape -lopencv_stitching -lopencv_superres  -lopencv_videoio -lopencv_video -lopencv_xfeatures2d -lopencv_face -lwiringX
tvs: base

base: main.o  
	mkdir -p apps
	${CC} ${CFLAGS} ${DIRECTIVES} main.o ${OBJECTS} -Ofast  ${LIBRARIES} -o example
main.o: main.c
	${CC} ${CFLAGS} -c main.c ${DIRECTIVES} ${LIBRARIES}

clean:
	rm -rf main.o example
