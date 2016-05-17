#ifndef ARDUINO
#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP

#ifndef LINUX_SAFE
#include <conio.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#endif // LINUX_SAFE
#ifdef FILESTREAM_SUPPORT
#include <fstream>
#include <sstream>
#include <iostream>
#endif
#ifdef WIRINGX_SUPPORT
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include "wiringX.h"
#include <fstream>
#include <sstream>
#include <iostream>
#endif
#ifndef EIDOS
#include "Support.h" 
#include <stdlib.h>
#include <string.h>
#else  // EIDOS
#include "../../VIPE/eidos_heap.h"
#include "../../VIPE/Support.h" 
#endif // EIDOS
#ifndef EIDOS
#include "GlobalVars.h"
#include "sdo.h"
#include "VirtualData.h"
#include "WinSockDebug.h"
#else  // EIDOS
#include "../../VIPE/GlobalVars.h"
#include "../../VIPE/sdo.h"
#include "../../VIPE/VirtualData.h"
#include "../../VIPE/WinSockDebug.h"
#endif // EIDOS
#ifdef CIMG
#include "CImg.h" 
using namespace cimg_library;
#endif // CIMG
#ifdef RADARDETECT
#include "PaPPRadarDetection.h"
#endif // RADARDETECT
#ifdef BPE
#include "getopt.h"
#include "global.h"
#include "tailor.h"
#include "lifting_97f.h"
#include "lifting_97M.h"
#include "AC_BitPlaneCoding.h"
#endif // BPE
#ifdef OPENCV
#include <cv.h>
#include <highgui.h>
#include "ml.hpp"
#ifndef VELCORE
#include "cvImageProc.h"
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/face.hpp>
#endif
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>	
using namespace cv;
using namespace ml;
#endif // OPENCV
#endif //ARDUINO
#ifdef ARDUINO
#include <Arduino.h>
#include "sdo.h"
#include "SDOArray.h"
#include "SDOQueue.h"
#include "Support.h"
#include "VirtualData.h"
#include "WinSockDebug.h"
#include "GlobalVars.h"
#include "arduino_additional_libs/Servo.h"
#include "arduino_additional_libs/Ultrasonic.h"
#include "arduino_additional_libs/SPI.h"
#include "arduino_additional_libs/Wire.h"
#include "arduino_additional_libs/Adafruit_GFX.h"
#include "arduino_additional_libs/Adafruit_SSD1306.h"
#include "arduino_additional_libs/I2Cdev.h"
#include "arduino_additional_libs/KalmanFilter.h"
#include "arduino_additional_libs/ADXL345.h"
#include "arduino_additional_libs/L3G4200D.h"
#define REMOTEXY_MODE__SERIAL
#include <RemoteXY.h> 
#include "RemoteXY_rc.h"
#endif //ARDUINO
#ifdef NUMPLATES
#include "send_recv.h"
#include "fname_wo_ext.h"
#include "string_manip.h"
#include "read_params.h"
#include "print_params.h"
#include "CLParams.h"
#include "DetectRegions.h"
#include "npTypes.h"
#include "OCR.h"
#endif
#ifdef OPENGL_SUPPORT
#include "OpenGLSupport.h"
#include <unistd.h>
#endif
#ifdef OPENVX_SUPPORT
#include <VX/vx.h>
#include <VX/vx_lib_debug.h>
#include <VX/vx_helper.h>
#endif
int GetTurnAngle(DataLink* in11, DataLink* in21, DataLink* in31, DataLink* out41)
{
	double width, height;
	int angle, center;
	cv::Rect r;
	memcpy(&r, in11->Data, sizeof(r));
	width = ReadDouble(in31);
	height = ReadDouble(in21);
	center = r.x + r.width / 2;
	if (center < width / 2 - 100)
		angle = 0;
	else if (center > width / 2 + 100)
		angle = 1;
	else angle = 2;
	out41->Size = sizeof(int);
	out41->Data = (char*)malloc(out41->Size);
	memcpy(out41->Data, &angle, out41->Size);
	return 0;
}
int intToChar(DataLink* in11, DataLink* out21)
{
	int inVal = ReadInteger(in11);
	std::string tmpStr = std::string("frame: ") + std::to_string(inVal) + std::string("\0");
	out21->Size = tmpStr.size();
	out21->Data = (char*)malloc(out21->Size);
	memcpy(out21->Data, tmpStr.c_str(), out21->Size);
	return 0;
}
int WiringXPutChar(DataLink* in11, DataLink* in21)
{
	int val;
	int fd = -1;
	val = ReadInteger(in21);
	fd = ReadInteger(in11);
	printf("Turn to: %d %d\n", fd, val);
#ifdef WIRINGX_SUPPORT
	if (val == 0)
	{
		wiringXSerialPutChar(fd, '0');
		wiringXSerialPutChar(fd, 0);
	}
	if (val == 1)
	{
		wiringXSerialPutChar(fd, '1');
		wiringXSerialPutChar(fd, 1);
	}
#endif
	return 0;
}
int WiringXSetup(DataLink* out11)
{
	int fd = -1;
#ifdef WIRINGX_SUPPORT
	struct wiringXSerial_t wiringXSerial = { 19200, 8, 'N', 1, 'x' };
	unsigned char data_send = 0x00;
	int date_receive = 0;
	wiringXSetup();
	if ((fd = wiringXSerialOpen("/dev/ttyS0", wiringXSerial)) < 0) {
		fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		return -1;
	}
#endif
	WriteInteger(out11, fd);
	return 0;
}
//<tdl>
//<module name="standard">
//<params>
//	<param name="inX1" type="int"/>
//	<param name="inY1" type="int"/>
//	<param name="out1" type="int"/>
//</params>
//</module>
//</tdl>
int llMod__i(DataLink *inX1, DataLink *inY1, DataLink *out1)
{
	int x, y;
	out1->Size = sizeof(int);
	out1->Data = (char*)malloc(sizeof(int));
	x = *((int*)(inX1->Data));
	y = *((int*)(inY1->Data));

	((int*)(out1->Data))[0] = x % y;
	return 0;
}
//<tdl>
//<module name="standard">
//<params>
//	<param name="in1" type="double"/>
//	<param name="out1" type="int"/>
//</params>
//</module>
//</tdl>  
int llFloatToInt__double(DataLink *in1, DataLink *out1)
{
	out1->Size = sizeof(int);
	out1->Data = (char*)malloc(out1->Size);
	((int*)(out1->Data))[0] = (int)(((double*)(in1->Data))[0]);
	return 0;
}
//<tdl>
//<module name="standard">
//<params>
//	<param name="inX1" type="double"/>
//	<param name="inY1" type="double"/>
//	<param name="out1" type="double"/>
//</params>
//</module>
//</tdl>  
int llSub__double(DataLink *inX1, DataLink *inY1, DataLink *out1)
{
	out1->Size = sizeof(double);
	out1->Data = (char*)malloc(sizeof(double));
	((double*)(out1->Data))[0] = ((double*)(inX1->Data))[0] - ((double*)(inY1->Data))[0];
	return 0;
}
//<tdl>
//<module name="standard">
//<params>
//	<param name="in1" type="int"/>
//	<param name="in2" type="int"/>
//	<param name="out1" type="int"/>
//</params>
//</module>
//</tdl>
int llAnd__int(DataLink *in1, DataLink *in2, DataLink *out1)
{
	out1->Size = sizeof(int);
	out1->Data = (char*)malloc(sizeof(int));

	if ((((int*)(in1->Data))[0] == 1) &&
		(((int*)(in2->Data))[0] == 1))
	{
		((int*)(out1->Data))[0] = 1;
	}
	else
	{
		((int*)(out1->Data))[0] = 0;
	}
	return 0;
}
//<tdl>
//<module name="standard">
//<params>
//	<param name="in1" type="int"/>
//	<param name="in2" type="int"/>
//	<param name="out1" type="int"/>
//</params>
//</module>
//</tdl>
int llEqual__int(DataLink *in1, DataLink *in2, DataLink *out1)
{
	out1->Size = sizeof(int);
	out1->Data = (char*)malloc(sizeof(int));
	if (((int*)(in1->Data))[0] == ((int*)(in2->Data))[0])
	{
		((int*)(out1->Data))[0] = 1;
	}
	else
	{
		((int*)(out1->Data))[0] = 0;
	}
	return 0;
}
//<tdl>
//<module name="standard">
//<params>
//	<param name="inX1" type="int"/>
//	<param name="inY1" type="int"/>
//	<param name="out1" type="int"/>
//</params>
//</module>
//</tdl>
int llGreaterThen(DataLink *inX1, DataLink *inY1, DataLink *out1)
{
	out1->Size = sizeof(int);
	out1->Data = (char*)malloc(sizeof(int));
	((int*)(out1->Data))[0] = ((int*)(inX1->Data))[0] > ((int*)(inY1->Data))[0] ? 1 : 0;
	return 0;
}
//<tdl>
//<module name="standard">
//<params>
//	<param name="in1" type="int"/>
//	<param name="in2" type="int"/>
//	<param name="out1" type="int"/>
//</params>
//</module>
//</tdl>
int llMul(DataLink *in1, DataLink *in2, DataLink *out1)
{
	out1->Size = sizeof(int);
	out1->Data = (char*)malloc(sizeof(int));
	((int*)(out1->Data))[0] = ((int*)(in1->Data))[0] * ((int*)(in2->Data))[0];
	return 0;
}
//<tdl>
//<module name="standard">
//<params>
//	<param name="in1" type="int"/>
//	<param name="in2" type="int"/>
//	<param name="out1" type="int"/>
//</params>
//</module>
//</tdl>
int llSum(DataLink *in1, DataLink *in2, DataLink *out1)
{
	out1->Size = sizeof(int);
	out1->Data = (char*)malloc(sizeof(int));
	((int*)(out1->Data))[0] = ((int*)(in1->Data))[0] + ((int*)(in2->Data))[0];
	return 0;
}
int GetTerminatorInfo(DataLink* in11, DataLink* out21, DataLink* out31, DataLink* out41, DataLink* out51)
{
	std::string tmpStr;
	int size;
	int maxStringSize = 20;
	char* tmpCharStr;
	memcpy(&size, in11->Data, sizeof(int));

	tmpCharStr = (char*)malloc(size);
	memcpy(tmpCharStr, in11->Data + sizeof(int), size);
	std::string str_name(tmpCharStr, size);
	tmpStr = std::string("VIPE CONSOLE\0");
	out21->Size = tmpStr.size();
	out21->Data = (char*)malloc(tmpStr.size());
	memcpy(out21->Data, tmpStr.c_str(), tmpStr.size());
	if (str_name != "Vera")
		tmpStr = std::string("VISUAL: MALE\0");
	else
		tmpStr = std::string("VISUAL: FEMALE\0");
	out31->Size = tmpStr.size();
	out31->Data = (char*)malloc(tmpStr.size());
	memcpy(out31->Data, tmpStr.c_str(), tmpStr.size());
	tmpStr = std::string("Detected: ") + str_name + std::string("\0");
	out41->Size = tmpStr.size();
	out41->Data = (char*)malloc(tmpStr.size());
	memcpy(out41->Data, tmpStr.c_str(), tmpStr.size());
	printf("%s\n", tmpStr.c_str());
	tmpStr = std::string("OBJ: ");
	if (str_name != "Vera")
		tmpStr = tmpStr + std::string("NEUTRAL\0");
	else
		tmpStr = tmpStr + std::string("TERMINATE\0");
	out51->Size = tmpStr.size();
	out51->Data = (char*)malloc(tmpStr.size());
	memcpy(out51->Data, tmpStr.c_str(), tmpStr.size());
	return 0;
}
int CVCaptureFromCam(DataLink *out1, int CameraID)
{
	VideoCapture* capture = new VideoCapture(0);
	WriteReference(out1, (int *)capture);
	return 0;
}
int CVFaceRecognizerCreateVal(DataLink *outR1, DataLink *outW1, DataLink *outH1, DataLink *outA1, char* file_name)
{
	std::vector<Mat> images;
	std::vector<int> labels;
	std::vector<std::string> names;
	cv::Ptr<cv::face::FaceRecognizer>* model = new cv::Ptr<cv::face::FaceRecognizer>(cv::face::createFisherFaceRecognizer());

	try {
		std::ifstream file(file_name, std::ifstream::in);
		if (!file) {
			std::string error_message = "No valid input file was given, please check the given filename.";
			CV_Error(Error::StsBadArg, error_message);
		}
		char separator = ';';
		std::string line, path, classlabel, name;
		while (getline(file, line)) {
			std::stringstream liness(line);
			getline(liness, path, separator);
			getline(liness, classlabel, separator);
			getline(liness, name);
			if (!path.empty() && !classlabel.empty() && !name.empty())
			{
				images.push_back(imread(path, 0));
				labels.push_back(atoi(classlabel.c_str()));
				name = name + std::string("\0");
				if (!((names.size() > 1) && (names.at(names.size() - 1) == name)))
					names.push_back(name);
			}
		}
	}
	catch (cv::Exception& e) {
		std::cerr << "Error opening file \"" << file_name << "\". Reason: " << e.msg << std::endl;
		// nothing more we can do
		exit(1);
	}
	int im_width = images[0].cols;
	int im_height = images[0].rows;
	((cv::Ptr<cv::face::FaceRecognizer>)(*model))->train(images, labels);

	WriteInteger(outW1, im_width);
	WriteInteger(outH1, im_height);
	outR1->Size = sizeof(model);
	outR1->Data = (char*)malloc(outR1->Size);
	memcpy(outR1->Data, &model, outR1->Size);

	int maxStringSize = 20;
	int customStringSize = maxStringSize + sizeof(int);
	outA1->Size = names.size()*(customStringSize);
	outA1->Data = (char*)malloc(outA1->Size);
	for (int k = 0; k < names.size(); k++)
	{
		int len = names.at(k).size();
		const char* tmpStr = names.at(k).c_str();
		char* strInMem = (char*)malloc(len);
		memcpy(strInMem, tmpStr, len);
		memcpy(outA1->Data + k*customStringSize, &len, sizeof(int));
		memcpy(outA1->Data + k*customStringSize + sizeof(int), strInMem, len);
	}
	printf("Face Recognizer is trained. Data Base contains %d images\n", images.size());
	return 0;
}
int CVFaceRecognizerPredictVal(DataLink *inR1, DataLink *inM1, DataLink *outS1)
{
	std::string box_text;
	cv::Mat face_resized;
	cv::Ptr<cv::face::FaceRecognizer>* model = new cv::Ptr<cv::face::FaceRecognizer>();

	memcpy(&model, inR1->Data, inR1->Size);
	DecodeMat(inM1, &face_resized);

	int prediction = -1;
	double predicted_confidence = 0.0;
	((cv::Ptr<cv::face::FaceRecognizer>)(*model))->predict(face_resized, prediction, predicted_confidence);

	outS1->Size = sizeof(prediction);
	outS1->Data = (char*)malloc(outS1->Size);
	memcpy(outS1->Data, &prediction, outS1->Size);
	return 0;
}
int CVLoadHaaraCascadeVal(DataLink *out11, char* cascade_path)
{
	int* p;
	CascadeClassifier* cascade = new CascadeClassifier();
	if (FILE *file = fopen(cascade_path, "r"))
	{
		fclose(file);
		cascade->load(cascade_path);
		p = (int*)(cascade);
		out11->Size = sizeof(p);
		out11->Data = (char*)malloc(out11->Size);
		memcpy(out11->Data, &p, sizeof(out11->Size));
	}
	else
	{
		printf("No file with name %s (LoadHaaraCascade)\n", cascade_path);
#ifndef DISABLE_INPUT
		getchar();
#endif // DISABLE_INPUT
		exit(1);
	}
	return 0;
}
int CVToColorVal(DataLink *in11, DataLink *out21, int flag)
{
	cv::Mat src, dst;

	DecodeMat(in11, &src);
	cvtColor(src, dst, flag);
	EncodeMat(&dst, out21);

	return 0;
}
int CVDetectFacesMatVal(DataLink *in11, DataLink *in21, DataLink *out31, DataLink *out41)
{
	cv::Mat src;
	int* p;
	std::vector< Rect_<int> > faces;
	CascadeClassifier* haar_cascade = new CascadeClassifier();
	int i, k, faces_count;
	cv::Rect new_rect;

	DecodeMat(in21, &src);
	p = (int*)(&haar_cascade);
	memcpy(&p, in11->Data, sizeof(p));
	haar_cascade = (CascadeClassifier*)p;
	haar_cascade->detectMultiScale(src, faces);

	faces_count = faces.size();	
	out31->Size = faces_count*(sizeof(cv::Rect));
	out31->Data = (char*)malloc(out31->Size);
	for (size_t i = 0; i < faces.size(); i++)
	{
		cv::Rect face_i = faces[i];
		new_rect = cv::Rect(face_i.x, face_i.y, face_i.width, face_i.height);
		memcpy(out31->Data + ((int)i)*sizeof(cv::Rect), &new_rect, sizeof(cv::Rect));
	}
	out41->Size = sizeof(int);
	out41->Data = (char*)malloc(sizeof(int));
	((int*)(out41->Data))[0] = faces_count;

	return 0;
}
int CVDrawRectMatVal(DataLink *in11, DataLink *in21, DataLink *in31, DataLink *in41, DataLink *out51, int thickness, int line_style)
{
	cv::Mat src_image;
	cv::Point ul;
	cv::Point lr;
	cv::Rect r;
	int diffx, diffy;

	DecodeMat(in11, &src_image);
	memcpy(&r, in21->Data, sizeof(r));
	diffx = ReadInteger(in31);
	diffy = ReadInteger(in41);
	ul.x = r.x + diffx;
	ul.y = r.y + diffy;
	lr.x = r.x + r.width + diffx;
	lr.y = r.y + r.height + diffy;
	rectangle(src_image, ul, lr, cv::Scalar(0, 255, 0), thickness, line_style);
	EncodeMat(&src_image, out51);

	return 0;
}
int CVFilter2DMatVal(DataLink *inS1, DataLink *inK1, DataLink *outR1, int x_anchor, int y_anchor)
{
	cv::Mat src, dst;
	int kernel_size;
	float* kernel;

	DecodeMat(inS1, &src);

	kernel_size = sqrt((float)(inK1->Size / sizeof(float)));
	kernel = new float[kernel_size*kernel_size]();
	for (int i = 0; i < kernel_size*kernel_size; ++i)
	{
		memcpy(&kernel[i], inK1->Data + i*sizeof(float), sizeof(float));
	}
	cv::Mat kernel_matrix = cv::Mat(kernel_size, kernel_size, CV_32FC1, kernel);
	cv::filter2D(src, dst, -1, kernel_matrix, cv::Point(-x_anchor, -y_anchor));

	delete kernel;
	EncodeMat(&dst, outR1);
	return 0;
}
int CVGetRegionMatVal(DataLink *in11, DataLink *in21, DataLink *out31, DataLink *out41, DataLink *out51)
{
	cv::Mat src_image;
	cv::Rect cur_face;
	int diffx = 0;
	int diffy = 0;

	DecodeMat(in11, &src_image);
	memcpy(&cur_face, in21->Data, sizeof(cur_face));
	cv::Mat image_reg = src_image(cur_face);
	diffx = cur_face.x; diffy = cur_face.y;
	EncodeMat(&image_reg.clone(), out31);
	WriteInteger(out41, diffx);
	WriteInteger(out51, diffy);
	return 0;
}
int CVResizeMatVal(DataLink *inS1, DataLink *inW1, DataLink *inH1, DataLink *outD1, double fx, double fy, int interpolation)
{
	cv::Mat inputMat;
	cv::Mat outputMat;

	DecodeMat(inS1, &inputMat);
	cv::resize(inputMat, outputMat, cv::Size(ReadInteger(inW1), ReadInteger(inH1)), fx, fy, interpolation);
	EncodeMat(&outputMat, outD1);
	inputMat.release();
	return 0;
}
int CVSetTransparencyMatVal(DataLink *in11, DataLink *in21, DataLink *out31, double transparency1, double transparency2)
{
	cv::Mat src1, src2, dst;

	DecodeMat(in11, &src1);
	DecodeMat(in21, &src2);

	cv::addWeighted(src1, transparency1, src2, transparency2, 0.0, dst);
	EncodeMat(&dst, out31);

	return 0;
}
int CVShowMatVal(DataLink *in11, DataLink *out21, char* window_name, int size)
{
	cv::Mat dst;

	cv::namedWindow(window_name, size);
	DecodeMat(in11, &dst);
	imshow(window_name, dst);
	return 0;
}
int CVExtractChannelMatVal(DataLink *in11, DataLink *out21)
{

	cv::Mat src, dst;
	DecodeMat(in11, &src);
	Mat planes[3];
	cv::split(src, planes);

	planes[0] = Mat::zeros(src.rows, src.cols, CV_8UC1);
	planes[1] = Mat::zeros(src.rows, src.cols, CV_8UC1);
	cv::merge(planes, 3, dst);
	EncodeMat(&dst, out21);

	return 0;
}
int CVPutTextVal(DataLink *inM1, DataLink *inS1, DataLink *inX1, DataLink *inY1, DataLink *outD1, int fontFace, int thickness)
{
	int pos_x, pos_y;
	cv::Mat image;
	char* inText;

	inText = (char*)malloc(inS1->Size);
	memcpy(inText, inS1->Data, inS1->Size);
	std::string text(inText, inS1->Size);
	pos_x = ReadInteger(inX1);
	pos_y = ReadInteger(inY1);
	DecodeMat(inM1, &image);

	cv::putText(image, text.c_str(), cv::Point(pos_x, pos_y), fontFace, 1.0, cv::Scalar(255, 255, 255), thickness);

	EncodeMat(&image, outD1);
	return 0;
}
int CVQueryFrameVal(DataLink *in1, DataLink *out1)
{
	VideoCapture* capture = (VideoCapture*)ReadReference(in1);

	cv::Mat tmpImg;
	capture->operator>>(tmpImg);

	EncodeMat(&tmpImg, out1);
	return 0;
}
int CVwaitVal(DataLink *in11, DataLink *out21, int time)
{
	cvWaitKey(time);
	return 0;
}
int CVCreateKernelMatrix(DataLink *out11, int kType)
{
	const int matr_size = 3;
	float kernel[matr_size*matr_size];
	float darkCoeff = 0.1;
	float smoothCoeff = 0.1;
	switch (kType)
	{
	case 1:	
		kernel[0] = smoothCoeff; kernel[1] = smoothCoeff; kernel[2] = smoothCoeff;
		kernel[3] = smoothCoeff; kernel[4] = smoothCoeff; kernel[5] = smoothCoeff;
		kernel[6] = smoothCoeff; kernel[7] = smoothCoeff; kernel[8] = smoothCoeff;
		break;
	case 2:	
		kernel[0] = -0.1; kernel[1] = -0.1; kernel[2] = -0.1;
		kernel[3] = -0.1;	kernel[4] = 2; kernel[5] = -0.1;
		kernel[6] = -0.1; kernel[7] = -0.1; kernel[8] = -0.1;
		break;
	case 3:	
		kernel[0] = -0.1; kernel[1] = 0.2; kernel[2] = -0.1;
		kernel[3] = 0.2; kernel[4] = 2; kernel[5] = 0.2;
		kernel[6] = -0.1; kernel[7] = 0.2; kernel[8] = -0.1;
		break;
	case 4:	
		kernel[0] = -darkCoeff; kernel[1] = darkCoeff; kernel[2] = -darkCoeff;
		kernel[3] = darkCoeff;  kernel[4] = 0.8; kernel[5] = darkCoeff;
		kernel[6] = -darkCoeff; kernel[7] = darkCoeff; kernel[8] = -darkCoeff;
		break;
	};
	out11->Size = matr_size*matr_size*sizeof(float);
	out11->Data = (char*)malloc(out11->Size);
	for (int i = 0; i < matr_size*matr_size; ++i)
	{
		memcpy(out11->Data + i*sizeof(float), &kernel[i], sizeof(float));
	}
	return 0;
}
int cvGetCapturePropsVal(DataLink *inC1, DataLink *outP1, int propID)
{
	cv::VideoCapture* capture = (cv::VideoCapture*)ReadReference(inC1);
	double outVal = capture->get(propID);
	WriteDouble(outP1, outVal);
	return 0;
}
int complex_4970(DataLink* out11, DataQueue* queue_8405, DataQueue* queue_8410)
{
	DataLink link_4997 = { NULL, 0, 0 };
	DataLink link_4998 = { NULL, 0, 0 };
	DataLink link_5014 = { NULL, 0, 0 };
	DataLink link_5016 = { NULL, 0, 0 };
	DataLink link_5017 = { NULL, 0, 0 };
	DataLink link_5015 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	CVCaptureFromCam(&link_4997, 0); //cvCapture FromCam (4973)
	splitter(&link_4997, &link_5014); //splitter 4992
	splitter(&link_4997, &link_4998); //splitter 4992
	splitter(&link_4997, &link_5015); //splitter 4992
	virtual_p_out(&link_4998, out11);
	FreeLink(&link_4998);
	FreeLink(&link_4997);
	cvGetCapturePropsVal(&link_5014, &link_5016, CV_CAP_PROP_FRAME_WIDTH); //cvGetCaptureProperties (5128)
	queue_write(&link_5016, queue_8405);
	FreeLink(&link_5014);
	FreeLink(&link_5016);
	cvGetCapturePropsVal(&link_5015, &link_5017, CV_CAP_PROP_FRAME_HEIGHT); //cvGetCaptureProperties (5131)
	queue_write(&link_5017, queue_8410);
	FreeLink(&link_5015);
	FreeLink(&link_5017);
	FreeLink(&result);
	return 0;
}

int if_8161_false(DataLink* in41, DataLink* in21, DataLink* out31)
{
	DataLink link_7403 = { NULL, 0, 0 };
	DataLink link_7404 = { NULL, 0, 0 };
	DataLink link_7405 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	virtual_p_in(in21, &link_7403);
	splitter(&link_7403, &link_7404); //splitter 7398
	virtual_p_out(&link_7404, out31);
	FreeLink(&link_7404);
	FreeLink(&link_7403);
	virtual_p_in(in41, &link_7405);
	FreeLink(&link_7405);
	FreeLink(&result);
	return 0;
}

int if_8161_true(DataLink* in41, DataLink* in21, DataLink* in1, DataLink* out31)
{
	DataLink link_7360 = { NULL, 0, 0 };
	DataLink link_7361 = { NULL, 0, 0 };
	DataLink link_7362 = { NULL, 0, 0 };
	DataLink link_7363 = { NULL, 0, 0 };
	DataLink link_7364 = { NULL, 0, 0 };
	DataLink link_7365 = { NULL, 0, 0 };
	DataLink link_7366 = { NULL, 0, 0 };
	DataLink link_7367 = { NULL, 0, 0 };
	DataLink link_8431 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	/*Assign constant 7337 to 95.0 */
	link_7362.Size = 8;
	link_7362.Data = (char*)malloc(8);
	SetDoubleValue(95.0, &link_7362);
	/*Assign constant 7348 to 10 */
	link_7363.Size = 4;
	link_7363.Data = (char*)malloc(4);
	SetControlValue(10, &link_7363);
	virtual_p_in(in41, &link_7364);
	intToChar(&link_7364, &link_7365); //To  char* (7350)
	FreeLink(&link_7364);
	virtual_p_in(in1, &link_8431);
	llSub__double(&link_8431, &link_7362, &link_7366); //- (7339)
	FreeLink(&link_7362);
	FreeLink(&link_8431);
	llFloatToInt__double(&link_7366, &link_7367); //ftoi (7353)
	FreeLink(&link_7366);
	virtual_p_in(in21, &link_7361);
	CVPutTextVal(&link_7361, &link_7365, &link_7367, &link_7363, &link_7360, FONT_HERSHEY_PLAIN, 2); //cvPutText (7326)
	virtual_p_out(&link_7360, out31);
	FreeLink(&link_7360);
	FreeLink(&link_7361);
	FreeLink(&link_7363);
	FreeLink(&link_7365);
	FreeLink(&link_7367);
	FreeLink(&result);
	return 0;
}

int complex_7445(DataLink* in21, DataLink* out11)
{
	DataLink link_7937 = { NULL, 0, 0 };
	DataLink link_7938 = { NULL, 0, 0 };
	DataLink link_7939 = { NULL, 0, 0 };
	DataLink link_7940 = { NULL, 0, 0 };
	DataLink link_7941 = { NULL, 0, 0 };
	DataLink link_7942 = { NULL, 0, 0 };
	DataLink link_7943 = { NULL, 0, 0 };
	DataLink link_7944 = { NULL, 0, 0 };
	DataLink link_7945 = { NULL, 0, 0 };
	DataLink link_7946 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	CVCreateKernelMatrix(&link_7943, 4); //Create kernel matrix (7932)
	virtual_p_in(in21, &link_7937);
	splitter(&link_7937, &link_7939); //splitter 7903
	splitter(&link_7937, &link_7944); //splitter 7903
	FreeLink(&link_7937);
	splitter(&link_7944, &link_7938); //splitter 7917
	FreeLink(&link_7944);
	CVExtractChannelMatVal(&link_7938, &link_7940); //Extract Red Channel (7907)
	FreeLink(&link_7938);
	splitter(&link_7940, &link_7941); //splitter 7920
	FreeLink(&link_7940);
	CVSetTransparencyMatVal(&link_7939, &link_7941, &link_7945, 0.35, 0.65); //Set transparency (cv::Mat) (7910)
	FreeLink(&link_7939);
	FreeLink(&link_7941);
	splitter(&link_7945, &link_7946); //splitter 7934
	FreeLink(&link_7945);
	CVFilter2DMatVal(&link_7946, &link_7943, &link_7942, -1, -1); //cvFilter2D (cv::Mat) (7925)
	virtual_p_out(&link_7942, out11);
	FreeLink(&link_7942);
	FreeLink(&link_7943);
	FreeLink(&link_7946);
	FreeLink(&result);
	return 0;
}

int complex_7887(DataLink* in11, DataLink* in1, DataArray* array_7794, DataLink* out31)
{
	DataLink link_7882 = { NULL, 0, 0 };
	DataLink link_7883 = { NULL, 0, 0 };
	DataLink link_7884 = { NULL, 0, 0 };
	DataLink link_7885 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	virtual_p_in(in11, &link_7882);
	virtual_p_in(in1, &link_7885);
	CVDetectFacesMatVal(&link_7885, &link_7882, &link_7884, &link_7883); //Detect object (cv::Mat) (7871)
	virtual_p_out(&link_7883, out31);
	FreeLink(&link_7883);
	array_write(&link_7884, array_7794);
	FreeLink(&link_7882);
	FreeLink(&link_7885);
	FreeLink(&link_7884);
	FreeLink(&result);
	return 0;
}

int if_7828_false(DataLink* in41, DataLink* out31)
{
	DataLink link_7500 = { NULL, 0, 0 };
	DataLink link_7501 = { NULL, 0, 0 };
	DataLink link_7502 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	/*Assign constant 7495 to 1 */
	link_7501.Size = 4;
	link_7501.Data = (char*)malloc(4);
	SetControlValue(1, &link_7501);
	splitter(&link_7501, &link_7502); //splitter 7497
	virtual_p_out(&link_7502, out31);
	FreeLink(&link_7502);
	FreeLink(&link_7501);
	virtual_p_in(in41, &link_7500);
	FreeLink(&link_7500);
	FreeLink(&result);
	return 0;
}

int if_7828_true(DataLink* in41, DataLink* in1, DataArray* array_7794, DataLink* out31)
{
	DataLink link_7897 = { NULL, 0, 0 };
	DataLink link_7898 = { NULL, 0, 0 };
	DataLink link_7899 = { NULL, 0, 0 };
	DataLink link_8589 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	virtual_p_in(in41, &link_7897);
	virtual_p_in(in1, &link_8589);
	complex_7887(&link_7897, &link_8589, array_7794, &link_7899);
	virtual_p_out(&link_7899, out31);
	FreeLink(&link_7899);
	FreeLink(&link_7897);
	FreeLink(&link_8589);
	FreeLink(&link_7898);
	FreeLink(&result);
	return 0;
}

int complex_7726(DataLink* in71, DataLink* in11, DataLink* in21, DataLink* in31, DataLink* in41, DataLink* in61, DataLink* out51)
{
	DataLink link_8059 = { NULL, 0, 0 };
	DataLink link_8060 = { NULL, 0, 0 };
	DataLink link_8061 = { NULL, 0, 0 };
	DataLink link_8062 = { NULL, 0, 0 };
	DataLink link_8063 = { NULL, 0, 0 };
	DataLink link_8064 = { NULL, 0, 0 };
	DataLink link_8065 = { NULL, 0, 0 };
	DataLink link_8066 = { NULL, 0, 0 };
	DataLink link_8067 = { NULL, 0, 0 };
	DataLink link_8068 = { NULL, 0, 0 };
	DataLink link_8069 = { NULL, 0, 0 };
	DataLink link_8070 = { NULL, 0, 0 };
	DataLink link_8071 = { NULL, 0, 0 };
	DataLink link_8072 = { NULL, 0, 0 };
	DataLink link_8073 = { NULL, 0, 0 };
	DataLink link_8074 = { NULL, 0, 0 };
	DataLink link_8075 = { NULL, 0, 0 };
	DataLink link_8076 = { NULL, 0, 0 };
	DataLink link_8077 = { NULL, 0, 0 };
	DataLink link_8078 = { NULL, 0, 0 };
	DataLink link_8079 = { NULL, 0, 0 };
	DataLink link_8080 = { NULL, 0, 0 };
	DataLink link_8081 = { NULL, 0, 0 };
	DataLink link_8082 = { NULL, 0, 0 };
	DataLink link_8083 = { NULL, 0, 0 };
	DataLink link_8084 = { NULL, 0, 0 };
	DataLink link_8085 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	/*Assign constant 8009 to 20 */
	link_8064.Size = 4;
	link_8064.Data = (char*)malloc(4);
	SetControlValue(20, &link_8064);
	/*Assign constant 8028 to 40 */
	link_8069.Size = 4;
	link_8069.Data = (char*)malloc(4);
	SetControlValue(40, &link_8069);
	/*Assign constant 8039 to 20 */
	link_8073.Size = 4;
	link_8073.Data = (char*)malloc(4);
	SetControlValue(20, &link_8073);
	/*Assign constant 7948 to 30 */
	link_8062.Size = 4;
	link_8062.Data = (char*)malloc(4);
	SetControlValue(30, &link_8062);
	/*Assign constant 7948 to 30 */
	link_8063.Size = 4;
	link_8063.Data = (char*)malloc(4);
	SetControlValue(30, &link_8063);
	/*Assign constant 7948 to 30 */
	link_8078.Size = 4;
	link_8078.Data = (char*)malloc(4);
	SetControlValue(30, &link_8078);
	splitter(&link_8078, &link_8077); //splitter 8041
	splitter(&link_8078, &link_8076); //splitter 8041
	FreeLink(&link_8078);
	virtual_p_in(in71, &link_8083);
	splitter(&link_8083, &link_8066); //splitter 8011
	splitter(&link_8083, &link_8067); //splitter 8011
	FreeLink(&link_8083);
	llSum(&link_8066, &link_8064, &link_8065); //+ (7996)
	FreeLink(&link_8064);
	FreeLink(&link_8066);
	virtual_p_in(in11, &link_8079);
	virtual_p_in(in21, &link_8080);
	CVPutTextVal(&link_8079, &link_8080, &link_8062, &link_8067, &link_8059, FONT_HERSHEY_PLAIN, 2); //cvPutText (7952)
	FreeLink(&link_8062);
	FreeLink(&link_8067);
	FreeLink(&link_8079);
	FreeLink(&link_8080);
	splitter(&link_8065, &link_8071); //splitter 8005
	splitter(&link_8065, &link_8068); //splitter 8005
	FreeLink(&link_8065);
	virtual_p_in(in31, &link_8084);
	CVPutTextVal(&link_8059, &link_8084, &link_8063, &link_8068, &link_8060, FONT_HERSHEY_PLAIN, 2); //cvPutText (7963)
	FreeLink(&link_8059);
	FreeLink(&link_8063);
	FreeLink(&link_8068);
	FreeLink(&link_8084);
	llSum(&link_8071, &link_8069, &link_8070); //+ (8015)
	FreeLink(&link_8069);
	FreeLink(&link_8071);
	splitter(&link_8070, &link_8075); //splitter 8024
	splitter(&link_8070, &link_8072); //splitter 8024
	FreeLink(&link_8070);
	virtual_p_in(in41, &link_8085);
	CVPutTextVal(&link_8060, &link_8085, &link_8076, &link_8072, &link_8061, FONT_HERSHEY_PLAIN, 2); //cvPutText (7974)
	FreeLink(&link_8060);
	FreeLink(&link_8072);
	FreeLink(&link_8076);
	FreeLink(&link_8085);
	llSum(&link_8075, &link_8073, &link_8074); //+ (8030)
	FreeLink(&link_8073);
	FreeLink(&link_8075);
	virtual_p_in(in61, &link_8082);
	CVPutTextVal(&link_8061, &link_8082, &link_8077, &link_8074, &link_8081, FONT_HERSHEY_PLAIN, 2); //cvPutText (7985)
	virtual_p_out(&link_8081, out51);
	FreeLink(&link_8081);
	FreeLink(&link_8061);
	FreeLink(&link_8074);
	FreeLink(&link_8077);
	FreeLink(&link_8082);
	FreeLink(&result);
	return 0;
}

int complex_7724(DataLink* in11, DataLink* in1, DataQueue* queue_8410, DataLink* in3)
{
	DataLink link_7386 = { NULL, 0, 0 };
	DataLink link_7387 = { NULL, 0, 0 };
	DataLink link_7389 = { NULL, 0, 0 };
	DataLink link_8425 = { NULL, 0, 0 };
	DataLink link_8426 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	virtual_p_in(in11, &link_7387);
	virtual_p_in(in1, &link_8426);
	queue_read(&link_8425, sdo_read, queue_8410);
	GetTurnAngle(&link_7387, &link_8425, &link_8426, &link_7389); //Where turn servo (7379)
	FreeLink(&link_7387);
	FreeLink(&link_8425);
	FreeLink(&link_8426);
	virtual_p_in(in3, &link_7386);
	WiringXPutChar(&link_7386, &link_7389); //WiringX PutChar (7372)
	FreeLink(&link_7386);
	FreeLink(&link_7389);
	FreeLink(&result);
	return 0;
}

int complex_7703(DataLink* in11, DataArray* array_4420, DataLink* in2, DataLink* in3, DataLink* in4, DataLink* out51, DataLink* out41, DataLink* out31, DataLink* out21)
{
	DataLink link_7568 = { NULL, 0, 0 };
	DataLink link_7569 = { NULL, 0, 0 };
	DataLink link_7572 = { NULL, 0, 0 };
	DataLink link_7573 = { NULL, 0, 0 };
	DataLink link_7574 = { NULL, 0, 0 };
	DataLink link_7575 = { NULL, 0, 0 };
	DataLink link_7576 = { NULL, 0, 0 };
	DataLink link_7577 = { NULL, 0, 0 };
	DataLink link_7578 = { NULL, 0, 0 };
	DataLink link_7579 = { NULL, 0, 0 };
	DataLink link_8748 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	virtual_p_in(in11, &link_7569);
	virtual_p_in(in4, &link_7573);
	virtual_p_in(in2, &link_7574);
	CVResizeMatVal(&link_7569, &link_7573, &link_7574, &link_7568, 0.5, 0.5, CV_INTER_NN); //Resize (cv::Mat) (7523)
	FreeLink(&link_7569);
	FreeLink(&link_7573);
	FreeLink(&link_7574);
	virtual_p_in(in3, &link_7575);
	CVFaceRecognizerPredictVal(&link_7575, &link_7568, &link_8748); //FaceRecognizer Predict (7532)
	FreeLink(&link_7568);
	FreeLink(&link_7575);
	array_read_el(&link_7572, GetControlValue(&link_8748), sdo_read, array_4420);
	FreeLink(&link_8748);
	GetTerminatorInfo(&link_7572, &link_7576, &link_7577, &link_7578, &link_7579); //Get terminate info (7550)
	virtual_p_out(&link_7576, out21);
	FreeLink(&link_7576);
	virtual_p_out(&link_7577, out31);
	FreeLink(&link_7577);
	virtual_p_out(&link_7578, out41);
	FreeLink(&link_7578);
	virtual_p_out(&link_7579, out51);
	FreeLink(&link_7579);
	FreeLink(&link_7572);
	FreeLink(&result);
	return 0;
}

int for_7617(DataLink* in41, DataArray* array_7794, DataLink* in61, DataLink* Vin_7596, int counter)
{
	DataLink link_7611 = { NULL, 0, 0 };
	DataLink link_7612 = { NULL, 0, 0 };
	DataLink link_7613 = { NULL, 0, 0 };
	DataLink link_7614 = { NULL, 0, 0 };
	DataLink link_7615 = { NULL, 0, 0 };
	DataLink link_8747 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	/*Assign counter 7585 to counter*/
	link_8747.Size = 4;
	link_8747.Data = (char*)malloc(4);
	memcpy(link_8747.Data, (char*)(&counter), link_8747.Size);
	array_read_el(&link_7613, GetControlValue(&link_8747), sdo_read_erase, array_7794);
	FreeLink(&link_8747);
	virtual_p_in(in41, &link_7611);
	virtual_p_in(in61, &link_7612);
	virtual_p_in(Vin_7596, &link_7614);
	CVDrawRectMatVal(&link_7614, &link_7613, &link_7612, &link_7611, &link_7615, 2, 8); //Draw rectangle (cv::Mat) (7598)
	virtual_r_out(&link_7615, Vin_7596);
	FreeLink(&link_7615);
	FreeLink(&link_7611);
	FreeLink(&link_7612);
	FreeLink(&link_7613);
	FreeLink(&link_7614);
	FreeLink(&result);
	return 0;
}

int if_7682_false(DataLink* in31, DataLink* in21, DataLink* in51, DataLink* in41, DataLink* out71)
{
	DataLink link_8139 = { NULL, 0, 0 };
	DataLink link_8140 = { NULL, 0, 0 };
	DataLink link_8141 = { NULL, 0, 0 };
	DataLink link_8142 = { NULL, 0, 0 };
	DataLink link_8143 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	virtual_p_in(in31, &link_8141);
	FreeLink(&link_8141);
	virtual_p_in(in41, &link_8142);
	FreeLink(&link_8142);
	virtual_p_in(in51, &link_8143);
	FreeLink(&link_8143);
	virtual_p_in(in21, &link_8139);
	splitter(&link_8139, &link_8140); //splitter 8130
	virtual_p_out(&link_8140, out71);
	FreeLink(&link_8140);
	FreeLink(&link_8139);
	FreeLink(&result);
	return 0;
}

int if_7682_true(DataArray* array_7794, DataLink* in31, DataLink* in21, DataLink* in51, DataLink* in41, DataLink* out71)
{
	DataLink link_7636 = { NULL, 0, 0 };
	DataLink link_7637 = { NULL, 0, 0 };
	DataLink link_7638 = { NULL, 0, 0 };
	DataLink link_7639 = { NULL, 0, 0 };
	DataLink link_7640 = { NULL, 0, 0 };
	DataLink link_7641 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	virtual_p_in(in21, &link_7636);
	virtual_p_in(in31, &link_7637);
	virtual_p_in(in41, &link_7638);
	virtual_p_in(in51, &link_7639);
	/* Call for 7617 */
	N = GetControlValue(&link_7639);
	FreeLink(&link_7639);
	for (i = 0; i < N; i++)
	{
		for_7617(&link_7638, array_7794, &link_7637, &link_7636, i);
	}
	if (link_7640.Size < link_7636.Size)
	{
		FreeLink(&link_7640);
		link_7640.Size = link_7636.Size;
		link_7640.Data = (char*)malloc(link_7640.Size);
	}
	memcpy(link_7640.Data, link_7636.Data, link_7636.Size);
	FreeLink(&link_7636);
	virtual_p_out(&link_7640, out71);
	FreeLink(&link_7640);
	FreeLink(&link_7636);
	FreeLink(&link_7637);
	FreeLink(&link_7638);
	FreeLink(&link_7641);
	FreeLink(&result);
	return 0;
}

int if_7771_false(DataLink* in101, DataLink* in31, DataLink* out111, DataLink* out41)
{
	DataLink link_7518 = { NULL, 0, 0 };
	DataLink link_7519 = { NULL, 0, 0 };
	DataLink link_7520 = { NULL, 0, 0 };
	DataLink link_7521 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	virtual_p_in(in101, &link_7520);
	splitter(&link_7520, &link_7521); //splitter 7515
	virtual_p_out(&link_7521, out111);
	FreeLink(&link_7521);
	FreeLink(&link_7520);
	virtual_p_in(in31, &link_7518);
	splitter(&link_7518, &link_7519); //splitter 7508
	virtual_p_out(&link_7519, out41);
	FreeLink(&link_7519);
	FreeLink(&link_7518);
	FreeLink(&result);
	return 0;
}

int if_7771_true(DataLink* in101, DataLink* in81, DataArray* array_7794, DataLink* in91, DataLink* in31, DataLink* in71, DataLink* in61, DataLink* in51, DataLink* in1, DataQueue* queue_8410, DataArray* array_4420, DataLink* in4, DataLink* in5, DataLink* in6, DataLink* in7, DataLink* out111, DataLink* out41)
{
	DataLink link_7734 = { NULL, 0, 0 };
	DataLink link_7735 = { NULL, 0, 0 };
	DataLink link_7736 = { NULL, 0, 0 };
	DataLink link_7737 = { NULL, 0, 0 };
	DataLink link_7738 = { NULL, 0, 0 };
	DataLink link_7739 = { NULL, 0, 0 };
	DataLink link_7740 = { NULL, 0, 0 };
	DataLink link_7741 = { NULL, 0, 0 };
	DataLink link_7742 = { NULL, 0, 0 };
	DataLink link_7743 = { NULL, 0, 0 };
	DataLink link_7744 = { NULL, 0, 0 };
	DataLink link_7745 = { NULL, 0, 0 };
	DataLink link_7746 = { NULL, 0, 0 };
	DataLink link_7747 = { NULL, 0, 0 };
	DataLink link_7748 = { NULL, 0, 0 };
	DataLink link_7749 = { NULL, 0, 0 };
	DataLink link_7750 = { NULL, 0, 0 };
	DataLink link_7751 = { NULL, 0, 0 };
	DataLink link_7752 = { NULL, 0, 0 };
	DataLink link_7753 = { NULL, 0, 0 };
	DataLink link_7754 = { NULL, 0, 0 };
	DataLink link_7755 = { NULL, 0, 0 };
	DataLink link_7756 = { NULL, 0, 0 };
	DataLink link_7757 = { NULL, 0, 0 };
	DataLink link_7758 = { NULL, 0, 0 };
	DataLink link_7759 = { NULL, 0, 0 };
	DataLink link_8513 = { NULL, 0, 0 };
	DataLink link_8540 = { NULL, 0, 0 };
	DataLink link_8569 = { NULL, 0, 0 };
	DataLink link_8573 = { NULL, 0, 0 };
	DataLink link_8577 = { NULL, 0, 0 };
	DataLink link_8581 = { NULL, 0, 0 };
	DataLink link_8585 = { NULL, 0, 0 };
	DataLink link_8758 = { NULL, 0, 0 };
	DataLink link_8760 = { NULL, 0, 0 };
	DataLink link_8762 = { NULL, 0, 0 };
	DataLink link_8764 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	/*Assign constant 7701 to 0 */
	link_7745.Size = 4;
	link_7745.Data = (char*)malloc(4);
	SetControlValue(0, &link_7745);
	/*Assign constant 7690 to 0 */
	link_7744.Size = 4;
	link_7744.Data = (char*)malloc(4);
	SetControlValue(0, &link_7744);
	/*Assign constant 7722 to 110 */
	link_7751.Size = 4;
	link_7751.Data = (char*)malloc(4);
	SetControlValue(110, &link_7751);
	/*Assign constant 7720 to 1 */
	link_7748.Size = 4;
	link_7748.Data = (char*)malloc(4);
	SetControlValue(1, &link_7748);
	/*Assign constant 8756 to 20 */
	link_8758.Size = 4;
	link_8758.Data = (char*)malloc(4);
	SetControlValue(20, &link_8758);
	/*Assign constant 7656 to 0 */
	link_7735.Size = 4;
	link_7735.Data = (char*)malloc(4);
	SetControlValue(0, &link_7735);
	/*Assign constant 7658 to 0 */
	link_7734.Size = 4;
	link_7734.Data = (char*)malloc(4);
	SetControlValue(0, &link_7734);
	virtual_p_in(in71, &link_8762);
	splitter(&link_8762, &link_7736); //splitter 8761
	splitter(&link_8762, &link_7752); //splitter 8761
	FreeLink(&link_8762);
	virtual_p_in(in101, &link_8764);
	splitter(&link_8764, &link_7747); //splitter 8763
	splitter(&link_8764, &link_7750); //splitter 8763
	FreeLink(&link_8764);
	virtual_p_in(in91, &link_7746);
	virtual_p_in(in4, &link_8573);
	virtual_p_in(in5, &link_8577);
	virtual_p_in(in6, &link_8581);
	complex_7703(&link_7746, array_4420, &link_8573, &link_8577, &link_8581, &link_7756, &link_7757, &link_7758, &link_7759);
	FreeLink(&link_7746);
	FreeLink(&link_8569);
	FreeLink(&link_8573);
	FreeLink(&link_8577);
	FreeLink(&link_8581);
	llAnd__int(&link_7744, &link_7745, &link_7743); //&& (7692)
	FreeLink(&link_7744);
	FreeLink(&link_7745);
	virtual_p_in(in31, &link_7737);
	CVDrawRectMatVal(&link_7737, &link_7736, &link_7735, &link_7734, &link_7738, 2, 8); //Draw rectangle (cv::Mat) (7662)
	FreeLink(&link_7734);
	FreeLink(&link_7735);
	FreeLink(&link_7736);
	FreeLink(&link_7737);
	llMul(&link_7750, &link_7751, &link_8760); //llMul (8749)
	FreeLink(&link_7750);
	FreeLink(&link_7751);
	virtual_p_in(in1, &link_8513);
	virtual_p_in(in7, &link_8585);
	complex_7724(&link_7752, &link_8513, queue_8410, &link_8585);
	FreeLink(&link_7752);
	FreeLink(&link_8513);
	FreeLink(&link_8540);
	FreeLink(&link_8585);
	llSum(&link_7748, &link_7747, &link_7749); //+ (7711)
	virtual_p_out(&link_7749, out111);
	FreeLink(&link_7749);
	FreeLink(&link_7747);
	FreeLink(&link_7748);
	llSum(&link_8760, &link_8758, &link_7754); //llSum (8753)
	FreeLink(&link_8758);
	FreeLink(&link_8760);
	virtual_p_in(in61, &link_7739);
	virtual_p_in(in51, &link_7740);
	virtual_p_in(in81, &link_7741);
	/* Call conditional control 7682 */
	conditionVar = GetControlValue(&link_7743);
	FreeLink(&link_7743);
	if ((conditionVar) != 0)
		if_7682_true(array_7794, &link_7739, &link_7738, &link_7741, &link_7740, &link_7755);
	else
		if_7682_false(&link_7739, &link_7738, &link_7741, &link_7740, &link_7755);
	FreeLink(&link_7738);
	FreeLink(&link_7739);
	FreeLink(&link_7740);
	FreeLink(&link_7741);
	FreeLink(&link_7742);
	complex_7726(&link_7754, &link_7755, &link_7759, &link_7758, &link_7757, &link_7756, &link_7753);
	virtual_p_out(&link_7753, out41);
	FreeLink(&link_7753);
	FreeLink(&link_7754);
	FreeLink(&link_7755);
	FreeLink(&link_7756);
	FreeLink(&link_7757);
	FreeLink(&link_7758);
	FreeLink(&link_7759);
	FreeLink(&result);
	return 0;
}

int for_7463(DataLink* in21, DataLink* Vin_7835, DataLink* Vin_7763, DataArray* array_7423, DataLink* in1, DataQueue* queue_8410, DataArray* array_4420, DataLink* in4, DataLink* in5, DataLink* in6, DataLink* in7, DataLink* in8, int counter)
{
	DataLink link_7839 = { NULL, 0, 0 };
	DataLink link_7842 = { NULL, 0, 0 };
	DataLink link_7843 = { NULL, 0, 0 };
	DataLink link_7844 = { NULL, 0, 0 };
	DataLink link_7845 = { NULL, 0, 0 };
	DataLink link_7846 = { NULL, 0, 0 };
	DataLink link_7847 = { NULL, 0, 0 };
	DataLink link_7848 = { NULL, 0, 0 };
	DataLink link_7849 = { NULL, 0, 0 };
	DataLink link_7850 = { NULL, 0, 0 };
	DataLink link_7851 = { NULL, 0, 0 };
	DataLink link_7852 = { NULL, 0, 0 };
	DataLink link_7853 = { NULL, 0, 0 };
	DataLink link_7854 = { NULL, 0, 0 };
	DataLink link_7855 = { NULL, 0, 0 };
	DataLink link_7856 = { NULL, 0, 0 };
	DataLink link_7857 = { NULL, 0, 0 };
	DataLink link_7858 = { NULL, 0, 0 };
	DataLink link_7859 = { NULL, 0, 0 };
	DataLink link_7860 = { NULL, 0, 0 };
	DataLink link_7861 = { NULL, 0, 0 };
	DataLink link_7862 = { NULL, 0, 0 };
	DataLink link_8519 = { NULL, 0, 0 };
	DataLink link_8543 = { NULL, 0, 0 };
	DataLink link_8593 = { NULL, 0, 0 };
	DataLink link_8597 = { NULL, 0, 0 };
	DataLink link_8601 = { NULL, 0, 0 };
	DataLink link_8605 = { NULL, 0, 0 };
	DataLink link_8609 = { NULL, 0, 0 };
	DataLink link_8613 = { NULL, 0, 0 };
	DataLink link_8765 = { NULL, 0, 0 };
	DataArray array_7794 = { (char*)malloc(320), (char*)malloc(20), 16, 20, "Array 7794" };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	for (i = 0; i < 20; i++)
		array_7794.Flags[i] = 0;
	/*Assign constant 7833 to 0 */
	link_7856.Size = 4;
	link_7856.Data = (char*)malloc(4);
	SetControlValue(0, &link_7856);
	/*Assign constant 7769 to 0 */
	link_7845.Size = 4;
	link_7845.Data = (char*)malloc(4);
	SetControlValue(0, &link_7845);
	/*Assign counter 7788 to counter*/
	link_8765.Size = 4;
	link_8765.Data = (char*)malloc(4);
	memcpy(link_8765.Data, (char*)(&counter), link_8765.Size);
	array_read_el(&link_7842, GetControlValue(&link_8765), sdo_read_erase, array_7423);
	FreeLink(&link_8765);
	virtual_p_in(in21, &link_7853);
	splitter(&link_7853, &link_7854); //splitter 7825
	FreeLink(&link_7853);
	splitter(&link_7842, &link_7843); //splitter 7790
	splitter(&link_7842, &link_7849); //splitter 7790
	FreeLink(&link_7842);
	CVGetRegionMatVal(&link_7854, &link_7849, &link_7852, &link_7850, &link_7851); //Get image region (cv::Mat) (7810)
	FreeLink(&link_7849);
	FreeLink(&link_7854);
	splitter(&link_7852, &link_7857); //splitter 7821
	splitter(&link_7852, &link_7855); //splitter 7821
	FreeLink(&link_7852);
	virtual_p_in(in8, &link_8613);
	/* Call conditional control 7828 */
	conditionVar = GetControlValue(&link_7856);
	FreeLink(&link_7856);
	if ((conditionVar) != 0)
		if_7828_true(&link_7857, &link_8613, &array_7794, &link_7859);
	else
		if_7828_false(&link_7857, &link_7859);
	FreeLink(&link_7857);
	FreeLink(&link_8613);
	FreeLink(&link_7858);
	splitter(&link_7859, &link_7848); //splitter 7806
	splitter(&link_7859, &link_7847); //splitter 7806
	FreeLink(&link_7859);
	llGreaterThen(&link_7847, &link_7845, &link_7846); //> (7797)
	FreeLink(&link_7845);
	FreeLink(&link_7847);
	virtual_p_in(Vin_7763, &link_7839);
	virtual_p_in(Vin_7835, &link_7860);
	virtual_p_in(in1, &link_8519);
	virtual_p_in(in4, &link_8597);
	virtual_p_in(in5, &link_8601);
	virtual_p_in(in6, &link_8605);
	virtual_p_in(in7, &link_8609);
	/* Call conditional control 7771 */
	conditionVar = GetControlValue(&link_7846);
	FreeLink(&link_7846);
	if ((conditionVar) != 0)
		if_7771_true(&link_7860, &link_7848, &array_7794, &link_7855, &link_7839, &link_7843, &link_7850, &link_7851, &link_8519, queue_8410, array_4420, &link_8597, &link_8601, &link_8605, &link_8609, &link_7861, &link_7862);
	else
		if_7771_false(&link_7860, &link_7839, &link_7861, &link_7862);
	virtual_r_out(&link_7861, Vin_7835);
	FreeLink(&link_7861);
	virtual_r_out(&link_7862, Vin_7763);
	FreeLink(&link_7862);
	FreeLink(&link_7839);
	FreeLink(&link_7843);
	FreeLink(&link_7844);
	FreeLink(&link_7848);
	FreeLink(&link_7850);
	FreeLink(&link_7851);
	FreeLink(&link_7855);
	FreeLink(&link_7860);
	FreeLink(&link_8519);
	FreeLink(&link_8543);
	FreeLink(&link_8593);
	FreeLink(&link_8597);
	FreeLink(&link_8601);
	FreeLink(&link_8605);
	FreeLink(&link_8609);
	FreeArray(&array_7794);
	FreeLink(&result);
	return 0;
}

int if_7429_false(DataLink* in51, DataLink* out41)
{
	DataLink link_8117 = { NULL, 0, 0 };
	DataLink link_8118 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	virtual_p_in(in51, &link_8117);
	splitter(&link_8117, &link_8118); //splitter 8114
	virtual_p_out(&link_8118, out41);
	FreeLink(&link_8118);
	FreeLink(&link_8117);
	FreeLink(&result);
	return 0;
}

int if_7429_true(DataLink* in71, DataArray* array_7423, DataLink* in31, DataLink* in51, DataLink* in1, DataQueue* queue_8410, DataArray* array_4420, DataLink* in4, DataLink* in5, DataLink* in6, DataLink* in7, DataLink* in8, DataLink* out41)
{
	DataLink link_7482 = { NULL, 0, 0 };
	DataLink link_7483 = { NULL, 0, 0 };
	DataLink link_7484 = { NULL, 0, 0 };
	DataLink link_7485 = { NULL, 0, 0 };
	DataLink link_7486 = { NULL, 0, 0 };
	DataLink link_7487 = { NULL, 0, 0 };
	DataLink link_8525 = { NULL, 0, 0 };
	DataLink link_8546 = { NULL, 0, 0 };
	DataLink link_8621 = { NULL, 0, 0 };
	DataLink link_8625 = { NULL, 0, 0 };
	DataLink link_8629 = { NULL, 0, 0 };
	DataLink link_8633 = { NULL, 0, 0 };
	DataLink link_8637 = { NULL, 0, 0 };
	DataLink link_8641 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	/*Assign constant 7480 to 0 */
	link_7487.Size = 4;
	link_7487.Data = (char*)malloc(4);
	SetControlValue(0, &link_7487);
	virtual_p_in(in71, &link_7484);
	virtual_p_in(in31, &link_7485);
	virtual_p_in(in51, &link_7486);
	virtual_p_in(in1, &link_8525);
	virtual_p_in(in4, &link_8625);
	virtual_p_in(in5, &link_8629);
	virtual_p_in(in6, &link_8633);
	virtual_p_in(in7, &link_8637);
	virtual_p_in(in8, &link_8641);
	/* Call for 7463 */
	N = GetControlValue(&link_7484);
	FreeLink(&link_7484);
	for (i = 0; i < N; i++)
	{
		for_7463(&link_7485, &link_7487, &link_7486, array_7423, &link_8525, queue_8410, array_4420, &link_8625, &link_8629, &link_8633, &link_8637, &link_8641, i);
	}
	if (link_7482.Size < link_7486.Size)
	{
		FreeLink(&link_7482);
		link_7482.Size = link_7486.Size;
		link_7482.Data = (char*)malloc(link_7482.Size);
	}
	memcpy(link_7482.Data, link_7486.Data, link_7486.Size);
	FreeLink(&link_7486);
	virtual_p_out(&link_7482, out41);
	FreeLink(&link_7482);
	FreeLink(&link_7483);
	FreeLink(&link_7485);
	FreeLink(&link_7486);
	FreeLink(&link_7487);
	FreeLink(&link_8525);
	FreeLink(&link_8546);
	FreeLink(&link_8621);
	FreeLink(&link_8625);
	FreeLink(&link_8629);
	FreeLink(&link_8633);
	FreeLink(&link_8637);
	FreeLink(&link_8641);
	FreeLink(&result);
	return 0;
}

int complex_7415(DataLink* in11, DataLink* in1, DataArray* array_7423, DataLink* out31)
{
	DataLink link_8105 = { NULL, 0, 0 };
	DataLink link_8106 = { NULL, 0, 0 };
	DataLink link_8107 = { NULL, 0, 0 };
	DataLink link_8108 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	virtual_p_in(in11, &link_8105);
	virtual_p_in(in1, &link_8108);
	CVDetectFacesMatVal(&link_8108, &link_8105, &link_8106, &link_8107); //Detect object (cv::Mat) (8094)
	virtual_p_out(&link_8107, out31);
	FreeLink(&link_8107);
	array_write(&link_8106, array_7423);
	FreeLink(&link_8105);
	FreeLink(&link_8108);
	FreeLink(&link_8106);
	FreeLink(&result);
	return 0;
}

int complex_8144(DataLink* in11, DataLink* in1, DataQueue* queue_8410, DataLink* in3, DataArray* array_4420, DataLink* in5, DataLink* in6, DataLink* in7, DataLink* in8, DataLink* in9, DataLink* out21)
{
	DataLink link_7448 = { NULL, 0, 0 };
	DataLink link_7449 = { NULL, 0, 0 };
	DataLink link_7450 = { NULL, 0, 0 };
	DataLink link_7451 = { NULL, 0, 0 };
	DataLink link_7452 = { NULL, 0, 0 };
	DataLink link_7453 = { NULL, 0, 0 };
	DataLink link_7454 = { NULL, 0, 0 };
	DataLink link_7455 = { NULL, 0, 0 };
	DataLink link_7456 = { NULL, 0, 0 };
	DataLink link_7457 = { NULL, 0, 0 };
	DataLink link_7458 = { NULL, 0, 0 };
	DataLink link_7459 = { NULL, 0, 0 };
	DataLink link_7460 = { NULL, 0, 0 };
	DataLink link_7461 = { NULL, 0, 0 };
	DataLink link_8531 = { NULL, 0, 0 };
	DataLink link_8549 = { NULL, 0, 0 };
	DataLink link_8565 = { NULL, 0, 0 };
	DataLink link_8649 = { NULL, 0, 0 };
	DataLink link_8653 = { NULL, 0, 0 };
	DataLink link_8657 = { NULL, 0, 0 };
	DataLink link_8661 = { NULL, 0, 0 };
	DataLink link_8665 = { NULL, 0, 0 };
	DataLink link_8669 = { NULL, 0, 0 };
	DataArray array_7423 = { (char*)malloc(480), (char*)malloc(30), 16, 30, "Array 7423" };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	for (i = 0; i < 30; i++)
		array_7423.Flags[i] = 0;
	virtual_p_in(in11, &link_7458);
	splitter(&link_7458, &link_7460); //splitter 7411
	splitter(&link_7458, &link_7456); //splitter 7411
	FreeLink(&link_7458);
	CVToColorVal(&link_7456, &link_7457, COLOR_BGR2GRAY); //To Color (cv::Mat) (7440)
	FreeLink(&link_7456);
	complex_7445(&link_7460, &link_7461);
	FreeLink(&link_7460);
	splitter(&link_7457, &link_7450); //splitter 7419
	splitter(&link_7457, &link_7448); //splitter 7419
	FreeLink(&link_7457);
	virtual_p_in(in3, &link_8565);
	complex_7415(&link_7448, &link_8565, &array_7423, &link_7453);
	FreeLink(&link_7448);
	FreeLink(&link_8565);
	FreeLink(&link_7449);
	splitter(&link_7453, &link_7455); //splitter 7436
	splitter(&link_7453, &link_7454); //splitter 7436
	FreeLink(&link_7453);
	virtual_p_in(in1, &link_8531);
	virtual_p_in(in5, &link_8653);
	virtual_p_in(in6, &link_8657);
	virtual_p_in(in7, &link_8661);
	virtual_p_in(in8, &link_8665);
	virtual_p_in(in9, &link_8669);
	/* Call conditional control 7429 */
	conditionVar = GetControlValue(&link_7454);
	FreeLink(&link_7454);
	if ((conditionVar) != 0)
		if_7429_true(&link_7455, &array_7423, &link_7450, &link_7461, &link_8531, queue_8410, array_4420, &link_8653, &link_8657, &link_8661, &link_8665, &link_8669, &link_7452);
	else
		if_7429_false(&link_7461, &link_7452);
	FreeLink(&link_7450);
	FreeLink(&link_7451);
	FreeLink(&link_7455);
	FreeLink(&link_7461);
	FreeLink(&link_8531);
	FreeLink(&link_8549);
	FreeLink(&link_8649);
	FreeLink(&link_8653);
	FreeLink(&link_8657);
	FreeLink(&link_8661);
	FreeLink(&link_8665);
	FreeLink(&link_8669);
	splitter(&link_7452, &link_7459); //splitter 7426
	virtual_p_out(&link_7459, out21);
	FreeLink(&link_7459);
	FreeLink(&link_7452);
	FreeArray(&array_7423);
	FreeLink(&result);
	return 0;
}

int if_7315_false(DataLink* in41, DataLink* in21, DataQueue* queue_8202)
{
	DataLink link_8215 = { NULL, 0, 0 };
	DataLink link_8254 = { NULL, 0, 0 };
	DataLink link_8327 = { NULL, 0, 0 };
	DataLink link_8328 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	/*Assign constant 8242 to 1 */
	link_8254.Size = 4;
	link_8254.Data = (char*)malloc(4);
	SetControlValue(1, &link_8254);
	virtual_p_in(in41, &link_8327);
	FreeLink(&link_8327);
	virtual_p_in(in21, &link_8215);
	FreeLink(&link_8215);
	splitter(&link_8254, &link_8328); //splitter 8249
	queue_write(&link_8328, queue_8202);
	FreeLink(&link_8254);
	FreeLink(&link_8328);
	FreeLink(&result);
	return 0;
}

int if_7315_true(DataLink* in41, DataLink* in21, DataLink* in31, DataQueue* queue_8405, DataQueue* queue_8410, DataLink* in3, DataArray* array_4420, DataLink* in5, DataLink* in6, DataLink* in7, DataLink* in8, DataLink* in9, DataQueue* queue_8202)
{
	DataLink link_8166 = { NULL, 0, 0 };
	DataLink link_8167 = { NULL, 0, 0 };
	DataLink link_8168 = { NULL, 0, 0 };
	DataLink link_8169 = { NULL, 0, 0 };
	DataLink link_8170 = { NULL, 0, 0 };
	DataLink link_8187 = { NULL, 0, 0 };
	DataLink link_8188 = { NULL, 0, 0 };
	DataLink link_8317 = { NULL, 0, 0 };
	DataLink link_8318 = { NULL, 0, 0 };
	DataLink link_8516 = { NULL, 0, 0 };
	DataLink link_8537 = { NULL, 0, 0 };
	DataLink link_8552 = { NULL, 0, 0 };
	DataLink link_8617 = { NULL, 0, 0 };
	DataLink link_8677 = { NULL, 0, 0 };
	DataLink link_8681 = { NULL, 0, 0 };
	DataLink link_8685 = { NULL, 0, 0 };
	DataLink link_8689 = { NULL, 0, 0 };
	DataLink link_8693 = { NULL, 0, 0 };
	DataLink link_8697 = { NULL, 0, 0 };
	DataLink link_8767 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	/*Assign constant 8159 to 1 */
	link_8168.Size = 4;
	link_8168.Data = (char*)malloc(4);
	SetControlValue(1, &link_8168);
	queue_read(&link_8767, sdo_read, queue_8405);
	splitter(&link_8767, &link_8516); //splitter 8766
	splitter(&link_8767, &link_8537); //splitter 8766
	FreeLink(&link_8767);
	virtual_p_in(in41, &link_8317);
	splitter(&link_8317, &link_8318); //splitter 8263
	queue_write(&link_8318, queue_8202);
	FreeLink(&link_8317);
	FreeLink(&link_8318);
	virtual_p_in(in21, &link_8187);
	virtual_p_in(in3, &link_8617);
	virtual_p_in(in5, &link_8681);
	virtual_p_in(in6, &link_8685);
	virtual_p_in(in7, &link_8689);
	virtual_p_in(in8, &link_8693);
	virtual_p_in(in9, &link_8697);
	complex_8144(&link_8187, &link_8537, queue_8410, &link_8617, array_4420, &link_8681, &link_8685, &link_8689, &link_8693, &link_8697, &link_8170);
	FreeLink(&link_8187);
	FreeLink(&link_8537);
	FreeLink(&link_8552);
	FreeLink(&link_8617);
	FreeLink(&link_8677);
	FreeLink(&link_8681);
	FreeLink(&link_8685);
	FreeLink(&link_8689);
	FreeLink(&link_8693);
	FreeLink(&link_8697);
	virtual_p_in(in31, &link_8188);
	/* Call conditional control 8161 */
	conditionVar = GetControlValue(&link_8168);
	FreeLink(&link_8168);
	if ((conditionVar) != 0)
		if_8161_true(&link_8188, &link_8170, &link_8516, &link_8169);
	else
		if_8161_false(&link_8188, &link_8170, &link_8169);
	FreeLink(&link_8170);
	FreeLink(&link_8188);
	FreeLink(&link_8516);
	CVShowMatVal(&link_8169, &link_8167, "image", 1); //Show Image (cv::Mat) (8154)
	FreeLink(&link_8169);
	CVwaitVal(&link_8167, &link_8166, 10); //Wait (8147)
	FreeLink(&link_8167);
	FreeLink(&link_8166);
	FreeLink(&result);
	return 0;
}

int if_6051_false(DataQueue* queue_8202, DataLink* in21)
{
	DataLink link_6929 = { NULL, 0, 0 };
	DataLink link_8303 = { NULL, 0, 0 };
	DataLink link_8304 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	queue_read(&link_8303, sdo_read_erase, queue_8202);
	splitter(&link_8303, &link_8304); //splitter 8298
	queue_write(&link_8304, queue_8202);
	FreeLink(&link_8303);
	FreeLink(&link_8304);
	virtual_p_in(in21, &link_6929);
	FreeLink(&link_6929);
	FreeLink(&result);
	return 0;
}

int if_6051_true(DataQueue* queue_8202, DataLink* in21, DataLink* in41, DataQueue* queue_8405, DataQueue* queue_8410, DataLink* in3, DataArray* array_4420, DataLink* in5, DataLink* in6, DataLink* in7, DataLink* in8, DataLink* in9)
{
	DataLink link_8173 = { NULL, 0, 0 };
	DataLink link_8174 = { NULL, 0, 0 };
	DataLink link_8177 = { NULL, 0, 0 };
	DataLink link_8180 = { NULL, 0, 0 };
	DataLink link_8308 = { NULL, 0, 0 };
	DataLink link_8522 = { NULL, 0, 0 };
	DataLink link_8555 = { NULL, 0, 0 };
	DataLink link_8645 = { NULL, 0, 0 };
	DataLink link_8701 = { NULL, 0, 0 };
	DataLink link_8705 = { NULL, 0, 0 };
	DataLink link_8709 = { NULL, 0, 0 };
	DataLink link_8713 = { NULL, 0, 0 };
	DataLink link_8717 = { NULL, 0, 0 };
	DataLink link_8721 = { NULL, 0, 0 };
	DataLink link_8769 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	queue_read(&link_8769, sdo_read_erase, queue_8202);
	splitter(&link_8769, &link_8177); //splitter 8768
	splitter(&link_8769, &link_8180); //splitter 8768
	FreeLink(&link_8769);
	virtual_p_in(in41, &link_8173);
	virtual_p_in(in21, &link_8174);
	virtual_p_in(in3, &link_8645);
	virtual_p_in(in5, &link_8705);
	virtual_p_in(in6, &link_8709);
	virtual_p_in(in7, &link_8713);
	virtual_p_in(in8, &link_8717);
	virtual_p_in(in9, &link_8721);
	/* Call conditional control 7315 */
	conditionVar = GetControlValue(&link_8177);
	FreeLink(&link_8177);
	if ((conditionVar) != 0)
		if_7315_true(&link_8180, &link_8174, &link_8173, queue_8405, queue_8410, &link_8645, array_4420, &link_8705, &link_8709, &link_8713, &link_8717, &link_8721, queue_8202);
	else
		if_7315_false(&link_8180, &link_8174, queue_8202);
	FreeLink(&link_8173);
	FreeLink(&link_8174);
	FreeLink(&link_8180);
	FreeLink(&link_8522);
	FreeLink(&link_8555);
	FreeLink(&link_8645);
	FreeLink(&link_8701);
	FreeLink(&link_8705);
	FreeLink(&link_8709);
	FreeLink(&link_8713);
	FreeLink(&link_8717);
	FreeLink(&link_8721);
	FreeLink(&link_8308);
	FreeLink(&result);
	return 0;
}

int while_28(DataQueue* queue_8202, DataLink* Vin_5236, DataArray* array_4420, DataLink* in91, DataLink* in61, DataLink* in71, DataLink* in51, DataLink* in31, DataLink* in41, DataLink* in21, DataQueue* queue_8405, DataQueue* queue_8410, DataLink* condition, int counter)
{
	DataLink link_62 = { NULL, 0, 0 };
	DataLink link_6075 = { NULL, 0, 0 };
	DataLink link_6077 = { NULL, 0, 0 };
	DataLink link_6097 = { NULL, 0, 0 };
	DataLink link_6913 = { NULL, 0, 0 };
	DataLink link_5264 = { NULL, 0, 0 };
	DataLink link_6937 = { NULL, 0, 0 };
	DataLink link_7031 = { NULL, 0, 0 };
	DataLink link_6101 = { NULL, 0, 0 };
	DataLink link_5262 = { NULL, 0, 0 };
	DataLink link_7311 = { NULL, 0, 0 };
	DataLink link_7313 = { NULL, 0, 0 };
	DataLink link_8287 = { NULL, 0, 0 };
	DataLink link_8511 = { NULL, 0, 0 };
	DataLink link_8528 = { NULL, 0, 0 };
	DataLink link_8558 = { NULL, 0, 0 };
	DataLink link_8673 = { NULL, 0, 0 };
	DataLink link_8725 = { NULL, 0, 0 };
	DataLink link_8729 = { NULL, 0, 0 };
	DataLink link_8733 = { NULL, 0, 0 };
	DataLink link_8737 = { NULL, 0, 0 };
	DataLink link_8741 = { NULL, 0, 0 };
	DataLink link_8745 = { NULL, 0, 0 };
	DataLink link_8775 = { NULL, 0, 0 };
	DataLink link_8778 = { NULL, 0, 0 };
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	/*Assign constant 6099 to 0 */
	link_6101.Size = 4;
	link_6101.Data = (char*)malloc(4);
	SetControlValue(0, &link_6101);
	/*Assign constant 6056 to 10 */
	link_6075.Size = 4;
	link_6075.Data = (char*)malloc(4);
	SetControlValue(10, &link_6075);
	/*Assign constant 8770 to 1 */
	link_8775.Size = 4;
	link_8775.Data = (char*)malloc(4);
	SetControlValue(1, &link_8775);
	/*Assign constant 5260 to 1 */
	link_5262.Size = 4;
	link_5262.Data = (char*)malloc(4);
	SetControlValue(1, &link_5262);
	splitter(&link_8775, &link_8778); //splitter 8772
	virtual_r_out(&link_8778, condition);
	FreeLink(&link_8778);
	FreeLink(&link_8775);
	virtual_p_in(in21, &link_62);
	CVQueryFrameVal(&link_62, &link_6913); //cvQuery Frame (36)
	FreeLink(&link_62);
	virtual_p_in(Vin_5236, &link_5264);
	splitter(&link_5264, &link_6077); //splitter 5245
	splitter(&link_5264, &link_7031); //splitter 5245
	splitter(&link_5264, &link_6937); //splitter 5245
	FreeLink(&link_5264);
	llSum(&link_6937, &link_5262, &link_8511); //+ (5254)
	virtual_r_out(&link_8511, Vin_5236);
	FreeLink(&link_8511);
	FreeLink(&link_6937);
	FreeLink(&link_5262);
	llMod__i(&link_6077, &link_6075, &link_6097); //Mod (6066)
	FreeLink(&link_6075);
	FreeLink(&link_6077);
	llEqual__int(&link_6097, &link_6101, &link_7311); //== (6092)
	FreeLink(&link_6097);
	FreeLink(&link_6101);
	virtual_p_in(in61, &link_8673);
	virtual_p_in(in41, &link_8729);
	virtual_p_in(in51, &link_8733);
	virtual_p_in(in31, &link_8737);
	virtual_p_in(in91, &link_8741);
	virtual_p_in(in71, &link_8745);
	/* Call conditional control 6051 */
	conditionVar = GetControlValue(&link_7311);
	FreeLink(&link_7311);
	if ((conditionVar) != 0)
		if_6051_true(queue_8202, &link_6913, &link_7031, queue_8405, queue_8410, &link_8673, array_4420, &link_8729, &link_8733, &link_8737, &link_8741, &link_8745);
	else
		if_6051_false(queue_8202, &link_6913);
	FreeLink(&link_6913);
	FreeLink(&link_7031);
	FreeLink(&link_7313);
	FreeLink(&link_8528);
	FreeLink(&link_8558);
	FreeLink(&link_8673);
	FreeLink(&link_8725);
	FreeLink(&link_8729);
	FreeLink(&link_8733);
	FreeLink(&link_8737);
	FreeLink(&link_8741);
	FreeLink(&link_8745);
	FreeLink(&link_8287);
	FreeLink(&result);
	return 0;
}

int main()
{
	DataLink link_34 = { NULL, 0, 0 };
	DataLink link_3760 = { NULL, 0, 0 };
	DataLink link_3770 = { NULL, 0, 0 };
	DataLink link_3059 = { NULL, 0, 0 };
	DataLink link_3060 = { NULL, 0, 0 };
	DataLink link_3061 = { NULL, 0, 0 };
	DataLink link_4497 = { NULL, 0, 0 };
	DataLink link_4929 = { NULL, 0, 0 };
	DataLink link_4950 = { NULL, 0, 0 };
	DataLink link_57 = { NULL, 0, 0 };
	DataLink link_5234 = { NULL, 0, 0 };
	DataLink link_7286 = { NULL, 0, 0 };
	DataLink link_8281 = { NULL, 0, 0 };
	DataLink link_8282 = { NULL, 0, 0 };
	DataLink link_5710 = { NULL, 0, 0 };
	DataLink link_4981 = { NULL, 0, 0 };
	DataLink link_8534 = { NULL, 0, 0 };
	DataLink link_8561 = { NULL, 0, 0 };
	DataArray array_4420 = { (char*)malloc(240), (char*)malloc(10), 24, 10, "Array 4420" };
	DataQueue queue_8202 = queue_create(4, 1, "Queue 8202");
	DataQueue queue_8405 = queue_create(8, 1, "Queue 8405");
	DataQueue queue_8410 = queue_create(8, 1, "Queue 8410");
	int N = 0;
	int i = 0;
	int conditionVar = 0;
	int STEP = 0;
	int START = 0;
	DataLink result = { (char*)malloc(sizeof(char*)), 1, 0 };
	for (i = 0; i < 10; i++)
		array_4420.Flags[i] = 0;
	/*Assign constant 7270 to 1 */
	link_8281.Size = 4;
	link_8281.Data = (char*)malloc(4);
	SetControlValue(1, &link_8281);
	/*Assign constant 5231 to 0 */
	link_5234.Size = 4;
	link_5234.Data = (char*)malloc(4);
	SetControlValue(0, &link_5234);
	/*Assign constant 32 to 1 */
	link_34.Size = 4;
	link_34.Data = (char*)malloc(4);
	SetControlValue(1, &link_34);
	CVFaceRecognizerCreateVal(&link_3059, &link_3061, &link_3060, &link_4929, "at2.txt"); //FaceRecognizer Create (4480)
	array_write(&link_4929, &array_4420);
	FreeLink(&link_4929);
	complex_4970(&link_57, &queue_8405, &queue_8410);
	FreeLink(&link_5710);
	FreeLink(&link_4981);
	WiringXSetup(&link_4950); //SetupWiringX (4940)
	CVLoadHaaraCascadeVal(&link_3770, "haarcascade_eye.xml"); //Load Haara cascade (3767)
	CVLoadHaaraCascadeVal(&link_3760, "haarcascade_frontalface_alt2.xml"); //Load Haara cascade (3757)
	splitter(&link_8281, &link_8282); //splitter 8276
	queue_write(&link_8282, &queue_8202);
	FreeLink(&link_8281);
	FreeLink(&link_8282);
	/* Call while 28 */
	N = GetControlValue(&link_34);
	FreeLink(&link_34);
	i = 0;
	while (N != 0)
	{
		while_28(&queue_8202, &link_5234, &array_4420, &link_4950, &link_3760, &link_3770, &link_3059, &link_3060, &link_3061, &link_57, &queue_8405, &queue_8410, &result, i);
		N = GetControlValue(&result);
		i++;
	}
	FreeLink(&link_3760);
	FreeLink(&link_3770);
	FreeLink(&link_3059);
	FreeLink(&link_3060);
	FreeLink(&link_3061);
	FreeLink(&link_4497);
	FreeLink(&link_4950);
	FreeLink(&link_57);
	FreeLink(&link_5234);
	FreeLink(&link_7286);
	FreeLink(&link_8534);
	FreeLink(&link_8561);
	FreeArray(&array_4420);
	queue_free(&queue_8202);
	queue_free(&queue_8405);
	queue_free(&queue_8410);
	FreeLink(&result);
	return 0;
}
