#ifndef DARKNETDETECTOR_H
#define DARKNETDETECTOR_H

extern "C" {
#include "/home/turtle1/Libs/darknet_AlexeyAB/src/utils.h"
#include "/home/turtle1/Libs/darknet_AlexeyAB/src/parser.h"
}
void detector_init(char *cfgfile, char *weightfile);

float* test_detector_file(char *filename, float thresh, float hier_thresh, int* num_output_class);

// *data is a image date buffer in which image data was stored as [bgrbgrbgr...bgr] by rows
float* test_detector_uchar(unsigned char *data, int w, int h, int c, float thresh, float hier_thresh, int* num_output_class);

void detector_uninit();

double what_is_the_time_now();
#endif // DARKNETDETECTOR_H
