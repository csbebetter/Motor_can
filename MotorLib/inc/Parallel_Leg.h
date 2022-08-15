#ifndef __PARALLEL_LEG__
#define __PARALLEL_LEG__

//#include "math.h"
//#include "stm32f4xx.h"
//#include "arm_math.h"
//#include <stdbool.h>
//#include "os.h"
//#include "motor_control.h"
#include "system_config.h"

float MotorStandingPos[8] = {84.1216634811075,65.4375097818913,84.1216634811075,65.4375097818913,84.1216634811075,65.4375097818913,84.1216634811075,65.4375097818913};
float StairFirstPos[8]={95.022,74.248,95.022,74.248,40.771,31.871,40.771,31.871};
float StairSecondPos[8]={40.771,31.871,40.771,31.871,107.05,84.548,107.05,84.548};
float MotorUpstair_Fix[8][Trot_LENGTH];
//����
float SlopeDownStanding[8]={67.86,52.755,67.86,52.755,120.22,96.974,120.22,96.974};
//����
float SlopeUpStanding[8]={120.22,96.974,120.22,96.974,67.86,52.755,67.86,52.755};
//float MotorTrot[8][Trot_LENGTH] = {
//{	117.241483,123.266844,138.602805,155.030489,143.043124,109.616574,88.919468,82.745264,82.743713,83.815466,90.102710,101.350003,111.336648,115.963309,117.115657,117.241483	},
//{	61.164113,66.315100,85.957604,124.422881,137.317568,115.776066,101.066345,95.663570,95.661869,95.471757,93.989442,89.036742,79.124601,68.246432,62.216795,61.164126	},
//{	82.743713,83.815466,90.102710,101.350003,111.336648,115.963309,117.115657,117.241483,117.241483,123.266844,138.602805,155.030489,143.043124,109.616574,88.919468,82.745264	},
//{	95.661869,95.471757,93.989442,89.036742,79.124601,68.246432,62.216795,61.164126,61.164113,66.315100,85.957604,124.422881,137.317568,115.776066,101.066345,95.663570	},
//{	117.241483,123.266844,138.602805,155.030489,143.043124,109.616574,88.919468,82.745264,82.743713,83.815466,90.102710,101.350003,111.336648,115.963309,117.115657,117.241483	},
//{	61.164113,66.315100,85.957604,124.422881,137.317568,115.776066,101.066345,95.663570,95.661869,95.471757,93.989442,89.036742,79.124601,68.246432,62.216795,61.164126	},
//{	82.743713,83.815466,90.102710,101.350003,111.336648,115.963309,117.115657,117.241483,117.241483,123.266844,138.602805,155.030489,143.043124,109.616574,88.919468,82.745264	},
//{	95.661869,95.471757,93.989442,89.036742,79.124601,68.246432,62.216795,61.164126,61.164113,66.315100,85.957604,124.422881,137.317568,115.776066,101.066345,95.663570	}
//};

float MotorTrot[8][Trot_LENGTH];
float MotorWalk[8][Walk_LENGTH];
float MotorSlope[8][Trot_LENGTH];
float Motor_Fh_Slope[8][Fh_Slope_LENGTH]={
{	89.401,98.352,111.24,125.28,136.39,137.53,124.49,107.01,94.67,90.458,92.152,93.292,93.907,94.035,93.712,92.969,91.836,90.332,88.469,86.252	}	,
{	42.541,47.284,56.561,71.504,91.918,109.52,111.81,105.18,98.872,96.315,91.492,86.366,81.01,75.493,69.871,64.188,58.477,52.753,47.023,41.277	}	,
{	91.319,92.901,93.792,94.045,93.712,92.84,91.47,89.629,87.331,84.573,87.71,96.734,109.87,124.42,136.39,138.3,124.61,106.09,93.294,89.007	}	,
{	94.107,88.448,82.457,76.236,69.871,63.428,56.951,50.462,43.961,37.429,38.694,43.601,53.375,69.366,91.918,112.53,115.76,108.68,102.01,99.325	}	,
{	44.378,54.83,67.191,77.406,81.783,77.963,67.451,55.142,45.77,42.333,44.071,45.428,46.394,46.957,47.101,46.805,46.035,44.742,42.844,40.2	}	,
{	24.523,31.504,41.422,53.015,64.35,72.077,73.931,71.18,67.124,65.173,61.973,58.533,54.86,50.958,46.83,42.468,37.855,32.958,27.717,22.022	}	,
{	43.19,44.931,46.175,46.906,47.101,46.73,45.744,44.061,41.54,37.898,42.376,53.464,66.44,77.152,81.783,77.774,66.736,53.978,44.422,40.968	}	,
{	63.71,59.938,55.862,51.492,46.83,41.868,36.579,30.907,24.746,17.875,20.621,28.223,38.983,51.681,64.35,73.269,75.773,73.21,69.145,67.167	}	,
};
float MotorUpSlope[8][Trot_LENGTH];
float MotorDownSlope[8][Trot_LENGTH];

float MotorTITA[8][Trot_LENGTH];
float MotorFlyJump[8][FlyJump_LENGTH] = {
{	84.123,97.494,111.96,128.21,147.52,174.2,174.2,174.2,107.82,50.471,50.471,174.2	}	,
{	65.44,71.492,78.195,86.508,98.635,122.73,122.73,122.73,36.055,-15.901,-15.901,122.73	}	,
{	84.123,97.494,111.96,128.21,147.52,174.2,174.2,174.2,107.82,50.471,50.471,174.2	}	,
{	65.44,71.492,78.195,86.508,98.635,122.73,122.73,122.73,36.055,-15.901,-15.901,122.73	}	,
{	84.123,97.494,111.96,128.21,147.52,174.2,174.2,174.2,107.82,50.471,50.471,174.2	}	,
{	65.44,71.492,78.195,86.508,98.635,122.73,122.73,122.73,36.055,-15.901,-15.901,122.73	}	,
{	84.123,97.494,111.96,128.21,147.52,174.2,174.2,174.2,107.82,50.471,50.471,174.2	}	,
{	65.44,71.492,78.195,86.508,98.635,122.73,122.73,122.73,36.055,-15.901,-15.901,122.73	}
};
float MotorJump[8][Jump_LENGTH] = {
{	84.123,97.494,111.96,128.21,147.52,174.2,174.2,174.2,107.82,50.471,50.471,174.2,151.36,119.1,88.23,60.292,34.175,37.015,39.805,42.545,45.232,47.866,50.442,52.959,55.412,57.799,60.116,62.359,64.524,66.607,68.605,70.514,72.331,74.053,75.676,77.198,78.618,79.933,81.141,82.243,83.237,84.123	}	,
{	65.44,71.492,78.195,86.508,98.635,122.73,122.73,122.73,36.055,-15.901,-15.901,122.73,125.96,115.8,106.74,98.688,89.819,89.839,89.78,89.64,89.42,89.119,88.735,88.269,87.719,87.085,86.366,85.56,84.668,83.689,82.622,81.469,80.229,78.903,77.493,76,74.426,72.774,71.046,69.245,67.376,65.44	}	,
{	84.123,97.494,111.96,128.21,147.52,174.2,174.2,174.2,107.82,50.471,50.471,174.2,151.36,119.1,88.23,60.292,34.175,37.015,39.805,42.545,45.232,47.866,50.442,52.959,55.412,57.799,60.116,62.359,64.524,66.607,68.605,70.514,72.331,74.053,75.676,77.198,78.618,79.933,81.141,82.243,83.237,84.123	}	,
{	65.44,71.492,78.195,86.508,98.635,122.73,122.73,122.73,36.055,-15.901,-15.901,122.73,125.96,115.8,106.74,98.688,89.819,89.839,89.78,89.64,89.42,89.119,88.735,88.269,87.719,87.085,86.366,85.56,84.668,83.689,82.622,81.469,80.229,78.903,77.493,76,74.426,72.774,71.046,69.245,67.376,65.44	}	,
{	84.123,97.494,111.96,128.21,147.52,174.2,174.2,174.2,107.82,50.471,50.471,174.2,147.52,128.21,111.96,97.494,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123	}	,
{	65.44,71.492,78.195,86.508,98.635,122.73,122.73,122.73,36.055,-15.901,-15.901,122.73,98.635,86.508,78.195,71.492,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44	}	,
{	84.123,97.494,111.96,128.21,147.52,174.2,174.2,174.2,107.82,50.471,50.471,174.2,147.52,128.21,111.96,97.494,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123	}	,
{	65.44,71.492,78.195,86.508,98.635,122.73,122.73,122.73,36.055,-15.901,-15.901,122.73,98.635,86.508,78.195,71.492,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44	}	,
};
float MotorLittleJump[8][LittleJump_LENGTH] = {
{	84.123,92.202,100.58,109.37,118.69,128.7,128.7,128.7,96.938,66.25,66.25,128.7,115.92,102.56,89.089,75.707,62.445,62.456,62.532,62.736,63.118,63.718,64.558,65.644,66.959,68.471,70.133,71.887,73.668,75.414,77.069,78.585,79.927,81.074,82.019,82.763,83.317,83.702,83.943,84.069,84.116,84.123	}	,
{	65.44,68.849,72.358,76.091,80.218,84.994,84.994,84.994,46.681,16.285,16.285,84.994,85.131,84.105,82.286,79.825,76.729,76.726,76.708,76.659,76.566,76.414,76.19,75.882,75.477,74.967,74.347,73.619,72.794,71.891,70.939,69.972,69.032,68.156,67.38,66.73,66.22,65.854,65.619,65.494,65.447,65.44	}	,
{	84.123,92.202,100.58,109.37,118.69,128.7,128.7,128.7,96.938,66.25,66.25,128.7,115.92,102.56,89.089,75.707,62.445,62.456,62.532,62.736,63.118,63.718,64.558,65.644,66.959,68.471,70.133,71.887,73.668,75.414,77.069,78.585,79.927,81.074,82.019,82.763,83.317,83.702,83.943,84.069,84.116,84.123	}	,
{	65.44,68.849,72.358,76.091,80.218,84.994,84.994,84.994,46.681,16.285,16.285,84.994,85.131,84.105,82.286,79.825,76.729,76.726,76.708,76.659,76.566,76.414,76.19,75.882,75.477,74.967,74.347,73.619,72.794,71.891,70.939,69.972,69.032,68.156,67.38,66.73,66.22,65.854,65.619,65.494,65.447,65.44	}	,
{	84.123,92.202,100.58,109.37,118.69,128.7,128.7,128.7,96.938,66.25,66.25,128.7,118.69,109.37,100.58,92.202,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123	}	,
{	65.44,68.849,72.358,76.091,80.218,84.994,84.994,84.994,46.681,16.285,16.285,84.994,80.218,76.091,72.358,68.849,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44	}	,
{	84.123,92.202,100.58,109.37,118.69,128.7,128.7,128.7,96.938,66.25,66.25,128.7,118.69,109.37,100.58,92.202,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123,84.123	}	,
{	65.44,68.849,72.358,76.091,80.218,84.994,84.994,84.994,46.681,16.285,16.285,84.994,80.218,76.091,72.358,68.849,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44,65.44	}	,
};

float MotorJump_First[8][Jump_LENGTH_FIRST] = {
{	84.123,97.494,111.96,128.21,147.52,174.2,174.2,174.2,105.83,59.005,59.005,91.105,121.78,174.2,149.6,120.39,92.759,67.532,43.933,43.959,44.142,44.628,45.542,46.979,48.996,51.605,54.776,58.434,62.464,66.723,71.045,75.263,79.223,82.8,85.908,88.506,90.593,92.196,93.367,94.164,94.656,94.913,95.008,95.022	}	,
{	65.44,71.492,78.195,86.508,98.635,122.73,122.73,122.73,46.855,9.0108,9.0108,34.709,60.803,122.73,119.61,110.24,102.17,94.924,87.168,87.17,87.185,87.221,87.286,87.373,87.467,87.538,87.545,87.436,87.161,86.674,85.944,84.963,83.753,82.366,80.881,79.393,77.998,76.781,75.799,75.077,74.609,74.357,74.262,74.248	}	,
{	84.123,97.494,111.96,128.21,147.52,174.2,174.2,174.2,105.83,59.005,59.005,91.105,121.78,174.2,149.6,120.39,92.759,67.532,43.933,43.959,44.142,44.628,45.542,46.979,48.996,51.605,54.776,58.434,62.464,66.723,71.045,75.263,79.223,82.8,85.908,88.506,90.593,92.196,93.367,94.164,94.656,94.913,95.008,95.022	}	,
{	65.44,71.492,78.195,86.508,98.635,122.73,122.73,122.73,46.855,9.0108,9.0108,34.709,60.803,122.73,119.61,110.24,102.17,94.924,87.168,87.17,87.185,87.221,87.286,87.373,87.467,87.538,87.545,87.436,87.161,86.674,85.944,84.963,83.753,82.366,80.881,79.393,77.998,76.781,75.799,75.077,74.609,74.357,74.262,74.248	}	,
{	84.123,97.494,111.96,128.21,147.52,174.2,174.2,174.2,105.83,59.005,59.005,91.105,121.78,174.2,132.72,106.24,84.163,63.357,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771	}	,
{	65.44,71.492,78.195,86.508,98.635,122.73,122.73,122.73,46.855,9.0108,9.0108,34.709,60.803,122.73,84.585,68.781,56.997,45.548,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871	}	,
{	84.123,97.494,111.96,128.21,147.52,174.2,174.2,174.2,105.83,59.005,59.005,91.105,121.78,174.2,132.72,106.24,84.163,63.357,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771,40.771	}	,
{	65.44,71.492,78.195,86.508,98.635,122.73,122.73,122.73,46.855,9.0108,9.0108,34.709,60.803,122.73,84.585,68.781,56.997,45.548,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871,31.871	}	,
};
float MotorJump_Second[8][Jump_LENGTH_SECOND] = {
{	95.022,106.3,118.79,133.22,151.31,184.4,184.4,184.4,111.3,72.261,72.261,98.308,125.81,184.4,171.44,163.53,154.58,146.06,140.36	}	,
{	74.248,80.372,87.694,97.408,112.64,154.1,154.1,154.1,55.48,19.914,19.914,43.078,70.493,154.1,135.4,131.64,130.19,125.71,119.62	}	,
{	95.022,106.3,118.79,133.22,151.31,184.4,184.4,184.4,111.3,72.261,72.261,98.308,125.81,184.4,171.44,163.53,154.58,146.06,140.36	}	,
{	74.248,80.372,87.694,97.408,112.64,154.1,154.1,154.1,55.48,19.914,19.914,43.078,70.493,154.1,135.4,131.64,130.19,125.71,119.62	}	,
{	40.771,54.621,67.258,79.465,91.666,104.15,104.15,104.15,78.287,39.911,39.911,68.555,87.157,104.15,104.15,104.15,104.15,104.15,104.15	}	,
{	31.871,38.998,44.731,49.655,54.076,58.216,58.216,58.216,29.279,-6.4581,-6.4581,19.565,38.628,58.216,58.216,58.216,58.216,58.216,58.216	}	,
{	40.771,54.621,67.258,79.465,91.666,104.15,104.15,104.15,78.287,39.911,39.911,68.555,87.157,104.15,104.15,104.15,104.15,104.15,104.15	}	,
{	31.871,38.998,44.731,49.655,54.076,58.216,58.216,58.216,29.279,-6.4581,-6.4581,19.565,38.628,58.216,58.216,58.216,58.216,58.216,58.216	}
};
float MotorJump_Third[8][Jump_LENGTH_THIRD] = {
{	95.022,106.3,118.79,133.22,151.31,184.4,184.4,184.4,111.3,72.261,72.261,98.308,125.81,184.4,93.288,40.771	}	,
{	74.248,80.372,87.694,97.408,112.64,154.1,154.1,154.1,55.48,19.914,19.914,43.078,70.493,154.1,65.184,31.871	}	,
{	95.022,106.3,118.79,133.22,151.31,184.4,184.4,184.4,111.3,72.261,72.261,98.308,125.81,184.4,93.288,40.771	}	,
{	74.248,80.372,87.694,97.408,112.64,154.1,154.1,154.1,55.48,19.914,19.914,43.078,70.493,154.1,65.184,31.871	}	,
{	40.771,52.339,62.95,73.084,82.998,92.843,92.843,92.843,72.655,39.911,39.911,64.483,79.88,92.843,103.55,113.77	}	,
{	31.871,37.281,41.598,45.181,48.211,50.793,50.793,50.793,25.865,-6.4581,-6.4581,17.048,34.204,50.793,69.122,90.71	}	,
{	40.771,52.339,62.95,73.084,82.998,92.843,92.843,92.843,72.655,39.911,39.911,64.483,79.88,92.843,103.55,113.77	}	,
{	31.871,37.281,41.598,45.181,48.211,50.793,50.793,50.793,25.865,-6.4581,-6.4581,17.048,34.204,50.793,69.122,90.71	}	,
};
float MotorJump_Fourth[8][Jump_LENGTH_FOURTH] = {
{	40.771,50.003,58.55,66.723,74.707,82.625,82.625,82.625,61.764,20.787,20.787,53.132,69.273,82.625,62.67,40.771	}	,
{	31.871,36.674,40.753,44.343,47.581,50.555,50.555,50.555,27.408,-9.8945,-9.8945,18.796,35.318,50.555,42.599,31.871	}	,
{	40.771,50.003,58.55,66.723,74.707,82.625,82.625,82.625,61.764,20.787,20.787,53.132,69.273,82.625,62.67,40.771	}	,
{	31.871,36.674,40.753,44.343,47.581,50.555,50.555,50.555,27.408,-9.8945,-9.8945,18.796,35.318,50.555,42.599,31.871	}	,
{	95.022,104.43,114.53,125.61,138.13,153.05,153.05,153.05,117.05,91.995,91.995,108.33,126.65,153.05,132.07,113.77	}	,
{	74.248,78.787,83.911,90.053,98.018,109.61,109.61,109.61,63.006,37.299,37.299,53.642,74.046,109.61,98.4,90.71	}	,
{	95.022,104.43,114.53,125.61,138.13,153.05,153.05,153.05,117.05,91.995,91.995,108.33,126.65,153.05,132.07,113.77	}	,
{	74.248,78.787,83.911,90.053,98.018,109.61,109.61,109.61,63.006,37.299,37.299,53.642,74.046,109.61,98.4,90.71	}	,
};
float MotorJump_Fifth[8][Jump_LENGTH_FIFTH] = {
{	40.771,54.621,67.258,79.465,91.666,104.15,104.15,104.15,78.287,39.911,39.911,68.555,87.157,104.15,104.15,104.15	}	,
{	31.871,38.998,44.731,49.655,54.076,58.216,58.216,58.216,29.279,-6.4581,-6.4581,19.565,38.628,58.216,58.216,58.216	}	,
{	40.771,54.621,67.258,79.465,91.666,104.15,104.15,104.15,78.287,39.911,39.911,68.555,87.157,104.15,104.15,104.15	}	,
{	31.871,38.998,44.731,49.655,54.076,58.216,58.216,58.216,29.279,-6.4581,-6.4581,19.565,38.628,58.216,58.216,58.216	}	,
{	95.022,106.3,118.79,133.22,151.31,184.4,184.4,184.4,111.3,72.261,72.261,98.308,125.81,184.4,159.25,140.36	}	,
{	74.248,80.372,87.694,97.408,112.64,154.1,154.1,154.1,55.48,19.914,19.914,43.078,70.493,154.1,131.19,119.62	}	,
{	95.022,106.3,118.79,133.22,151.31,184.4,184.4,184.4,111.3,72.261,72.261,98.308,125.81,184.4,159.25,140.36	}	,
{	74.248,80.372,87.694,97.408,112.64,154.1,154.1,154.1,55.48,19.914,19.914,43.078,70.493,154.1,131.19,119.62	}
};
float MotorUpstair[8][Upstair_LENGTH]={
{84.122,84.122,81.498,78.902,76.324,73.757,71.19,68.623,66.051,63.455,60.837,58.184,54.406,50.01,45.013,39.428,33.252,47.104,59.805,72.594,86.763,105.18,106.07,104.25,93.123,72.038,47.43,25.017,7.3556,-4.4776,-10.19,-11.333,-11.333,-11.333,-11.333,-11.333,-11.333,-11.333,-11.333,-11.333,-11.333,-11.333,-11.333,-11.333,-11.333,-11.333,-11.333},
{65.438,65.438,63.369,61.329,59.313,57.319,55.329,53.345,51.358,49.36,47.344,45.301,48.505,51.187,53.321,54.876,55.816,69.248,82.431,96.795,114.42,140.78,143.21,147.45,146.91,139.68,129.86,119.62,109.84,101.88,97.174,95.89,95.89,95.89,95.89,95.89,95.89,95.89,95.89,95.89,95.89,95.89,95.89,95.89,95.89,95.89,95.89},
{84.122,84.122,81.498,78.902,76.324,73.757,71.19,68.623,66.051,63.455,60.837,58.184,54.406,50.01,45.013,39.428,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,47.104,59.805,72.594,86.763,105.18,106.07,104.25,93.123,72.038,47.43,25.017,7.3556,-4.4776,-10.19,-11.333},
{65.438,65.438,63.369,61.329,59.313,57.319,55.329,53.345,51.358,49.36,47.344,45.301,48.505,51.187,53.321,54.876,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,69.248,82.431,96.795,114.42,140.78,143.21,147.45,146.91,139.68,129.86,119.62,109.84,101.88,97.174,95.89},
{84.122,84.122,81.498,78.902,76.324,73.757,71.19,68.623,66.051,63.455,60.837,58.184,54.406,50.01,45.013,39.428,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252},	
{65.438,65.438,63.369,61.329,59.313,57.319,55.329,53.345,51.358,49.36,47.344,45.301,48.505,51.187,53.321,54.876,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816},
{84.122,84.122,81.498,78.902,76.324,73.757,71.19,68.623,66.051,63.455,60.837,58.184,54.406,50.01,45.013,39.428,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252,33.252},
{65.438,65.438,63.369,61.329,59.313,57.319,55.329,53.345,51.358,49.36,47.344,45.301,48.505,51.187,53.321,54.876,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816,55.816},
};
float MotorLeftRotate[8][LeftRotate_LENGTH]={
{	84.123,83.294,82.435,81.548,80.631,79.685,78.712,77.711,76.684,75.629,74.549,77.923,87.398,101.02,115.25,123.57,120.44,109.39,96.983,87.628,84.123	}	,
{	65.44,66.242,67.02,67.774,68.503,69.207,69.887,70.541,71.169,71.773,72.351,75.282,83.314,94.746,106.54,111.58,103.71,89.418,76.673,68.331,65.44	}	,
{	84.123,87.628,96.983,109.39,120.44,123.57,115.25,101.02,87.398,77.923,74.549,75.629,76.684,77.711,78.712,79.685,80.631,81.548,82.435,83.294,84.123	}	,
{	65.44,68.331,76.673,89.418,103.71,111.58,106.54,94.746,83.314,75.282,72.351,71.773,71.169,70.541,69.887,69.207,68.503,67.774,67.02,66.242,65.44	}	,
{	84.123,84.922,85.691,86.429,87.137,87.813,88.458,89.072,89.654,90.205,90.724,94.38,104.22,117.47,129.74,134.37,126.87,112.28,97.849,87.736,84.123	}	,
{	65.44,64.614,63.764,62.892,61.997,61.079,60.14,59.18,58.199,57.197,56.176,58.824,66.489,78.302,92.044,100.78,97.274,86.528,75.806,68.223,65.44	}	,
{	84.123,87.736,97.849,112.28,126.87,134.37,129.74,117.47,104.22,94.38,90.724,90.205,89.654,89.072,88.458,87.813,87.137,86.429,85.691,84.922,84.123	}	,
{	65.44,68.223,75.806,86.528,97.274,100.78,92.044,78.302,66.489,58.824,56.176,57.197,58.199,59.18,60.14,61.079,61.997,62.892,63.764,64.614,65.44	}	,
};
//float MotorRightRotate[8][RightRotate_LENGTH]={
//{84.122,87.789,98.268,113.57,129.22,137.16,132.7,120.74,107.68,97.85,94.171,93.736,93.174,92.481,91.668,90.722,89.656,88.459,87.135,85.692,84.122},
//{65.438,68.171,75.361,84.947,93.203,92.733,80.93,66.285,54.644,47.409,45,47.361,49.665,51.904,54.076,56.176,58.201,60.138,61.994,63.764,65.438},
//{84.122,82.437,80.632,78.713,76.685,74.548,72.313,69.981,67.552,65.042,62.447,65.586,74.731,88.499,104.16,115.74,116.4,107.79,96.538,87.577,84.122},
//{65.438,67.019,68.503,69.884,71.167,72.353,73.43,74.404,75.281,76.054,76.731,79.67,87.6,98.52,109.46,114.15,106.03,90.722,77.097,68.383,65.438},
//{84.122,87.577,96.538,107.79,116.4,115.74,104.16,88.499,74.731,65.586,62.447,65.042,67.552,69.981,72.313,74.548,76.685,78.713,80.632,82.437,84.122},
//{65.438,68.383,77.097,90.722,106.03,114.15,109.46,98.52,87.6,79.67,76.731,76.054,75.281,74.404,73.43,72.353,71.167,69.884,68.503,67.019,65.438},
//{84.122,85.692,87.135,88.459,89.656,90.722,91.668,92.481,93.174,93.736,94.171,97.85,107.68,120.74,132.7,137.16,129.22,113.57,98.268,87.789,84.122},
//{65.438,63.764,61.994,60.138,58.201,56.176,54.076,51.904,49.665,47.361,45,47.409,54.644,66.285,80.93,92.733,93.203,84.947,75.361,68.171,65.438},
//};
float MotorRightRotate[8][RightRotate_LENGTH]={
{	84.123,87.628,96.983,109.39,120.44,123.57,115.25,101.02,87.398,77.923,74.549,75.629,76.684,77.711,78.712,79.685,80.631,81.548,82.435,83.294,84.123	}	,
{	65.44,68.331,76.673,89.418,103.71,111.58,106.54,94.746,83.314,75.282,72.351,71.773,71.169,70.541,69.887,69.207,68.503,67.774,67.02,66.242,65.44	}	,
{	84.123,83.294,82.435,81.548,80.631,79.685,78.712,77.711,76.684,75.629,74.549,77.923,87.398,101.02,115.25,123.57,120.44,109.39,96.983,87.628,84.123	}	,
{	65.44,66.242,67.02,67.774,68.503,69.207,69.887,70.541,71.169,71.773,72.351,75.282,83.314,94.746,106.54,111.58,103.71,89.418,76.673,68.331,65.44	}	,
{	84.123,87.736,97.849,112.28,126.87,134.37,129.74,117.47,104.22,94.38,90.724,90.205,89.654,89.072,88.458,87.813,87.137,86.429,85.691,84.922,84.123	}	,
{	65.44,68.223,75.806,86.528,97.274,100.78,92.044,78.302,66.489,58.824,56.176,57.197,58.199,59.18,60.14,61.079,61.997,62.892,63.764,64.614,65.44	}	,
{	84.123,84.922,85.691,86.429,87.137,87.813,88.458,89.072,89.654,90.205,90.724,94.38,104.22,117.47,129.74,134.37,126.87,112.28,97.849,87.736,84.123	}	,
{	65.44,64.614,63.764,62.892,61.997,61.079,60.14,59.18,58.199,57.197,56.176,58.824,66.489,78.302,92.044,100.78,97.274,86.528,75.806,68.223,65.44	}	,
};


float MotorStairLeftRotate[8][StairLeftRotate_LENGTH]={
{	95.022,94.355,93.669,92.963,92.239,91.496,90.734,89.954,89.157,88.342,87.509,90.142,97.606,108.33,119.22,125.29,122.96,114.58,104.97,97.719,95.022	}	,
{	74.248,74.898,75.53,76.147,76.747,77.33,77.896,78.446,78.979,79.495,79.994,82.318,88.764,97.957,107.09,110.66,104.75,93.652,83.351,76.573,74.248	}	,
{	95.022,97.719,104.97,114.58,122.96,125.29,119.22,108.33,97.606,90.142,87.509,88.342,89.157,89.954,90.734,91.496,92.239,92.963,93.669,94.355,95.022	}	,
{	74.248,76.573,83.351,93.652,104.75,110.66,107.09,97.957,88.764,82.318,79.994,79.495,78.979,78.446,77.896,77.33,76.747,76.147,75.53,74.898,74.248	}	,
{	40.771,41.238,41.689,42.124,42.542,42.944,43.328,43.697,44.048,44.382,44.699,47.806,55.262,63.549,69.575,71.196,67.761,60.394,51.435,43.837,40.771	}	,
{	31.871,31.389,30.895,30.388,29.869,29.337,28.793,28.237,27.669,27.088,26.495,28.842,34.664,41.571,47.411,50.445,49.729,45.629,39.677,34.188,31.871	}	,
{	40.771,43.837,51.435,60.394,67.761,71.196,69.575,63.549,55.262,47.806,44.699,44.382,44.048,43.697,43.328,42.944,42.542,42.124,41.689,41.238,40.771	}	,
{	31.871,34.188,39.677,45.629,49.729,50.445,47.411,41.571,34.664,28.842,26.495,27.088,27.669,28.237,28.793,29.337,29.869,30.388,30.895,31.389,31.871	}	,
};

float MotorStairRightRotate[8][StairRightRotate_LENGTH]={
{	95.022,97.719,104.97,114.58,122.96,125.29,119.22,108.33,97.606,90.142,87.509,88.342,89.157,89.954,90.734,91.496,92.239,92.963,93.669,94.355,95.022	}	,
{	74.248,76.573,83.351,93.652,104.75,110.66,107.09,97.957,88.764,82.318,79.994,79.495,78.979,78.446,77.896,77.33,76.747,76.147,75.53,74.898,74.248	}	,
{	95.022,94.355,93.669,92.963,92.239,91.496,90.734,89.954,89.157,88.342,87.509,90.142,97.606,108.33,119.22,125.29,122.96,114.58,104.97,97.719,95.022	}	,
{	74.248,74.898,75.53,76.147,76.747,77.33,77.896,78.446,78.979,79.495,79.994,82.318,88.764,97.957,107.09,110.66,104.75,93.652,83.351,76.573,74.248	}	,
{	40.771,43.837,51.435,60.394,67.761,71.196,69.575,63.549,55.262,47.806,44.699,44.382,44.048,43.697,43.328,42.944,42.542,42.124,41.689,41.238,40.771	}	,
{	31.871,34.188,39.677,45.629,49.729,50.445,47.411,41.571,34.664,28.842,26.495,27.088,27.669,28.237,28.793,29.337,29.869,30.388,30.895,31.389,31.871	}	,
{	40.771,41.238,41.689,42.124,42.542,42.944,43.328,43.697,44.048,44.382,44.699,47.806,55.262,63.549,69.575,71.196,67.761,60.394,51.435,43.837,40.771	}	,
{	31.871,31.389,30.895,30.388,29.869,29.337,28.793,28.237,27.669,27.088,26.495,28.842,34.664,41.571,47.411,50.445,49.729,45.629,39.677,34.188,31.871	}	,
};
/**************************Walk*****************************/
//�Ӳ��濴������ֱ�Ϊ��A E���غϣ���ǰ�ؽ�ΪB���׹ؽ�ΪC����ؽ�ΪD�����ΪF
#define half_length	200		//ǰ���㳤�ȵ�һ��
#define half_width	200		//��������ȵ�һ��

#define LENGTH			Walk_LENGTH / 6		//�����ߵ���
#define	Support1		8		//֧��1����
#define	Support13		12		//֧��1���� + ֧��3����
#define	Support123	20		//֧��1���� + ֧��2���� + ֧��3����
#define TmW					600		//����������
#define SW					200		//����
#define HW					100		//����
#define HBW			    0		//֧�Ų���
#define YM1_ADD		  50		//ZMP1������
#define YM2_ADD		  100		//ZMP2������
#define L3W					210		//��ʼAC
#define HJ					30		//֧����һ���½��߶�

/**************************Trot*****************************/
//�Ӳ��濴������ֱ�Ϊ��A E���غϣ���ǰ�ؽ�ΪB���׹ؽ�ΪC����ؽ�ΪD�����ΪF
//#define LENGTH		8
#define Tm			400
#define S				150		//����
#define H       75
//#define H_Front				75		//����50
//#define H_Back        75    
#define HB			20		//֧�Ų���
#define L1			105		//����
#define L2 	    200		//С��
#define L3      180
//#define L3_Front			180		//��ʼAC160
//#define L3_Back       180
#define L4			90.2		//�㳤

float S0 = S;
float S1 = S;
float S2 = S;
float S3 = S;
float walk_delay = Tm / (Trot_LENGTH / 2 - 1);
float FOOTPOINT_X[4][Trot_LENGTH];
float FOOTPOINT_Y[4][Trot_LENGTH];
float MOTOR_ANGLE[8][Trot_LENGTH];

float YM1, YM2;							//ZMP����λ����
float walk_delayW = TmW/(Walk_LENGTH / 6 - 1);
float FOOTPOINT_XW[4][Walk_LENGTH];		//�������X
float FOOTPOINT_YW[4][Walk_LENGTH];		//�������Y
float MOTOR_ANGLEW[8][Walk_LENGTH];		//����ǣ���ǰ����0��1����ǰ����2��3���Һ���4��5�������6��7��

#endif