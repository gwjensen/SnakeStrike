//
// file i/o functions
//
#ifndef _FILE_IO_H_
#define _FILE_IO_H_

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <Eigen/Dense>
#include "Triangulation.h"
#include "MultipleViewTriangulation.h"

using namespace Eigen;

// load point data
bool
load_point_data(std::string filename, MatrixXd data[], int CamNum, int PtNum);

// load projection matices
bool 
load_proj_mat(std::string filename, Matrix34d Proj[], int CamNum,
              double f0=MultipleViewTriangulation::Default_f0);

// output ply file
bool
save_data_as_ply(std::string plyfile, Vector3d rp[], int PtNum);

#endif // _FILE_IO_H_
