//
// FILE I/O functions for multiple view triangulation
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "fileio.h"

using namespace std;
using namespace Eigen;

// load point data
bool
load_point_data(string filename, MatrixXd data[], int CamNum, int PtNum)
{
   ifstream ifs;
   string   str;

   ifs.open(filename.c_str());

   if (ifs.fail())
   {
      cerr << "Cannot open data file: " << filename << endl;
      return false;
   }

   for (int pt = 0; pt < PtNum; pt++)
   {
      // separate csv data at each line
      if (!getline(ifs, str))
      {
         cerr << "We need more point data!" << endl;
         return false;
      }

      string token;
      istringstream stream(str);

      for (int cm = 0; cm < CamNum; cm++)
      {
         Vector2d pos;
         
         // x
         if (!getline(stream, token, ',')) return false;
         pos(0) = atof(token.c_str());
         // y
         if (!getline(stream, token, ',')) return false;
         pos(1) = atof(token.c_str());

         data[pt].col(cm) = pos;
      }
   }

   return true;
}

// load projection matices
bool
load_proj_mat(string filename, Matrix34d Prj[], int CamNum, double f0)
{
   ifstream ifs;
   string str;

   // file open
   ifs.open(filename.c_str());
   if (ifs.fail())
   {
      cerr << "Cannot open file: " << filename << endl;
      return false;
   }

   for (int cm = 0; cm < CamNum; cm++)
   {
      double p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34;
      Matrix34d Pt;
      Matrix3d Qm, QQt_i, C, C_i, R, K;
      Vector3d Qv, tr;
      
      if (!getline(ifs, str))  return false;
      sscanf(str.data(), "%lf,%lf,%lf,%lf", &p11, &p12, &p13, &p14);
      if (!getline(ifs, str))  return false;
      sscanf(str.data(), "%lf,%lf,%lf,%lf", &p21, &p22, &p23, &p24);
      if (!getline(ifs, str))  return false;
      sscanf(str.data(), "%lf,%lf,%lf,%lf", &p31, &p32, &p33, &p34);

      Pt << p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34;

      // convert to Projection matrix which includes f0
      Qm = Pt.block<3,3>(0,0);
      Qv = Pt.block<3,1>(0,3);
      
      if(Qm.determinant() < 0){
         Qm *= -1.0;
         Qv *= -1.0;
      }
      tr = -Qm.inverse() * Qv;
		
      QQt_i = (Qm * Qm.transpose()).inverse();
      LLT<Matrix3d> CholD(QQt_i);
      C = CholD.matrixL().transpose();
      C_i = C.inverse();
      
      R = (C * Qm).transpose();
      K = C_i;
      K /= C_i(2,2);
      K(2,2) = f0;
      Prj[cm] << K*R.transpose(), -K*R.transpose()*tr; 
   }

   return true;
}

// output ply file
bool
save_data_as_ply(string plyfile, Vector3d rp[], int PtNum)
{
   ofstream ofs;

   // open
   ofs.open(plyfile.c_str());
   if (ofs.fail())
   {
      cerr << "Cannot open file: " << plyfile << endl;
      return false;
   }

   // headers
   ofs << "ply" << endl;
   ofs << "format ascii 1.0" << endl;
   ofs << "element vertex " << PtNum << endl;
   ofs << "property float x" << endl;
   ofs << "property float y" << endl;
   ofs << "property float z" << endl;
   ofs << "end_header" << endl;

   // point data
   for (int p = 0; p < PtNum; p++)
      ofs << rp[p].transpose() << endl;

   // close
   ofs.close();

   return true;
}
