//
// Multiple View Triangulation
//

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <Eigen/Dense>
#include "Triangulation.h"
#include "MultipleViewTriangulation.h"

#include "fileio.h"

using namespace Eigen;

// main
int
main()
{
   // Dinosaur data from Oxford University
   std::string pointfile = "points.dat";
   std::string projmatfile = "projmat.dat";
   std::string plyfile = "output.ply";
   int CamNumAll = 36;
   int PtNum = 4983;

   Matrix34d Proj[CamNumAll];
   MatrixXd pt[PtNum], ptc[PtNum];
   double rerr[PtNum];
   Vector3d rp[PtNum];
   MatrixXi idx(PtNum,CamNumAll);

   // resize each point data matrix whose size is 2 x CamNum
   for (int i = 0; i < PtNum; i++)
   {
      pt[i].resize(2,CamNumAll);
      ptc[i].resize(2,CamNumAll);
   }

   // read projection matrices
   std::cerr << "loading projection matrices ... ";
   load_proj_mat(projmatfile, Proj, CamNumAll);
   std::cerr << "done" << std::endl;

   // read point data
   std::cerr << "loading data ... ";
   load_point_data(pointfile, pt, CamNumAll, PtNum);
   std::cerr << "done" << std::endl;

   // optimal correction and triangulation
   std::cerr << "doing optimal correction ...";
   MultipleViewTriangulation::optimal_correction_all(Proj, CamNumAll,
                                                     pt, ptc, idx,
                                                     rerr, PtNum);
   std::cerr << "done" << std::endl;

   std::cerr << "doing triangulation ...";
   MultipleViewTriangulation::triangulation_all(Proj, CamNumAll,
                                                ptc, rp, PtNum,
                                                idx);
   std::cerr << "done" << std::endl;
   
   // save data as ply format
   std::cerr << "saving a ply file ...";
   save_data_as_ply(plyfile, rp, PtNum);
   std::cerr << "done" << std::endl;

   std::cerr << "all done!" << std::endl;
}
