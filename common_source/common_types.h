#ifndef common_types_h
#define common_types_h

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Dense>
#include <set>
#include <string>

	// class integer generator:
	struct IndexFiller {
	  int current;
	  IndexFiller() { current=0; }
	  int operator()() { return current++; }
	} ;


    //In memory this object has full paths, when it is written out or read in, they are relative so the
    //projDir needs to be applied to each relative path on construction.
    struct TrackerConfigFile {
        std::string projDir = "";
        std::string dataFileLocation = "";
        std::string maskFileLocation = "";
        std::string calibFileLocation = "";
        std::string writeUndistImages = "";
        std::string triangulationOutput = "";
        int64_t vizUndistortedImages = 0; //can be -1
        int64_t vizPointCorrespondences = 0; //can be -1
        int64_t vizCameraPose = 0; //can be -1
        int64_t vizThresholds = 0; //can be -1
        bool viz3dTriangulatedPoints = false;
        bool undistortImagesBool = false;
        bool tryToUseSavedMarkedPoints = false;
        bool allowHiddenPointMarking = false;
        unsigned int minNumCamsTriangulation = 0;
        unsigned int maxNumPoints = 0;
        unsigned int hLeftbound = 0;
        unsigned int sLeftbound = 0;
        unsigned int vLeftbound = 0;
        unsigned int hRightbound = 0;
        unsigned int sRightbound = 0;
        unsigned int vRightbound = 0;
        unsigned int numCameras = 0;
        unsigned int noiseFilterSize = 0;
        unsigned int noiseIterations = 0;
        unsigned int noiseThreshold = 0;
        std::set<uint64_t> camIndexesToExclude;

    };

#endif //common_types_h
