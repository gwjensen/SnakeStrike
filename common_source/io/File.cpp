#include <sys/types.h>
#include <dirent.h>
#include <iostream>
#include <regex>
#include <math.h>
#include <string>
#include <utility>
#include <assert.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/error/en.h>

#include "common_types.h"
#include "visualization/opencv_viz.h"
#include "image/ImageSet.h"
#include "eigen_cv_conversions.h"
#include "ThreadPool.h"
#include "processing/CamMats.h"
#include "utils/utils.h"

#include "File.h"

using namespace rapidjson;

std::string GetFileExt(const std::string& iPath)
{

   size_t i = iPath.rfind('.', iPath.length());
   if (i != std::string::npos) {
      return(iPath.substr(i+1, iPath.length() - i));
   }

   return("");
}

double CalcRotatedImageBufferSize( const cv::Mat& iImage )
{
    return sqrt( pow( iImage.rows/2, 2) + pow( iImage.cols/2, 2) ) * 2;
}

void ReadXMLImages( const std::string& iLocation,
                        const std::string& iBaseDir,
                        std::vector< std::string >& oImageLocations,
                        uint32_t& oNumCams,
                        uint64_t& oNumImagesFound )
{
    DIR *directory = opendir( iLocation.c_str() );
    oNumImagesFound = 0;

    if (directory == NULL)
    {
        std::fprintf(stderr, "Given config file, reading file.\n");

        //Was given a filename. Check to see if it works as a OpenCV storage file
        cv::FileStorage fs( iLocation, cv::FileStorage::READ );
        int tmp_num = 0;
        fs["nCameras"] >> tmp_num;
        oNumCams = static_cast<uint32_t>( tmp_num );
        assert( oNumCams > 0 );

        cv::FileNode node = fs["images"];
        if (node.type() != cv::FileNode::SEQ)
        {
            if( node.type() != cv::FileNode::STR)
            {
                std::cerr << "Bad FileNode! FAIL" << std::endl;
            }
            else {
                //fprintf( stdout, "The images are stored in a video, not at separate images.\n");
            }
        }

        cv::FileNodeIterator it = node.begin(), it_end = node.end();
        for ( ; it != it_end; ++it)
        {
            std::stringstream tmp_str;
            tmp_str << iBaseDir << "/" << (std::string)*it;
            oImageLocations.push_back( tmp_str.str() );
            //std::fprintf(stderr, "Found file: %s\n", tmpStr.str().c_str());
            ++oNumImagesFound;
        }
        fs.release();
    }
    else
    {
        std::fprintf( stderr, "Given directory path...searching for files...\n" );

        // Read the directory, and pull in every file that doesn't start with '.'
        struct dirent *entry;
        while (NULL != ( entry = readdir(directory) ))
        {
            // by convention, UNIX files beginning with '.' are invisible.
            // and . and .. are special anyway.
            if (entry->d_name[0] != '.')
            {
                oImageLocations.push_back( entry->d_name );
                //std::fprintf( stderr, "Found file: %s \n", entry->d_name );
                ++oNumImagesFound;
            }
        }
    }
    closedir( directory );
}

void ReadImageTimestepGroup( const uint32_t iStartTimestep,
                             const uint32_t iEndTimestep,
                             const std::pair<unsigned long, std::set<unsigned long> >& iCamIndexesToExclude,
                             const std::map< std::string, std::string >& iFileList,
                             bool (*CancelFunc)(),
                             const std::vector< SmtImage >& iCamMasks,
                             ImageSet& oTimestepImages
                             )
{
    //We do this weird count up thing so we can be sure we group the images correctly in the case when not all cameras have a view at a specific timestamp.
    for (uint32_t j = iStartTimestep; j < iEndTimestep; ++j)
    { //for each timestep
        if ((*CancelFunc)())
        {
            std::fprintf( stdout, "Cancel called during operation. Cancelling...\n");
            return;
        }
        std::vector< SmtImage > cams_in_timestep;
        for (uint32_t i = 0; i <  iCamIndexesToExclude.first; ++i)
        {
            if ((*CancelFunc)())
            {
                std::fprintf( stdout, "Cancel called during operation. Cancelling...\n");
                return;
            }
            if (1 == iCamIndexesToExclude.second.count( i ))
            {
                continue;
            }
            std::ostringstream string_stream;
            string_stream << i << "-" << j;

            std::map< std::string, std::string >::const_iterator file_iter = iFileList.find(string_stream.str());
            if (file_iter != iFileList.end())
            {
                SmtImage image_in( file_iter->second,
                                   i,
                                   j,
                                   cv::imread( file_iter->second, CV_LOAD_IMAGE_COLOR ),
                                   false /*RBG*/);

                if (!image_in.data)
                {
                    std::fprintf( stderr,
                                  "SHOULD NOT HAPPEN: Could not open or find the image at '%s', '%s'\n",
                                  string_stream.str().c_str(),
                                  file_iter->second.c_str() );
                    continue;
                }
                else
                {
                    if (iCamMasks.size() > 0)
                    {
                        image_in.BackgroundSubtraction( iCamMasks[i] ); //second method
                    }
                    cams_in_timestep.push_back( image_in );
                }
            }
            else
            {
                std::cerr << "Didn't find file " << string_stream.str().c_str() << "\n" << std::endl;
            }
        }
        oTimestepImages[j] = cams_in_timestep;
    }
}

std::string CreateVideoFilename(const unsigned int& iCamNum,
                                const unsigned int& iImageWidth,
                                const unsigned int& iImageHeight,
                                const uint64_t& iTotalFrames,
                                const float& iOrigCamHz,
                                const float& iVideoPlaybackHz)
{
    std::string orig_cam_hz_str = std::to_string( iOrigCamHz );
    FindAndReplaceAll(orig_cam_hz_str, ".", "-");

    std::string video_play_hz_str = std::to_string( iVideoPlaybackHz );
    FindAndReplaceAll(video_play_hz_str, ".", "-");

    std::string filename = "Cam" + std::to_string( iCamNum ) +
                           "_s" + std::to_string( iImageWidth ) + "x" + std::to_string( iImageHeight ) +
                           "_f" + std::to_string( iTotalFrames ) +
                           "_h" + orig_cam_hz_str +
                           "_p" + video_play_hz_str +
                           ".avi";
    return filename;
}

bool ParseVideoFilename( const std::string& iFilename,
                         unsigned int& oCamNum,
                         unsigned int& oImageWidth,
                         unsigned int& oImageHeight,
                         uint64_t& oTotalFrames,
                         float& oOrigCamHz,
                         float& oVideoPlaybackHz,
                         std::string& oFileType)
{
    ///////////////////////////////////
    ////pull relevant info from filename
    //new regex to pull info from video filename
    //Pulls cam#, image size, num frames, frame rate during capture, framerate of video, and video type
    std::string regex_str = "Cam([0-9]+)_s([0-9]+)x([0-9]+)_f([0-9]+)_h([0-9\\-]+)_p([0-9\\-]+)\\.(\\w+)";
    bool reg_found = false;
    std::smatch match_result;

    try
    {
        std::regex regex_pattern( regex_str );
        reg_found = regex_search( iFilename, match_result, regex_pattern );
    }
    catch (const std::regex_error& e)
    {
        std::cerr << "Problem with regex instantiation: " << e.what() << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Problem with regex_search: " << e.what() << std::endl;
    }

    if (!reg_found)
    {
        std::cerr << "Regular expression not found in filename candidate. Possible error with regex or with candidate" << std::endl;
        std::cerr << "Regex: " << regex_str << std::endl;
        std::cerr << "Candidate: " << iFilename.c_str() << std::endl;
    }
    else
    {
        oCamNum = stoui( match_result[1] );
        oImageWidth = stoui( match_result[2] );
        oImageHeight = stoui( match_result[3] );
        oTotalFrames = std::stoull( match_result[4] );

        std::string original_cam_hz_str = match_result[5];
        FindAndReplaceAll(original_cam_hz_str, "-", ".");
        oOrigCamHz = std::stof( original_cam_hz_str );

        std::string video_playback_hz_str = match_result[6];
        FindAndReplaceAll(video_playback_hz_str, "-", ".");
        oVideoPlaybackHz = std::stof( video_playback_hz_str );

        oFileType = match_result[7];
    }
    return reg_found;
}

void PopulateImagesFromVideos( std::vector< std::string > iVideoFileNames,
                               const TrackerConfigFile& iConfig,
                              ImageSet& oTimestepImages,
                              bool (*CancelFunc)() )
{
    //::TRICKY:: we will be using this nested vector in changed vector order. ie. camera [timestep_image] not timestep{camera_image]
    std::vector<std::vector<SmtImage> > camera_timestep_set;
    //std::vector<FILE*> pipe_list;

    unsigned int cam_num = 0;
    unsigned int image_width = 0;
    unsigned int image_height = 0;
    uint64_t    total_frames = 0;
    float cam_hz = 0;
    float video_hz = 0;
    std::string file_type = "";

    //Need to put the images into the correct format. they are currently [camera[timestep_image]]
    //They need to be [timestep[camera_image]] to work with the rest of the code base
    std::pair<unsigned long, std::set<unsigned long> > cam_indexes_to_exclude_pair( iConfig.numCameras,
                                                                                iConfig.camIndexesToExclude);
    camera_timestep_set.resize( iVideoFileNames.size() - cam_indexes_to_exclude_pair.second.size() );

    {
        ThreadPool pool( iVideoFileNames.size() );
        uint32_t index = 0;
        for (uint32_t i=0; i < iVideoFileNames.size(); ++i)
        {
            bool parseable = ParseVideoFilename( iVideoFileNames[i],
                                                 cam_num,
                                                 image_width,
                                                 image_height,
                                                 total_frames,
                                                 cam_hz,
                                                 video_hz,
                                                 file_type);
            if (parseable)
            {
                assert( 0 == file_type.compare("avi") );
                if (1 == cam_indexes_to_exclude_pair.second.count( cam_num ))
                {
                    //If we try to include excluded cameras we are gonna have problems with Empty images or empty arrays.
                    continue;
                }
                pool.enqueue( boost::bind( ReadImages,
                                           iVideoFileNames[i],
                                           iConfig,
                                           total_frames,
                                           image_width,
                                           image_height,
                                           cam_num,
                                           boost::ref(camera_timestep_set[index]),
                                           CancelFunc) );
                ++index;
            }
        }
    }//this bracket ensures we wait until the pool is done before continuing onwards

    if ((*CancelFunc)())
    {
        //cancel was called, don't need to populate, should probably clean up though.
    }
    else
    {

        //::TRICKY:: We have to do this in order to make sure the cv::Mat objects get deleted.
        {
            ImageSet delete_vector;
            delete_vector.swap(oTimestepImages);
        }
        oTimestepImages.resize( total_frames );


        for ( int j = 0; j < total_frames; ++j)
        {
            for (int i =0; i < camera_timestep_set.size(); ++i)
            {
                //Just making sure the cameras are sorted by camera index number.
                std::vector< SmtImage >::iterator iter = oTimestepImages[j].begin();
                for (; iter != oTimestepImages[j].end(); ++iter )
                {
                    if ( camera_timestep_set[i][j].Cam() > iter->Cam() )
                    {
                        continue;
                    }
                    else
                    {
                        oTimestepImages[j].insert(iter, camera_timestep_set[i][j]);
                        iter = oTimestepImages[j].begin();
                        break;
                    }
                }
                if ( iter == oTimestepImages[j].end())
                {
                    oTimestepImages[j].push_back(camera_timestep_set[i][j]);
                }

            }
        }
    }
}

void ReadImages( const std::string& iFilename,
                 const TrackerConfigFile& iConfig,
                 const uint64_t iTotalFrames,
                 const int iImageWidth,
                 const int iImageHeight,
                 const int iCamNum,
                 std::vector<SmtImage>& oTimestepImages,
                 bool (*CancelFunc)() )
{
    //This function does make the assumption that all of the images to be read have the same filetype
    std::vector< std::string > image_locations;

    //Get the background masks if they exist
    bool use_masks = false;
    std::vector< SmtImage > cam_masks;
    uint32_t back_cam_num = 0;
    uint64_t back_image_count = 0;

    ///@todo this could be improved, we are currently reading in all the masks for every camera instead of the specific mask
    if (iConfig.maskFileLocation != "")
    {
        use_masks = true;
        ReadXMLImages( iConfig.maskFileLocation, iConfig.projDir, image_locations, back_cam_num, back_image_count );

        for (uint32_t i = 0; i < back_cam_num; ++i)
        {
            SmtImage image_in( "",
                               i,
                               -1,
                               cv::imread( image_locations[ i ], CV_LOAD_IMAGE_COLOR ),
                               false /*RGB*/);
            cam_masks.push_back( image_in );
        }
        image_locations.clear();
    }

    uint64_t image_count = 0;

    //::TRICKY:: We have to do this in order to make sure the cv::Mat objects get deleted.
    {
        std::vector<SmtImage> delete_vector;
        delete_vector.swap(oTimestepImages);
    }
    //oTimestepImages.clear();

    bool continue_polling = true;
    cv::VideoCapture video(iFilename);
    while(continue_polling)
    {
        if ((*CancelFunc)())
        {
            //::TRICKY:: We have to do this in order to make sure the cv::Mat objects get deleted.
            {
                std::vector<SmtImage> delete_vector;
                delete_vector.swap(oTimestepImages);
            }
            //oTimestepImages.clear();
            return;
        }

        cv::Mat frame;
        video >> frame;

        if (frame.empty() || image_count >= iTotalFrames)
        {
            continue_polling = false;
        }
        else
        {
            cv::cvtColor( frame, frame, CV_BGR2RGB);

            SmtImage new_image( iConfig.dataFileLocation,
                                iCamNum,
                                image_count,
                                frame, //the constructor does a copy
                                true);
            if (use_masks)
            {
                new_image.BackgroundSubtraction( cam_masks[iCamNum] );
            }
            oTimestepImages.push_back( new_image );
            ++image_count;
        }
    }
    assert( image_count == iTotalFrames );
    std::fprintf( stderr," imageCount of image locations found: %lu  \n", image_count );
}

uint64_t ReadImages( const TrackerConfigFile& iConfig,
                     ImageSet& oTimestepImages,
                     bool (*CancelFunc)() )
{
    //This function does make the assumption that all of the images to be read have the same filetype
    std::vector< std::string > image_locations;
    std::pair<unsigned long, std::set<unsigned long> > camIndexesToExcludePair( iConfig.numCameras,
                                                                                iConfig.camIndexesToExclude);


    //Get the background masks if they exist
    bool use_masks = false;
    std::vector< SmtImage > cam_masks;
    uint32_t back_cam_num = 0;
    uint64_t back_image_count = 0;
    if (iConfig.maskFileLocation != "")
    {
        use_masks = true;
        ReadXMLImages( iConfig.maskFileLocation, iConfig.projDir, image_locations, back_cam_num, back_image_count );

        for (uint32_t i = 0; i < back_cam_num; ++i)
        {
            SmtImage image_in( "",
                               i,
                               -1,
                               cv::imread( image_locations[ i ], CV_LOAD_IMAGE_COLOR ),
                               false /*RGB*/);
            cam_masks.push_back( image_in );
        }
        image_locations.clear();
    }

    //first find out the highest number of cameras listed for the images, then the highest timestep number
    ///@TODO these numbers could/should be read from the python generated xml image list files.
    uint32_t cam_num = 0;
    uint64_t image_count = 0;
    uint32_t max_timestep = 0;
    std::string regex_str = "([0-9]+)-([0-9]+)\\.(\\w+)";

    std::map< std::string, std::string > file_list;
    std::string suffix;

    ReadXMLImages( iConfig.dataFileLocation, iConfig.projDir, image_locations, cam_num, image_count );

    if( use_masks ){
        assert( cam_num == back_cam_num );
    }


    std::fprintf( stderr," imageCount of image locations found: %lu  \n", image_count );

    //Create a map of the files with the timestep and camera they belong
    DIR *directory = opendir( iConfig.dataFileLocation.c_str() );
    for (uint32_t index = 0; index < image_locations.size(); ++index)
    {
        bool reg_found = false;
        std::smatch match_result;

        try
        {
            std::regex regex_pattern( regex_str );
            reg_found = regex_search( image_locations[index], match_result, regex_pattern );
        }
        catch (const std::regex_error& e)
        {
            std::cerr << "Problem with regex instantiation: " << e.what() << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Problem with regex_search: " << e.what() << std::endl;
        }

        if (!reg_found && directory != NULL)
        {
            //directory search could find filenames that don't fit what we are looking for...
            continue;
        }
        else if (!reg_found)
        {
            std::cerr << "Regular expression not found in filename candidate. Possible error with regex or with candidate" << std::endl;
            std::cerr << "Regex: " << regex_str << std::endl;
            std::cerr << "Candidate: " << image_locations[index].c_str() << std::endl;
        }
        else
        {
            if (match_result[1] > cam_num)
            {
                cam_num = std::stoi(match_result[1]);
            }
            if (stoui(match_result[2]) > max_timestep)
            {
                max_timestep = std::stoi(match_result[2]);
            }
            std::string res1( match_result[1] );
            std::string res2( match_result[2] );
            std::string filename( res1 + "-" + res2 );
            file_list.insert(  make_pair( filename, image_locations[index] ) );
            //if( suffix.empty() ){
                suffix = match_result[3];
            //}
        }
    }
    closedir( directory );

    //Initialize the size of the arrays.
    //::TRICKY:: We have to do this in order to make sure the cv::Mat objects get deleted.
    {
        ImageSet delete_vector;
        delete_vector.swap(oTimestepImages);
    }
    oTimestepImages.resize( max_timestep + 1 ); //have to add one because of 0 start in timestep counting
    //Call threadpool code, put it in a block so that we can check the status of the cancel function
    //as that requires the pool to be destructed. If we put it in the same scope, then the cancel check
    //is done while the code is still running.
    {
        ThreadPool pool( 12 ); //6 real cores in our server, but we are io bottlenecked, not CPU
        uint32_t numTimestepsPerThread = 500;
        for (uint32_t j = 0; j < oTimestepImages.size() ; )
        { //for each timestep
            if (oTimestepImages.size() - j < numTimestepsPerThread)
            {
                numTimestepsPerThread = oTimestepImages.size() - j;
            }
            pool.enqueue( boost::bind( ReadImageTimestepGroup,
                                       j,
                                       j + numTimestepsPerThread,
                                       camIndexesToExcludePair,
                                       boost::ref( file_list ),
                                       CancelFunc,
                                       boost::ref( cam_masks ),
                                       boost::ref( oTimestepImages ) ));
            j += numTimestepsPerThread;
        }
    }
    if ((*CancelFunc)())
    {
        //::TRICKY:: We have to do this in order to make sure the cv::Mat objects get deleted.
        {
            ImageSet delete_vector;
            delete_vector.swap(oTimestepImages);
        }
        return 0;
    }

    return image_count;
}

void WriteTriangulationInfoToFile( cv::FileStorage& iFs,
                                   const std::string& iName,
                                   std::vector< std::vector< cv::Point3d > >& iList,
                                   const int iMaxNumPoints)
{
     uint32_t timesteps = iList.size();
     uint32_t missing = 0;
     for (uint32_t j = 0; j< timesteps; ++j)
     {
         if (iList[j].size() == 0)
         {
             ++missing;
         }
     }
     // typedefs
     iFs << iName << "{";
     iFs << "MaxNumPoints" << iMaxNumPoints;
     iFs << "MaxTimestep" << static_cast<int32_t>(timesteps);
     iFs << "NumMissingTimesteps" << static_cast<int32_t>(missing);
     for (uint32_t j=0; j< timesteps; ++j)
     {
         std::stringstream ss;
         std::string s;
         std::string a;
         a="Timestep";
         ss << (j);
         s = ss.str();
         a+=s;

         iFs << a << iList[j];
     }
     iFs << "}";
}




template<typename _Tp>inline void ReadFileNodeList(const cv::FileNode& fn, std::vector<_Tp>& result)
{
    if (fn.type() == cv::FileNode::SEQ)
    {
        for (cv::FileNodeIterator it = fn.begin(); it != fn.end();)
        {
            _Tp item;
            it >> item;
            result.push_back(item);
        }
    }
}

std::vector < std::vector< cv::Point3d > > ReadTriangulationInfoToFile( const cv::FileNode& iFn )
{
    std::vector< std::vector< cv::Point3d > > output;

    for (uint32_t j=0; j < iFn.size(); ++j)
    {
        std::stringstream ss;
        std::string s;
        std::string a;
        a="Timestep";
        ss << j;
        s = ss.str();
        a+=s;
        cv::FileNode temp_Id;
        temp_Id = iFn[a];
        std::vector< cv::Point3d > cam_points;
        ReadFileNodeList( temp_Id, cam_points);
        output.push_back( cam_points );
    }
    return output;
}

void WriteConfig( const std::string& iFileName, TrackerConfigFile& iConfigStruct )
{
    ///@todo change this to rapidjson. See ProjectDialog::WriteConfigFile
    //The tracker file should have relative paths according to the projDir on writeout, on read in they should be
    //constructed back to absolute paths.
    std::ofstream stream;
    stream.open( iFileName );
    stream << "{\n";
    stream << "\"projDir\" : \"" << iConfigStruct.projDir << "\",\n";

    stream << "\"dataFileLocation\" : \"" <<
              EraseSubString(iConfigStruct.dataFileLocation, iConfigStruct.projDir) <<
              "\",\n";
    stream << "\"maskFileLocation\" : \"" <<
              EraseSubString(iConfigStruct.maskFileLocation, iConfigStruct.projDir) <<
              "\",\n";
    stream << "\"calibFileLocation\"   : \"" <<
              EraseSubString(iConfigStruct.calibFileLocation, iConfigStruct.projDir) <<
              "\",\n";
    stream << "\"writeUndistImages\" : \"" <<
              EraseSubString(iConfigStruct.writeUndistImages, iConfigStruct.projDir) <<
              "\",\n";
    stream << "\"triangulationOutput\" : \"" <<
              EraseSubString(iConfigStruct.triangulationOutput, iConfigStruct.projDir) <<
              "\",\n";
    stream << "\"vizUndistortedImages\": " << iConfigStruct.vizUndistortedImages << ",\n";
    stream << "\"vizPointCorrespondences\" : " << iConfigStruct.vizPointCorrespondences << ",\n";
    stream << "\"vizCameraPose\" : " << iConfigStruct.vizCameraPose << ",\n";
    stream << "\"vizThresholds\" : " << iConfigStruct.vizThresholds << ",\n";

    stream << "\"camIndexesToExclude\" : [ ";
    std::set<uint64_t>::iterator it;
    for (it = iConfigStruct.camIndexesToExclude.begin();
         it != iConfigStruct.camIndexesToExclude.end();   )
    {
        stream << *it;
        ++it;
        if( it != iConfigStruct.camIndexesToExclude.end() ){
            stream << ", ";
        }
    }
    stream << " ],\n";
    std::string tmp_str = iConfigStruct.tryToUseSavedMarkedPoints ? "true" : "false";
    stream << "\"tryToUseSavedMarkedPoints\" : " << tmp_str << ",\n";
    tmp_str = iConfigStruct.undistortImagesBool ? "true" : "false";
    stream << "\"undistortImages\" : " << tmp_str << ",\n";
    tmp_str = iConfigStruct.viz3dTriangulatedPoints ? "true" : "false";
    stream << "\"viz3dTriangulatedPoints\" : " << tmp_str << ",\n";
    stream << "\"noiseFilterSize\" : " << iConfigStruct.noiseFilterSize << ",\n";
    stream << "\"noiseIterations\" : " << iConfigStruct.noiseIterations << ",\n";
    stream << "\"noiseThreshold\" :  " << iConfigStruct.noiseThreshold << ",\n";
    stream << "\"numCameras\" : " << iConfigStruct.numCameras << ",\n";
    stream << "\"minNumCamsTriangulation\" : " << iConfigStruct.minNumCamsTriangulation << ",\n";
    stream << "\"maxNumPoints\" : " << iConfigStruct.maxNumPoints << ",\n";
    stream << "\"hLeftbound\" : " << iConfigStruct.hLeftbound << ",\n";
    stream << "\"sLeftbound\" : " << iConfigStruct.sLeftbound << ",\n";
    stream << "\"vLeftbound\" : " << iConfigStruct.vLeftbound << ",\n";
    stream << "\"hRightbound\" : " << iConfigStruct.hRightbound << ",\n";
    stream << "\"sRightbound\" : " << iConfigStruct.sRightbound << ",\n";
    stream << "\"vRightbound\" : " << iConfigStruct.vRightbound << "\n"; //No Comma on the last in list
    stream << "}\n";
    stream.close();
}

//The tracker file should have relative paths according to the projDir on writeout, on read in they should be
//constructed back to absolute paths.
void ProcessConfigFile( const std::string& iFileName, TrackerConfigFile& oConfigStruct )
{
    //Read in file as a string
    std::fprintf( stderr, "Reading in Config file...\n" );
    std::ifstream t(iFileName);
    std::stringstream buffer;
    if ( t )
    {
        buffer << t.rdbuf();
    }
    else
    {
        std::cerr << "File could not be opened!\n"; // Report error
        std::cerr << "File was \"" << iFileName << "\"" << std::endl;
        std::cerr << "Error code: " << strerror(errno); // Get some info as to why
    }
    std::string buf_str = buffer.str();
    std::fprintf( stderr, "\nConfig file:\n %s\n\n", buf_str.c_str() );

    // 1. Parse a JSON config file into DOM.
    const char* json = buf_str.c_str();
    Document doc;


    //ParseResult result = doc.Parse(json);
    if (doc.Parse( json ).HasParseError())
    {
        fprintf(stderr,
                "\nError(offset %u): %s\n",
                (unsigned)doc.GetErrorOffset(),
                GetParseError_En(doc.GetParseError()));
        exit( -1 );
    }

    std::fprintf( stderr, "Reading in projDir...\n" );
    //tmpstring = doc.FindMember("dataFileLocation");
    assert( doc.HasMember("projDir") );
    Value& tmp_str = doc["projDir"];

    assert(tmp_str.IsString());
    oConfigStruct.projDir = tmp_str.GetString();
    fprintf(stderr, "\t%s\n", oConfigStruct.projDir.c_str());


    std::fprintf( stderr, "Reading in dataFileLocation...\n" );
    //tmpstring = doc.FindMember("dataFileLocation");
    assert( doc.HasMember("dataFileLocation") );
    tmp_str = doc["dataFileLocation"];

    assert(tmp_str.IsString());
    oConfigStruct.dataFileLocation = oConfigStruct.projDir + "/" + tmp_str.GetString();
    fprintf(stderr, "\t%s\n", oConfigStruct.dataFileLocation.c_str());


    std::fprintf( stderr, "Reading in maskFileLocation...\n" );
    //tmpstring = doc.FindMember("maskFileLocation");
    assert( doc.HasMember("maskFileLocation") );
    tmp_str = doc["maskFileLocation"];

    assert( tmp_str.IsString() );
    std::string check_str =  tmp_str.GetString();
    if (!check_str.empty())
    {
        oConfigStruct.maskFileLocation = oConfigStruct.projDir + "/" + check_str;
    }
    std::fprintf( stderr, "maskFileLocation: \"%s\"\n", oConfigStruct.maskFileLocation.c_str() );


    std::fprintf( stderr, "Reading in calibFileLocation...\n" );
    //tmpstring = doc.FindMember("calibFileLocation");
    assert( doc.HasMember("calibFileLocation") );
    tmp_str = doc["calibFileLocation"];

    assert(tmp_str.IsString());
    oConfigStruct.calibFileLocation = oConfigStruct.projDir + "/" + tmp_str.GetString();
    std::fprintf( stderr, "calibFileLocation: \"%s\"\n", oConfigStruct.calibFileLocation.c_str() );


    std::fprintf( stderr, "Reading in writeUndistImages...\n" );
    assert( doc.HasMember("writeUndistImages") );
    tmp_str = doc["writeUndistImages"];
    check_str =  tmp_str.GetString();
    if (!check_str.empty())
    {
        //assert(tmpstring.IsString());
        oConfigStruct.writeUndistImages = oConfigStruct.projDir + "/" + check_str;
    }

    std::fprintf( stderr, "writeUndistImages: \"%s\"\n", oConfigStruct.writeUndistImages.c_str() );

    std::fprintf( stderr, "Reading in triangulationOutput...\n" );
    assert( doc.HasMember("triangulationOutput") );
    tmp_str = doc["triangulationOutput"];
    if (!tmp_str.IsNull())
    {
        //assert(tmpstring.IsString());
        oConfigStruct.triangulationOutput = oConfigStruct.projDir + "/" + tmp_str.GetString();
    }
    std::fprintf( stderr, "triangulationOutput: \"%s\"\n", oConfigStruct.triangulationOutput.c_str() );

    std::fprintf( stderr, "Reading in vizUndistortedImages...\n" );
    assert( doc.HasMember("vizUndistortedImages") );
    assert( doc["vizUndistortedImages"].IsInt64() );
    oConfigStruct.vizUndistortedImages = doc["vizUndistortedImages"].GetInt64();
    std::fprintf( stderr, "vizUndistortedImages: \"%" PRId64 "\"\n", oConfigStruct.vizUndistortedImages );

    std::fprintf( stderr, "Reading in vizPointCorrespondences...\n" );
    assert( doc.HasMember("vizPointCorrespondences") );
    assert( doc["vizPointCorrespondences"].IsInt64() );
    oConfigStruct.vizPointCorrespondences = doc["vizPointCorrespondences"].GetInt64();
    std::fprintf( stderr, "vizPointCorrespondences: \"%" PRId64 "\"\n", oConfigStruct.vizPointCorrespondences );

    std::fprintf( stderr, "Reading in vizCameraPose...\n" );
    assert( doc.HasMember("vizCameraPose") );
    assert( doc["vizCameraPose"].IsInt64() );
    oConfigStruct.vizCameraPose = doc["vizCameraPose"].GetInt64();
    std::fprintf( stderr, "vizCameraPose: \"%" PRId64 "\"\n", oConfigStruct.vizCameraPose );

    std::fprintf( stderr, "Reading in vizThresholds...\n" );
    assert( doc.HasMember("vizThresholds") );
    assert( doc["vizThresholds"].IsInt64() );
    oConfigStruct.vizThresholds = doc["vizThresholds"].GetInt64();
    std::fprintf( stderr, "vizThresholds: \"%" PRId64 "\"\n", oConfigStruct.vizThresholds );

    std::fprintf( stderr, "Reading in viz3dTriangulatedPoints...\n" );
    assert( doc.HasMember("viz3dTriangulatedPoints") );
    assert( doc["viz3dTriangulatedPoints"].IsBool() );
    oConfigStruct.viz3dTriangulatedPoints = doc["viz3dTriangulatedPoints"].GetBool();
    std::fprintf( stderr, "viz3dTriangulatedPoints: \"%s\"\n", oConfigStruct.viz3dTriangulatedPoints ? "true" : "false" );

    std::fprintf( stderr, "Reading in undistortImages...\n" );
    assert( doc.HasMember("undistortImages") );
    assert( doc["undistortImages"].IsBool() );
    oConfigStruct.undistortImagesBool = doc["undistortImages"].GetBool();
    std::fprintf( stderr, "undistortImages: \"%s\"\n", oConfigStruct.undistortImagesBool ? "true" : "false" );

    std::fprintf( stderr, "Reading in minNumCamsTriangulation...\n" );
    assert( doc.HasMember("minNumCamsTriangulation") );
    assert( doc["minNumCamsTriangulation"].IsInt() );
    oConfigStruct.minNumCamsTriangulation = doc["minNumCamsTriangulation"].GetInt();
    std::fprintf( stderr, "minNumCamsTriangulation: \"%d\"\n", oConfigStruct.minNumCamsTriangulation );

    std::fprintf( stderr, "Reading in maxNumPoints...\n" );
    assert( doc.HasMember("maxNumPoints") );
    assert( doc["maxNumPoints"].IsInt() );
    oConfigStruct.maxNumPoints = doc["maxNumPoints"].GetInt();
    std::fprintf( stderr, "maxNumPoints: \"%d\"\n", oConfigStruct.maxNumPoints );
    //
    std::fprintf( stderr, "Reading in noiseFilterSize...\n" );
    assert( doc.HasMember("noiseFilterSize") );
    assert( doc["noiseFilterSize"].IsInt() );
    oConfigStruct.noiseFilterSize = doc["noiseFilterSize"].GetInt();
    std::fprintf( stderr, "noiseFilterSize: \"%d\"\n", oConfigStruct.noiseFilterSize );

    std::fprintf( stderr, "Reading in noiseIterations...\n" );
    assert( doc.HasMember("noiseIterations") );
    assert( doc["noiseIterations"].IsInt() );
    oConfigStruct.noiseIterations = doc["noiseIterations"].GetInt();
    std::fprintf( stderr, "noiseIterations: \"%d\"\n", oConfigStruct.noiseIterations );

    std::fprintf( stderr, "Reading in noiseThreshold...\n" );
    assert( doc.HasMember("noiseThreshold") );
    assert( doc["noiseThreshold"].IsInt() );
    oConfigStruct.noiseThreshold = doc["noiseThreshold"].GetInt();
    std::fprintf( stderr, "noiseThreshold: \"%d\"\n", oConfigStruct.noiseThreshold );
    //

    std::fprintf( stderr, "Reading in numCameras...\n" );
    assert( doc.HasMember("numCameras") );
    assert( doc["numCameras"].IsInt() );
    oConfigStruct.numCameras = doc["numCameras"].GetInt();
    std::fprintf( stderr, "numCameras: \"%d\"\n", oConfigStruct.numCameras );


    std::fprintf( stderr, "Reading in hLeftbound...\n" );
    assert( doc.HasMember("hLeftbound") );
    assert( doc["hLeftbound"].IsInt() );
    oConfigStruct.hLeftbound = doc["hLeftbound"].GetInt();
    std::fprintf( stderr, "hLeftbound: \"%d\"\n", oConfigStruct.hLeftbound );

    std::fprintf( stderr, "Reading in sLeftbound...\n" );
    assert( doc.HasMember("sLeftbound") );
    assert( doc["sLeftbound"].IsInt() );
    oConfigStruct.sLeftbound = doc["sLeftbound"].GetInt();
    std::fprintf( stderr, "sLeftbound: \"%d\"\n", oConfigStruct.sLeftbound );

    std::fprintf( stderr, "Reading in vLeftbound...\n" );
    assert( doc.HasMember("vLeftbound") );
    assert( doc["vLeftbound"].IsInt() );
    oConfigStruct.vLeftbound = doc["vLeftbound"].GetInt();
    std::fprintf( stderr, "vLeftbound: \"%d\"\n", oConfigStruct.vLeftbound );

    std::fprintf( stderr, "Reading in hRightbound...\n" );
    assert( doc.HasMember("hRightbound") );
    assert( doc["hRightbound"].IsInt() );
    oConfigStruct.hRightbound = doc["hRightbound"].GetInt();
    std::fprintf( stderr, "hRightbound: \"%d\"\n", oConfigStruct.hRightbound );

    std::fprintf( stderr, "Reading in sRightbound...\n" );
    assert( doc.HasMember("sRightbound") );
    assert( doc["sRightbound"].IsInt() );
    oConfigStruct.sRightbound = doc["sRightbound"].GetInt();
    std::fprintf( stderr, "sRightbound: \"%d\"\n", oConfigStruct.sRightbound );

    std::fprintf( stderr, "Reading in vRightbound...\n" );
    assert( doc.HasMember("vRightbound") );
    assert( doc["vRightbound"].IsInt() );
    oConfigStruct.vRightbound = doc["vRightbound"].GetInt();
    std::fprintf( stderr, "vRightbound: \"%d\"\n", oConfigStruct.vRightbound );

    ////////Not required items in the JSON
    std::fprintf(stderr, "Reading in cam Indexes to exclude.\n");
    if (doc.HasMember( "camIndexesToExclude" ))
    {
        for (uint32_t i = 0; i < doc["camIndexesToExclude"].Size(); ++i)
        {
            oConfigStruct.camIndexesToExclude.insert( doc["camIndexesToExclude"][i].GetInt() );
        }
    }
    else
    {
        fprintf(stderr, "    Tag Not present.\n");
    }

    std::fprintf( stderr, "Reading in whether we should used saved Marked points...\n" );
    if (doc.HasMember("tryToUseSavedMarkedPoints"))
    {
        assert( doc["tryToUseSavedMarkedPoints"].IsBool() );
        oConfigStruct.tryToUseSavedMarkedPoints = doc["tryToUseSavedMarkedPoints"].GetBool();
        std::fprintf( stderr,
                      "tryToUseSavedMarkedPoints: \"%s\"\n",
                      oConfigStruct.tryToUseSavedMarkedPoints ? "true" : "false" );
    }
    else
    {
        fprintf(stderr, "    Tag Not present.\n");
        oConfigStruct.tryToUseSavedMarkedPoints = false;
    }
}

void WriteMarkedPointsToFile( const std::string& iFilename/*should be the image file name*/,
                              const std::vector< std::vector< cv::Point2d > >& iMarkedPoints,
                              const std::vector< std::vector< std::vector< int32_t > > >& iBestFit)
{
    std::ofstream myfile;
    boost::filesystem::path path( iFilename );
    path.remove_filename();
    path.append( "UserMarkedCorrespondencePoints.txt");
    myfile.open( path.string() );
    myfile << "Marked Points File\n";
    myfile << "Associated with: " << iFilename << '\n';
    for (uint32_t i = 0; i < iMarkedPoints.size(); ++i)
    {
        myfile << "Cam[";
        for (uint32_t p =0; p < iMarkedPoints[i].size(); ++p)
        {
            myfile << std::to_string(iMarkedPoints[i][p].x) << "|" << std::to_string(iMarkedPoints[i][p].y);
            if (p + 1 < iMarkedPoints[i].size())
            {
                myfile << ", ";
            }
        }
        myfile << "]\n";
    }
    for (uint32_t t = 0; t < iBestFit.size(); ++t)
    {
        myfile << "PointTimestepIdx[" ;
        for (uint32_t p = 0; p < iBestFit[t].size(); ++p)
        {
            for (uint32_t i = 0; i < iBestFit[t][p].size(); ++i)
            {
                myfile << std::to_string( iBestFit[t][p][i] );
                if (i+1 < iBestFit[t][p].size())
                {
                    myfile << "|";
                }
            }
            if (p + 1 < iBestFit[t].size())
            {
                myfile << ", ";
            }
        }
        myfile << "]\n";
    }
    myfile.close();
}

void ReadMarkedPointsFromFile( const std::string& iFilename,
                               std::vector< std::vector< SmtPixel > >& oMarkedPoints,
                               std::vector< std::vector< std::vector< int32_t > > >& oBestFit)
{
    oMarkedPoints.clear();
    oBestFit.clear();
    std::ifstream myfile;
    boost::filesystem::path p( iFilename );
    p.remove_filename();
    p.append( "UserMarkedCorrespondencePoints.txt");
    myfile.open( p.string() );
    std::string line;

    if (myfile.is_open())
    {
        uint32_t cam_num = 0;
        while (getline( myfile, line ))
        {
            if (line.substr(0,4) == "Cam[")
            {
                std::vector< SmtPixel > cam_point_list;
                std::vector< std::string > points;
                boost::split(points, line,boost::is_any_of(",[]Cam\n"));
                for (uint32_t p = 0; p < points.size(); ++p)
                {
                    if (points[p].size() == 0)
                    {
                        continue;
                    }
                    //std::fprintf(stderr, "%s\n",points[p].c_str());
                    std::vector< std::string > pointParts;
                    boost::split( pointParts, points[p], boost::is_any_of("|") );
                    double x = atof( pointParts[0].c_str() );
                    double y = atof( pointParts[1].c_str() );
                    SmtPixel point( x, y, cam_num );
                    cam_point_list.push_back( point );
                }
                oMarkedPoints.push_back( cam_point_list );
                ++cam_num;
            }
            if (line.substr( 0, 17 ) == "PointTimestepIdx[")
            {
                std::vector< std::vector< int32_t > > cams_ordered_by_point;
                std::vector< std::string > points;
                boost::split( points, line, boost::is_any_of(",[]PointTimestepIdx\n"));
                for (uint32_t p = 0; p < points.size(); ++p)
                {
                    if (points[p].size() == 0)
                    {
                        continue;
                    }
                    std::vector< std::string> ordered_cams;
                    std::vector< int32_t > ordered_cams_nums;
                    boost::split( ordered_cams, points[p], boost::is_any_of("|"));
                    for (uint32_t i = 0; i < ordered_cams.size(); ++i)
                    {
                        ordered_cams_nums.push_back( stoi( ordered_cams[i] ) );
                    }
                    cams_ordered_by_point.push_back( ordered_cams_nums );
                }
                oBestFit.push_back( cams_ordered_by_point );
            }
        }
    }
    myfile.close();
}





