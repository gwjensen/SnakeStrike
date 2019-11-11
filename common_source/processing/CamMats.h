#ifndef CamMats_h
#define CamMats_h

#include <string>

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Dense>


class CamMats
{
    public:
        static CamMats* Instance();
        ~CamMats(){}
        void Init( const std::string& iConfigLocation );

        std::vector< cv::Mat_< double > > const K()  const { return mK; }
        std::vector< cv::Mat_< double > > const KH()  const { return mKH; }
        std::vector< cv::Mat_< double > > const KInvH()  const { return mKInvH; }
        std::vector< cv::Mat_< double > > const Pose()  const { return mPose; }
        std::vector< cv::Mat_< double > > const Extrinsic()  const { return mExtrinsic; }
        std::vector< cv::Mat_< double > > const R()  const { return mR; }
        std::vector< cv::Mat_< double > > const RInv()  const { return mRInv; }
        std::vector< cv::Mat_< double > > const T()  const { return mT; }
        std::vector< cv::Mat_< double > > const TInv()  const { return mTInv; }
        std::vector< cv::Point3d >        const C()  const { return mC; }
        std::vector< cv::Mat_< double > > const Distortion() const { return mDistortion; }
        std::vector< cv::Mat_< double > > const M() const { return mM; }
        std::vector< cv::Mat_< double > > const MInv() const { return mMInv; }
        std::vector< Eigen::Matrix< double, 3, 4 > > const EigenM() const { return mEigenM; }

        void GetPrincipalPointInfo( const int&      iCamIdx,
                                    double&         oFocalLength,
                                    cv::Point2d&    oOpticalCenter ) const;

        void GetPrincipalPointInfo( std::vector< double >&      oFocalLengths,
                                    std::vector< cv::Point2d >& oOpticalCenters) const;

        bool isInit(){ return mIsInit; }


    private:
        static CamMats* mpsInstance;
        bool mIsInit;

        std::vector< cv::Mat_< double > > mK;//, tmpM33);
        std::vector< cv::Mat_< double > > mKH;
        std::vector< cv::Mat_< double > > mKInvH;
        std::vector< cv::Mat_< double > > mPose;
        std::vector< cv::Mat_< double > > mExtrinsic;//, tmpM44);
        std::vector< cv::Mat_< double > > mR;//, tmpM33);
        std::vector< cv::Mat_< double > > mRInv;
        std::vector< cv::Mat_< double > > mT;//, tmpM13);
        std::vector< cv::Mat_< double > > mTInv;
        std::vector< cv::Point3d > mC;
        std::vector< cv::Mat_< double > > mDistortion;//, tmpM15);
        std::vector< cv::Mat_< double > > mM; //Camera Projection Matrix
        std::vector< cv::Mat_< double > > mMInv;
        std::vector< Eigen::Matrix< double, 3, 4 > > mEigenM;

        CamMats():mIsInit(false){ }

        CamMats(CamMats const&);        // Don't Implement
        void operator=(CamMats const&); // Don't implement
        void ClearData();

};


#endif // CamMats_h
