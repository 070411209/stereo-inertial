#include "ygz/TrackerLK.h"
#include "ygz/ORBExtractor.h"
#include "ygz/ORBMatcher.h"
#include "ygz/LKFlow.h"
#include "ygz/Feature.h"
#include "ygz/BackendInterface.h"
#include "ygz/MapPoint.h"
#include "ygz/Viewer.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ygz {

    TrackerLK::TrackerLK(const string &settingFile) {

        cv::FileStorage fSettings(settingFile, cv::FileStorage::READ);
        if (fSettings.isOpened() == false) {
            cerr << "Setting file not found." << endl;
            return;
        }

        // create camera object
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];
        float bf = fSettings["Camera.bf"];

        mpCam = shared_ptr<CameraParam>(new CameraParam(fx, fy, cx, cy, bf));
        mpExtractor = shared_ptr<ORBExtractor>(new ORBExtractor(ORBExtractor::FAST_SINGLE_LEVEL));
        mpMatcher = shared_ptr<ORBMatcher>(new ORBMatcher);
        mState = NO_IMAGES_YET;
    }

    TrackerLK::TrackerLK() {
        mState = NO_IMAGES_YET;
    }

    SE3d TrackerLK::InsertStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp,
                                 const VecIMU &vimu) {

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        if (setting::trackerUseHistBalance) {
            // perform a histogram equalization
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
            Mat imgLeftAfter, imgRightAfter;
            clahe->apply(imRectLeft, imgLeftAfter);
            clahe->apply(imRectRight, imgRightAfter);
            mpCurrentFrame = shared_ptr<Frame>(new Frame(imgLeftAfter, imgRightAfter, timestamp, mpCam, vimu));
        } else {
            mpCurrentFrame = shared_ptr<Frame>(new Frame(imRectLeft, imRectRight, timestamp, mpCam, vimu));
        }

        if (this->mbVisionOnlyMode == false)
            mvIMUSinceLastKF.insert(mvIMUSinceLastKF.end(), vimu.begin(), vimu.end());   // ??????imu??????

        if (mpLastKeyFrame)
            mpCurrentFrame->mpReferenceKF = mpLastKeyFrame;

        // DO TRACKING !!
        LOG(INFO) << "\n\n********* Tracking frame " << mpCurrentFrame->mnId << " **********" << endl;
        Track();
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        LOG(INFO) << "Insert stereo cost time: " << timeCost << endl;

        if (mpViewer) {
            if (mpCurrentFrame->IsKeyFrame()) {
                mpViewer->AddFrame(mpCurrentFrame);
            } else {
                mpViewer->SetCurrentFrame(mpCurrentFrame);
            }
            mpViewer->SetTrackStatus(static_cast<int>(mState), mTrackInliersCnt);
        }

        LOG(INFO) << "Tracker returns, pose = \n" << mpCurrentFrame->GetPose().matrix() << endl;
        if (mbVisionOnlyMode == false)
            LOG(INFO) << "speed and bias = \n" << mpCurrentFrame->mSpeedAndBias.transpose() << endl;
        return mpCurrentFrame->GetPose();
    }

    void TrackerLK::Track() {

        mTrackInliersCnt = 0;

        if (mState == NO_IMAGES_YET) {

            // ??????????????????????????????????????????
            // first we build the pyramid and compute the features
            LOG(INFO) << "Detecting features" << endl;
            mpExtractor->Detect(mpCurrentFrame, true, false);    // extract the keypoint in left eye
            LOG(INFO) << "Compute stereo matches" << endl;
            mpMatcher->ComputeStereoMatches(mpCurrentFrame, ORBMatcher::OPTIFLOW_CV);

            if (StereoInitialization() == false) {
                LOG(INFO) << "Stereo init failed." << endl;
                return;
            }

            // ????????????????????????
            InsertKeyFrame();

            mpLastFrame = mpCurrentFrame;
            mpLastKeyFrame = mpCurrentFrame;

            // ?????????????????????????????????
            mState = NOT_INITIALIZED;

            return;

        } else if (mState == NOT_INITIALIZED) {

            // IMU???????????????????????????????????????????????????????????????
            bool bOK = false;
            mpCurrentFrame->SetPose(mpLastFrame->GetPose() * mSpeed);  // assume the speed is constant

            bOK = TrackLastFrame(false);

            if (bOK) {
                bOK = TrackLocalMap(mTrackInliersCnt);
            }

            if (bOK == false) {
                // ???????????????????????????reset????????????
                // ????????????????????????????????????
                Reset();
                return;
            }

            CleanOldFeatures();

            if (NeedNewKeyFrame(mTrackInliersCnt)) {

                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                CreateStereoMapPoints();
                InsertKeyFrame();
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                double timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
                LOG(INFO) << "Insert KF cost time: " << timeCost << endl;
            } else {

            }

            // ????????????IMU?????????
            if (mbVisionOnlyMode == false) {
                if (IMUInitialization() == true) {
                    // ??????????????????????????????OK
                    mState = OK;
                }
            }

            mSpeed = mpLastFrame->GetPose().inverse() * mpCurrentFrame->GetPose();
            mpLastFrame = mpCurrentFrame;
            return;

        } else if (mState == OK) {
            // ????????????
            // ???imu????????????????????????????????????????????????????????????
            bool bOK = false;

            /*
            if (rand() % 10 == 0) {
                // ??????????????????
                LOG(INFO) << "Set to lost!" << endl;
                mpCurrentFrame->mImLeft.setTo(0);
                mpCurrentFrame->mImRight.setTo(0);
            }
             */

            PredictCurrentPose();
            // ?????????????????????
            bOK = TrackLastFrame(false);

            if (bOK) {
                // ???lastframe???????????????current???????????????pose???????????????????????????local map????????????
                bOK = TrackLocalMap(mTrackInliersCnt);
            }

            if (bOK) {
                // ????????????
                CleanOldFeatures();

                if (NeedNewKeyFrame(mTrackInliersCnt)) {
                    // ???????????????
                    CreateStereoMapPoints();
                    InsertKeyFrame();
                }

                mpLastFrame = mpCurrentFrame;
                mSpeed = mpLastFrame->GetPose().inverse() * mpCurrentFrame->GetPose();    // ????????????

            } else {
                mState = WEAK;
                mpLastFrame = mpCurrentFrame;
                LOG(INFO) << "Set into WEAK mode" << endl;
                // in this case, don't save current as last frame
            }
        } else if (mState == WEAK) {
            LOG(WARNING) << "Running WEAK mode" << endl;
            // use imu only to propagate the pose and try to initialize stereo vision
            PredictCurrentPose();   // ?????????????????????

            // first let's try to track the last (may be bit longer) frame
            bool bOK = false;
            bOK = TrackLastFrame(false);
            bOK = TrackLocalMap(mTrackInliersCnt);
            if (bOK) {
                // we successfully tracked the last frame and local map, set state back to OK
                mState = OK;
                // ???????????????
                if (NeedNewKeyFrame(mTrackInliersCnt))
                    InsertKeyFrame();
                mpLastFrame = mpCurrentFrame;
            } else {
                // track failed, try use current frame to init the stereo vision
                // grab the features and do stereo matching
                mpExtractor->Detect(mpCurrentFrame, true, false);
                mpMatcher->ComputeStereoMatches(mpCurrentFrame, ORBMatcher::OPTIFLOW_CV);
                mpBackEnd->Reset();
                CreateStereoMapPoints();
                CleanOldFeatures();
                InsertKeyFrame();
                mState = OK;
                mpLastFrame = mpCurrentFrame;
                LOG(INFO) << "Recovered from WEAK into NOT_INITIALIZAED." << endl;
            }
        }
    }

    bool TrackerLK::TrackLastFrame(bool usePoseInfo) {

        // Track the points in last frame and create new features in current
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        VecVector2f trackedPts, refPts;
        vector<size_t> idxRef;
        SE3d TCW = mpCurrentFrame->GetTCW();
        unique_lock<mutex> lock(mpLastFrame->mMutexFeature);

        for (size_t i = 0; i < mpLastFrame->mFeaturesLeft.size(); i++) {
            shared_ptr<Feature> feat = mpLastFrame->mFeaturesLeft[i];
            if (feat == nullptr) {
                continue;
            }

            idxRef.push_back(i);
            if (feat->mpPoint && feat->mpPoint->isBad() == false) {
                // associated with a good map point, predict the projected pixel in current
                refPts.push_back(feat->mPixel);
                // trackedPts.push_back(feat->mPixel);
                Vector2d px = mpCurrentFrame->World2Pixel(feat->mpPoint->GetWorldPos(), TCW);
                trackedPts.push_back(Vector2f(px[0], px[1]));
            } else {
                refPts.push_back(feat->mPixel);
                trackedPts.push_back(feat->mPixel);
            }
        }

        // ?????? LK ????????????2D?????????????????????2D??????????????????3D?????????
        int cntMatches = LKFlowCV(mpLastFrame, mpCurrentFrame, refPts, trackedPts);
        // int cntMatches = LKFlow(mpLastFrame, mpCurrentFrame, trackedPts);

        int validMatches = 0;

        for (size_t i = 0; i < trackedPts.size(); i++) {
            if (trackedPts[i][0] < 0 || trackedPts[i][1] < 0)
                continue;

            // create a feature assigned with this map point
            shared_ptr<Feature> feat(new Feature);
            feat->mPixel = trackedPts[i];
            feat->mpPoint = mpLastFrame->mFeaturesLeft[idxRef[i]]->mpPoint;
            mpCurrentFrame->mFeaturesLeft.push_back(feat);
            if (feat->mpPoint->Status() == MapPoint::GOOD) {
                validMatches++;
            }

        }

        LOG(INFO) << "Current features: " << mpCurrentFrame->mFeaturesLeft.size() << endl;

        if (validMatches <= setting::minTrackLastFrameFeatures) {
            LOG(WARNING) << "Track last frame not enough valid matches: " << validMatches << ", I will abort this frame"
                         << endl;
            return false;
        }

        // do pose optimization
        LOG(INFO) << "LK tracked points: " << cntMatches << ", valid: " << validMatches << ", last frame features: "
                  << mpLastFrame->mFeaturesLeft.size() << endl;

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        double timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();
        LOG(INFO) << "LK cost time: " << timeCost << ", pts: " << refPts.size() << endl;

        mTrackInliersCnt = OptimizeCurrentPose();
        // LOG(INFO) << "Track last frame inliers: " << inliers << endl;

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        LOG(INFO) << "Track last frame cost time: " << timeCost << endl;

        if (mTrackInliersCnt >= setting::minTrackLastFrameFeatures) {
            return true;
        } else {
            return false;
        }
    }

    bool TrackerLK::TrackLocalMap(int &inliers) {

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Step 1. ???Local Map?????????????????????
        set<shared_ptr<MapPoint>> localmap = mpBackEnd->GetLocalMap();

        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
        double timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t1).count();
        LOG(INFO) << "get local map points cost time: " << timeCost << endl;

        for (auto mp: localmap)
            if (mp)
                mp->mbTrackInView = false;

        for (auto feat: mpCurrentFrame->mFeaturesLeft)
            if (feat && feat->mpPoint && feat->mbOutlier == false)
                feat->mpPoint->mbTrackInView = true;

        // ????????????????????????
        set<shared_ptr<MapPoint> > mpsInView;
        for (auto &mp: localmap) {
            if (mp && mp->isBad() == false && mp->mbTrackInView == false && mpCurrentFrame->isInFrustum(mp, 0.5)) {
                mpsInView.insert(mp);
            }
        }

        if (mpsInView.empty())
            return inliers >= setting::minTrackLocalMapInliers;

        LOG(INFO) << "Call Search by direct projection" << endl;
        int cntMatches = mpMatcher->SearchByDirectProjection(mpCurrentFrame, mpsInView);
        LOG(INFO) << "Track local map matches: " << cntMatches << ", current features: "
                  << mpCurrentFrame->mFeaturesLeft.size() << endl;

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        LOG(INFO) << "Search local map points cost time: " << timeCost << endl;

        // Optimize Pose
        int optinliers = OptimizeCurrentPose();

        // Update MapPoints Statistics
        inliers = 0;
        for (shared_ptr<Feature> feat : mpCurrentFrame->mFeaturesLeft) {
            if (feat->mpPoint) {
                if (!feat->mbOutlier) {
                    feat->mpPoint->IncreaseFound();
                    if (feat->mpPoint->Status() == MapPoint::GOOD)
                        inliers++;
                } else {
                }
            }
        }

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();
        LOG(INFO) << "Track local map cost time: " << timeCost << endl;

        LOG(INFO) << "Track Local map inliers: " << inliers << endl;

        // Decide if the tracking was succesful
        if (inliers < setting::minTrackLocalMapInliers)
            return false;
        else
            return true;
    }

    void TrackerLK::Reset() {

        LOG(INFO) << "Tracker is reseted" << endl;
        mpCurrentFrame->SetPose(mpLastFrame->GetPose());
        LOG(INFO) << "Current pose = \n" << mpCurrentFrame->GetPose().matrix() << endl;
        mpBackEnd->Reset();

        // test if we can just recover from stereo
        mpCurrentFrame->mFeaturesLeft.clear();

        LOG(INFO) << "Try init stereo" << endl;
        mpExtractor->Detect(mpCurrentFrame, true, false);    // extract the keypoint in left eye
        mpMatcher->ComputeStereoMatches(mpCurrentFrame, ORBMatcher::OPTIFLOW_BASED);

        if (StereoInitialization() == false) {
            LOG(INFO) << "Stereo init failed." << endl;
            // ?????????????????????????????????
            mState = NOT_INITIALIZED;
            return;
        } else {
            LOG(INFO) << "Stereo init succeed." << endl;
            // set the current as a new kf and track it
            InsertKeyFrame();

            mpLastFrame = mpCurrentFrame;
            mpLastKeyFrame = mpCurrentFrame;
        }
        return;
    }

    void TrackerLK::CreateStereoMapPoints() {

        // ?????????????????????????????????????????????
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        mpCurrentFrame->AssignFeaturesToGrid();
        mpExtractor->Detect(mpCurrentFrame, true, false);    // extract new keypoints in left eye

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        double timeCost2 = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t1).count();
        LOG(INFO) << "Detect new feature cost time: " << timeCost2 << endl;

        // ???????????????????????????????????????????????????
        mpMatcher->ComputeStereoMatchesOptiFlow(mpCurrentFrame, true);

        float meanInvDepth = 1.0 / (mpCurrentFrame->ComputeSceneMedianDepth(2) + 1e-9);

        int cntMono = 0, cntStereo = 0, cntUpdate = 0;
        // ??????????????????
        for (size_t i = 0; i < mpCurrentFrame->mFeaturesLeft.size(); i++) {
            shared_ptr<Feature> feat = mpCurrentFrame->mFeaturesLeft[i];
            if (feat == nullptr)
                continue;
            if (feat->mpPoint == nullptr) {
                // ?????????????????????
                if (feat->mfInvDepth > setting::minNewMapPointInvD && feat->mfInvDepth < setting::maxNewMapPointInvD) {
                    // ??????????????????
                    // we create a map point here
                    shared_ptr<MapPoint> mp(new MapPoint(mpCurrentFrame, i));   // ??????????????????
                    feat->mpPoint = mp;
                    cntStereo++;
                } else {
                    // ?????????????????????????????????????????????
                    feat->mfInvDepth = meanInvDepth;
                    shared_ptr<MapPoint> mp(new MapPoint(mpCurrentFrame, i));   // ??????????????????
                    feat->mpPoint = mp;
                    mp->SetStatus(MapPoint::IMMATURE);
                    cntMono++;
                }
            } else {
                // ????????????????????????
                if (feat->mpPoint->Status() == MapPoint::IMMATURE) {
                    if (feat->mpPoint->mpRefKF.expired() ||
                        (feat->mpPoint->mpRefKF.expired() == false &&
                         feat->mpPoint->mpRefKF.lock()->mbIsKeyFrame == false))
                        feat->mpPoint->mpRefKF = mpCurrentFrame;    // change its reference

                    if (feat->mfInvDepth > setting::minNewMapPointInvD &&
                        feat->mfInvDepth < setting::maxNewMapPointInvD) {
                        // ???????????????????????????????????????????????????????????????????????????
                        feat->mpPoint->SetStatus(MapPoint::GOOD);
                        Vector3d ptFrame =
                                mpCurrentFrame->mpCam->Img2Cam(feat->mPixel) * (1.0 / double(feat->mfInvDepth));
                        feat->mpPoint->SetWorldPos(mpCurrentFrame->mRwc * ptFrame + mpCurrentFrame->mOw);
                        cntUpdate++;
                    } else {
                        // ????????????????????????invDepth????????????????????????????????????????????????????????????????????????IMMATURE
                        Vector3d pw = feat->mpPoint->mWorldPos;
                        feat->mfInvDepth = 1.0 / (mpCurrentFrame->mRcw * pw + mpCurrentFrame->mtcw)[2];
                    }
                }
            }
        }

        LOG(INFO) << "new stereo: " << cntStereo << ", new Mono: " << cntMono << ", update immature: " << cntUpdate
                  << ", total features: " << mpCurrentFrame->mFeaturesLeft.size() << endl;

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        LOG(INFO) << "Create map point cost time: " << timeCost << endl;
    }

    void TrackerLK::CleanOldFeatures() {

        LOG(INFO) << "Cleaning old features" << endl;
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        int cntPassed = 0;
        int cntOutlier = 0;
        for (shared_ptr<Feature> &feat: mpCurrentFrame->mFeaturesLeft) {
            if (feat->mbOutlier) {
                feat = nullptr; // ??????????????????
                // ????????????????????????vector????????????????????????????????????????????????????????????
                cntOutlier++;
                continue;
            }

            shared_ptr<MapPoint> mp = feat->mpPoint;
            auto status = mp->Status();
            if (status == MapPoint::BAD) {
                // ?????????????????????????????????
                feat->mpPoint = nullptr;
                feat = nullptr;
            } else if (status == MapPoint::GOOD) {
                // ???????????????
                if (mp->mpRefKF.expired() || mp->mpRefKF.lock()->IsKeyFrame() == false) {
                    // ?????????????????????????????????
                    mp->mpRefKF = mpCurrentFrame; // ????????????
                    cntPassed++;
                }
            } else {
                // ?????????????????????
                if (mp->mpRefKF.expired() || mp->mpRefKF.lock()->IsKeyFrame() == false) {
                    // ?????????????????????????????????
                    mp->mpRefKF = mpCurrentFrame; // ????????????
                    cntPassed++;
                }
            }
        }
        LOG(INFO) << "passed " << cntPassed << " features into current, delete "
                  << cntOutlier << " outliers." << endl;

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double timeCost = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        LOG(INFO) << "Clean old features cost time: " << timeCost << endl;

    }


}
