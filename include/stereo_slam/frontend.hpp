#pragma once

// opencv
#include <opencv2/features2d.hpp>

// sophus
#include <sophus/se3.hpp>

// local header
#include "stereo_slam/frame.hpp"
#include "stereo_slam/map.hpp"
#include "stereo_slam/backend.hpp"

//class Viewer;

enum class FrontendStatus
{
  INITING,
  TRACKING_GOOD,
  TRACKING_BAD,
  LOST
};

class Frontend
{
  public:
    using Ptr = std::shared_ptr<Frontend>;
    Frontend();

    bool Addframe(Frame::Ptr frame);

    void SetBackend(Backend::Ptr backend) { backend_ = backend; }

    void SetMap(Map::Ptr map) { map_ = map; }

    //void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

    FrontendStatus GetStatus() const;

    void SetCameras(Camera::Ptr left, Camera::Ptr right);

  private:
    /**
     * Try init the frontend with stereo images saved in current_frame_
     * @return true if success
     */
    bool StereoInit();

    /**
     * Track in normal mode
     * @return true if success
     */
    bool Track();

    /**
     * Reset when lost
     * @return true if success
     */
    bool Reset();

    /**
     * Track with last frame
     * @return num of tracked points
     */
    int TrackLastFrame();

    /**
     * estimate current frame's pose
     * @return num of inliers
     */
    int EstimateCurrentPose();

    /**
     * Detect features in left image in current_frame_
     * keypoints will be saved in current_frame_
     * @return
     */
    int DetectFeatures();

    /**
     * Find the corresponding features in right image of current_frame_
     * @return num of features found
     */
    int FindFeaturesInRight();

    /**
     * Build the initial map with single image
     * @return true if succeed
     */
    bool BuildInitMap();

    /**
     * set current frame as a keyframe and insert it into backend
     * @return true if success
     */
    bool InsertKeyframe();

    /**
     * Set the features in keyframe as new observation of the map points
     */
    void SetObservationsForKeyFrame();

    /**
     * Triangulate the 2D points in current frame
     * @return num of triangulated points
     */
    int TriangulateNewPoints();

    // data
    FrontendStatus status_ = FrontendStatus::INITING;

    Frame::Ptr current_frame_ = nullptr;
    Frame::Ptr last_frame_ = nullptr;
    Camera::Ptr camera_left_ = nullptr;
    Camera::Ptr camera_right_ = nullptr;

    Map::Ptr map_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    //Viewer::Ptr viewer_ = nullptr;

    Sophus::SE3d relative_motion_;  // relative motion between current and last frame used for current pose estimation

    int tracking_inliers_ = 0;  // inliers, used for testing new keyframes

    // params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt_;  // feature detector in opencv
};
