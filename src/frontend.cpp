#include <iostream>
#include <vector>

// opencv header
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>

// g2o header
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

// local header
#include "stereo_slam/algorithm.hpp"
#include "stereo_slam/frontend.hpp"
#include "stereo_slam/feature.hpp"
#include "stereo_slam/mappoint.hpp"
#include "stereo_slam/g2o_types.hpp"


Frontend::Frontend(double num_features, double num_features_init)
{
  gftt_ = cv::GFTTDetector::create(num_features, 0.01, 20);
  num_features_ = num_features;
  num_features_init_ = num_features_init;
}

bool Frontend::Addframe(Frame::Ptr frame)
{
  current_frame_ = frame;

  switch (status_) {
    case FrontendStatus::INITING:
      StereoInit();
      break;
    case FrontendStatus::TRACKING_GOOD:
    case FrontendStatus::TRACKING_BAD:
      Track();
      break;
    case FrontendStatus::LOST:
    default:
      Reset();
      break;
  }

  last_frame_ = current_frame_;
  return true;
}

FrontendStatus Frontend::GetStatus() const
{
  return status_;
}

void Frontend::SetCameras(Camera::Ptr left, Camera::Ptr right)
{
  camera_left_ = left;
  camera_right_ = right;
}

bool Frontend::StereoInit()
{
  int num_features_left = DetectFeatures();
  int num_coor_features = FindFeaturesInRight();
  if (num_coor_features < num_features_init_) {
    return false;
  }

  bool build_map_success = BuildInitMap();
  if (build_map_success) {
    status_ = FrontendStatus::TRACKING_GOOD;
    if (viewer_) {
     viewer_->AddCurrentFrame(current_frame_);
     viewer_->UpdateMap();
    }
    return true;
  }
  return false;
}

bool Frontend::Track()
{
  if (last_frame_) {
    current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
  }

  int num_track_last = TrackLastFrame();
  tracking_inliers_ = EstimateCurrentPose();

  if (tracking_inliers_ > num_features_tracking_) {
    // tracking good
    status_ = FrontendStatus::TRACKING_GOOD;
  } else if (tracking_inliers_ > num_features_tracking_bad_) {
    // tracking bad
    status_ = FrontendStatus::TRACKING_BAD;
  } else {
    // lost
    status_ = FrontendStatus::LOST;
  }

  InsertKeyframe();
  relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

  if (viewer_) {
   viewer_->AddCurrentFrame(current_frame_);
  }
  return true;
}

bool Frontend::Reset()
{
  std::cout << "Reset the VO. Clean-up Map and Viewer. Re-init Frontend." << std::endl;

  // Clean-up Map and Viewer. Nothing to clean for the Backend.
  map_->Reset();
  viewer_->Reset();

  // Re-init Frontend. Camera setting stays the same.
  status_ = FrontendStatus::INITING;
  current_frame_ = nullptr;
  last_frame_ = nullptr;

  return true;
}

int Frontend::TrackLastFrame()
{
  // use LK flow to estimate points in the right image
  std::vector<cv::Point2f> kps_last, kps_current;
  for (auto &kp : last_frame_->features_left_) {
    if (kp->map_point_.lock()) {
      // use project point
      auto mp = kp->map_point_.lock();
      auto px = camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
      kps_last.push_back(kp->position_.pt);
      kps_current.push_back(cv::Point2f(px[0], px[1]));
    } else {
      kps_last.push_back(kp->position_.pt);
      kps_current.push_back(kp->position_.pt);
    }
  }

  std::vector<uchar> status;
  cv::Mat error;
  cv::calcOpticalFlowPyrLK(
    last_frame_->left_img_, current_frame_->left_img_, kps_last,
    kps_current, status, error, cv::Size(11, 11), 3,
    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
    cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;

  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      cv::KeyPoint kp(kps_current[i], 7);
      Feature::Ptr feature(new Feature(current_frame_, kp));
      feature->map_point_ = last_frame_->features_left_[i]->map_point_;
      current_frame_->features_left_.push_back(feature);
      num_good_pts++;
    }
  }

  std::cout << "Find " << num_good_pts << " in the last image." << std::endl;
  return num_good_pts;
}

int Frontend::EstimateCurrentPose()
{
  // setup g2o
  using BlockSolverType = g2o::BlockSolver_6_3;
  using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
    std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  // vertex
  VertexPose* vertex_pose = new VertexPose();  // camera vertex_pose
  vertex_pose->setId(0);
  vertex_pose->setEstimate(current_frame_->Pose());
  optimizer.addVertex(vertex_pose);

  // K
  Mat33 K = camera_left_->K();

  // edges
  int index = 1;
  std::vector<EdgeProjectionPoseOnly*> edges;
  std::vector<Feature::Ptr> features;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    auto mp = current_frame_->features_left_[i]->map_point_.lock();
    if (mp) {
      features.push_back(current_frame_->features_left_[i]);
      EdgeProjectionPoseOnly* edge = new EdgeProjectionPoseOnly(mp->pos_, K);
      edge->setId(index);
      edge->setVertex(0, vertex_pose);
      edge->setMeasurement(toVec2(current_frame_->features_left_[i]->position_.pt));
      edge->setInformation(Eigen::Matrix2d::Identity());
      edge->setRobustKernel(new g2o::RobustKernelHuber);
      edges.push_back(edge);
      optimizer.addEdge(edge);
      index++;
    }
  }

  // estimate the Pose the determine the outliers
  const double chi2_th = 5.991;
  int cnt_outlier = 0;
  for (int iteration = 0; iteration < 4; ++iteration) {
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cnt_outlier = 0;

    // count the outliers
    for (size_t i = 0; i < edges.size(); ++i) {
      auto e = edges[i];
      if (features[i]->is_outlier_) {
        e->computeError();
      }
      if (e->chi2() > chi2_th) {
        features[i]->is_outlier_ = true;
        e->setLevel(1);
        cnt_outlier++;
      } else {
        features[i]->is_outlier_ = false;
        e->setLevel(0);
      };

      if (iteration == 2) {
        e->setRobustKernel(nullptr);
      }
    }
  }

  std::cout << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/" << features.size() - cnt_outlier << std::endl;
  // Set pose and outlier
  current_frame_->SetPose(vertex_pose->estimate());

  std::cout << "Current Pose = \n" << current_frame_->Pose().matrix() << std::endl;

  for (auto& feat: features) {
    if (feat->is_outlier_) {
      feat->map_point_.reset();
      feat->is_outlier_ = false;  // maybe we can still use it in future
    }
  }
  return features.size() - cnt_outlier;
}

bool Frontend::InsertKeyframe()
{
  if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
    // still have enough features, don't insert keyframe
    return false;
  }
  // current frame is a new keyframe
  current_frame_->SetKeyFrame();
  map_->InsertKeyFrame(current_frame_);

  std::cout << "Set frame " << current_frame_->id_ << " as keyframe " << current_frame_->keyframe_id_ << std::endl;

  SetObservationsForKeyFrame();
  DetectFeatures();  // detect new features

  // track in right image
  FindFeaturesInRight();
  // triangulate map points
  TriangulateNewPoints();
  // update backend because we have a new keyframe
  backend_->UpdateMap();

  if (viewer_) viewer_->UpdateMap();

  return true;
}

void Frontend::SetObservationsForKeyFrame()
{
  for (auto& feat: current_frame_->features_left_) {
    auto mp = feat->map_point_.lock();
    if (mp) {
      mp->AddObservation(feat);
    }
  }
}

int Frontend::TriangulateNewPoints()
{
  std::vector<Sophus::SE3d> poses{camera_left_->pose(), camera_right_->pose()};
  Sophus::SE3d current_pose_Twc = current_frame_->Pose().inverse();
  int cnt_triangulated_pts = 0;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    if (current_frame_->features_left_[i]->map_point_.expired() &&
      current_frame_->features_right_[i] != nullptr) {
      // The feature points of the left map are not associated with map points
      // and there are right map matches, try triangulation
      std::vector<Vec3> points{
        camera_left_->pixel2camera(
          Vec2(current_frame_->features_left_[i]->position_.pt.x,
               current_frame_->features_left_[i]->position_.pt.y)),
        camera_right_->pixel2camera(
          Vec2(current_frame_->features_right_[i]->position_.pt.x,
               current_frame_->features_right_[i]->position_.pt.y))};
      Vec3 pworld = Vec3::Zero();

      if (triangulation(poses, points, pworld) && pworld[2] > 0) {
        auto new_map_point = MapPoint::CreateNewMappoint();
        pworld = current_pose_Twc * pworld;
        new_map_point->SetPos(pworld);
        new_map_point->AddObservation(current_frame_->features_left_[i]);
        new_map_point->AddObservation(current_frame_->features_right_[i]);

        current_frame_->features_left_[i]->map_point_ = new_map_point;
        current_frame_->features_right_[i]->map_point_ = new_map_point;
        map_->InsertMapPoint(new_map_point);
        cnt_triangulated_pts++;
      }
    }
  }
  std::cout << "new landmarks: " << cnt_triangulated_pts << std::endl;
  return cnt_triangulated_pts;
}

int Frontend::DetectFeatures()
{
  cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
  for (auto& feat: current_frame_->features_left_) {
    cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
      feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
  }

  std::vector<cv::KeyPoint> keypoints;
  gftt_->detect(current_frame_->left_img_, keypoints, mask);
  int cnt_detected = 0;
  for (auto& kp: keypoints) {
    current_frame_->features_left_.push_back(
      Feature::Ptr(new Feature(current_frame_, kp)));
    ++cnt_detected;
  }

  std::cout << "Detect " << cnt_detected << " new features" << std::endl;
  return cnt_detected;
}

int Frontend::FindFeaturesInRight()
{
  // use LK flow to estimate points in the right image
  std::vector<cv::Point2f> kps_left, kps_right;
  for (auto& kp: current_frame_->features_left_) {
    kps_left.push_back(kp->position_.pt);
    auto mp = kp->map_point_.lock();
    if (mp) {
      // use projected points as initial guess
      auto px = camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
      kps_right.push_back(cv::Point2f(px[0], px[1]));
    } else {
      // use same pixel in left iamge
      kps_right.push_back(kp->position_.pt);
    }
  }

  std::vector<uchar> status;
  cv::Mat error;
  cv::calcOpticalFlowPyrLK(
    current_frame_->left_img_, current_frame_->right_img_, kps_left,
    kps_right, status, error, cv::Size(11, 11), 3,
    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
    cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      cv::KeyPoint kp(kps_right[i], 7);
      Feature::Ptr feat(new Feature(current_frame_, kp));
      feat->is_on_left_image_ = false;
      current_frame_->features_right_.push_back(feat);
      num_good_pts++;
    } else {
      current_frame_->features_right_.push_back(nullptr);
    }
  }
  std::cout << "Find " << num_good_pts << " in the right image." << std::endl;
  return num_good_pts;
}

bool Frontend::BuildInitMap()
{
  std::vector<Sophus::SE3d> poses{camera_left_->pose(), camera_right_->pose()};
  size_t cnt_init_landmarks = 0;
  for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
    if (current_frame_->features_right_[i] == nullptr) {
      continue;
    }
    // create map point from triangulation
    std::vector<Vec3> points{
      camera_left_->pixel2camera(
        Vec2(current_frame_->features_left_[i]->position_.pt.x,
             current_frame_->features_left_[i]->position_.pt.y)),
      camera_right_->pixel2camera(
        Vec2(current_frame_->features_right_[i]->position_.pt.x,
             current_frame_->features_right_[i]->position_.pt.y))};
    Vec3 pworld = Vec3::Zero();

    if (triangulation(poses, points, pworld) && pworld[2] > 0) {
      auto new_map_point = MapPoint::CreateNewMappoint();
      new_map_point->SetPos(pworld);
      new_map_point->AddObservation(current_frame_->features_left_[i]);
      new_map_point->AddObservation(current_frame_->features_right_[i]);
      current_frame_->features_left_[i]->map_point_ = new_map_point;
      current_frame_->features_right_[i]->map_point_ = new_map_point;
      cnt_init_landmarks++;
      map_->InsertMapPoint(new_map_point);
    }
  }
  current_frame_->SetKeyFrame();
  map_->InsertKeyFrame(current_frame_);
  backend_->UpdateMap();

  std::cout << "Initial map created with " << cnt_init_landmarks << " map points" << std::endl;

  return true;
}
