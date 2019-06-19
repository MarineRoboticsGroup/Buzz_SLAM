#ifndef BUZZ_CONTROLLER_QUADMAPPER_NO_SENSING_H
#define BUZZ_CONTROLLER_QUADMAPPER_NO_SENSING_H

#include <argos/buzz_controller_quadmapper.h>
#include <map>

using namespace argos;
namespace buzz_quadmapper {

/*
*  Buzz controller to support 3D robust distributed pose graph optimization.
*  This version is tighly coupled with the argos3 simulation without sensing.
*  The fake measurements are generated by adding gaussian noise to the ground truth provided by the simulator.
*/
class CBuzzControllerQuadMapperNoSensing : public CBuzzControllerQuadMapper {

public:

   CBuzzControllerQuadMapperNoSensing();
   
   virtual ~CBuzzControllerQuadMapperNoSensing();

   virtual void Init(TConfigurationNode& t_node);

   // Control functions
   int MoveForwardFakeOdometry(const CVector3& distance, const int& simulation_time_divider);

   // Fake measurements generation
   int ComputeNoisyFakeSeparatorMeasurement(const CQuaternion& gt_orientation, const CVector3& gt_translation, 
                                          const int& pose_id, const int& robot_id, const int& this_robot_pose_id);

   void LoadParameters(const double& sensor_range, const double& outlier_probability);

private:

   // Fake measurements generation
   void ComputeNoisyFakeOdometryMeasurement();

   gtsam::Pose3 AddGaussianNoiseToMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t);

   gtsam::Pose3 OutlierMeasurement(const gtsam::Rot3& R, const gtsam::Point3& t);

   void SavePoseGroundTruth();

protected:

   // Functions for link with buzz VM
   virtual buzzvm_state RegisterFunctions();

   virtual bool CompareCentralizedAndDecentralizedError();

   void ComputeCentralizedEstimate(const std::string& centralized_extension);

   void ComputeCentralizedEstimateIncremental(std::set<int> robots, const std::string& centralized_extension);

   virtual void WriteInitialDataset();

   virtual void WriteOptimizedDataset();

   std::set<std::pair<gtsam::Key, gtsam::Key>> AggregateOutliersKeys(const std::set<int>& robots);

   std::pair<int, int> CountInliersAndOutliers(const std::set<int>& robots);

   virtual void SaveRejectedKeys(const std::set<std::pair<gtsam::Key, gtsam::Key>>& rejected_keys);

   void RemoveRejectedKeys();

   virtual void AbortOptimization(const bool& log_info);

private:

   // Ground truth information to compute fake measurements
   std::map<int, gtsam::Pose3> ground_truth_poses_;
   boost::shared_ptr<gtsam::Values> ground_truth_data_;
   argos::CCI_PositioningSensor::SReading previous_simulation_gt_pose_;

   // Random numbers generation
   std::random_device rd_{};
   std::mt19937 gen_translation_, gen_rotation_, gen_outliers_;
   std::normal_distribution<> normal_distribution_translation_, normal_distribution_rotation_;
   std::uniform_real_distribution<> uniform_distribution_outliers_translation_, 
                                    uniform_distribution_outliers_rotation_,
                                    uniform_distribution_draw_outlier_;

   // Current state of the simulation
   int simulation_step_;
   int number_of_outliers_added_;
   int number_of_inliers_added_;
   double outlier_probability_;
   double sensor_range_;
   std::set<std::pair<gtsam::Key, gtsam::Key>> outliers_keys_;
   std::set<std::pair<gtsam::Key, gtsam::Key>> inliers_keys_;
   std::set<std::pair<gtsam::Key, gtsam::Key>> rejected_keys_;

};
}
#endif
