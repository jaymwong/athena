#ifndef ATHENA_POINTCLOUD_REGISTRATION_H
#define ATHENA_POINTCLOUD_REGISTRATION_H

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>

#define kDefaultMaximumIterations 100
#define kInfinity 9999999

struct RegistrationResult{
  Eigen::Matrix4d transform;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

struct CorrespondenceScoreResult{
  // Scene to model and model to scene correspondences
  double s_to_m, m_to_s;
};

class RegistrationOptions{
  public:
    int max_iterations;
    bool global_init, strict_update;
    double correspondence_thresh;
    RegistrationOptions(){
      max_iterations = kDefaultMaximumIterations;

      global_init = false;
      strict_update = false;
      correspondence_thresh = 0.005;
    }
    ~RegistrationOptions(){}
};


namespace athena{
  namespace pointcloud{
    RegistrationResult performTemplateBasedRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, RegistrationOptions options=RegistrationOptions());
    RegistrationResult performPointBasedIterativeRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene);

    int findNumCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, double thresh);
    CorrespondenceScoreResult computeCorrespondenceScore(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, double thresh);
  };

  class Registrator{
    private:
      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_model_cloud;
    public:
      pcl::PointCloud<pcl::PointXYZ>::Ptr model, scene, result_cloud;
      RegistrationOptions options;
      bool has_global_init;
      Eigen::Matrix4d transform;
      double max_score;

      Registrator(pcl::PointCloud<pcl::PointXYZ>::Ptr model, RegistrationOptions options=RegistrationOptions()){
        this->model = model;
        this->options = options;
        transform = Eigen::Matrix4d::Identity();
        has_global_init = false || !options.global_init;
        max_score = -kInfinity;
        result_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      }
      ~Registrator(){}

      RegistrationResult performRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud){
        scene = scene_cloud;
        RegistrationResult result;
        if (!has_global_init){ // First run template-based registration to identify an initial alignment
          auto reg_result = athena::pointcloud::performTemplateBasedRegistration(model, scene);
          auto score = athena::pointcloud::computeCorrespondenceScore(scene, reg_result.cloud, options.correspondence_thresh);

          if (score.s_to_m + score.m_to_s > max_score || !options.strict_update){
            result_cloud = reg_result.cloud;
            transform = reg_result.transform;
            max_score = score.s_to_m + score.m_to_s;
            has_global_init = true;
          }
        }
        else{ // Then using that seed, run ICP to refine the registration and only update when the result is "better"
          transformed_model_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
          pcl::transformPointCloud(*model, *transformed_model_cloud, transform);
          auto reg_result = athena::pointcloud::performPointBasedIterativeRegistration(transformed_model_cloud, scene);
          auto score = athena::pointcloud::computeCorrespondenceScore(scene, reg_result.cloud, options.correspondence_thresh);
          if (score.s_to_m + score.m_to_s > max_score || !options.strict_update){
            result_cloud = reg_result.cloud;
            transform = reg_result.transform * transform;
            max_score = score.s_to_m + score.m_to_s;
          }
        }
        result.cloud = result_cloud;
        result.transform = transform;
        return result;
      }
  };
};

// The following was adapted from the template alignment tutorial:
// from https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/sources/template_alignment/template_alignment.cpp

class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (100)
    {
      // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }
    void setMaximumIterations(int iterations){
      nr_iterations_ = iterations;
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputSource (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

#endif
