/*
 Copywrite 2017. All rights reserved.

 Author: Dong-Ki Kim
 Contact: dkkim93@mit.edu

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "../include/elas_autel.hpp"

bool initial = true;

class Elas_Proc {
  public:
    Elas_Proc(const std::string& transport) {
      if (initial) {
        ros::NodeHandle local_nh("~");
        local_nh.param("queue_size", queue_size_, 5);
  
        // Subscribe topic setup
        image_transport::ImageTransport it(nh);
        std::string stereo_ns = nh.resolveName("narrow_stereo");
        std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image_rect")); left_sub_.subscribe(it, left_topic, 1, transport);
        std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh.resolveName("image_rect")); right_sub_.subscribe(it, right_topic, 1, transport);
        std::string left_info_topic = stereo_ns + "/left/camera_info"; left_info_sub_.subscribe(nh, left_info_topic, 1);
        std::string right_info_topic = stereo_ns + "/right/camera_info"; right_info_sub_.subscribe(nh, right_info_topic, 1);
        ROS_INFO("Subscribing to:\n%s\n%s\n%s\n%s", left_topic.c_str(), right_topic.c_str(), left_info_topic.c_str(), right_info_topic.c_str());

        // Pusblish topic setup
        image_transport::ImageTransport local_it(local_nh);
        disp_pub_.reset(new Publisher(local_it.advertise("image_disparity", 1)));
        depth_pub_.reset(new Publisher(local_it.advertise("depth", 1)));
        pc_pub_.reset(new ros::Publisher(local_nh.advertise<PointCloud>("point_cloud", 1)));
        elas_fd_pub_.reset(new ros::Publisher(local_nh.advertise<elas_ros::ElasFrameData>("frame_data", 1)));
        pub_disparity_ = local_nh.advertise<stereo_msgs::DisparityImage>("disparity", 1);

        // Create the elas processing class
        elas_parameter_setup();

        // Synchronize input topics. Optionally do approximate synchronization.
        bool approx; local_nh.param("approximate_sync", approx, false);
        if (approx) {
          std::cout << "[ INFO ] Sync policy: approximate sync" << std::endl;
          approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_), left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
          approximate_sync_->registerCallback(boost::bind(&Elas_Proc::process, this, _1, _2, _3, _4));
        }
        else {
          std::cout << "[ INFO ] Sync policy: exact sync" << std::endl;
          exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_), left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
          exact_sync_->registerCallback(boost::bind(&Elas_Proc::process, this, _1, _2, _3, _4));
        }

        initial = false;
      }

      // Reconfigure callback
      reconfigure_callback_f = boost::bind(&Elas_Proc::reconfigure_callback, this, _1, _2);
      reconfigure_server.setCallback(reconfigure_callback_f);
    }

    void reconfigure_callback(elas_ros::reconfigureConfig &config, uint32_t level);

    void elas_parameter_setup() {
      std::cout << "[ INFO ] Setting up ELAS parameters ..." << std::endl;

      param.reset(new Elas::parameters);
      param->disp_min              = disp_min;
      param->disp_max              = disp_max;
      param->support_threshold     = support_th;
      param->support_texture       = support_texture;
      param->candidate_stepsize    = candidate_stepsize;
      param->incon_window_size     = incon_window_size;
      param->incon_threshold       = incon_th;
      param->incon_min_support     = incon_min_support;
      param->add_corners           = add_corners;
      param->grid_size             = grid_size;
      param->beta                  = beta;
      param->gamma                 = gamma;
      param->sigma                 = sigma;
      param->sradius               = sradius;
      param->match_texture         = match_texture;
      param->lr_threshold          = lr_th;
      param->speckle_sim_threshold = speckle_sim_th;
      param->speckle_size          = speckle_size;
      param->ipol_gap_width        = ipol_gap_width;
      param->filter_median         = filter_median;
      param->filter_adaptive_mean  = filter_adaptive_mean;
      param->postprocess_only_left = postprocess_only_left;
      param->subsampling           = subsampling;

      std::cout << " ======= [ ELAS PARAM ] ======= " << std::endl;
      std::cout << "disp_min             : " << param->disp_min << std::endl;
      std::cout << "disp_max             : " << param->disp_max << std::endl;
      std::cout << "support_th           : " << param->support_threshold << std::endl;
      std::cout << "support_texture      : " << param->support_texture << std::endl;
      std::cout << "candidate_stepsize   : " << param->candidate_stepsize << std::endl;
      std::cout << "incon_window_size    : " << param->incon_window_size << std::endl;
      std::cout << "incon_threshold      : " << param->incon_threshold << std::endl;
      std::cout << "incon_min_support    : " << param->incon_min_support << std::endl;
      std::cout << "add_corners          : " << param->add_corners << std::endl;
      std::cout << "grid_size            : " << param->grid_size << std::endl;
      std::cout << "beta                 : " << param->beta << std::endl;
      std::cout << "gamma                : " << param->gamma << std::endl;
      std::cout << "sigma                : " << param->sigma << std::endl;
      std::cout << "sradius              : " << param->sradius << std::endl;
      std::cout << "match_texture        : " << param->match_texture << std::endl;
      std::cout << "lr_threshold         : " << param->lr_threshold << std::endl;
      std::cout << "speckle_sim_threshold: " << param->speckle_sim_threshold << std::endl;
      std::cout << "speckle_size         : " << param->speckle_size << std::endl;
      std::cout << "ipol_gap_width       : " << param->ipol_gap_width << std::endl;
      std::cout << "filter_median        : " << param->filter_median << std::endl;
      std::cout << "filter_adaptive_mean : " << param->filter_adaptive_mean << std::endl;
      std::cout << "postprocess_only_left: " << param->postprocess_only_left << std::endl;
      std::cout << "subsampling          : " << param->subsampling << std::endl;
      std::cout << " ============================== " << std::endl;
  
      elas_.reset(new Elas(*param));
    }

    void process(const sensor_msgs::ImageConstPtr& l_image_msg, const sensor_msgs::ImageConstPtr& r_image_msg,
                 const sensor_msgs::CameraInfoConstPtr& l_info_msg, const sensor_msgs::CameraInfoConstPtr& r_info_msg) {
      // Update the camera model
      model_.fromCameraInfo(l_info_msg, r_info_msg);
  
      // Allocate new disparity image message
      stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage>();
      disp_msg->header         = l_info_msg->header;
      disp_msg->image.header   = l_info_msg->header;
      disp_msg->image.height   = l_image_msg->height;
      disp_msg->image.width    = l_image_msg->width;
      disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      disp_msg->image.step     = disp_msg->image.width * sizeof(float);
      disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);
      disp_msg->min_disparity = param->disp_min;
      disp_msg->max_disparity = param->disp_max;
  
      // Stereo parameters
      float f = model_.right().fx();
      float T = model_.baseline();
      float depth_fact = T*f*1000.0f;
      uint16_t bad_point = std::numeric_limits<uint16_t>::max();
  
      // Have a synchronised pair of images, now to process using elas
      // Convert images if necessary
      uint8_t *l_image_data, *r_image_data;
      int32_t l_step, r_step;
      cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
      if (l_image_msg->encoding == sensor_msgs::image_encodings::MONO8) {
        l_image_data = const_cast<uint8_t*>(&(l_image_msg->data[0]));
        l_step = l_image_msg->step;
      }
      else {
        l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
        l_image_data = l_cv_ptr->image.data;
        l_step = l_cv_ptr->image.step[0];
      }
      if (r_image_msg->encoding == sensor_msgs::image_encodings::MONO8) {
        r_image_data = const_cast<uint8_t*>(&(r_image_msg->data[0]));
        r_step = r_image_msg->step;
      }
      else {
        r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
        r_image_data = r_cv_ptr->image.data;
        r_step = r_cv_ptr->image.step[0];
      }
      ROS_ASSERT(l_step == r_step);
      ROS_ASSERT(l_image_msg->width == r_image_msg->width);
      ROS_ASSERT(l_image_msg->height == r_image_msg->height);
  
      int32_t width = l_image_msg->width;
      int32_t height = l_image_msg->height;
  
      // Allocate
      const int32_t dims[3] = {l_image_msg->width, l_image_msg->height, l_step};
      float* l_disp_data = reinterpret_cast<float*>(&disp_msg->image.data[0]);
      float* r_disp_data = new float[width*height*sizeof(float)];
  
      // Process
      elas_->process(l_image_data, r_image_data, l_disp_data, r_disp_data, dims);
  
      // Find the max for scaling the image colour
      float disp_max = 0;
      for (int32_t i=0; i<width*height; i++) {
        if (l_disp_data[i]>disp_max) disp_max = l_disp_data[i];
        if (r_disp_data[i]>disp_max) disp_max = r_disp_data[i];
      }
  
      cv_bridge::CvImage out_depth_msg;
      out_depth_msg.header = l_image_msg->header;
      out_depth_msg.encoding = sensor_msgs::image_encodings::MONO16;
      out_depth_msg.image = cv::Mat(height, width, CV_16UC1);
      uint16_t * out_depth_msg_image_data = reinterpret_cast<uint16_t*>(&out_depth_msg.image.data[0]);
  
      cv_bridge::CvImage out_msg;
      out_msg.header = l_image_msg->header;
      out_msg.encoding = sensor_msgs::image_encodings::MONO8;
      out_msg.image = cv::Mat(height, width, CV_8UC1);
      std::vector<int32_t> inliers;
      for (int32_t i=0; i<width*height; i++) {
        out_msg.image.data[i] = (uint8_t)std::max(255.0*l_disp_data[i]/disp_max,0.0);
        float disp =  l_disp_data[i];

        // In milimeters
        out_depth_msg_image_data[i] = disp <= 0.0f ? bad_point : (uint16_t)(depth_fact/disp);
        if (l_disp_data[i] > 0) inliers.push_back(i);
      }
  
      // Publish
      std::cout << "[ STATUS ] Publishing ... " << std::endl;
      disp_pub_->publish(out_msg.toImageMsg());
      depth_pub_->publish(out_depth_msg.toImageMsg());
      publish_point_cloud(l_image_msg, l_disp_data, inliers, width, height, l_info_msg, r_info_msg);
      pub_disparity_.publish(disp_msg);
  
      // Clean-up
      delete r_disp_data;
    }
  
    void publish_point_cloud(const sensor_msgs::ImageConstPtr& l_image_msg, 
                             float* l_disp_data, const std::vector<int32_t>& inliers,
                             int32_t l_width, int32_t l_height,
                             const sensor_msgs::CameraInfoConstPtr& l_info_msg, 
                             const sensor_msgs::CameraInfoConstPtr& r_info_msg) {
      try {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
        image_geometry::StereoCameraModel model;
        model.fromCameraInfo(*l_info_msg, *r_info_msg);
        pcl::PCLHeader l_info_header = pcl_conversions::toPCL(l_info_msg->header);
  
        PointCloud::Ptr point_cloud(new PointCloud());
        point_cloud->header.frame_id = l_info_header.frame_id;
        point_cloud->header.stamp = l_info_header.stamp;
        point_cloud->width = 1;
        point_cloud->height = inliers.size();
        point_cloud->points.resize(inliers.size());
  
        elas_ros::ElasFrameData data;
        data.header.frame_id = l_info_msg->header.frame_id;
        data.header.stamp = l_info_msg->header.stamp;
        data.width = l_width;
        data.height = l_height;
        data.disparity.resize(l_width * l_height);
        data.r.resize(l_width * l_height);
        data.g.resize(l_width * l_height);
        data.b.resize(l_width * l_height);
        data.x.resize(l_width * l_height);
        data.y.resize(l_width * l_height);
        data.z.resize(l_width * l_height);
        data.left = *l_info_msg;
        data.right = *r_info_msg;
  
        // Copy into the data
        for (int32_t u=0; u<l_width; u++) {
          for (int32_t v=0; v<l_height; v++) {
            int index = v*l_width + u;
            data.disparity[index] = l_disp_data[index];
            cv::Vec3b col = cv_ptr->image.at<cv::Vec3b>(v,u);

            data.r[index] = col[0];
            data.g[index] = col[1];
            data.b[index] = col[2];
          }
        }
  
        for (size_t i=0; i<inliers.size(); i++) {
          cv::Point2d left_uv;
          int32_t index = inliers[i];
          left_uv.x = index % l_width;
          left_uv.y = index / l_width;

          cv::Point3d point;
          model.projectDisparityTo3d(left_uv, l_disp_data[index], point);
          point_cloud->points[i].x = point.x;
          point_cloud->points[i].y = point.y;
          point_cloud->points[i].z = point.z;
          point_cloud->points[i].r = data.r[index];
          point_cloud->points[i].g = data.g[index];
          point_cloud->points[i].b = data.b[index];
  
          data.x[index] = point.x;
          data.y[index] = point.y;
          data.z[index] = point.z;
        }
  
        pc_pub_->publish(point_cloud);
        elas_fd_pub_->publish(data);
      }

      catch (cv_bridge::Exception& e) { ROS_ERROR("cv_bridge exception: %s", e.what()); }
    }
  
  private:
    ros::NodeHandle nh;
    Subscriber left_sub_, right_sub_;
    InfoSubscriber left_info_sub_, right_info_sub_;
    boost::shared_ptr<Publisher> disp_pub_;
    boost::shared_ptr<Publisher> depth_pub_;
    boost::shared_ptr<ros::Publisher> pc_pub_;
    boost::shared_ptr<ros::Publisher> elas_fd_pub_;
    boost::shared_ptr<ExactSync> exact_sync_;
    boost::shared_ptr<ApproximateSync> approximate_sync_;
    boost::shared_ptr<Elas> elas_;
    int queue_size_;
    image_geometry::StereoCameraModel model_;
    ros::Publisher pub_disparity_;
    boost::scoped_ptr<Elas::parameters> param;
    dynamic_reconfigure::Server<elas_ros::reconfigureConfig> reconfigure_server;
    dynamic_reconfigure::Server<elas_ros::reconfigureConfig>::CallbackType reconfigure_callback_f;
    int32_t disp_min, disp_max, support_texture, candidate_stepsize, incon_window_size, incon_th, incon_min_support,
            grid_size, match_texture, lr_th, speckle_size, ipol_gap_width;
    float support_th, beta, gamma, sigma, sradius, speckle_sim_th;
    bool add_corners, filter_median, filter_adaptive_mean, postprocess_only_left, subsampling;
};

void Elas_Proc::reconfigure_callback(elas_ros::reconfigureConfig &config, uint32_t level) {
  std::cout << "[ INFO ] Reconfigure ... " << std::endl;

  disp_min              = config.disp_min;
  disp_max              = config.disp_max;
  support_th            = config.support_th;      
  support_texture       = config.support_texture;        
  candidate_stepsize    = config.candidate_stepsize;     
  incon_window_size     = config.incon_window_size;     
  incon_th              = config.incon_th;      
  incon_min_support     = config.incon_min_support;    
  add_corners           = config.add_corners;          
  grid_size             = config.grid_size;            
  beta                  = config.beta;             
  gamma                 = config.gamma;            
  sigma                 = config.sigma;                  
  sradius               = config.sradius;               
  match_texture         = config.match_texture;         
  lr_th                 = config.lr_th;          
  speckle_sim_th        = config.speckle_sim_th; 
  speckle_size          = config.speckle_size;      
  ipol_gap_width        = config.ipol_gap_width;      
  filter_median         = config.filter_median;         
  filter_adaptive_mean  = config.filter_adaptive_mean;   
  postprocess_only_left = config.postprocess_only_left;  
  subsampling           = config.subsampling;            
  
  elas_parameter_setup();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "elas_ros");

  // Trasport possible options: "raw", "compressed", "theora"
  // ref: http://wiki.ros.org/image_transport
  Elas_Proc processor("raw");

  ros::spin();

  return 0;
}
