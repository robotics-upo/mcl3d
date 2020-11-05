#ifndef __PARTICLEFILTER_HPP__
#define __PARTICLEFILTER_HPP__

#include <vector>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include "grid3d.hpp"
#include <time.h>

using std::isnan;

double ang_dist(double a, double a_) 
{
	double dist_a = a - a_;
	return dist_a - floor( (dist_a + M_PI) /M_PI*0.5)*M_PI*2.0;
}

//Class definition
class ParticleFilter
{
public:

	//! Struct that contains the data concerning one particle
	struct particle
	{
		//Position
		float x;
		float y;
		float z;
		
		// Yaw angle
		float a;

		// Weight
		float w;
	};

	//!Default contructor 
	ParticleFilter(std::string &node_name) : 
	m_grid3d(node_name)
	{
		// Setup random number generator from GSL
		gsl_rng_env_setup();
		m_randomType = gsl_rng_default;
		m_randomValue = gsl_rng_alloc(m_randomType);
		
		// Read node parameters
		ros::NodeHandle lnh("~");
		if(!lnh.getParam("in_cloud", m_inCloudTopic))
			m_inCloudTopic = "/pointcloud";	
		if(!lnh.getParam("base_frame_id", m_baseFrameId))
			m_baseFrameId = "base_link";	
		if(!lnh.getParam("odom_frame_id", m_odomFrameId))
			m_odomFrameId = "odom";	
		if(!lnh.getParam("global_frame_id", m_globalFrameId))
			m_globalFrameId = "map";	
		if (!lnh.getParam("use_imu", m_use_imu)) 
			m_use_imu = false;
		m_roll = m_pitch = m_yaw = 0.0;
		if (!lnh.getParam("use_2d_odom", m_use_2d_odom)) 
			m_use_2d_odom = false;
		
		// Read amcl parameters
		if(!lnh.getParam("update_rate", m_updateRate))
			m_updateRate = 10.0;
		if(!lnh.getParam("particles", m_nParticles))
			m_nParticles = 600;
		if(!lnh.getParam("odom_x_mod", m_odomXMod))
			m_odomXMod = 0.2;
		if(!lnh.getParam("odom_y_mod", m_odomYMod))
			m_odomYMod = 0.2;
		if(!lnh.getParam("odom_z_mod", m_odomZMod))
			m_odomZMod = 0.2;
		if(!lnh.getParam("odom_a_mod", m_odomAMod))
			m_odomAMod = 0.2;
		if(!lnh.getParam("odom_x_bias", m_odomXBias))
			m_odomXBias = 0.05;
		if(!lnh.getParam("odom_y_bias", m_odomYBias))
			m_odomYBias = 0.05;
		if(!lnh.getParam("odom_z_bias", m_odomZBias))
			m_odomZBias = 0.05;
		if(!lnh.getParam("odom_a_bias", m_odomABias))
			m_odomABias = 0.05;
		if(!lnh.getParam("initial_x", m_initX))
			m_initX = 0.0;
		if(!lnh.getParam("initial_y", m_initY))
			m_initY = 0.0;
		if(!lnh.getParam("initial_z", m_initZ))
			m_initZ = 0.0;
		if(!lnh.getParam("initial_a", m_initA))
			m_initA = 0.0;	
		if(!lnh.getParam("initial_x_dev", m_initXDev))
			m_initXDev = 0.3;
		if(!lnh.getParam("initial_y_dev", m_initYDev))
			m_initYDev = 0.3;
		if(!lnh.getParam("initial_z_dev", m_initZDev))
			m_initZDev = 0.1;
		if(!lnh.getParam("initial_a_dev", m_initADev))
			m_initADev = 0.2;	
		if(!lnh.getParam("update_min_d", m_dTh))
			m_dTh = 0.2;
		if(!lnh.getParam("update_min_a", m_aTh))
			m_aTh = 0.2;
		if(!lnh.getParam("resample_interval", m_resampleInterval))
			m_resampleInterval = 0;
        if(!lnh.getParam("initial_z_offset", m_initZOffset))
            m_initZOffset = 0.0;  
        if(!lnh.getParam("cloud_voxel_size", m_voxelSize))
            m_voxelSize = 0.05;  

		// Init internal variables
		m_nUpdates = 0;
		m_init = false;
		m_doUpdate = false;
		m_tfCache = false;
		m_p.resize(m_nParticles);
		
		// Launch subscribers
		m_pcSub = m_nh.subscribe(m_inCloudTopic, 1, &ParticleFilter::pointcloudCallback, this);
		m_initialPoseSub = lnh.subscribe("initial_pose", 2, &ParticleFilter::initialPoseReceived, this);
		if(m_use_imu)
			m_imuSub = m_nh.subscribe("imu", 1, &ParticleFilter::imuCallback, this);

		// Launch publishers
		m_posesPub = lnh.advertise<geometry_msgs::PoseArray>("particle_cloud", 1, true);
		//m_pcPub = lnh.advertise<sensor_msgs::PointCloud2>("/cloud", 1, true);

		// Launch updater timer
		updateTimer = m_nh.createTimer(ros::Duration(1.0/m_updateRate), &ParticleFilter::checkUpdateThresholdsTimer, this);
		
		// Initialize TF from odom to map as identity
		m_lastGlobalTf.setIdentity();
				
		if(m_initX != 0 || m_initY != 0 || m_initZ != 0 || m_initA != 0)
		{
			tf::Pose pose;
			tf::Vector3 origin(m_initX, m_initY, m_initZ);
			tf::Quaternion q;
			q.setRPY(0,0,m_initA);

			pose.setOrigin(origin);
			pose.setRotation(q);
			
			setInitialPose(pose, m_initXDev, m_initYDev, m_initZDev, m_initADev);
			m_init = true;
		}
	}

	//!Default destructor
	~ParticleFilter()
	{
		gsl_rng_free(m_randomValue);
	}
		
	//! Check motion and time thresholds for AMCL update
	bool checkUpdateThresholds()
	{
		// If the filter is not initialized then exit
		if(!m_init)
			return false;
					
		// Publish current TF from odom to map
		m_tfBr.sendTransform(tf::StampedTransform(m_lastGlobalTf, ros::Time::now(), m_globalFrameId, m_odomFrameId));
		
		// Compute odometric translation and rotation since last update 
		tf::StampedTransform odomTf;
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(.0));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("MCL3D error: %s",ex.what());
			return false;
		}
		tf::Transform T = m_lastOdomTf.inverse()*odomTf;
		
		// Check translation threshold
		if(T.getOrigin().length() > m_dTh)
		{
            ROS_INFO("Translation update");
            m_doUpdate = true;
			return true;
		}
		
		// Check yaw threshold
		double yaw, pitch, roll;
		T.getBasis().getRPY(roll, pitch, yaw);
		if(fabs(yaw) > m_aTh)
		{
            ROS_INFO("Rotation update");
			m_doUpdate = true;
			return true;
		}
		
	
		return false;
	}
	
	void publishParticles()
	{
		static int seq = 0;

		// If the filter is not initialized then exit
		if(!m_init)
			return;
			
		// Build the msg based on the particles position and orinetation	
		geometry_msgs::PoseArray particlesMsg;
		particlesMsg.header.stamp = ros::Time::now();
		particlesMsg.header.frame_id = m_globalFrameId;
		particlesMsg.header.seq = seq++;
		particlesMsg.poses.resize(m_p.size());
		for(int i=0; i<m_p.size(); i++)
		{
			particlesMsg.poses[i].position.x = m_p[i].x;
			particlesMsg.poses[i].position.y = m_p[i].y;
			particlesMsg.poses[i].position.z = m_p[i].z;
			particlesMsg.poses[i].orientation.x = 0.0;
			particlesMsg.poses[i].orientation.y = 0.0;
			particlesMsg.poses[i].orientation.z = sin(m_p[i].a*0.5);
			particlesMsg.poses[i].orientation.w = cos(m_p[i].a*0.5);
		}
		
		// Publisg particle cloud
		m_posesPub.publish(particlesMsg);
	}
		                                   
private:

	void checkUpdateThresholdsTimer(const ros::TimerEvent& event)
	{
		checkUpdateThresholds();
	}

	void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
	{
		// We only accept initial pose estimates in the global frame
		if(msg->header.frame_id != m_globalFrameId)
		{
			ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
			msg->header.frame_id.c_str(),
			m_globalFrameId.c_str());
			return;	
		}
		
		// Transform into the global frame
		tf::Pose pose;
		tf::poseMsgToTF(msg->pose.pose, pose);
		
		// Initialize the filter
		setInitialPose(pose, m_initXDev, m_initYDev, m_initZDev, m_initADev);
	}
	
	//! IMU callback
	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
	{
		double r = m_roll;
		double p = m_pitch;
		double y = m_yaw;
		auto o = msg->orientation;
		tf::Quaternion q;
		tf::quaternionMsgToTF(o, q);
		tf::Matrix3x3 M(q);
		M.getRPY(m_roll, m_pitch, m_yaw);
		if (isnan(m_roll) || isnan(m_pitch) || isnan(m_yaw)) 
		{
			m_roll = r;
			m_pitch = p;
			m_yaw = y;
		}
	}

	//! 3D point-cloud callback
	void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
	{		
		// If the filter is not initialized then exit
		if(!m_init)
			return;
			
		// Check if an update must be performed or not
		if(!m_doUpdate)
			return;
		
		// Pre-cache transform for point-cloud to base frame and transform the pc
		if(!m_tfCache)
		{	
			try
			{
                m_tfListener.waitForTransform(m_baseFrameId, cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
                m_tfListener.lookupTransform(m_baseFrameId, cloud->header.frame_id, ros::Time(0), m_pclTf);
				m_tfCache = true;
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
				return;
			}
		}
		sensor_msgs::PointCloud2 baseCloud, downCloud;
		pcl_ros::transformPointCloud(m_baseFrameId, m_pclTf, *cloud, baseCloud);
		
		// Apply voxel grid
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(baseCloud, *cloud_src);
		sor.setInputCloud(cloud_src);
		sor.setLeafSize(m_voxelSize, m_voxelSize, m_voxelSize);
		sor.filter(*cloud_down);
		cloud_down->header = cloud_src->header;
		pcl::toROSMsg(*cloud_down, downCloud);
		
		// Compute odometric translation and rotation since last update 
		tf::StampedTransform odomTf;
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), odomTf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			return;
		}

		// Get roll and pitch from odom if no IMU available
		if(!m_use_imu)
			odomTf.getBasis().getRPY(m_roll, m_pitch, m_yaw);
		// Perform particle prediction based on odometry
		float delta_x, delta_y, delta_z;
		double delta_r, delta_p, delta_a;
		tf::Transform T = m_lastOdomTf.inverse()*odomTf;
		delta_x = T.getOrigin().getX();
		delta_y = T.getOrigin().getY();
		delta_z = T.getOrigin().getZ();

		T.getBasis().getRPY(delta_r, delta_p, delta_a);
		if(!predict(delta_x, delta_y, delta_z, (float)delta_a))
		{
			ROS_ERROR("Prediction error!");
			return;
		}

		// Perform particle update based on current point-cloud
		if(!update(downCloud))
		{
			ROS_ERROR("Update error!");
			return;
		}

		// Update time and transform information
		m_lastOdomTf = odomTf;
		m_doUpdate = false;
		computeGlobalTfAndPose();
						
		// Publish particles
		publishParticles();
	}

	//!This function implements the PF prediction stage. Translation in X, Y and Z 
	//!in meters and yaw angle incremenet in rad
	bool predict(float delta_x, float delta_y, float delta_z, float delta_a)
	{
		float temp_dx = delta_x;
		float temp_dy = delta_y;
		float temp_dz = delta_z;
		// if(m_use_2d_odom) { // Adapt the 2d odometry according to the IMU measurements
			// float cr, sr, cp, sp, cy, sy, rx, ry; 
			// float r00, r01, r02, r10, r11, r12, r20, r21, r22;
			// sr = sin(m_roll);
			// cr = cos(m_roll);
			// sp = sin(m_pitch);
			// cp = cos(m_pitch);
			// r00 = cp; 	r01 = sp*sr; 	
			// r10 =  0; 	r11 = cr;		
			// r20 = -sp;	r21 = cp*sr;	
			// delta_z = delta_x*r20 + delta_y*r21;
			// delta_x = delta_x*r00 + delta_y*r01;
			// delta_y = delta_y*r11;

		float cr, sr, cp, sp, cy, sy, rx, ry;
		float r00, r01, r02, r10, r11, r12, r20, r21, r22;
		sr = sin(m_roll);
		cr = cos(m_roll);
		sp = sin(m_pitch);
		cp = cos(m_pitch);
		r00 = cp; 	r01 = sp*sr; 	r02 = cr*sp;
		r10 =  0; 	r11 = cr;		r12 = -sr;
		r20 = -sp;	r21 = cp*sr;	r22 = cp*cr;
		// }
		delta_x = r00 * temp_dx + r01 * temp_dy + r02 * temp_dz; 
		delta_y = r10 * temp_dx + r11 * temp_dy + r12 * temp_dz;
		delta_z = r20 * temp_dx + r21 * temp_dy + r22 * temp_dz;

		float xDev, yDev, zDev, aDev;
		xDev = fabs(delta_x*m_odomXMod) + m_odomXBias;
		yDev = fabs(delta_y*m_odomYMod) + m_odomYBias;
		zDev = fabs(delta_z*m_odomZMod) + m_odomZBias;
		aDev = fabs(delta_a*m_odomAMod) + m_odomABias;
//		ROS_INFO("Odom Z: %f,Last Z: %f, Delta X: %f, Delta Y: %f, Delta Z: %f",odomTf.getOrigin.getZ(), m_lastOdomTf.getOrigin().getZ(), delta_x, delta_y, delta_z);
		//Make a prediction for all particles according to the odometry
		for(int i=0; i<(int)m_p.size(); i++)
		{
			float sa = sin(m_p[i].a);
			float ca = cos(m_p[i].a);
			float randX = delta_x + gsl_ran_gaussian(m_randomValue, std::max(xDev, yDev));
			float randY = delta_y + gsl_ran_gaussian(m_randomValue, std::max(xDev, yDev));
			m_p[i].x += ca*randX - sa*randY;
			m_p[i].y += sa*randX + ca*randY;
			m_p[i].z += delta_z + gsl_ran_gaussian(m_randomValue, zDev);
			m_p[i].a += delta_a + gsl_ran_gaussian(m_randomValue, aDev);
			 
		}
		
		return true;
	}
	
	// Update Particles with a pointcloud update
	bool update(sensor_msgs::PointCloud2 &cloud)
	{		
		// Compensate for the current roll and pitch of the base-link
		std::vector<pcl::PointXYZ> points;
		float cr, sr, cp, sp, cy, sy, rx, ry;
		float r00, r01, r02, r10, r11, r12, r20, r21, r22;
		sr = sin(m_roll);
		cr = cos(m_roll);
		sp = sin(m_pitch);
		cp = cos(m_pitch);
		r00 = cp; 	r01 = sp*sr; 	r02 = cr*sp;
		r10 =  0; 	r11 = cr;		r12 = -sr;
		r20 = -sp;	r21 = cp*sr;	r22 = cp*cr;
		sensor_msgs::PointCloud2Iterator<float> iterX(cloud, "x");
		sensor_msgs::PointCloud2Iterator<float> iterY(cloud, "y");
		sensor_msgs::PointCloud2Iterator<float> iterZ(cloud, "z");
		//points.resize(cloud.width*cloud.height);
		for(int i=0; i<cloud.width*cloud.height; ++i, ++iterX, ++iterY, ++iterZ) 
		{
			float x = *iterX, y = *iterY, z = *iterZ;
			pcl::PointXYZ p;
			p.x = x*r00 + y*r01 + z*r02;
			p.y = x*r10 + y*r11 + z*r12;
			p.z = x*r20 + y*r21 + z*r22;
			//if(x>=0)
				points.push_back(p);
		}
		//publishPointCloud(points);
		
		// Incorporate measurements
		float wt = 0.0;
		std::vector<pcl::PointXYZ> new_points;
		new_points.resize(points.size());
		for(int i=0; i<m_p.size(); i++)
		{
			if(isnan(m_p[i].x) || isnan(m_p[i].y) || isnan(m_p[i].z) || isnan(m_p[i].a) || isnan(m_p[i].w))
			{
				std::cout << "Particles with NAN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
				return false;;
			}
		}
		for(int i=0; i<m_p.size(); i++)
		{
			// Get particle information
			float tx = m_p[i].x;
			float ty = m_p[i].y;
			float tz = m_p[i].z;
			float sa = sin(m_p[i].a);
			float ca = cos(m_p[i].a);
			
			// Check the particle is into the map
			if(!m_grid3d.isIntoMap(tx, ty, tz))
			{
			 	m_p[i].w = 0;
			 	continue;
			}
			
			// Transform every point of the point-cloud to current particle position
			for(int j=0; j<points.size(); j++)
			{
				// Get point
				const pcl::PointXYZ& p = points[j];
				
				// Translate and rotate it in yaw
				new_points[j].x = ca*p.x - sa*p.y + tx;
				new_points[j].y = sa*p.x + ca*p.y + ty;
				new_points[j].z = p.z + tz;
				/*
				if (isnan(new_points[j].x) || isnan(new_points[j].y) || isnan(new_points[j].z)) 
				{
					new_points[j].x = -1;
					new_points[j].y = -1;
					new_points[j].z = -1;
				}*/
			}
			
			// Evaluate the weight of the point-cloud
			m_p[i].w = m_grid3d.computeCloudWeight(new_points);
				
			//Increase the summatory of weights
			wt += m_p[i].w;
		}
		
		//Normalize all weights
		for(int i=0; i<(int)m_p.size(); i++) 
		{
			if (wt > 0.0) 
				m_p[i].w /= wt;
			else 
				m_p[i].w = 1.0/static_cast<float>(m_p.size());
		}
		
		// Re-compute global TF according to new weight of samples
		//computeGlobalTfAndPose();

		//Do the resampling if needed
		m_nUpdates++;
		if(m_nUpdates > m_resampleInterval)
		{
			m_nUpdates = 0;
			resample();
		}

		return true;
	}

	//! Set the initial pose of the particle filter
	void setInitialPose(tf::Pose initPose, float xDev, float yDev, float zDev, float aDev)
	{
		// Log
		tf::Pose &pose = initPose;
		ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f %.3f", ros::Time::now().toSec(), pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(), getYawFromTf(pose));

		// Resize particle set
		m_p.resize(m_nParticles);
		
		// Sample the given pose
		tf::Vector3 t = initPose.getOrigin();
		float a = getYawFromTf(initPose);
		float dev = std::max(std::max(xDev, yDev), zDev);
		float gaussConst1 = 1./(dev*sqrt(2*M_PI));
		float gaussConst2 = 1./(2*dev*dev);
		float dist = 0.0, wt = 0.0;
		m_p[0].x = t.x();
		m_p[0].y = t.y();
        m_p[0].z = t.z()+m_initZOffset;
		m_p[0].a = a;
		m_p[0].w = gaussConst1;
		wt = m_p[0].w;
		for(int i=1; i<(int)m_p.size(); i++)
		{
			m_p[i].x = t.x() + gsl_ran_gaussian(m_randomValue, xDev);
			m_p[i].y = t.y() + gsl_ran_gaussian(m_randomValue, yDev);
            m_p[i].z = t.z() + m_initZOffset + gsl_ran_gaussian(m_randomValue, zDev);
			m_p[i].a = a + gsl_ran_gaussian(m_randomValue, aDev);
			dist = sqrt((m_p[i].x - t.x())*(m_p[i].x - t.x()) + (m_p[i].y - t.y())*(m_p[i].y - t.y()) + (m_p[i].z - t.z())*(m_p[i].z - t.z()));
			m_p[i].w = gaussConst1*exp(-dist*dist*gaussConst2);
			wt += m_p[i].w;
		}
		for(int i=0; i<(int)m_p.size(); i++)
			m_p[i].w /= wt;
				
		// Extract TFs for future updates
		bool got_tf = false;
		while (!got_tf && ros::ok()) {
			try
			{
				m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
				m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), m_lastOdomTf);
				got_tf = true;
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("%s",ex.what());
			}
		}
		if (!got_tf)
		{
			ROS_ERROR("MCL3D Error: Initialization:Could not get initial TF from %s to %s.", m_odomFrameId.c_str(), m_baseFrameId.c_str());
			return;
		}
		computeGlobalTfAndPose();
		m_doUpdate = false;
		m_init = true;
		
		// Publish particles
		publishParticles();
	}
	
	//! Return yaw from a given TF
	float getYawFromTf(tf::Pose& pose)
	{
		double yaw, pitch, roll;
		
		pose.getBasis().getRPY(roll, pitch, yaw);
		
		return (float)yaw;
	}

	//! resample the set of particules using low-variance sampling
	int resample()
	{
		int i, m;
		float r, u, c, factor;
		std::vector<particle> newP;

		//Initialize data and vectors
		newP.resize(m_p.size());
		factor = 1.0/((float)m_p.size());
		i = 0;
		c = m_p[0].w;
		r = factor*gsl_rng_uniform(m_randomValue);

		//Do resampling
		for(m=0; m<m_p.size(); m++)
		{
			u = r + factor*(float)m;
			while(u > c)
			{
				i++;
				c += m_p[i].w;
			}
			newP[m] = m_p[i];
			newP[m].w = factor;
		}
		
		//Asign the new particles set
		m_p = newP;

		return 0;
	}
	
	// Computes TF from odom to global frames
	void computeGlobalTfAndPose()
	{				
		// Compute mean value from particles
		float mx, my, mz, ma, varx, vary, varz, vara;
		computeVar(mx, my, mz, ma, varx, vary, varz, vara);	

		// Compute the TF from odom to global
		tf::Quaternion q;
		q.setRPY(m_roll, m_pitch, ma);
		m_lastGlobalTf = tf::Transform(q, tf::Vector3(mx, my, mz))*m_lastOdomTf.inverse();
		tf::Vector3 t = m_lastGlobalTf.getOrigin();
		double r, p, y;
		m_lastGlobalTf.getBasis().getRPY(r, p, y);
		ROS_INFO("\tNew TF. tx: %f, ty: %f, tz: %f, a: %f",t.x(), t.y(), t.z(), y);
	}
	
	void computeDev(float &mX, float &mY, float &mZ, float &mA, float &devX, float &devY, float &devZ, float &devA)
	{				
		computeVar(mX, mY, mZ, mA, devX, devY, devZ, devA);
		devX = sqrt(devX);
		devY = sqrt(devY);
		devZ = sqrt(devZ);
		devA = sqrt(devA);
	}

	void computeVar(float &mX, float &mY, float &mZ, float &mA, float &varX, float &varY, float &varZ, float &varA)
	{				
		// Compute mean value from particles
		varX = mX = 0.0;
		varY = mY = 0.0;
		varZ = mZ = 0.0;
		varA = mA = 0.0;
		for(int i=0; i<m_p.size(); i++)
		{
			mX += m_p[i].w * m_p[i].x;
			mY += m_p[i].w * m_p[i].y;
			mZ += m_p[i].w * m_p[i].z;
			mA += m_p[i].w * m_p[i].a;
		}
		for(int i=0; i<m_p.size(); i++)
		{
			varX += m_p[i].w * (m_p[i].x-mX) * (m_p[i].x-mX);
			varY += m_p[i].w * (m_p[i].y-mY) * (m_p[i].y-mY);
			varZ += m_p[i].w * (m_p[i].z-mZ) * (m_p[i].z-mZ);
			varA += m_p[i].w * (m_p[i].a-mA) * (m_p[i].a-mA);
		}
	}

	void publishPointCloud(std::vector<pcl::PointXYZ> &points)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		
		cloud->width = points.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		for(unsigned int i=0; i<points.size(); i++)
			cloud->points[i] = points[i];
		
		// Create the point cloud msg for publication
		sensor_msgs::PointCloud2 pcMsg;
		pcl::toROSMsg(*cloud, pcMsg);
		pcMsg.header.frame_id = m_baseFrameId;
		m_pcPub.publish(pcMsg);
	}
	
	//! Indicates if the filter was initialized
	bool m_init;

	//! If true, the odometry is assumed 2d and thus we proyect it into 3d
	bool m_use_imu, m_use_2d_odom;
	
	//! Indicates that the local transfrom for the pint-cloud is cached
	bool m_tfCache;
	tf::StampedTransform m_pclTf;

	//! Particles 
	std::vector<particle> m_p;
	
	//! Particles roll and pich (given by IMU)
	double m_roll, m_pitch, m_yaw;
	
	//! Number of particles in the filter
	int m_nParticles;
	
	//! Odometry characterization
	double m_odomXMod, m_odomYMod, m_odomZMod, m_odomAMod, m_odomAModMin, m_odomZModMin;
	double m_odomXBias,m_odomYBias,m_odomZBias, m_odomABias; 
    double m_initX, m_initY, m_initZ, m_initA, m_initZOffset;
	double m_initXDev, m_initYDev, m_initZDev, m_initADev;
	double m_voxelSize;
	
	//! Resampling control
	int m_nUpdates;
	int m_resampleInterval;
	
	//! Thresholds for filter updating
	double m_dTh, m_aTh;
	tf::StampedTransform m_lastOdomTf;
	tf::Transform m_lastGlobalTf;
	bool m_doUpdate;
	double m_updateRate;
		
	//! Node parameters
	std::string m_inCloudTopic;
	std::string m_baseFrameId;
	std::string m_odomFrameId;
	std::string m_globalFrameId;
	
	//! ROS msgs and data
	ros::NodeHandle m_nh;
	tf::TransformBroadcaster m_tfBr;
	tf::TransformListener m_tfListener;
    ros::Subscriber m_pcSub, m_initialPoseSub, m_odomTfSub, m_imuSub;
	ros::Publisher m_posesPub, m_pcPub;
	ros::Timer updateTimer;
		
	//! Random number generator
	const gsl_rng_type *m_randomType;
	gsl_rng *m_randomValue;
	
	//! Probability grid for point-cloud evaluation
	Grid3d m_grid3d;
};

#endif


