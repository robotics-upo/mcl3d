#ifndef __PARTICLEFILTER_HPP__
#define __PARTICLEFILTER_HPP__

#include <vector>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include "grid3d.hpp"
#include <time.h>
#include <range_msgs/P2PRangeWithPose.h>


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
		float wp;
		float wr;
	};

	//! Range-only data class
	struct rangeData
	{
		uint64_t id;
		float x;
		float y;
		float z;
		std::vector<float> range;
		float variance;
		float getMeanRange(void)
		{
			if(range.size() == 0)
				return -1.0;
			float mean = 0;
			for(size_t i=0; i<range.size(); i++)
				mean += range[i];
			mean = mean/((float)range.size());
			return mean;
		}
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
		
		// Read range-only parameters
		std::string id;
		if(!lnh.getParam("use_range_only", m_useRageOnly))
            m_useRageOnly = false;  
        if(!lnh.getParam("in_range", m_inRangeTopic))
			m_inRangeTopic = "/range";	
		if(!lnh.getParam("range_only_tag", id))
            id = "0";
        if(id.find("0x") == 0 || id.find("0X") == 0)
			sscanf(id.c_str(), "%lx", &m_tagId);
		else
			sscanf(id.c_str(), "%lu", &m_tagId);
		if(!lnh.getParam("range_only_dev", m_rangeOnlyDev))
            m_rangeOnlyDev = -1.0;

		// Read amcl parameters
		if(!lnh.getParam("update_rate", m_updateRate))
			m_updateRate = 10.0;
		if(!lnh.getParam("min_particles", m_minParticles))
			m_minParticles = 300;
		if(!lnh.getParam("max_particles", m_maxParticles))
			m_maxParticles = 600;
		if(m_minParticles > m_maxParticles)
			m_maxParticles = m_minParticles;
		if(!lnh.getParam("odom_x_mod", m_odomXMod))
			m_odomXMod = 0.2;
		if(!lnh.getParam("odom_y_mod", m_odomYMod))
			m_odomYMod = 0.2;
		if(!lnh.getParam("odom_z_mod", m_odomZMod))
			m_odomZMod = 0.2;
		if(!lnh.getParam("odom_a_mod", m_odomAMod))
			m_odomAMod = 0.2;
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
		
		m_nUpdates = 0;
		m_init = false;
		m_doUpdate = false;
		m_tfCache = false;
		m_p.resize(m_maxParticles);
		
		// Launch subscribers
		m_pcSub = m_nh.subscribe(m_inCloudTopic, 1, &ParticleFilter::pointcloudCallback, this);
		m_initialPoseSub = m_nh.subscribe(node_name+"/initial_pose", 2, &ParticleFilter::initialPoseReceived, this);
		if(m_useRageOnly)
			m_rangeSub = m_nh.subscribe(m_inRangeTopic, 1, &ParticleFilter::rangeDataCallback, this);
		
		// Launch publishers
		m_posesPub = m_nh.advertise<geometry_msgs::PoseArray>(node_name+"/particle_cloud", 1, true);
		m_visPub = m_nh.advertise<visualization_msgs::Marker>(node_name+"/uav", 0);

		// Launch updater timer
		updateTimer = m_nh.createTimer(ros::Duration(1.0/m_updateRate), &ParticleFilter::checkUpdateThresholdsTimer, this);
		
		// Initialize TF from odom to map as identity
		m_lastGlobalTf.setIdentity();
		
		// Borrar
		pf = fopen("/home/fernando/catkin_ws/mcl3d.txt", "w");
	}

	//!Default destructor
	~ParticleFilter(void)
	{
		gsl_rng_free(m_randomValue);
		fclose(pf);
	}
		
	//! Check motion and time thresholds for AMCL update
	bool checkUpdateThresholds(void)
	{
		// Publish current TF from odom to map
		m_tfBr.sendTransform(tf::StampedTransform(m_lastGlobalTf, ros::Time::now(), m_globalFrameId, m_odomFrameId));
		
		// If the filter is not initialized then exit
		if(!m_init)
			return false;
		std::cout << "Checking for AMCl3D update" << std::endl;
					
		// Compute odometric translation and rotation since last update 
		tf::StampedTransform odomTf;
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
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
		if(fabs(yaw) > m_dTh)
		{
            ROS_INFO("Rotation update");
			m_doUpdate = true;
			return true;
		}
		
<<<<<<< HEAD
	
=======
>>>>>>> 8ae66df0571d1b961f024a271dbe26be89b16fe2
		return false;
	}
	
	void publishParticles(void)
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
		ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f %.3f", ros::Time::now().toSec(), pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(), getYawFromTf(pose));
		
		// Initialize the filter
		setInitialPose(pose, m_initXDev, m_initYDev, m_initZDev, m_initADev);
	}
	
	//! Range-only data callback
	void rangeDataCallback(const range_msgs::P2PRangeWithPose::ConstPtr& msg)
	{
		// Check if tag id matches
		if(m_tagId != 0 && msg->source_id != m_tagId)
		{
			return;
		}
		
		// Check if range frame_id is corect
		if(msg->header.frame_id != m_globalFrameId)
		{
			std::cout << "[MCL3D] Incoming range-only data frame_id does not match with global_frame_id!!" << std::endl;
			std::cout << "\tDescarting range" << std::endl;
			return;
		}
		
		// Do we have already measurements from this anchor?
		int index = -1;
		for(size_t i = 0; i<m_rangeData.size(); i++)
			if(msg->destination_id == m_rangeData[i].id)
			{
				index = i;
				break;
			}
	
		// Save range meaurement
		if(index < 0)
		{
			rangeData anchor;
			anchor.id = msg->destination_id;
			anchor.x = msg->position.point.x;
			anchor.y = msg->position.point.y;
			anchor.z = msg->position.point.z;
			anchor.variance = msg->variance;
			anchor.range.push_back(msg->range); 
			m_rangeData.push_back(anchor);
		}
		else
			m_rangeData[index].range.push_back(msg->range);
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
		
		// Compute odometric trasnlation and rotation since last update 
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
		
		// Extract current robot roll and pitch
		double yaw, r, p;
		odomTf.getBasis().getRPY(r, p, yaw);
		//odomTf.getBasis().getRPY(m_roll, m_pitch, yaw);
		
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
		computeGlobalTf();
						
		// Publish particles
		publishParticles();
	}

	//!This function implements the PF prediction stage. Translation in X, Y and Z 
	//!in meters and yaw angle incremenet in rad
	bool predict(float delta_x, float delta_y, float delta_z, float delta_a)
	{
		float xDev, yDev, zDev, aDev;
		xDev = fabs(delta_x*m_odomXMod);
		yDev = fabs(delta_y*m_odomYMod);
		zDev = fabs(delta_z*m_odomZMod);
		aDev = fabs(delta_a*m_odomAMod);
		
		//Make a prediction for all particles according to the odometry
		for(int i=0; i<(int)m_p.size(); i++)
		{
			float sa = sin(m_p[i].a);
			float ca = cos(m_p[i].a);
			float randX = delta_x + gsl_ran_gaussian(m_randomValue, xDev);
			float randY = delta_y + gsl_ran_gaussian(m_randomValue, yDev);
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
		points.resize(cloud.width);
		for(int i=0; i<cloud.width; ++i, ++iterX, ++iterY, ++iterZ)
		{
			points[i].x = *iterX*r00 + *iterY*r01 + *iterZ*r02;
			points[i].y = *iterX*r10 + *iterY*r11 + *iterZ*r12;
			points[i].z = *iterX*r20 + *iterY*r21 + *iterZ*r22;
		}
		
		// Incorporate measurements
		float alpha, wtp = 0, wtr = 0;
		std::vector<pcl::PointXYZ> new_points;
		new_points.resize(points.size());
		if(m_resampleInterval > 0)
			alpha = 1.0/(float)m_resampleInterval;
		else
			alpha = 1.0;
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
			}
			
			// Evaluate the weight of the point-cloud
			m_p[i].wp = m_grid3d.computeCloudWeight(new_points);
			
			// Evaluate the weight of the range sensors
			if(m_rangeData.size() > 0 && m_useRageOnly)
				m_p[i].wr = computeRangeWeight(tx, ty, tz);
			else
				m_p[i].wr = 0;
				
			//Increase the summatory of weights
			wtp += m_p[i].wp;
			wtr += m_p[i].wr;
		}
		
		// Clean the range buffer
		m_rangeData.clear(); 
		
		//Normalize all weights
		for(int i=0; i<(int)m_p.size(); i++)
		{
			m_p[i].wp /= wtp;
			if(m_rangeData.size() > 0 && m_useRageOnly)
			{
				m_p[i].wr /= wtr;
				m_p[i].w = m_p[i].wp*0.5 + m_p[i].wr*0.5;  
			}
			else
			{
				m_p[i].wr = 0;
				m_p[i].w = m_p[i].wp;  
			}
			
		}	
		
		// Re-compute global TF according to new weight of samples
		computeGlobalTf();

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
		// Resize particle set
		m_p.resize(m_maxParticles);
		
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
		try
		{
			m_tfListener.waitForTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), ros::Duration(1.0));
			m_tfListener.lookupTransform(m_odomFrameId, m_baseFrameId, ros::Time(0), m_lastOdomTf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			return;
		}
		computeGlobalTf();
		m_doUpdate = false;
		m_init = true;
		
		// Publish particles
		publishParticles();
		
		// Clean the range buffer
		m_rangeData.clear(); 
	}
	
	//! Return yaw from a given TF
	float getYawFromTf(tf::Pose& pose)
	{
		double yaw, pitch, roll;
		
		pose.getBasis().getRPY(roll, pitch, yaw);
		
		return (float)yaw;
	}

	//! resample the set of particules using low-variance sampling
	int resample(void)
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

		//Do resamplig
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
	void computeGlobalTf(void)
	{				
		// Compute mean value from particles
		particle p;
		p.x = 0.0;
		p.y = 0.0;
		p.z = 0.0;
		p.a = 0.0;
		p.w = 0.0;
		for(int i=0; i<m_p.size(); i++)
		{
			p.x += m_p[i].w * m_p[i].x;
			p.y += m_p[i].w * m_p[i].y;
			p.z += m_p[i].w * m_p[i].z;
			p.a += m_p[i].w * m_p[i].a;
		}
		
		// Compute the TF from odom to global
		std::cout << "New TF:\n\t" << p.x << ", " << p.y << ", " << p.z << std::endl;
		m_lastGlobalTf = tf::Transform(tf::Quaternion(0.0, 0.0, sin(p.a*0.5), cos(p.a*0.5)), tf::Vector3(p.x, p.y, p.z))*m_lastOdomTf.inverse();
	}

	float computeRangeWeight(float x, float y, float z)
	{
		int n=0;
		float w, sigma, k1, k2;
		
		w = 1;
		sigma = m_rangeOnlyDev;
		k1 = 1.0/(sigma*sqrt(2*M_PI));
		k2 = 0.5/(sigma*sigma);
		for(int i=0; i<m_rangeData.size(); i++)
		{
			float ax, ay, az, ar, r;
			ax = m_rangeData[i].x;
			ay = m_rangeData[i].y;
			az = m_rangeData[i].z;
			ar = m_rangeData[i].getMeanRange();
			r = sqrt((x-ax)*(x-ax) + (y-ay)*(y-ay) + (z-az)*(z-az));
			w *= k1*exp(-k2*(r-ar)*(r-ar));
			n++;
		}
		
		if(n > 0)
			return w;
		else
			return -1;
	}
	
	
	void computeDev(float &mX, float &mY, float &mZ, float &mA, float &devX, float &devY, float &devZ, float &devA)
	{				
		// Compute mean value from particles
		devX = mX = 0.0;
		devY = mY = 0.0;
		devZ = mZ = 0.0;
		devA = mA = 0.0;
		for(int i=0; i<m_p.size(); i++)
		{
			mX += m_p[i].w * m_p[i].x;
			mY += m_p[i].w * m_p[i].y;
			mZ += m_p[i].w * m_p[i].z;
			mA += m_p[i].w * m_p[i].a;
		}
		for(int i=0; i<m_p.size(); i++)
		{
			devX += m_p[i].w * (m_p[i].x-mX) * (m_p[i].x-mX);
			devY += m_p[i].w * (m_p[i].y-mY) * (m_p[i].y-mY);
			devZ += m_p[i].w * (m_p[i].z-mZ) * (m_p[i].z-mZ);
			devA += m_p[i].w * (m_p[i].a-mA) * (m_p[i].a-mA);
		}
		devX = sqrt(devX);
		devY = sqrt(devY);
		devZ = sqrt(devZ);
		devA = sqrt(devA);
	}
	
	//! Indicates if the filter was initialized
	bool m_init;
	
	//! Indicates that the local transfrom for the pint-cloud is cached
	bool m_tfCache;
	tf::StampedTransform m_pclTf;

	//! Particles 
	std::vector<particle> m_p;
	
	//! Particles roll and pich (given by IMU)
	double m_roll, m_pitch;
	
	//! Number of particles in the filter
	int m_maxParticles;
	int m_minParticles;
	
	//! Odometry characterization
	double m_odomXMod, m_odomYMod, m_odomZMod, m_odomAMod;
    double m_initX, m_initY, m_initZ, m_initA, m_initZOffset;
	double m_initXDev, m_initYDev, m_initZDev, m_initADev;
	double m_voxelSize;
	
	//! Resampling control
	int m_nUpdates;
	int m_resampleInterval;
	
	//! Thresholds for filter updating
	double m_dTh, m_aTh, m_tTh;
	tf::StampedTransform m_lastOdomTf;
	tf::Transform m_lastGlobalTf;
	bool m_doUpdate;
	double m_updateRate;
	
	//! Range only updates
	uint64_t m_tagId;
	bool m_useRageOnly;
	std::vector<rangeData> m_rangeData;
	std::string m_inRangeTopic;
	float m_rangeOnlyDev;
	
	//! Node parameters
	std::string m_inCloudTopic;
	std::string m_baseFrameId;
	std::string m_odomFrameId;
	std::string m_globalFrameId;
    std::string m_inOdomTfTopic;
	
	//! ROS msgs and data
	ros::NodeHandle m_nh;
	tf::TransformBroadcaster m_tfBr;
	tf::TransformListener m_tfListener;
    ros::Subscriber m_pcSub, m_initialPoseSub, m_odomTfSub, m_rangeSub;
	ros::Publisher m_posesPub, m_visPub;
	ros::Timer updateTimer;
	
	//! Random number generator
	const gsl_rng_type *m_randomType;
	gsl_rng *m_randomValue;
	
	//! Probability grid for point-cloud evaluation
	Grid3d m_grid3d;
	
	//!Borrar
	FILE *pf;
};

#endif


