template<typename PointOutT>
void Cloud::computeDescriptors(FeatureInitializer<PointOutT> initFeature,
								pcl::Feature<pcl::PointXYZRGB,PointOutT>& featureEstimation,
								const typename pcl::PointCloud<PointOutT>::Ptr& out) const
{
	//set descriptor engine parameters
	initFeature(*this, featureEstimation);

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());

	//general parameters
	featureEstimation.setSearchMethod(kdtree);
	featureEstimation.setInputCloud(this->keypoints.p);
	featureEstimation.setSearchSurface(this->points.p);
	
	featureEstimation.compute(*out);
}