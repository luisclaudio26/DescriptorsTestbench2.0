template<typename PointOutT>
void Cloud::computeDescriptors(FeatureInitializer<PointOutT> initFeature,
								pcl::Feature<pcl::PointXYZRGBNormal,PointOutT>& featureEstimation,
								const typename pcl::PointCloud<PointOutT>::Ptr& out) const
{
	//set descriptor engine parameters
	initFeature(*this, featureEstimation);

	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());

	//general parameters
	featureEstimation.setSearchMethod(kdtree);
	featureEstimation.setInputCloud(this->keypoints);
	featureEstimation.setSearchSurface(this->points);
	
	featureEstimation.compute(*out);
}