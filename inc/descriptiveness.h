#ifndef _DESCRIPTIVENESS_H_
#define _DESCRIPTIVENESS_H_

#include "cloud.h"
#include <pcl/registration/correspondence_estimation.h>

namespace Descriptiveness
{
	void groundtruthCorrespondences(const Cloud& target, Cloud& source);


	void filterCorrectMatches(const pcl::Correspondences& groundtruth, 
								const pcl::Correspondences& matches,
								pcl::Correspondences& out);
}

#endif