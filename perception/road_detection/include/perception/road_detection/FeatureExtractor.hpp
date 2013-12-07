#ifndef __FeatureExtractor_H_
#define __FeatureExtractor_H_

#include <image_transport/image_transport.h>

#include "Feature.h"

namespace Perception {
namespace RoadDetection {

	/**
	 * Abstract base class describing feature extraction functionality.
	 * A feature is extracted from the input image.
	 * 
	 * @author Tanmay Patil
	 * @see Feature.hpp
	 */
	class FeatureExtractor {
		
	public:
		/**
		 * Function to extract a feature from the given image.
		 * @param inputImage is the image out of which the features are to be extracted.
		 * @return Feature extracted from the image
		 */
		virtual Feature extractFeature(const sensor_msgs::ImageConstPtr& inputImage);
		
	};
	
}
}

#endif
