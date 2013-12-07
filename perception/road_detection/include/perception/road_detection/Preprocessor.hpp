#ifndef __Preprocessor_H_
#define __Preprocessor_H_

#include <image_transport/image_transport.h>

namespace Perception {
namespace RoadDetection {

	/**
	 * Abstract base class describing image pre-processing functionality.
	 * This is done before features are extracted from the input image.
	 * @author Tanmay Patil
	 */
	class Preprocessor {
		
	public:
		/**
		 * Function to process image for better feature extraction.
		 * @param inputImage is the image to be processed
		 * @return the processed image
		 */
		virtual sensor_msgs::ImageConstPtr process(const sensor_msgs::ImageConstPtr& inputImage);
	
	};

}
}

#endif
