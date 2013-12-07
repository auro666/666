#ifndef __RoadDetector_H_
#define __RoadDetector_H_

#include "FeatureExtractor.hpp"
#include "Classifier.hpp"
#include <list>

using namespace std;

namespace Perception {
namespace RoadDetection {

	/**
	 * Encapsulation of Road Detection strategies in following steps:
	 * 1) Preprocessing
	 * 2) Feature Extraction
	 * 3) Classification
	 * which also encorporates Road model and Vehicle model.
	 * 
	 * @author Tanmay Patil
	 * @see preprocessor.hpp
	 * @see Feature.hpp
	 * @see FeatureExtractor.hpp
	 * @see Classifier.hpp
	 */
	class RoadDetector {
	
	private:
		list<Preprocessor> preprocessors;			/**< List of pre-processors to be used */
		list<FeatureExtractor> featureExtractors;	/**< List of feature extractors to be used */
		Classifier classifier;						/**< Classification model abstraction */
		
	public:
		/**
		 * Constructor to initialize with no pre-processors and feature-extractors.
		 * It is recommended to add them before usage.
		 * @param classifier is the holder for classification algorithm.
		 */
		RoadDetector(Classifier &classifier) {
			this->classifier = classifier;
		}
		
		/**
		 * Instantiation with given list of pre-processors and feature-extractors.
		 * @param preprocessors is the list of pre-processors to be used before feature extraction.
		 * @param featureExtractors is the list of feature-extractors to be used before classification.
		 * @param classifier is the holder for classification algorithm.
		 */
		RoadDetector(list<Preprocessor> &preprocessors, list<FeatureExtractor> &featureExtractors, Classifier &classifier) {
			this->preprocessors = preprocessors;
			this->featureExtractors = featureExtractors;
			this->classifier = classifier;
		}
		
		/**
		 * Returns the list of pre-processors to be used before feature extraction.
		 * @return list of pre-processors
		 */
		list<Preprocessor> getPreprocessors() {
			return preprocessors;
		}
		
		/**
		 * Returns the list of feature-extractors to be used before classification.
		 * @return list of feature-extractors
		 */
		list<FeatureExtractor> getFeatureExtractors() {
			return featureExtractors;
		}
		
	};

}
}

#endif
