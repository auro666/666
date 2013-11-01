#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <dc1394/conversions.h>
#include <dc1394/control.h>
#include <dc1394/utils.h>

#include "pgr_registers.h"
#include "pgr_stereocam.h"

#include "transport2.h"
#include <iostream>
#include <algorithm>    // std::fill

void cleanupAndExit(dc1394camera_t* camera) {
    dc1394_capture_stop(camera);
    dc1394_video_set_transmission(camera, DC1394_OFF);
    dc1394_camera_free(camera);
    exit (0);
}

void initStereo(PGRStereoCamera_t& stereoCamera, TriclopsError& e, TriclopsContext& triclops, dc1394camera_t*& camera) {
	dc1394error_t err;
    dc1394_t* d;
    dc1394camera_list_t* list;
    unsigned int nThisCam;
    
    d = dc1394_new();

    err = dc1394_camera_enumerate(d, &list);
    if (err != DC1394_SUCCESS) {
		ROS_ERROR("[stereopsis]: Unable to enumerate stereo camera(s)");
        exit (0);
    }

    if (list->num == 0) {
        ROS_ERROR("[stereopsis]: No stereo cameras found");
        exit (0);
    }

    for (nThisCam = 0; nThisCam < list->num; nThisCam++) {
        camera = dc1394_camera_new(d, list->ids[nThisCam].guid);

        if (!camera) {
            ROS_INFO("[stereopsis]: Failed to initialize camera with guid %llx", list->ids[nThisCam].guid);
            continue;
        }

        if (isStereoCamera(camera)) {
            break;
        }
        dc1394_camera_free(camera);
    }

    if (nThisCam == list->num) {
        ROS_ERROR("[stereopsis]: No stereo cameras detected");
        exit (0);
    }

    dc1394_camera_free_list(list);

    err = queryStereoCamera(camera, &stereoCamera);
    if (err != DC1394_SUCCESS) {
        ROS_ERROR("[stereopsis]: Cannot query all information from stereo camera");
        cleanupAndExit(camera);
    }

    err = setStereoVideoCapture(&stereoCamera);
    if (err != DC1394_SUCCESS) {
        ROS_ERROR("[stereopsis]: Could not set up video capture mode");
        cleanupAndExit(stereoCamera.camera);
    }

    err = startTransmission(&stereoCamera);
    if (err != DC1394_SUCCESS) {
        ROS_ERROR("[stereopsis]: Unable to start camera iso transmission");
        cleanupAndExit(stereoCamera.camera);
    }

    /// Stereopsis
    e = getTriclopsContextFromCamera(&stereoCamera, &triclops);
    if (e != TriclopsErrorOk) {
        ROS_ERROR("[stereopsis]: Unable to get context from stereo camera");
        cleanupAndExit(camera);
    }
    
    triclopsSetSubpixelInterpolation(triclops, 1);
	triclopsSetMaxThreadCount(triclops, 1);
}

PointCloud generatePointCloud(TriclopsImage16 depthImage16, TriclopsContext triclops) {
	PointCloud msg;
	msg.width = 500;
	msg.height = 500;
	std::fill(msg.data, msg.data + msg.width * msg.height, 0);
	
	int pixelinc = depthImage16.rowinc / 2;
	
	for (int i = 0; i < depthImage16.nrows; i++) {
		unsigned short * row = depthImage16.data + i * pixelinc;
		for (int j = 0; j < depthImage16.ncols; j++) {
			int disparity = row[j];
			// filter invalid points
			if (disparity < 0xFF00) {
				float x, y, z;
				
				// convert the 16 bit disparity value to floating point x,y,z
				triclopsRCD16ToXYZ(triclops, i, j, disparity, &x, &y, &z);
				
				/// World of centimeters
				
				int depth = 154;
				int max_depth = 255;
				int baseline = 12;
				
				// look at points within a range
				if (((x > -(msg.width + baseline / 2.) / 100.) && (x < (msg.width - baseline / 2.) / 100.)) && 
				    ((y > (-max_depth + depth) / 100.) && (y < depth / 100.)) && 
				    ((z > 0.) && (z < msg.height * 2. / 100.))) {						
					int ix = (int) (msg.width + baseline / 2. + 100 * x);
					unsigned char iy = (unsigned char) (depth - 100 * y);
					int iz = (int) (msg.height * 2. - 100 * z);
					
					msg.data[iz/2 * msg.width + ix/2] = iy;
				}
			}
		}
	}
	msg.width = 500;
	msg.height = 500;
	msg.stamp = time(0);

	return msg;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "stereopsis");
    
    dc1394camera_t* camera;
    PGRStereoCamera_t stereoCamera;
	TriclopsError e;
    TriclopsContext triclops;
    
    initStereo(stereoCamera, e, triclops, camera);
    
	Publisher<PointCloud> tcp_publisher(6007);
	
	unsigned int nBufferSize = stereoCamera.nRows * stereoCamera.nCols * stereoCamera.nBytesPerPixel;
    unsigned char* pucDeInterlacedBuffer = new unsigned char[nBufferSize];
    unsigned char* pucRGBBuffer = new unsigned char[3 * nBufferSize];
    unsigned char* pucGreenBuffer = new unsigned char[nBufferSize];

    while (ros::ok()) {
		TriclopsInput input;
        
		unsigned char* pucRightRGB = NULL;
		unsigned char* pucLeftRGB = NULL;
		unsigned char* pucCenterRGB = NULL;
		extractImagesColor(&stereoCamera, DC1394_BAYER_METHOD_NEAREST, pucDeInterlacedBuffer, pucRGBBuffer, pucGreenBuffer, &pucRightRGB, &pucLeftRGB, &pucCenterRGB, &input);

        e = triclopsRectify(triclops, &input);
        if (e != TriclopsErrorOk) {
            ROS_ERROR("[stereopsis]: triclopsRectify failed");
            triclopsDestroyContext(triclops);
            cleanupAndExit(camera);
        }

        e = triclopsStereo(triclops);
        if (e != TriclopsErrorOk) {
            ROS_ERROR("[stereopsis]: triclopsStereo failed");
            triclopsDestroyContext(triclops);
            cleanupAndExit(camera);
        }

        TriclopsImage16 image16;
        triclopsGetImage16(triclops, TriImg16_DISPARITY, TriCam_REFERENCE, &image16);
        
        tcp_publisher.publish(generatePointCloud(image16, triclops));
    }
    
    if (dc1394_video_set_transmission(stereoCamera.camera, DC1394_OFF) != DC1394_SUCCESS) {
        ROS_ERROR("[stereopsis]: Unable to stop the camera");
    }

    delete[] pucDeInterlacedBuffer;
    delete[] pucRGBBuffer;
    delete[] pucGreenBuffer;

	tcp_publisher.finish();

    cleanupAndExit(camera);
}
