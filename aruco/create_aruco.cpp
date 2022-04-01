# include "create_aruco.hpp"
# include <stdio.h> 
# include <vector>
# include <map>
// openCV libs
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/imgproc/imgproc.hpp"
# include "opencv2/opencv.hpp"
# include "opencv2/core/core.hpp"
# include <opencv2/aruco.hpp> //added

# include <sys/time.h>

# include <math.h>

// create aruco markers if needed for printing or for image library 
void createArucoMarkers(){ 
	Mat markerImage;
	
	Ptr <aruco::Dictionary> markerDictionary= aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	
    //Drawing white marker
	aruco::drawMarker(markerDictionary,5,300,markerImage, 1);
	ostringstream convert;
	string ImageName = "4x4Marker_WHITE_ID_5";
	convert << ImageName << ".jpg";
	imwrite(convert.str(),markerImage);

	//Drawing red marker
	aruco::drawMarker(markerDictionary,19,300,markerImage, 1);
	ostringstream convert2;
	string ImageName2 = "4x4Marker_RED_ID_19";
	convert2 << ImageName2 << ".jpg";
	imwrite(convert2.str(),markerImage);
}

int main( int argc, char** argv )
{
    createArucoMarkers();
}