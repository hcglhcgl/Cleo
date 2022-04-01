#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <ctime>
#include <raspicam/raspicam_cv.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <fstream>

using namespace cv;
using namespace std;


const float calibrationSqaureDim = 0.020f; // meters
//const float arucoSqaureDim = 0.0175f; // meters
const float arucoSqaureDim = 0.1000f; // meters
const Size chessboardDim  = Size(7,9);


// create aruco markers if needed for printing or for image library 
void createArucoMarkers(){ 
	Mat markerImage;
	
	Ptr <aruco::Dictionary> markerDictionary= aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	
	for(int i = 1; i < 5; i++)
	{
	aruco::drawMarker(markerDictionary,i,100,markerImage, 1);
	ostringstream convert;
	string ImageName = "4x4Marker_";
	convert << ImageName << i << ".jpg";
	imwrite(convert.str(),markerImage);
	}
}

//create known board posistions
void createKnownBoardPos(Size boardSize, float squareEdgeLength, vector<Point3f>& corners){
	
	for(int i = 0; i<boardSize.height ;i++)
	{
			for(int j =0; j<boardSize.width ; j++)
			{
				corners.push_back(Point3f(j*squareEdgeLength,i*squareEdgeLength,0.0f));
			}
	}
}

// get chess board Corners
void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allCorners, bool ShowResults = false){
	
	for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++){
		
		vector<Point2f> pointBuf;
		bool found = cv::findChessboardCorners(*iter, Size(7,9), pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
		
		
		if(found)
		{
			allCorners.push_back(pointBuf);
			
		}
		
		if(ShowResults)
		{
			cv::drawChessboardCorners(*iter, Size(7,9) , pointBuf, found);
			imshow("Looking for Corners", *iter);
			waitKey(0);
		}
		
	}
}

//Marker detection 
int MarkerDetection(Mat& cameraMatrix, Mat& distortionCoef, float arucoSqaureDim,  vector<Vec3d>& rotationVectors, vector<Vec3d>& translationVectors){

	time_t timer_begin,timer_end;
    raspicam::RaspiCam_Cv Camera;
	Mat image;
	
	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners, rejectedcandidates;
	aruco::DetectorParameters parameters; 
	
	Ptr <aruco::Dictionary> markerDictionary= aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
			
    //set camera params
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
    //Open camera
    cout<<"Opening Camera..."<<endl;
	if (!Camera.open()){
		cerr<<"Error opening the camera"<<endl;
		return -1;
	}
	
	namedWindow("Raspicam", CV_WINDOW_AUTOSIZE);
	
	
	while(true)
	{
		if(!Camera.grab()) //if we are unable to grab anything we break
			break;
		
		Camera.grab();		//grab image
		Camera.retrieve(image);		//retrives image from grab buffer
		
		//detect marker if present in image and markerdictionary
		aruco::detectMarkers(image, markerDictionary, markerCorners, markerIds);
		//Estimate pose of single marker and return rotation and translation vectors for marker position from camera in camera coordiantes system
		aruco::estimatePoseSingleMarkers(markerCorners, arucoSqaureDim, cameraMatrix, distortionCoef, rotationVectors, translationVectors);
		
		//cout << markerCorners[0] << endl;  // if found marker corners is wanted
		
		cout << "RotationVec" << endl;
		for(int i = 0; i<rotationVectors.size(); i++)
		{
			cout << rotationVectors[i]; 			//output live rotation vector
		}
		cout << endl;
		
		cout << "TranslationVec" << endl;
		for(int i=0; i<translationVectors.size(); i++)
		{
			cout << translationVectors[i] << endl;  //output live translation vector
		}
		cout << endl;
		
		for(unsigned j=0; j<markerIds.size(); j++)
		{
			aruco::drawAxis(image, cameraMatrix, distortionCoef, rotationVectors[j], translationVectors[j], 0.1f); //draws marker coordinate vectors on feducial
		}
		imshow("Raspicam", image);
		if(waitKey(30)>= 0) 
		{
			cv::imwrite("raspicam_cv_image1.jpg",image); //save image from marker found when breaking from marker detection
			break; 	
		}
			
		//cout<< markerIds.at(0) <<endl; //If marker id is wanted 
		
		

	}
	
	return 1;
}

//Calibration of camera with saved images in image buffer

void CamCalibration(vector<Mat> calibrationImages, Size boardsize, float sqaureEdgeLength, Mat& cameraMatrix, Mat& distortionCoef){
	
	vector<vector<Point2f>> chessboardImgSpacePoint;
	vector<vector<Point3f>> worldSpaceCornerPoints(1);
	vector<Mat> rvector, tvector; 
	distortionCoef = Mat::zeros(8,1,CV_64F);
	
	getChessboardCorners(calibrationImages, chessboardImgSpacePoint, false);  
	
	createKnownBoardPos(boardsize, sqaureEdgeLength, worldSpaceCornerPoints[0]);
	
	worldSpaceCornerPoints.resize(chessboardImgSpacePoint.size(), worldSpaceCornerPoints[0]); 
	
	cv::calibrateCamera(worldSpaceCornerPoints, chessboardImgSpacePoint, boardsize, cameraMatrix, distortionCoef, rvector, tvector);
	
}


//Save Camera calibration to file 

bool saveCamCalib(string name, Mat cameraMatrix, Mat distortionCoef){
	
	ofstream outStream(name);
	if(outStream)
	{
		uint16_t rows = cameraMatrix.rows; 
		uint16_t columns = cameraMatrix.cols;
		
		outStream << rows << endl; 
		outStream << columns << endl; 
		
		for(int r = 0; r < rows; r++)
		{
				for(int c = 0; c<columns; c++)
				{
						double val = cameraMatrix.at<double>(r,c);
						outStream << val << endl; 
				}
		}
		
		rows = distortionCoef.rows;
		columns = distortionCoef.cols; 
	
		outStream << rows << endl; 
		outStream << columns << endl; 
	
		for(int r = 0; r < rows; r++)
		{
				for(int c = 0; c<columns; c++)
				{
						double val = distortionCoef.at<double>(r,c);
						outStream << val << endl; 
				}
		}
	outStream.close();
	return true;
	
	}
	return false; 
} 


bool loadCamCalib(string name, Mat& cameraMatrix, Mat& distortionCoef)
{
	ifstream inStream(name);
	if(inStream)
	{
		uint16_t rows;
		uint16_t columns; 
		
		inStream >> rows; 
		inStream >> columns; 
		
		// cameraMatrix 
		
		cameraMatrix = Mat(Size(columns, rows), CV_64F);
		
		for(int r=0; r <rows; r++)
		{
			for(int c = 0; c<columns; c++)
			{
				double read= 0.0f;
				inStream >> read; 
				cameraMatrix.at<double>(r,c) = read; 
				cout << cameraMatrix.at<double>(r,c) << "\n";
			}
		}
		
		//Distortion Ceofficents 
		
		inStream >> rows; 
		inStream >> columns; 
		
		
		distortionCoef = Mat::zeros(rows, columns, CV_64F);
		
		for(int r= 0; r <rows; r++)
		{
			for(int c = 0; c<columns; c++)
			{
				double read= 0.0f;
				inStream >> read; 
				distortionCoef.at<double>(r,c) = read; 
				cout << distortionCoef.at<double>(r,c) << "\n";
			}
		}
		inStream.close();
		return true;
		
	}
	return false; 
}


int main(int argc,char ** argv)
{
	
	//createArucoMarkers(); // creates aruco markers form the 4x4_50_100 dictionary 
	
	Mat frame; 
	Mat drawtoframe; 
	
	Mat cameraMatrix = Mat::eye(3,3,CV_64F);
	
	Mat distortionCoef; 
	
	vector<Mat> savedImages; 
	
	//vector<vector<Point2f>> markerCorners;
	
	vector<Vec3d> rotationVectors, translationVectors;
	
	raspicam::RaspiCam_Cv Camera;
    Mat image;
    //set camera params
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
	if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return 0;}
	
	int FPS = 20; 
	
	namedWindow("RaspiCam" , CV_WINDOW_AUTOSIZE);
	/*
	loadCamCalib("cameraMatrix coef", cameraMatrix, distortionCoef);
	
	MarkerDetection(cameraMatrix, distortionCoef, arucoSqaureDim, rotationVectors, translationVectors);
	
	vector<Point2f> imagePoints;
	
	vector<Point3f> ObjectPoints;
	
	
	//Get Image points from marker world points 
	ObjectPoints.push_back(Point3f(0.0,0.0,0.0));
	cv::projectPoints(ObjectPoints, rotationVectors, translationVectors, cameraMatrix, distortionCoef, imagePoints);
		
	
	cout << "cameraMatrix" << endl; 
	cout << cameraMatrix<< endl; 
	
	cout << "distcoef" << endl; 
	cout << distortionCoef << endl; 
	
	cout << "ImagePoint" << endl;
	cout << imagePoints << endl;
	
	Mat R; 
	
	cv::Rodrigues(rotationVectors, R);
	
	cout << "R mat" << endl; 
	cout << R << endl; 
	
	Mat cameraRot, cameraTrans, transVec(translationVectors), rotVec(rotationVectors);
	
	
	cout << translationVectors[0]<< endl;
	
	//Vec3d new_trans(translationVectors[1] , translationVectors[1] , translationVectors[1] );
	
	//cout << rotVec << endl;   

	cv::Rodrigues(R.t(), cameraRot);
	
	
	cameraTrans = -R.t()*Mat(translationVectors[0]);
	
	cout << "test2" << endl;
	
	cout << "camRot" << endl; 
	cout <<  cameraRot << endl; 
	
	cout << "camTrans" << endl;
	cout << cameraTrans << endl; 
	*/
	
	
	  //Code for live camera calibration 
	while(true)
	{
		if(!Camera.grab())
			break;
		
		Camera.grab();
		Camera.retrieve(frame);
		
		vector<Vec2f> foundPoints;
		bool found = false;
		
		found = cv::findChessboardCorners(frame, chessboardDim, foundPoints,  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
		frame.copyTo(drawtoframe);
		cv::drawChessboardCorners(drawtoframe, chessboardDim, foundPoints, found);
		
		if(found){
			imshow("RaspiCam", drawtoframe);
			cout << "found"<<endl;
		}
		else{
			imshow("RaspiCam", frame);
			cout << "Not found"<<endl;
		}
		char character = waitKey(1000/FPS); 
		
		switch(character)
		{
		case ' ': 			//Space key pressed with the Raspicam window highlighted !NOT THE TERMINAL!
			// Image save
			if(found)
			{
				Mat temp; 
				frame.copyTo(temp);
				savedImages.push_back(temp);
				cout << "Imagesaved" << endl;
				cv::imwrite("Calibration_image.jpg",drawtoframe); //save image
				
			}
			break; 
		case 13: 			//Return/enter key pressed with the Raspicam window highlighted !NOT THE TERMINAL!
			//start Calibration
			if(savedImages.size() > 20)
			{
			cout << "Calibrating" << endl; 
			CamCalibration(savedImages, chessboardDim, calibrationSqaureDim, cameraMatrix, distortionCoef);
			saveCamCalib("cameraMatrix coef", cameraMatrix, distortionCoef);
			cout << "Calibrated and saved to file" << endl; 
			}
			break;
		case 27: 			//Escape key pressed with the Raspicam window highlighted !NOT THE TERMINAL!
			//exit 
			return 0;
			break;  
		}
		
	}
	
	return 0 ; 

}
