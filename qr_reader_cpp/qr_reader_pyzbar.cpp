
#include <iostream>
#include <fstream>
#include <utility>
#include <vector>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <math.h>       /* floor */
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */
#include <stdio.h>
#include <zbar.h>

using namespace cv;
using namespace std;
using namespace zbar;


//  sudo apt install libzbar-dev

typedef struct
{
  string type;
  string data;
  vector <Point> location;
} decodedObject;

// Find and decode barcodes and QR codes
void decode(Mat &im, vector<decodedObject>&decodedObjects)
{
   
  // Create zbar scanner
  ImageScanner scanner;
 
  // Configure scanner
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
   
  // Convert image to grayscale
  Mat imGray;
  cvtColor(im, imGray,COLOR_BGR2GRAY);
 
  // Wrap image data in a zbar image
  Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);
 
  // Scan the image for barcodes and QRCodes
  int n = scanner.scan(image);
   
  // Print results
  for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
  {
    decodedObject obj;
     
    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();
     
    // Print type and data
    cout << "Type : " << obj.type << endl;
    cout << "Data : " << obj.data << endl << endl;
     
    // Obtain location
    for(int i = 0; i< symbol->get_location_size(); i++)
    {
      obj.location.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
    }
     
    decodedObjects.push_back(obj);
  }
}

// Display barcode and QR code location  
void display(Mat &im, vector<decodedObject>&decodedObjects)
{
  // Loop over all decoded objects
  for(int i = 0; i < decodedObjects.size(); i++)
  {
    vector<Point> points = decodedObjects[i].location;
    vector<Point> hull;
     
    // If the points do not form a quad, find convex hull
    if(points.size() > 4)
      convexHull(points, hull);
    else
      hull = points;
     
    // Number of points in the convex hull
    int n = hull.size();
     
    for(int j = 0; j < n; j++)
    {
      line(im, hull[j], hull[ (j+1) % n], Scalar(255,0,0), 3);
    }
     
  }
   
  // Display results 
  imshow("Results", im);
  waitKey(1);
   
}

int main(int argc, char* argv[])
{

    Mat frame;
    // Mat im = imread("zbar-test.jpg");
    
    //--- INITIALIZE VIDEOCAPTURE
    //VideoCapture cap("rtsp://192.168.0.219:554/media1.sdp");
    VideoCapture cap(0);

    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    //int deviceID = 0;             // 0 = open default camera
    //int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    //cap.open(deviceID + apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    //--- GRAB AND WRITE LOOP
    cout << "Start grabbing" << endl
        << "Press any key to terminate" << endl;

    namedWindow( "Results", WINDOW_NORMAL );
    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        //cv::Rect roi;
        //roi.x = frame.size().width/2 - 400;
        //roi.y = frame.size().height/2 -400;
        //roi.width =  800;
        //roi.height = 800;

        /* Crop the original image to the defined ROI */
        //cv::Mat frame_crop = frame(roi);

        // Variable for decoded objects 
        vector<decodedObject> decodedObjects;
        
        // Find and decode barcodes and QR codes
        decode(frame, decodedObjects);
        
        // Display location 
        display(frame, decodedObjects);

        // show live and wait for a key with timeout long enough to show images
        //imshow("Live", frame);
        if (waitKey(5) >= 0)
            break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return EXIT_SUCCESS;
}
