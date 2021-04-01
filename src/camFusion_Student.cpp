
#include <iostream>
#include <algorithm>
#include <numeric>
#include <iomanip>
#include <fstream>
#include <unordered_map>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

struct stat st2;

int FindVectorOccurrences(vector<int> value);

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, int imgIndex , cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT )
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }                

    } // eof loop over all Lidar points

    cout << left;
    cout << setfill(' ');
    cout << setw(12) << "image_num";        
    cout << setfill(' ');
    cout << setw(8) << "boxID";
    cout << setfill(' ');
    cout << setw(8) << "ClassID"; 
    cout << setfill(' '); 
    cout << setw(13) << "Class_Label";
    cout << setfill(' ');
    cout << setw(13) << "confidence";
    cout << setfill(' ');
    cout << setw(8) << "roi_y";
    cout << setfill(' ');
    cout << setw(8) << "roi_x"; 
    cout << setfill(' ');       
    cout << setw(12) << "roi_width";
    cout << setfill(' ');
    cout << setw(12) << "roi_height";
    cout << setfill(' ');
    cout << setw(15) << "roi_points";
    cout << setfill(' ');
    cout << endl;

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {

        cout << left;   
        cout << setfill(' ');
        cout << setw(12) << imgIndex+1;              
        cout << setfill(' ');
        cout << setw(8) << (*it1).boxID;
        cout << setfill(' ');
        cout << setw(8) << (*it1).classID;
        cout << setfill(' ');
        cout << setw(13) << (*it1).className;         
        cout << setfill(' ');
        cout << setw(13) << (*it1).confidence;
        cout << setfill(' ');
        cout << setw(8) << (*it1).roi.y;
        cout << setfill(' ');
        cout << setw(8) << (*it1).roi.x; 
        cout << setfill(' ');       
        cout << setw(12) << (*it1).roi.width;
        cout << setfill(' ');
        cout << setw(12) << (*it1).roi.height;
        cout << setfill(' ');
        cout << setw(15) << (*it1).lidarPoints.size();
        cout << setfill(' ');
        cout << endl;       
        
        
        fstream report;
        report.open("../output/ObjectDetc2D_lidar_ROI.txt", std::ios_base::app);
        report <<imgIndex+1<<","<<(*it1).boxID<<"," <<
                (*it1).classID<<","<<(*it1).className <<","<<
                (*it1).confidence<<","<<(*it1).roi.y<<","<<(*it1).roi.x<<
                ","<< (*it1).roi.width <<","<<(*it1).roi.height<<","<<(*it1).lidarPoints.size()<<"\n";
        report.close(); 
    }

}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, int imgIndex,bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    //string windowName = "3D Objects";
    //cv::namedWindow(windowName, 0);
    //cv::imshow(windowName, topviewImg);
    ostringstream saving_name;
    saving_name << imgIndex<<"_LIDAR_ROI";
    string file_name = saving_name.str();    
    string path_file = "../output/Lidar_ROI_3D_View/";
    // creating directory with detector type name
    const char* p_c_str = path_file.c_str();
    
    //checking and creating directory
    if(stat(p_c_str,&st2) != 0)
      mkdir( p_c_str , 0777);

    imwrite(path_file +"img"+ file_name+".jpg",topviewImg);       
    //cv::waitKey(0);
    
    /*
    if(bWait)
    {
        cv::waitKey(); // wait for key to be pressed
    }
    */
}

//clusterKptMatchesWithROI(*currBB, (dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->kptMatches);
// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    vector<double> DistanceEuclideanCoord;
    
    // iterating over matching points in order to create a vector with all distances
    for(auto it1 = kptMatches.begin(); it1 != kptMatches.end(); it1++)
    {
        if(boundingBox.roi.contains(kptsCurr.at(it1->trainIdx).pt)) // checking if the keypoint is inside of the bounding box of the current frame.
        { 
            // calculating euclidean Distance with opencv function for each matched points
            DistanceEuclideanCoord.push_back(cv::norm(kptsCurr.at(it1->trainIdx).pt - kptsPrev.at(it1->queryIdx).pt)); 
        }
    }



    // calculating mean and standar deviation.
    double sum = accumulate(begin(DistanceEuclideanCoord), end(DistanceEuclideanCoord), 0.0);
    double m =  sum / DistanceEuclideanCoord.size();
    
    double accum = 0.0;
    for_each (begin(DistanceEuclideanCoord), std::end(DistanceEuclideanCoord),[&](const double d){accum += (d - m) * (d - m);});
    double stdev = sqrt(accum / (DistanceEuclideanCoord.size()-1));

    double min_acceptable_value = m - stdev;
    double max_acceptable_value = m + stdev;

    // removing outliers and pushing the acceptable values to bounding boxes of the current frame.
    for(auto it2 = kptMatches.begin(); it2 != kptMatches.end(); it2++)
    {
        if(boundingBox.roi.contains(kptsCurr.at(it2->trainIdx).pt)) // checking if the keypoint is inside of the bounding box of the current frame.
        { 
            // calculating euclidean Distance with opencv function for each matched points
            double points_distance = cv::norm(kptsCurr.at(it2->trainIdx).pt - kptsPrev.at(it2->queryIdx).pt);

            if (( points_distance > min_acceptable_value ) && ( points_distance < max_acceptable_value))
            {
                boundingBox.kptMatches.push_back(*it2);
                boundingBox.keypoints.push_back(kptsCurr.at(it2->trainIdx));
            }
        }
    }
}




// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg) 
{
    vector <double> filtered_dist_ratio;
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // calculating mean and standar deviation.
    double sum = accumulate(begin(distRatios), end(distRatios), 0.0);
    double m =  sum / distRatios.size();
    
    double accum = 0.0;
    for_each (begin(distRatios), std::end(distRatios),[&](const double d){accum += (d - m) * (d - m);});
    double stdev = sqrt(accum / (distRatios.size()-1));

    double min_acceptable_value = m - stdev;
    double max_acceptable_value = m + stdev;

    
    for (int it3 = 0 ; it3 < distRatios.size(); ++it3)
    {
        if ((distRatios[it3]>min_acceptable_value) && ( distRatios[it3] < max_acceptable_value))
        {
            filtered_dist_ratio.push_back(distRatios[it3]);
        }
    }
    if (filtered_dist_ratio.size() == 0)
    {
        TTC = NAN;
        return;
    }  	
    std::sort(filtered_dist_ratio.begin(), filtered_dist_ratio.end());
    long medIndex = floor(filtered_dist_ratio.size() / 2.0);
    // compute median dist. ratio to remove outlier influence over the Filtered distance ratio vector.
    double medDistRatio = filtered_dist_ratio.size() % 2 == 0 ? (filtered_dist_ratio[medIndex - 1] + filtered_dist_ratio[medIndex]) / 2.0 : filtered_dist_ratio[medIndex];
    // compute median dist. ratio to remove outlier influence
    

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
    // EOF STUDENT TASK
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)

{
    // auxiliary variables
    double dT = 1 / frameRate;  // time between two measurements in seconds

    float A_Ycoord = -0.5;
    float B_Ycoord = 0;
    float C_Ycoord = 0.5;
    float zone_width = .15; // in order to filter the points only in front of the car.   

    float xwmin_prev_zA=1e8;
    float xwmin_prev_zB=1e8;
    float xwmin_prev_zC=1e8;

    for (auto it1 = lidarPointsPrev.begin(); it1 != lidarPointsPrev.end(); ++it1)
    {
        // world coordinates
        float xw_prev = (*it1).x; // world position in m with x facing forward from sensor
        float yw_prev = (*it1).y; // world position in m with y facing left from sensor
      
        if ((yw_prev > A_Ycoord- zone_width/2) && (yw_prev < A_Ycoord + zone_width/2 ))
        {
            xwmin_prev_zA = xwmin_prev_zA<xw_prev ? xwmin_prev_zA : xw_prev;
        }
        if ((yw_prev > B_Ycoord- zone_width/2) && (yw_prev < B_Ycoord + zone_width/2 ))
        {
            xwmin_prev_zB = xwmin_prev_zB<xw_prev ? xwmin_prev_zB : xw_prev;
        }        
        if ((yw_prev > C_Ycoord- zone_width/2) && (yw_prev < C_Ycoord + zone_width/2 ))
        {
            xwmin_prev_zC = xwmin_prev_zC<xw_prev ? xwmin_prev_zC : xw_prev; 
        }   
    }
    //cout <<xwmin_prev_zA << ","<< xwmin_prev_zB << ","<< xwmin_prev_zC<<endl;


    float xwmin_curr_zA=1e8;
    float xwmin_curr_zB=1e8;
    float xwmin_curr_zC=1e8;

    for (auto it2 = lidarPointsCurr.begin(); it2 != lidarPointsCurr.end(); ++it2)
    {
        // world coordinates
        float xw_curr = (*it2).x; // world position in m with x facing forward from sensor
        float yw_curr = (*it2).y; // world position in m with y facing left from sensor
        
        if ((yw_curr > A_Ycoord- zone_width/2) && (yw_curr < A_Ycoord + zone_width/2 ))
        {
            xwmin_curr_zA = xwmin_curr_zA<xw_curr ? xwmin_curr_zA : xw_curr;
        }
        if ((yw_curr > B_Ycoord- zone_width/2) && (yw_curr < B_Ycoord + zone_width/2 ))
        {
            xwmin_curr_zB = xwmin_curr_zB<xw_curr ? xwmin_curr_zB : xw_curr;
        }        
        if ((yw_curr > C_Ycoord- zone_width/2) && (yw_curr < C_Ycoord + zone_width/2 ))
        {
            xwmin_curr_zC = xwmin_curr_zC<xw_curr ? xwmin_curr_zC : xw_curr;
        }   
    }    

    //cout << xwmin_curr_zA << ","<< xwmin_curr_zB << ","<< xwmin_curr_zC<<endl;
   

    double TTC_A = abs(xwmin_curr_zA * dT / (xwmin_prev_zA - xwmin_curr_zA));
    double TTC_B = abs(xwmin_curr_zB * dT / (xwmin_prev_zB - xwmin_curr_zB));
    double TTC_C = abs(xwmin_curr_zC * dT / (xwmin_prev_zC - xwmin_curr_zC));

    // compute TTC from both measurements
    TTC = (TTC_A+TTC_B+TTC_C)/3;
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{ 
    vector<vector<int>>final_result(currFrame.boundingBoxes.size());
    int bBox_curr = 0;
    int bBox_prev = 0;

    // iterating over matching points in order to create a vector with all distances
    for(auto it1 = matches.begin(); it1 != matches.end(); it1++)
    {   
        double prev_X = prevFrame.keypoints.at(it1->queryIdx).pt.x ;
        double prev_Y = prevFrame.keypoints.at(it1->queryIdx).pt.y ;
        double curr_X = currFrame.keypoints.at(it1->trainIdx).pt.x ;
        double curr_Y = currFrame.keypoints.at(it1->trainIdx).pt.y ;        
        for(auto it2 = prevFrame.boundingBoxes.begin(); it2 != prevFrame.boundingBoxes.end(); it2++)
        {        
            //checking x vector.
            if ( prev_X >(*it2).roi.x && prev_X < ((*it2).roi.x + (*it2).roi.width))
            {
                //checking y vector.
                if ( (prev_Y >(*it2).roi.y) && (prev_Y < ((*it2).roi.y + (*it2).roi.height)))
                {          
                    bBox_prev = it2->boxID;                   
                }
            }          
        }
        
        for(auto it3 = currFrame.boundingBoxes.begin(); it3 != currFrame.boundingBoxes.end(); it3++)
        {         
            //checking x vector.
            if ( curr_X >(*it3).roi.x && curr_X < ((*it3).roi.x + (*it3).roi.width))
            {
                //checking y vector.
                if ( (curr_Y >(*it3).roi.y) && (curr_Y < ((*it3).roi.y + (*it3).roi.height)))
                {          
                    bBox_curr = it3->boxID;
                }
            }           
        }

        final_result[bBox_curr].push_back(bBox_prev);
      
    } // eol loop the mathes vector
    
    // storing in bbBestMatches the pairs of the matching bounding boxes < previous , current >

    //cout << "the size of final vector of bbmatches is : " << final_result.size()<<endl;
    int bbox_current = 0;
    for(int it4 = 0 ; it4< final_result.size() ; it4++ )
    {
        int bBox_matching_prev = FindVectorOccurrences(final_result[it4]);
        bbBestMatches.insert(pair<int, int>(bBox_matching_prev, bbox_current));
        //cout << bbox_current << ","<< bBox_matching_prev << endl;
        bbox_current ++;
    }
}

// stackoverflow solution https://stackoverflow.com/questions/2488941/find-which-numbers-appears-most-in-a-vector/55478213#55478213
int FindVectorOccurrences(vector<int> value)
{ 
    int index = 0;
    int highest = 0;
    for (unsigned int a = 0; a < value.size(); a++)
    {
        int count = 1;
        int Position = value.at(a);
        for (unsigned int b = a + 1; b < value.size(); b++)
        {
            if (value.at(b) == Position)
            {
                count++;
            }
        }
        if (count >= index)
        {
            index = count;
            highest = Position;
        }
    }
    return highest;
}