
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


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<double> euclideanDistance;

    for(auto it = kptMatches.begin(); it != kptMatches.end(); it++)
    {
        int currKptIndex = (*it).trainIdx;
        const auto &currKeyPoint = kptsCurr[currKptIndex];

        if(boundingBox.roi.contains(currKeyPoint.pt))
        {
            int prevKptIndex = (*it).queryIdx;
            const auto &prevKeyPoint = kptsPrev[prevKptIndex];

            euclideanDistance.push_back(cv::norm(currKeyPoint.pt - prevKeyPoint.pt));
        }
    }

    int pair_num =  euclideanDistance.size();
    double euclideanDistanceMean = std::accumulate(euclideanDistance.begin(), euclideanDistance.end(), 0.0) / pair_num;

    for(auto it = kptMatches.begin(); it != kptMatches.end(); it++)
    {
        int currKptIndex = (*it).trainIdx;
        const auto &currKeyPoint = kptsCurr[currKptIndex];

        if(boundingBox.roi.contains(currKeyPoint.pt))
        {
            int prevKptIndex = (*it).queryIdx;
            const auto &prevKeyPoint = kptsPrev[prevKptIndex];

            double temp = cv::norm(currKeyPoint.pt - prevKeyPoint.pt);

            double euclideanDistanceMean_Augment = euclideanDistanceMean * 1.3;
            if(temp < euclideanDistanceMean_Augment)
            {
                boundingBox.keypoints.push_back(currKeyPoint);
                boundingBox.kptMatches.push_back(*it);
            }
        }
    }
    std::cout << "mean value: " << euclideanDistanceMean << "Before filtering there are: " << pair_num << " and after filtering, there are " << boundingBox.keypoints.size() << std::endl;
}




// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{

    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

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
    }// eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();

    double dT = 1 / frameRate;
    TTC = -dT / (1 - meanDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)

{
    // auxiliary variables
    double dT = 1 / frameRate;  // time between two measurements in seconds
    double laneWidth = 4.0; // assumed width of the ego lane

    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        minXPrev = minXPrev > it->x ? it->x : minXPrev;
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        minXCurr = minXCurr > it->x ? it->x : minXCurr;
    }

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{ 
    vector<vector<int>>final_result(currFrame.boundingBoxes.size());
    
    // creating variable to store the corresponding boundingbox 
    int bBox_prev; 
    int bBox_curr;

    double gr_conf = 0;

    // loop through every Matching points:
    for(auto it1 = matches.begin(); it1 != matches.end(); it1++ )
    {
        //-----------------------------------------------------------------------------------------------------------
        //                         correponding the keypoint and Bounding Box on the previous frame
        //-----------------------------------------------------------------------------------------------------------
        //Getting pixel coordinates on priveous frame 
        double prev_X = prevFrame.keypoints.at(it1->queryIdx).pt.x ;
        double prev_Y = prevFrame.keypoints.at(it1->queryIdx).pt.y ;

        // creating vector to store the bbox that could be the correspondent to take decision on a further step
        map<int, double > prev_bBox_matching_checking; // 

        // iterating over boundingBoxes to check if the points is inside of box , if have more than 1 box for the same points
        // the one with the greatest confidence will get the keypoint.
        for(auto it2 = prevFrame.boundingBoxes.begin(); it2 != prevFrame.boundingBoxes.end(); it2++)
        {
      
            //checking x vector.
            if ( prev_X >(*it2).roi.x && prev_X < ((*it2).roi.x + (*it2).roi.width))
            {
                //checking y vector.
                if ( prev_Y >(*it2).roi.y && prev_Y < ((*it2).roi.y + (*it2).roi.height))
                {          
                    int boxId = it2->boxID;
                    double confidence = it2->confidence;         
                    prev_bBox_matching_checking.insert(pair<int, double>(boxId, confidence));
                    
                }
            }
        }
        
        // checking if the keypoints have more than 1 box to be considered.
        if ( prev_bBox_matching_checking.size() >= 1)
        {
            gr_conf = 0;
            for (auto it3 = prev_bBox_matching_checking.begin(); it3 != prev_bBox_matching_checking.end(); it3++)
            {
                if (it3-> second > gr_conf)
                {
                    bBox_prev = it3 -> first;
                    gr_conf = it3-> second;
                }               
                
            }
        }

        else
        {
            bBox_prev = 0;
        }
        
        //-----------------------------------------------------------------------------------------------------------
        //                         correponding the keypoint and Bounding Box on the Current frame
        //-----------------------------------------------------------------------------------------------------------

        //Getting pixel coordinates on current frame 
        double curr_X = currFrame.keypoints.at(it1->trainIdx).pt.x ;
        double curr_Y = currFrame.keypoints.at(it1->trainIdx).pt.y ;

        // creating vector to store the bbox that could be the correspondent to take decision on a further step
        map<int, double> curr_bBox_matching_checking; // 

        // iterating over boundingBoxes to check if the points is inside of box , if have more than 1 box for the same points
        // the one with the greatest confidence will get the keypoint.
        for(auto it4 = currFrame.boundingBoxes.begin(); it4 != currFrame.boundingBoxes.end(); it4++)
        {
      
            //checking x vector.
            if ( curr_X >(*it4).roi.x && curr_X < ((*it4).roi.x + (*it4).roi.width))
            {
                //checking y vector.
                if ( curr_Y >(*it4).roi.y && curr_Y < ((*it4).roi.y + (*it4).roi.height))
                {          
                    int boxId_curr = it4->boxID;
                    double confidence_curr = it4->confidence;         
                    curr_bBox_matching_checking.insert(pair<int, double>(boxId_curr, confidence_curr));


                }
            }
        }
     
        
        // checking if the keypoints have more than 1 box to be considered.
        if ( curr_bBox_matching_checking.size() >= 1)
        {
            gr_conf = 0;
            for (auto it5 = curr_bBox_matching_checking.begin(); it5 != curr_bBox_matching_checking.end(); it5++)
            {
                if (it5-> second > gr_conf)
                {
                    bBox_curr = it5 -> first;
                    gr_conf = it5-> second;
                }               
                
            }
        }
        else
        {
            bBox_curr = 0;
        }

        final_result[bBox_curr].push_back(bBox_prev);

    } // eol loop the mathes vector

    // storing in bbBestMatches the pairs of the matching bounding boxes < previous , current >

    //cout << "the size of final vector of bbmatches is : " << final_result.size()<<endl;
    int bbox_current = 0;
    for(int it6 = 0 ; it6< final_result.size() ; it6++ )
    {
        int bBox_matching_prev = FindVectorOccurrences(final_result[it6]);
        bbBestMatches.insert(pair<int, int>(bBox_matching_prev, bbox_current));
        //cout << bbox_current << ","<< bBox_matching_prev << endl;
        bbox_current ++;
    }

}


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