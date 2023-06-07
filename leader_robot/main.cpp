#include "opencv2/opencv.hpp"
#include "line.hpp"
#include <iostream>
#include <sys/time.h>
#include <signal.h>
#include <cmath>
using namespace cv;
using namespace std;
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}
int main()
{
    string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12, framerate=(fraction)120/1 ! \
                nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)320, format=(string)BGRx! videoconvert ! \
                video/x-raw, format=(string)BGR !appsink";
    string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.62.63 port=8001 sync=false";

    struct timeval start,end1,end2,end3;
    double diff1,diff2,diff3;

    VideoCapture source(src, CAP_GSTREAMER); //Gstreamer 명령어로 카메라를 열어줌
    if (!source.isOpened()){ cout << "Video error" << endl; return -1; }

    VideoWriter writer(dst, 0, (double)30, cv::Size(640,320/4), true);
    if (!writer.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}

    signal(SIGINT, ctrlc);

    Mat frame;
    Mat roi;
    Mat gray;
    Mat bright_mean;
    Mat ths;
    Mat img_labels, stats, centroids;
    Mat ths_gbr;
    int oi =0;
    //int r_speed = 54, l_speed = 54; //gain = 0.15
    //int r_spd = 54, l_spd = 54;
    int r_speed = 70, l_speed = 70;
    int r_spd = 70, l_spd = 70;
    int old_centroids_x= 0;
    Dxl test;

    test.dxl_open();
    while(true)
    {
        gettimeofday(&start,NULL);
        source >> frame;
        if(frame.empty())
        {
            cerr << "frame empty!" <<endl;
            break;
        }
        int x =0;
        int y = 3* frame.rows / 4;
        int width = frame.cols;
        int height = frame.rows / 4;
        int max = 1;
        int error = 0;
        int area_count =0;
        int object_index[100] = {0};
        int k =0;
        int min_object_index = 0;
        int distance =0;
        roi = frame(Rect(x,y,width,height));
        cvtColor(roi,gray,COLOR_BGR2GRAY);
        bright_mean = gray + (150 - mean(gray)[0]);
        threshold(bright_mean,ths,200,255,THRESH_BINARY);
        int i = connectedComponentsWithStats(ths,img_labels,stats,centroids);

        for(int j =1; j <i; j++)
        {
            int area = stats.at<int>(j,4);
            int x = stats.at<int>(j,0);
            int y = stats.at<int>(j,1);
            int width = stats.at<int>(j,2);
            int height = stats.at<int>(j,3);
            int centroids_x = centroids.at<double>(j,0);
            int centroids_y = centroids.at<double>(j,1);
            if(area > max)
            {
                max = area;
                oi = j;
            }
            cvtColor(ths,ths_gbr,COLOR_GRAY2BGR);
            if(area >1500 && area < 5000) //크기만 일치하는 영역 표시
            {
                rectangle(ths_gbr,Rect(x, y, width, height),Scalar(0,0,255),2);
                circle(ths_gbr,Point(centroids_x,centroids_y),1,Scalar(0,0,255),2);
                area_count += 1;
                object_index[k++] = j;
            }
        }
        int max_x = stats.at<int>(oi,0);
        int max_y = stats.at<int>(oi,1);
        int max_width = stats.at<int>(oi,2);
        int max_height = stats.at<int>(oi,3);
        int max_centroids_x = centroids.at<double>(oi,0);
        int max_centroids_y = centroids.at<double>(oi,1);

        distance = old_centroids_x - max_centroids_x;
        
        if(max >1500 && max <5000)
        {
            rectangle(ths_gbr,Rect(max_x,max_y,max_width,max_height),Scalar(0,0,255),2);
            circle(ths_gbr,Point(max_centroids_x,max_centroids_y),1,Scalar(0,0,255),2);
        }
        else
        {
            max_centroids_x =0;
        }
        if(max_centroids_x == 0)
            error = 0;
        else if(i < 2)
        {
             error = ths_gbr.cols/2 - old_centroids_x;
        }
        else
        {
            if(area_count > 1)
            {
                if(abs(distance) > 30)
                {
                    error = ths_gbr.cols/2 - old_centroids_x;
                }
                else
                {
                    error = ths_gbr.cols/2 - max_centroids_x;
                    old_centroids_x = max_centroids_x;
                }
            }
            else if(area_count == 0)
            {
                error = ths_gbr.cols/2 - old_centroids_x;
            }
            else
            {
                error = ths_gbr.cols/2 - max_centroids_x;
                old_centroids_x = max_centroids_x;
            }
        }
        //cout << "error: " << error <<endl;
        //cout << "max_centroids_x: " <<max_centroids_x <<endl;

        l_speed = l_spd - (0.17 * error);
        r_speed = r_spd + (0.17 * error);

        test.dxl_set_velocity(l_speed, -r_speed); 

        gettimeofday(&end1,NULL);
        //writer << frame;
        writer << ths_gbr;
        if(waitKey(1) == 'q' || waitKey(1) == 'Q') break;
        if(ctrl_c_pressed)
        {
            break;
        }
        diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;
        cout << "time1:" << diff1 << endl; 
        double fps = source.get(CAP_PROP_FPS);
        //cout << "FPS: " << fps << endl; 
        //cout << "width: " << width << "  height: " << height <<endl;
        //cout << "bright_mean: " << mean(frame)[0] <<endl;
        //cout << "max_area: " << max <<endl;
        //cout << "i: " <<i <<endl;
        //cout << "distance: " << abs(distance) <<endl;
        //cout << "area_count: " <<area_count <<endl;
        //cout << "max_centroids_x: " << max_centroids_x <<endl;
        //cout << "old_centroids_x: " << old_centroids_x <<endl;
    }
    test.dxl_close();
    return 0;
}
