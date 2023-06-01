#include <iostream>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <signal.h>
#include "dxl.hpp"
using namespace std;
using namespace cv;
using namespace cv::dnn;
const float CONFIDENCE_THRESHOLD = 0.9;
const float NMS_THRESHOLD = 0.5;
const int NUM_CLASSES = 1;
// colors for bounding boxes
const cv::Scalar colors[] = {
{0, 255, 255},
{255, 255, 0},
{0, 255, 0},
{255, 0, 0}
};
const auto NUM_COLORS = sizeof(colors) / sizeof(colors[0]);
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}
int main()
{
    //string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12, framerate=(fraction)120/1 ! \
                nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx! videoconvert ! \
                video/x-raw, format=(string)BGR !appsink";
    string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12 ! \
                nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx! videoconvert ! \
                video/x-raw, format=(string)BGR !appsink";
    string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.169 port=8004 sync=false";

    struct timeval start,end1,end2,end3;
    double diff1,diff2,diff3;
    bool mode =false;
	char ch;

    int l_spd = 130 ,r_spd = 130;
    int l_speed = 0 , r_speed =0;
    int size = 0;
    int error = 0;
    double gain = 0.0015;
    int old_error = 0;
    int size_100 =0;
    int size_1000 =0;
    float max =0;
    Dxl test;

    VideoCapture source(src, CAP_GSTREAMER); //Gstreamer 명령어로 카메라를 열어줌
    if (!source.isOpened()){ cout << "Video error" << endl; return -1; }

    VideoWriter writer(dst, 0, (double)30, cv::Size(640,360), true);
    if (!writer.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}

    signal(SIGINT, ctrlc);

    std::vector<std::string> class_names={"leader"};
    auto net = cv::dnn::readNetFromDarknet("yolov4-tiny-leader.cfg", "yolov4-tiny-leader_final.weights");
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    //net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    //net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    auto output_names = net.getUnconnectedOutLayersNames();

    test.open();
    while(true)
    {
        gettimeofday(&start,NULL);
        Mat frame;
        source >> frame;
        cv::Mat blob;
        std::vector<cv::Mat> detections;
        std::vector<int> indices[NUM_CLASSES];
        std::vector<cv::Rect> boxes[NUM_CLASSES];
        std::vector<float> scores[NUM_CLASSES];
        cv::dnn::blobFromImage(frame, blob, 1/255.f, cv::Size(288, 288), cv::Scalar(), true, false, CV_32F);
        net.setInput(blob);
        net.forward(detections, output_names);
        for (auto& output : detections)
        {
            const auto num_boxes = output.rows;
            for (int i = 0; i < num_boxes; i++)
            {
                auto x = output.at<float>(i, 0) * frame.cols;
                auto y = output.at<float>(i, 1) * frame.rows;
                auto width = output.at<float>(i, 2) * frame.cols;
                auto height = output.at<float>(i, 3) * frame.rows;
                cv::Rect rect(x - width / 2, y - height / 2, width, height);
                for (int c = 0; c < NUM_CLASSES; c++)
                {
                    auto confidence = *output.ptr<float>(i, 5 + c);
                    if (confidence >= CONFIDENCE_THRESHOLD)
                    {
                        boxes[c].push_back(rect);
                        scores[c].push_back(confidence);
                    }
                }
            }
        }
        for (int c = 0; c < NUM_CLASSES; c++)
            cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD, indices[c]);
        for (int c = 0; c < NUM_CLASSES; c++)
        {
            if(indices[c].size() == 0)
            {
                error = old_error;
            }
            cout <<"갯수: " << indices[c].size();
            for (int i = 0; i < indices[c].size(); ++i)
            {
                const auto color = colors[c % NUM_COLORS];
                auto idx = indices[c][i];
                const auto& rect = boxes[c][idx];
                cv::rectangle(frame, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width, rect.y + rect.height), color, 3);
                std::string label_str = class_names[c] + ": " + cv::format("%.02lf",scores[c][idx]);
                int baseline;
                auto label_bg_sz = cv::getTextSize(label_str,
                cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
                cv::rectangle(frame, cv::Point(rect.x, rect.y - label_bg_sz.height - baseline - 10), cv::Point(rect.x + label_bg_sz.width, rect.y), color, cv::FILLED);
                cv::putText(frame, label_str, cv::Point(rect.x, rect.y - baseline - 5),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));

                size = rect.width * rect.height;
                size_100 = size % 1000;

                size_1000 = size - size_100;

                int centroids_x = rect.x + (rect.width / 2);
                int centroids_y = rect.y + (rect.height / 2);
                cv::circle(frame, cv::Point(centroids_x,centroids_y),3,Scalar(0,255,255),1);
                error = frame.cols/2 - centroids_x;
                old_error = error;
                cout << "  centroids_x: " <<centroids_x <<"  centroids_y: " << centroids_y << "  확률: " << scores[c][idx];
            }
        }
        l_speed = gain * (100000 - size_1000) + 0.1 * error; //0.0015 lidar_following
        r_speed = gain * (100000 - size_1000) - 0.1 * error;
        //l_speed = gain * (110000 - size_1000) + 0.15 * error; //0.0015 speed-70
        //r_speed = gain * (110000 - size_1000) - 0.15 * error;

        if(mode) test.setVelocity(r_speed, -l_speed);

        if(test.kbhit())
		{ 
			ch = test.getch();
			if(ch == 'q') break;
			else if(ch == 's') mode = true;			
		}

        gettimeofday(&end1,NULL);
        writer << frame;
        //if(waitKey(1) == 'q' || waitKey(1) == 'Q') break;
        if(ctrl_c_pressed)
        {
            break;
        }
        diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;
        cout << " time1:" << diff1;
        cout << "  size:" << size_1000 << endl;
        cout << "  error: " <<error << "  1_spd: " << l_speed <<"  r_spd: " << r_speed << endl;
    }
    test.close();
    return 0;
}