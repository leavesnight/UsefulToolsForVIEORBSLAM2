#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, char **argv) {
    cout << "Hello, world!" << endl;
    cv::Mat mat(4,4,CV_32F,1.5),mat2(3,1,CV_32F,2);
    //cv::Mat B=mat(cv::Rect(3,0,1,3))=mat2.clone();
    cv::Mat B=mat.rowRange(0,3).col(3);
    mat2.copyTo(mat.rowRange(0,3).col(3));
    cout<<mat<<endl;
    cout<<mat(cv::Rect(3,0,1,3))<<endl;
    cout<<B;
    
    return 0;
}
