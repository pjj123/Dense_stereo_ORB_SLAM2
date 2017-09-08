#include <cmath>
#include <opencv2/opencv.hpp>
#include <numeric>
#include <stdint.h>
#include <stdio.h>
using namespace cv;
using namespace std;

class NCC{
    public:
    Mat limg;
    Mat rimg;
    int hei;
    int wid;
    bool left_or_right;
    int maxdis;
    int weight;
    Mat x_w;
    Mat xx_w;
    Mat y_w;
    Mat * xy_w;
    Mat yy_w;
    Mat weight_ncc;
    Mat second_weight_ncc;
    Mat * ncc_w;
    Mat dis;
public:
    Mat getdis(void);
    void buildcv();
    NCC(const Mat l,const Mat r,const int maxdis,bool y_n,const int weight_s);
    ~NCC(void);
private:

};

