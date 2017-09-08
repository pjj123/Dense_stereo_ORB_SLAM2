#include"ncc.h"
//#include"guidedfilter.h"

NCC::NCC(const Mat l,const Mat r,const int d,bool y_n,const int weight_s)
    :limg(l),rimg(r),maxdis(d),left_or_right(y_n),weight(weight_s){
    CV_Assert(limg.type()==CV_32F&&rimg.type()==CV_32F&&limg.rows==rimg.rows&&limg.cols==rimg.cols);
    hei=limg.rows;
     wid=limg.cols;
     dis=cv::Mat::zeros(hei,wid,CV_8UC1);
     xy_w=new Mat[maxdis];
     ncc_w=new Mat[maxdis];
     for(int i=0;i<maxdis;i++){
         xy_w[i]=cv::Mat::zeros(hei,wid,CV_32F);
         ncc_w[i]=cv::Mat::zeros(hei,wid,CV_32F);
     }
     x_w=cv::Mat::zeros(hei,wid,CV_32F);
     xx_w=cv::Mat::zeros(hei,wid,CV_32F);
     y_w=cv::Mat::zeros(hei,wid,CV_32F);
     yy_w=cv::Mat::zeros(hei,wid,CV_32F);
     second_weight_ncc=cv::Mat::zeros(hei,wid,CV_32F);
     weight_ncc=cv::Mat(hei,wid,CV_32F);
}
NCC::~NCC(void){
    delete [] xy_w;
    delete [] ncc_w;
}
Mat NCC::getdis(){
     return dis.clone();
}
 void NCC::buildcv(){
     Mat lp=limg.clone();
     Mat rp=rimg.clone();
     Mat lrp=Mat::zeros(hei,wid,CV_32F);
     int number_ncc=0;
     if(left_or_right){
         cv::boxFilter(limg,x_w,-1,cvSize(weight,weight));
         cv::boxFilter(rimg,y_w,-1,cvSize(weight,weight));
         Mat pjjy=rimg.mul(rimg);
         cv::boxFilter(pjjy,yy_w,-1,cvSize(weight,weight));
          Mat pjjx=limg.mul(limg);
         cv::boxFilter(pjjx,xx_w,-1,cvSize(weight,weight));
         for(int d=0;d<maxdis;d++){
             for(int y=0;y<hei;y++){
                 float * rdp=(float*)rp.ptr<float>(y);
                 float* lrdp=(float*)lrp.ptr<float>(y);
                 for(int x=0;x<wid;x++){
                      if(x-d<0){
                          lrdp[x]=0;
                      }else{
                          lrdp[x]=rdp[x-d];
                      }
                 }
             }
             Mat pjj_xy=lp.mul(lrp);
             cv::boxFilter(pjj_xy,xy_w[d],-1,cvSize(weight,weight));
         }
     for(int d=0;d<maxdis;d++){
         for(int y=(weight-1)/2;y<hei-(weight-1)/2;y++){
             float *wyy=(float*)yy_w.ptr<float>(y);
             float *wx=(float*)x_w.ptr<float>(y);
             float *wy=(float*)y_w.ptr<float>(y);
             float * wxy=(float*)xy_w[d].ptr<float>(y);
             float* wxx=(float*)xx_w.ptr<float>(y);
             float* wncc=(float*)ncc_w[d].ptr<float>(y);
             for(int x=(weight-1)/2+d;x<wid-(weight-1)/2;x++){
                 wncc[x]=(wxy[x]-wx[x]*wy[x-d])/sqrt((wyy[x-d]-wy[x-d]*wy[x-d])*(wxx[x]-wx[x]*wx[x]));
                 if(wncc[x]>1){
                   wncc[x]=0;
                   //  number_ncc++;
                   //cout<<"error wncc l="<<wncc[x]<<"fenzhi="<<wxy[x]-wx[x]*wy[x-d]<<"fenmuy="<<wyy[x-d]-wy[x-d]*wy[x-d]<<"femnux="<<wxx[x]-wx[x]*wx[x]<<endl;
                 }
             }
         }
      //ncc_w[d]=guidedFilter(limg,ncc_w[d],3,0.01);
         // cv::boxFilter(ncc_w[d],ncc_w[d],-1,cvSize(3,3));
     }
}
     else{
         cv::boxFilter(rimg,x_w,-1,cvSize(weight,weight));
         cv::boxFilter(limg,y_w,-1,cvSize(weight,weight));
         Mat pjjy=limg.mul(limg);
         cv::boxFilter(pjjy,yy_w,-1,cvSize(weight,weight));
         Mat pjjx=rimg.mul(rimg);
         cv::boxFilter(pjjx,xx_w,-1,cvSize(weight,weight));
         for(int d=0;d<maxdis;d++){
             for(int y=0;y<hei;y++){
                 float * ldp=(float*)lp.ptr<float>(y);
                 float* lrdp=(float*)lrp.ptr<float>(y);
                 for(int x=0;x<wid;x++){
                        if(x+d>wid-1){
                            lrdp[x]=0;
                        }else{
                            lrdp[x]=ldp[x+d];
                        }
                 }
             }
             Mat pjj_xy=rp.mul(lrp);
             cv::boxFilter(pjj_xy,xy_w[d],-1,cvSize(weight,weight));
         }
     for(int d=0;d<maxdis;d++){
         for(int y=(weight-1)/2;y<hei-(weight-1)/2;y++){
             float *wyy=(float*)yy_w.ptr<float>(y);
             float *wx=(float*)x_w.ptr<float>(y);
             float *wy=(float*)y_w.ptr<float>(y);
             float *wxx=(float*)xx_w.ptr<float>(y);
             float * wxy=(float*)xy_w[d].ptr<float>(y);
             float* wncc=(float*)ncc_w[d].ptr<float>(y);
             for(int x=(weight-1)/2;x<wid-(weight-1)/2-d;x++){
                 wncc[x]=(wxy[x]-wx[x]*wy[x+d])/sqrt((wyy[x+d]-wy[x+d]*wy[x+d])*(wxx[x]-wx[x]*wx[x]));
                 if(wncc[x]>1){
                     wncc[x]=0;
                     //number_ncc++;
                     //cout<<"error with wncc[x]"<<wncc[x]<<endl;
                 }

             }
         }
      // ncc_w[d]=guidedFilter(rimg,ncc_w[d],3,0.01);
         //cv::boxFilter(ncc_w[d],ncc_w[d],-1,cvSize(3,3));
     }
     }
  //cout<<"number_ncc="<<number_ncc<<endl;
     for(int d=0;d<maxdis;d++){
         for(int y=(weight-1)/2;y<hei;y++){
             uchar *wdis=(uchar*)dis.ptr<uchar>(y);
             float* wncc=(float*)ncc_w[d].ptr<float>(y);
             float* weight_=(float*)weight_ncc.ptr<float>(y);
             float* second_weight_=(float*)second_weight_ncc.ptr<float>(y);
             for(int x=(weight-1)/2;x<wid;x++){
                 if(wncc[x]>weight_[x]){
                      second_weight_[x]=weight_[x];
                     weight_[x]=wncc[x];
                     wdis[x]=(uchar)d;
                 }
             }
         }
     }

for(int y=0;y<hei;y++){
    float*wei=(float*)weight_ncc.ptr<float>(y);
    float *second_wei=(float*)second_weight_ncc.ptr<float>(y);
    uchar* best_dis=(uchar*)dis.ptr<uchar>(y);
    for(int x=0;x<wid;x++){
        if(wei[x]<0.85||(wei[x]-second_wei[x])/wei[x]<0.1)
            best_dis[x]=(uchar)0;
    }
 }
 }

