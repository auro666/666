#include "stdafx.h"
#include <stdio.h>
#include <highgui.h>
#include <cv.h>
#include <math.h>
#include <cxcore.h>
#include "opencv2/opencv.hpp"
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/ocl/ocl.hpp"
#include<conio.h>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <time.h>
#define vl 1



using namespace std;
float a,b;
float X(int y) {
    return a*y+b;
}
void midline(IplImage *im,CvSeq*lines,CvPoint *c,CvPoint *d)
{
    CvPoint * line=NULL;
    float m,cos,sin;
    float sumx=0,sumy=0,midx,midy,midang,sumang=0,ang,slope;
    for( int i = 0; i < lines->total; i++ )
    {
        line = (CvPoint*)cvGetSeqElem(lines,i);
        sumx+=((float)(line[0].x+line[1].x))/2;
        sumy+=((float)(line[0].y+line[1].y))/2;
        slope=((float)(line[0].y-line[1].y))/((float)(line[0].x-line[1].x));
        ang=atan(slope)*180/CV_PI;
        if(ang<0.0f)
            ang+=180;
        sumang+=ang;
    }
    midx=sumx/lines->total;
    midy=sumy/lines->total;
    midang=sumang/lines->total;
    m=tan(midang*CV_PI/180);
    cos=1.0/sqrt(1+m*m);
    sin=fabs(cos*m);
    if(m<0.0f)
        cos*=-1;



    //mid line is of form x=ay+b
    a=1.0/m;
    b=midx-((float)midy)/m;


    float cnt1=0,cnt2=0;
    float sumx1=0,sumy1=0,sumang1=0,midx1,midy1,midang1;
    float sumx2=0,sumy2=0,sumang2=0,midx2,midy2,midang2;


    float xm,ym;

    for( int i = 0; i < lines->total; i++ )
    {

        line = (CvPoint*)cvGetSeqElem(lines,i);
        xm=((float)(line[0].x+line[1].x))/2;
        ym=((float)(line[0].y+line[1].y))/2;
        slope=((float)(line[0].y-line[1].y))/((float)(line[0].x-line[1].x));
        ang=atan(slope)*180/CV_PI;
        if(ang<0.0f)
            ang+=180;
        if(X(ym)>(float)xm)
        {
            sumx1+=xm;
            sumy1+=ym;
            sumang1+=ang;
            cnt1++;
        }
        else
        {
            sumx2+=xm;
            sumy2+=ym;
            sumang2+=ang;
            cnt2++;
        }
    }

    midx1=sumx1/cnt1;
    midy1=sumy1/cnt1;
    midang1=sumang1/cnt1;

    {
        m=tan(midang1*CV_PI/180);
        cos=1.0/sqrt(1+m*m);
        sin=fabs(cos*m);
        if(m<0.0f)
            cos*=-1;

        c->x=int(midx1+100*cos);
        c->y=int(midy1+100*sin);
        d->x=int(midx1-100*cos);
        d->y=int(midy1-100*sin);
        cvLine(im, *c, *d, cvScalar(255,255,255));
    }
    midx2=sumx2/cnt2;
    midy2=sumy2/cnt2;
    midang2=sumang2/cnt2;
    {
        m=tan(midang2*CV_PI/180);
        cos=1.0/sqrt(1+m*m);
        sin=fabs(cos*m);
        if(m<0.0f)
            cos*=-1;

        c->x=int(midx2+100*cos);
        c->y=int(midy2+100*sin);
        d->x=int(midx2-100*cos);
        d->y=int(midy2-100*sin);
        cvLine(im, *c, *d, cvScalar(255,255,255));
    }
    midx=(midx1+midx2)/2;
    midy=(midy1+midy2)/2;
    midang=(midang1+midang2)/2;
    {
        m=tan(midang*CV_PI/180);
        cos=1.0/sqrt(1+m*m);
        sin=fabs(cos*m);
        if(m<0.0f)
            cos*=-1;

        c->x=int(midx+100*cos);
        c->y=int(midy+100*sin);
        d->x=int(midx-100*cos);
        d->y=int(midy-100*sin);
        cvLine(im, *c, *d, cvScalar(0,0,255));
    }

}

long long int calcData(IplImage *close, long long int *Y_mean,long long int *Y_sd,long long int *Cr_mean,long long int *Cr_sd,long long int *Cb_mean,long long int *Cb_sd);
long long int segment(IplImage *road, IplImage *segmented, long long int Y_mean, long long int Y_sd, long long int Cr_mean, long long int Cr_sd, long long int Cb_mean, long long int Cb_sd);

int main()
{
    clock_t begin, end;
    double time_spent;
    begin = clock();


    long long int Y_mean, Y_sd, Cr_mean, Cr_sd, Cb_mean, Cb_sd;

    IplImage *road = cvLoadImage("whitlane.jpg");
    cvShowImage("Original",road);

    IplImage *whitefil = cvCreateImage(cvGetSize(road), IPL_DEPTH_8U,3);
    IplImage *edgelane = cvCreateImage(cvGetSize(road), IPL_DEPTH_8U,1);
    IplImage *hough=cvCreateImage(cvGetSize(road), IPL_DEPTH_8U,3);
    IplImage *white=cvLoadImage("whitesample.jpg");
    IplImage *rc2=cvCreateImage(cvGetSize(white), IPL_DEPTH_8U,3);
    //IplImage *rc3=cvCreateImage(cvGetSize(road), IPL_DEPTH_8U,3);
    cvCvtColor(white,rc2,CV_BGR2YCrCb);

    CvScalar white_pix=cvGet2D(rc2,1,1);

    Y_mean=white_pix.val[0];
    Cr_mean=white_pix.val[1];
    Cb_mean=white_pix.val[2];

    Y_sd=30;
    Cr_sd=3;
    Cb_sd=3;
    //calcData(rc2, &Y_mean, &Y_sd, &Cr_mean, &Cr_sd, &Cb_mean, &Cb_sd);

    printf("%lld   %lld   %lld  %lld   %lld   %lld",Y_mean, Y_sd, Cr_mean, Cr_sd, Cb_mean, Cb_sd);
    segment(road,whitefil,Y_mean,Y_sd,Cr_mean, Cr_sd, Cb_mean, Cb_sd);
    cvShowImage("Lane_white",whitefil);
    cvCanny(whitefil,edgelane,100,95);
    cvShowImage("Edgelane",edgelane);





    long int count=0;
    char c;
    /*while(count==0) *///{
    count++;

    CvMemStorage * huff=cvCreateMemStorage(0);//,* store_cont=cvCreateMemStorage(0);
    //CvSeq* seg=0,*cont=0;






    cvCvtColor(whitefil,edgelane,CV_BGR2GRAY);
    CvSeq* lines = cvHoughLines2(edgelane,huff,CV_HOUGH_PROBABILISTIC,1,CV_PI/180, 50, 50, 5);
    CvPoint *line=NULL,a,b;
    float m,sin,cos,p;
    if(lines)
        printf( "(%d)",lines->total);
    //int A[50][3];
    float min=0,max=0,p1=0,p2=0;
    for( int i = 0; i < lines->total; i++ )
    {
        line = (CvPoint*)cvGetSeqElem(lines,i);

        m=((float)(line[0].y-line[1].y))/((float)(line[0].x-line[1].x));
        cos=1.0/sqrt(1+m*m);
        sin=cos*m;
        CvPoint c,d;
        int l=20;

        {
            c.x=int(line[0].x-l*cos);
            c.y=int(line[0].y-l*sin);
            d.x=int(line[1].x+l*cos);
            d.y=int(line[1].y+l*sin);
        }

        cvLine(hough, c, d, cvScalar(255,0,0));


    }
    midline(hough,lines,&a,&b);
    cvShowImage("HoughLine",hough);



    cvClearSeq(lines);
    cvReleaseMemStorage(&huff);
    cvWaitKey(0);
    end = clock();
    time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    printf("%f fps",(time_spent));
    return 0;
}

long long int calcData(IplImage *img, long long int *Y_mean,long long int *Y_sd,long long int *Cr_mean,long long int *Cr_sd,long long int *Cb_mean,long long int *Cb_sd) {
    unsigned long long int n, sum_Y=0, sum_Cr=0, sum_Cb=0;
    IplImage* close = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
    cvCopy(img, close);
    n = 5000;
    CvScalar pixel;
    long long int wid,h1,hm,sm,vm;
    h1=close->height;
    wid=close->width;

    for(long long int i = 0; i<h1; i++) {
        for(long long int j = 0; j< wid; j++) {
            pixel = cvGet2D(close, i, j);
            sum_Y += pixel.val[0];
            sum_Cr += pixel.val[1];
            sum_Cb += pixel.val[2];
        }
    }

    *Y_mean = sum_Y / n;
    *Cr_mean = sum_Cr / n;
    *Cb_mean = sum_Cb / n;
    sum_Y = sum_Cr = sum_Cb = 0;
    for(long long int i = 0; i<h1; i++) {
        for(long long int j = 0; j< wid; j++) {
            //for(long long int i = 0; i< close->height; i++){
            //for(long long int j = 0; j< close->width; j++){
            pixel = cvGet2D(close, i, j);
            sum_Y += (*Y_mean - pixel.val[0])*(*Y_mean - pixel.val[0]);
            sum_Cr += (*Cr_mean - pixel.val[1])*(*Cr_mean - pixel.val[1]);
            sum_Cb += (*Cb_mean - pixel.val[2])*(*Cb_mean - pixel.val[2]);
        }
    }
    *Y_sd = sum_Y / (close->height*close->width);
    *Y_sd = sqrt((float)*Y_sd);
    *Cr_sd = sum_Cr / (close->height*close->width);
    *Cr_sd = sqrt((float)*Cr_sd);
    *Cb_sd = sum_Cb / (close->height*close->width);
    *Cb_sd = sqrt((float)*Cb_sd);
    cvReleaseImage(&close);
    return 0;
}

long long int segment(IplImage *road, IplImage *segmented, long long int Y_mean, long long int Y_sd, long long int Cr_mean, long long int Cr_sd, long long int Cb_mean, long long int Cb_sd) {
    CvScalar pix1, pix0, pixel;
    pix0.val[0] = 0;
    pix0.val[1] = 0;
    pix0.val[2] = 0;

    IplImage *roadYCrCb = cvCreateImage(cvGetSize(road), IPL_DEPTH_8U, 3);
    cvSmooth(road,road);
    cvCvtColor(road,roadYCrCb,CV_BGR2YCrCb);
    //cvCopy(road, roadYCrCb);
    long long int h, s, v, h1,h2,w1,w2;
    h2=road->height;
    for(long long int i = 0; i<h2; i++) {
        for(long long int j = 0; j< road->width; j++) {
            pixel = cvGet2D(roadYCrCb, i, j);
            h = pixel.val[0];
            s = pixel.val[1];
            v = pixel.val[2];
            pix1 = cvGet2D(road, i, j);
            double dd;
            if(fabs((float)(Y_mean-h))<(Y_sd) && fabs((float)(Cr_mean-s))<(Cr_sd) && fabs((float)(Cb_mean-v))<(Cb_sd))
                cvSet2D(segmented, i, j, pix1);
            else
                cvSet2D(segmented, i, j, pix0);
        }
    }
    cvReleaseImage(&roadYCrCb);
    return 0;
}
