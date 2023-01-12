
#ifndef ASSOCIATION_H
#define ASSOCIATION_H

#include <iostream>
#include <vector>
#include <math.h>
#include <string.h>
#include <algorithm>

struct proposal_type
{
    double x1;
    double y1;

    double x2;
    double y2;

    double x3;
    double y3;

    double x4;
    double y4;
};

struct MyPoint
{
    double x, y;
};

double IOU(const proposal_type & r1, const proposal_type & r2);
double intersection_area(const proposal_type & r1, const proposal_type & r2);
MyPoint intersection(MyPoint a,MyPoint b,MyPoint c,MyPoint d);
double calcularea(const proposal_type & r);
double PolygonArea(MyPoint p[], int n);
double cross(MyPoint a,MyPoint b,MyPoint c);
int dcmp(double x);
double SPIA(MyPoint a[], MyPoint b[], int na, int nb);///SimplePolygonIntersectArea 调用此函数
double CPIA(MyPoint a[], MyPoint b[], int na, int nb);//ConvexPolygonIntersectArea
bool cmpr(const proposal_type &a,const proposal_type &b);
#endif
