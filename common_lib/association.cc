#include "association.h"

const double eps = 1e-8;

double IOU(const proposal_type & r1, const proposal_type & r2)
{
    double inter = intersection_area(r1,r2);

    double o = inter / (calcularea(r1) + calcularea(r2) - inter);

    return (o >= 0) ? o : 0;
}


double intersection_area(const proposal_type & r1, const proposal_type & r2){
    MyPoint p1[10],p2[10];

    p1[0].x=r1.x2;
    p1[0].y=r1.y2;
    p1[1].x=r1.x3;
    p1[1].y=r1.y3;
    p1[2].x=r1.x4;
    p1[2].y=r1.y4;
    p1[3].x=r1.x1;
    p1[3].y=r1.y1;

    p2[0].x=r2.x2;
    p2[0].y=r2.y2;
    p2[1].x=r2.x3;
    p2[1].y=r2.y3;
    p2[2].x=r2.x4;
    p2[2].y=r2.y4;
    p2[3].x=r2.x1;
    p2[3].y=r2.y1;
    double area = SPIA(p1, p2, 4, 4);
    return area;
}


double SPIA(MyPoint a[], MyPoint b[], int na, int nb)///SimplePolygonIntersectArea 调用此函数
{
    int i, j;
    MyPoint t1[4], t2[4];
    double res = 0, num1, num2;
    a[na] = t1[0] = a[0], b[nb] = t2[0] = b[0];

    for(i = 2; i < na; i++)
    {
        t1[1] = a[i-1], t1[2] = a[i];
        num1 = dcmp(cross(t1[1], t1[2],t1[0]));
        if(num1 < 0) std::swap(t1[1], t1[2]);

        for(j = 2; j < nb; j++)
        {

            t2[1] = b[j - 1], t2[2] = b[j];
            num2 = dcmp(cross(t2[1], t2[2],t2[0]));
            if(num2 < 0) std::swap(t2[1], t2[2]);
            res += CPIA(t1, t2, 3, 3) * num1 * num2;
        }
    }
    return res;
}

int dcmp(double x)
{
    if(x > eps) return 1;
    return x < -eps ? -1 : 0;
}

double cross(MyPoint a,MyPoint b,MyPoint c) ///叉积
{
    return (a.x-c.x)*(b.y-c.y)-(b.x-c.x)*(a.y-c.y);
}

double CPIA(MyPoint a[], MyPoint b[], int na, int nb)
{
    MyPoint p[20], tmp[20];
    int tn, sflag, eflag;
    a[na] = a[0], b[nb] = b[0];
    memcpy(p,b,sizeof(MyPoint)*(nb + 1));
    for(int i = 0; i < na && nb > 2; i++)
    {
        sflag = dcmp(cross(a[i + 1], p[0],a[i]));
        for(int j = tn = 0; j < nb; j++, sflag = eflag)
        {
            if(sflag>=0) tmp[tn++] = p[j];
            eflag = dcmp(cross(a[i + 1], p[j + 1],a[i]));
            if((sflag ^ eflag) == -2)
                tmp[tn++] = intersection(a[i], a[i + 1], p[j], p[j + 1]); ///求交点
        }
        memcpy(p, tmp, sizeof(MyPoint) * tn);
        nb = tn, p[nb] = p[0];
    }
    if(nb < 3) return 0.0;
    return PolygonArea(p, nb);
}


double calcularea(const proposal_type & r){
    float d12=sqrt(pow(r.x2-r.x1,2)+pow(r.y2-r.y1,2));
    float d14=sqrt(pow(r.x4-r.x1,2)+pow(r.y4-r.y1,2));
    float d24=sqrt(pow(r.x2-r.x4,2)+pow(r.y2-r.y4,2));
    float d32=sqrt(pow(r.x2-r.x3,2)+pow(r.y2-r.y3,2));
    float d34=sqrt(pow(r.x3-r.x4,2)+pow(r.y3-r.y4,2));
    float p1=(d12+d14+d24)/2;
    float p2=(d24+d32+d34)/2;
    float s1=sqrt(p1*(p1-d12)*(p1-d14)*(p1-d24));
    float s2=sqrt(p2*(p2-d32)*(p2-d34)*(p2-d24));
    return s1+s2;
}

MyPoint intersection(MyPoint a,MyPoint b,MyPoint c,MyPoint d)
{
    MyPoint p = a;
    double t =((a.x-c.x)*(c.y-d.y)-(a.y-c.y)*(c.x-d.x))/((a.x-b.x)*(c.y-d.y)-(a.y-b.y)*(c.x-d.x));
    p.x +=(b.x-a.x)*t;
    p.y +=(b.y-a.y)*t;
    return p;
}

//计算多边形面积
double PolygonArea(MyPoint p[], int n)
{
    if(n < 3) return 0.0;
    double s = p[0].y * (p[n - 1].x - p[1].x);
    p[n] = p[0];
    for(int i = 1; i < n; ++ i)
        s += p[i].y * (p[i - 1].x - p[i + 1].x);
    return fabs(s * 0.5);
}



