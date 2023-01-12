#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFile>
#include <QDebug>

#include "iostream"
#include <QTextStream>


#include "common_lib/association.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::on_pushButton_released()
{
    // 读取本地文件
    QFile file("/home/charles/simple_track/test.csv");
    file.open(QIODevice::ReadOnly);
    QTextStream read22(&file);
    QStringList data = read22.readAll().split("\n", QString::SkipEmptyParts);
    QRegExp mySeparator = QRegExp(":| |,");

    std::vector<dets_t> dets_list;

    //获取检测结果
    for(int idx=0; idx<data.count(); idx++)
    {
        uint pos = 1;
        dets_t det_;

        QStringList tempData = data.at(idx).split(mySeparator, QString::SkipEmptyParts);
        det_.x = tempData[pos].toDouble();pos += 2;
        det_.y = tempData[pos].toDouble();pos += 2;
        det_.z = tempData[pos].toDouble();pos += 2;
        det_.heading = tempData[pos].toDouble();pos += 2;
        det_.length = tempData[pos].toDouble();pos += 2;
        det_.width = tempData[pos].toDouble();pos += 2;
        det_.height = tempData[pos].toDouble();pos += 2;
        det_.score = tempData[pos].toDouble();

        dets_list.emplace_back(det_);
    }

    std::cout << dets_list.size() << std::endl;

    // do Simple Tracker
    simple_trace_proc(dets_list);
}



std::vector<simple_tracker> trace_list;
void MainWindow::simple_trace_proc(std::vector<dets_t> &dets_list)
{
    // predict
    std::vector<Eigen::VectorXd> x_predict_list;
    for(auto &trace : trace_list)
    {
        trace.trace_update(0.01);
        x_predict_list.push_back(trace.X_);
    }

    //match and assign
//    for()
//    {

//    }



//    // trace management
//    for(auto &trace : trace_list)
//    {

//    }


    // new trace
    for(auto det: dets_list)
    {
        if(det.valid)
        {
            Eigen::VectorXd x_int;
            Eigen::MatrixXd p_init;
            simple_tracker new_trace(x_int, p_init);

            trace_list.push_back(new_trace);
        }
    }
}



void bipartite_matcher(const std::vector<dets_t> dets_list,
                       const std::vector<Eigen::VectorXd> trace_list,
                       const std::string assign_mode)
{
    if( (assign_mode == "iou") || (assign_mode == "giou") )
    {
        compute_iou_distance(dets_list, trace_list, assign_mode);
    }
}


void compute_iou_distance(const std::vector<dets_t> dets_list,
                          const std::vector<Eigen::VectorXd> trace_list,
                          const std::string assign_mode)
{
    if(assign_mode == "iou")
    {

    }

    if(assign_mode == "giou")
    {

    }
}

void creat_box_2d(std::vector<double> center_and_shape)
{

}


void creat_box_3d(std::vector<double> center_and_shape)
{

}


void MainWindow::on_pushButton_2_released()
{
    //iou测试
    proposal_type r1;
    proposal_type r2;

    double p1[4] = {2,3,2,4};
    double half_wid, half_len;

    half_wid = 2;
    half_len = 4;

    double theta = 0.2;
    double rot[4] = {cos(theta), -1.0*sin(theta), sin(theta), cos(theta)};
    theta = -0.2;
    double rot2[4] = {cos(theta), -1.0*sin(theta), sin(theta), cos(theta)};

    double temp_x, temp_y;

    temp_x = -1.0 * half_wid;
    temp_y =  half_len;
    r1.x1 = rot[0] * temp_x + rot[1] * temp_y + p1[0];
    r1.y1 = rot[2] * temp_x + rot[3] * temp_y + p1[1];

    temp_x = half_wid;
    temp_y = half_len;
    r1.x2 = rot[0] * temp_x + rot[1] * temp_y + p1[0];
    r1.y2 = rot[2] * temp_x + rot[3] * temp_y + p1[1];

    temp_x = half_wid;
    temp_y = -1.0 * half_len;
    r1.x3 = rot[0] * temp_x + rot[1] * temp_y + p1[0];
    r1.y3 = rot[2] * temp_x + rot[3] * temp_y + p1[1];

    temp_x = -1.0 * half_wid;
    temp_y = -1.0 * half_len;
    r1.x4 = rot[0] * temp_x + rot[1] * temp_y + p1[0];
    r1.y4 = rot[2] * temp_x + rot[3] * temp_y + p1[1];

    double p2[4] = {3,4,2,4};

    temp_x = -1.0 * half_wid;
    temp_y =  half_len;
    r2.x1 = rot2[0] * temp_x + rot2[1] * temp_y + p2[0];
    r2.y1 = rot2[2] * temp_x + rot2[3] * temp_y + p2[1];

    temp_x = half_wid;
    temp_y = half_len;
    r2.x2 = rot2[0] * temp_x + rot2[1] * temp_y + p2[0];
    r2.y2 = rot2[2] * temp_x + rot2[3] * temp_y + p2[1];

    temp_x = half_wid;
    temp_y = -1.0 * half_len;
    r2.x3 = rot2[0] * temp_x + rot2[1] * temp_y + p2[0];
    r2.y3 = rot2[2] * temp_x + rot2[3] * temp_y + p2[1];

    temp_x = -1.0 * half_wid;
    temp_y = -1.0 * half_len;
    r2.x4 = rot2[0] * temp_x + rot2[1] * temp_y + p2[0];
    r2.y4 = rot2[2] * temp_x + rot2[3] * temp_y + p2[1];


    std::cout << r1.x1 << "," << r1.y1 << std::endl;
    std::cout << r1.x2 << "," << r1.y2 << std::endl;
    std::cout << r1.x3 << "," << r1.y3 << std::endl;
    std::cout << r1.x4 << "," << r1.y4 << std::endl;

    std::cout << r2.x1 << "," << r2.y1 << std::endl;
    std::cout << r2.x2 << "," << r2.y2 << std::endl;
    std::cout << r2.x3 << "," << r2.y3 << std::endl;
    std::cout << r2.x4 << "," << r2.y4 << std::endl;

    double iou = IOU(r1, r2);



    std::cout << iou << std::endl;
}








