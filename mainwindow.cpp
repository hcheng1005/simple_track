#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFile>
#include <QDebug>

#include "iostream"
#include <QTextStream>


#include "common_lib/association.h"
#include "common_lib/iou.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{

        ui->setupUi(this);

    QPen pen;
    pen.setWidth(2);
    pen.setStyle(Qt::PenStyle::SolidLine);
    pen.setColor(Qt::darkGray);
    ui->widget->addGraph();
    ui->widget->graph(0)->setPen(pen);
    ui->widget->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->widget->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 3));
    ui->widget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

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
    // iou_test
    iou2d_test();

    GreedyTest_Main();
}


void MainWindow::iou2d_test(void)
{
    //iou测试
    rect_basic_struct r1;
    rect_basic_struct r2;

    r1.center_pos[0]= 2;
    r1.center_pos[1] = 3;
    r1.center_pos[2] = 1;
    r1.box_len = 8;
    r1.box_wid = 4;
    r1.box_height = 2;
    r1.heading = 0.2;

    r2.center_pos[0]= 3;
    r2.center_pos[1] = 4;
    r2.center_pos[2] = 0;
    r2.box_len = 8;
    r2.box_wid = 4;
    r2.box_height = 2;
    r2.heading = -0.2;

    double iou = IOU_2D(r1, r2);
    double giou = GIOU_2D(r1, r2);
    double iou3d = IOU_3D(r1, r2);
    double giou3d = GIOU_3D(r1, r2);


    std::cout << iou << std::endl;
    std::cout << giou << std::endl;

    std::cout << iou3d << std::endl;
    std::cout << giou3d << std::endl;
}


void MainWindow::GreedyTest_Main(void)
{
    double cost_c[10][10] = {
        1.74816487, 2.38917233, 1.71076069, 3.39056081 ,8.97918614 ,8.8463572,
         5.72492498, 3.51112043, 2.83719919, 1.47646523, 6.90482939 ,6.54823362,
         2.97598861 ,8.58927493 ,1.27471997, 6.63336473 ,8.00192929 ,5.53644708,
         8.17686098 ,6.53984023 ,5.12970743, 5.15610536 ,6.76563599 ,3.63906049,
         9.5657066, 0.9938076, 4.65050956, 9.40180311, 7.40438347, 2.76392061,
         0.14804986, 2.46669343, 7.33323472 ,5.8211227 , 7.97660068, 4.25715621,
         8.70762212, 3.84921524, 3.19890027 ,2.28689383 ,8.34067808, 2.06432393,
         5.28740296, 4.65337427, 7.83300603 ,8.53227427 ,5.38427513, 1.03191784,
         6.6057736 , 7.68792094, 4.62337316 ,9.95950717, 7.65598475, 2.33958583,
         4.71393984, 8.73278614, 5.13445941 ,8.88417655 ,5.28262101, 1.08137045,
         6.5017676 , 3.71347059, 8.90070478 ,6.89908671, 9.3396071 , 9.69360009,
         8.39359751, 9.25831462, 9.28462701 ,4.67101498, 0.19922305, 8.61400931,
         4.97661521, 2.94110684, 4.14077323 ,4.74816978, 4.42211109, 3.70811997,
         2.46486932, 6.42482562, 7.4039233  ,3.37486973 ,0.27083053 ,0.18565782,
         5.25106232, 2.51429459, 8.12555989 ,2.01157174, 9.21221066, 2.54981598,
         7.40352095, 7.36382558, 0.7780371  ,1.78572676 ,1.72834597 ,8.56007773,
         8.72230221, 7.66976083, 7.88648666, 0.24672
         };


       std::vector<std::vector<cost_type>> cost_matrix;

       for(uint8_t trk_idx=0; trk_idx<10; trk_idx++)
       {
           std::vector<cost_type> sub_matrix;
           cost_type cost_pair_;
           for(uint8_t meas_idx=0; meas_idx<10; meas_idx++)
           {
                cost_pair_.first.first = trk_idx;
                cost_pair_.first.second = meas_idx;
                cost_pair_.second = cost_c[trk_idx][meas_idx];
                sub_matrix.push_back(cost_pair_);
           }

           cost_matrix.push_back(sub_matrix);
       }

       greedy_test(cost_matrix);
}


void greedy_test(std::vector<std::vector<cost_type>> &cost_matrix)
{
    int trace_num = cost_matrix.size();
    int meas_num = cost_matrix.at(0).size();

     //获取所有航迹与所有航迹的“距离”
    std::vector<cost_type> new_cost_mat;
    for(const auto &subList:cost_matrix)
    {
       for(auto sub_pair:subList)
       {
            new_cost_mat.push_back(sub_pair);
       }
    }

    //从小到大排列
    std::sort(new_cost_mat.begin(), new_cost_mat.end(), myTestFunc);

    //遍历所有成员，进行分配
    std::vector<int> trace_assign_info(trace_num, 255);
    std::vector<int> meas_assign_info(meas_num, 255);
    for(const auto sub_pair:new_cost_mat)
    {
        if((trace_assign_info.at(sub_pair.first.first) == 255) && \
            (meas_assign_info.at(sub_pair.first.second) == 255))
        {
            trace_assign_info.at(sub_pair.first.first) = sub_pair.first.second;
            meas_assign_info.at(sub_pair.first.second) = sub_pair.first.first;
        }
    }

#if 0
    double total_cost = 0.0;
    for(uint idx=0; idx<trace_assign_info.size(); idx++)
    {
        if(trace_assign_info.at(idx) != 255)
        {
            total_cost += cost_matrix.at(idx).at(trace_assign_info.at(idx)).second;
        }
    }

    std::cout << "total is: " << std::to_string(total_cost) << std::endl;
#endif

}


bool myTestFunc(cost_type c1, cost_type c2)
{
    return (c1.second < c2.second);
}



