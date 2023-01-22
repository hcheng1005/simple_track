#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFile>
#include <QDebug>

#include "iostream"
#include <QTextStream>


#include "common_lib/association.h"
#include "common_lib/iou.h"

#include <sys/time.h>

/* simple trace vari */
std::vector<simple_tracker> trace_list;

/* ---------------end-------------- */



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

    pen.setColor(Qt::green);
    ui->widget->addGraph();
    ui->widget->graph(1)->setPen(pen);
    ui->widget->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui->widget->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 3));
    ui->widget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    pen.setColor(Qt::red);
    ui->widget->addGraph();
    ui->widget->graph(2)->setPen(pen);
    ui->widget->graph(2)->setLineStyle(QCPGraph::lsNone);
    ui->widget->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 3));
    ui->widget->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    ui->widget->xAxis->setRange(7000, 7500);
    ui->widget->yAxis->setRange(-1500, -1200);

    ui->widget->setOpenGl(true);

    LA_solver = new Linear_Assigment();
}


MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_released()
{
    simple_demo();
}


void MainWindow::simple_demo()
{
 //遍历文件夹下面的子文件夹
    QString basePath = "/home/zdhjs-05/myGitHubCode/simple_track/demo_data/";
    QDir dir(basePath);
    //  设置过滤器
    QFileInfoList fileInfoList = dir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot | QDir::Dirs);

    QString Summary_Info_File_Path = "";
    QString Det_Path, Ego_Path, Pc_Path;

    // foreach的循环不能修改元素内容
    foreach (auto fileInfo, fileInfoList)
    {
        if(fileInfo.isFile())
        {
            qDebug() << fileInfo.absoluteFilePath();
            Summary_Info_File_Path = fileInfo.absoluteFilePath();
        }
        else
        {
            if(fileInfo.isDir())
            {
                qDebug() << fileInfo.absoluteFilePath();

                if(fileInfo.absoluteFilePath().contains("dets")){
                    Det_Path = fileInfo.absoluteFilePath();
                }

                if(fileInfo.absoluteFilePath().contains("ego")){
                    Ego_Path = fileInfo.absoluteFilePath();
                }

                if(fileInfo.absoluteFilePath().contains("pc")){
                    Pc_Path = fileInfo.absoluteFilePath();
                }
            }
        }
    }

    QFile summary_file(Summary_Info_File_Path);
    summary_file.open(QIODevice::ReadOnly);
    QTextStream summary_text(&summary_file);
    QStringList summary_text_list = summary_text.readAll().split("\n", QString::SkipEmptyParts);

    for(int idx=1; idx<summary_text_list.count(); idx++)
    {
        QStringList tempData = summary_text_list.at(idx).split("\n", QString::SkipEmptyParts);
        QString timeStamp = tempData.at(0);

        //获取dets文件
        QString det_file_path = Det_Path + "/" + timeStamp + "_dets_result.csv";
        std::vector<dets_t> dets_list;
        simple_demo_get_dets(det_file_path, dets_list);

        // do Simple Tracker
        simple_trace_proc(dets_list);

        simple_demo_plot(dets_list, trace_list);

        mySleep(500);
    }
}

void MainWindow::simple_demo_plot(const std::vector<dets_t> &dets_list, const std::vector<simple_tracker> &trace_list)
{
    QVector<double> x_pos_det, y_pos_det;
    QVector<double> x_pos_temp_trk, y_pos_temp_trk;
    QVector<double> x_pos_confir_trk, y_pos_confir_trk;

    rect_basic_struct rect;

    ui->widget->clearItems();

    for(auto sub_det:dets_list)
    {
        x_pos_det.append(sub_det.x);
        y_pos_det.append(sub_det.y);

        rect.center_pos[0] = sub_det.x;
        rect.center_pos[1] = sub_det.y;
        rect.center_pos[2] = sub_det.z;

        rect.box_len = sub_det.length;
        rect.box_wid = sub_det.width;
        rect.box_height = sub_det.height;
        rect.heading = sub_det.heading;

        simple_demo_plot_add_det_rect(rect, "det");
    }

    for(auto sub_trk:trace_list)
    {
        if(sub_trk.track_manage.track_status == TRK_Invalid)
        {
            continue;
        }

        rect.center_pos[0] = sub_trk.X_(0);
        rect.center_pos[1] = sub_trk.X_(1);
        rect.center_pos[2] = sub_trk.X_(2);

        rect.box_len = sub_trk.X_(4);
        rect.box_wid = sub_trk.X_(5);
        rect.box_height = sub_trk.X_(6);
        rect.heading = sub_trk.X_(3);

        if(sub_trk.track_manage.track_status == TRK_Detected)
        {
            x_pos_temp_trk.append(sub_trk.X_(0));
            y_pos_temp_trk.append(sub_trk.X_(1));

            simple_demo_plot_add_det_rect(rect, "temp_trk");
        }

        if(sub_trk.track_manage.track_status == TRK_Confirmed)
        {
            x_pos_confir_trk.append(sub_trk.X_(0));
            y_pos_confir_trk.append(sub_trk.X_(1));

            simple_demo_plot_add_det_rect(rect, "confir_trk");
        }
    }

    ui->widget->graph(0)->setData(x_pos_det, y_pos_det);
    ui->widget->graph(1)->setData(x_pos_temp_trk, y_pos_temp_trk);
    ui->widget->graph(2)->setData(x_pos_confir_trk, y_pos_confir_trk);
    ui->widget->replot();
}


void MainWindow::simple_demo_plot_add_det_rect(const rect_basic_struct &rect, QString plot_obj)
{
    QCPItemLine *L1 = new QCPItemLine(ui->widget);
    QCPItemLine *L2 = new QCPItemLine(ui->widget);
    QCPItemLine *L3 = new QCPItemLine(ui->widget);
    QCPItemLine *L4 = new QCPItemLine(ui->widget);

    QPen pen;
    pen.setWidth(3);

    if(plot_obj == "det")
    {
        pen.setColor(Qt::darkGray);
    }

    if(plot_obj == "temp_trk")
    {
        pen.setColor(Qt::green);
    }

    if(plot_obj == "confir_trk")
    {
        pen.setColor(Qt::red);
    }

    L1->setPen(pen);
    L2->setPen(pen);
    L3->setPen(pen);
    L4->setPen(pen);

    rect_corners_struct box_corners;
    creat_rect_box_point(rect, box_corners);

    L1->start->setCoords(QPointF(box_corners.corners[0].x, box_corners.corners[0].y));
    L1->end->setCoords(QPointF(box_corners.corners[1].x, box_corners.corners[1].y));

    L2->start->setCoords(QPointF(box_corners.corners[1].x, box_corners.corners[1].y));
    L2->end->setCoords(QPointF(box_corners.corners[2].x, box_corners.corners[2].y));

    L3->start->setCoords(QPointF(box_corners.corners[2].x, box_corners.corners[2].y));
    L3->end->setCoords(QPointF(box_corners.corners[3].x, box_corners.corners[3].y));

    L4->start->setCoords(QPointF(box_corners.corners[3].x, box_corners.corners[3].y));
    L4->end->setCoords(QPointF(box_corners.corners[0].x, box_corners.corners[0].y));
}


void MainWindow::simple_demo_get_dets(const QString filePath, std::vector<dets_t> &dets_list)
{
    QFile file(filePath);
    file.open(QIODevice::ReadOnly);
    QTextStream read22(&file);
    QStringList data = read22.readAll().split("\n", QString::SkipEmptyParts);
    QRegExp mySeparator = QRegExp(":| |,");
    //获取检测结果
    for(int idx=1; idx<data.count(); idx++)
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
}


void MainWindow::simple_trace_proc(std::vector<dets_t> &dets_list)
{
    // 航迹预测
    std::vector<Eigen::VectorXd> x_predict_list;
    for(auto &trace : trace_list){
        trace.trace_predict(0.1);
        x_predict_list.push_back(trace.X_);
    }

    // "距离"计算
    std::vector<std::vector<cost_pair>> cost_matrix;
    compute_iou_distance(dets_list, x_predict_list, "giou", cost_matrix);

    // 分配算法
    std::vector<int> trace_assign_info(trace_list.size(), 255);
    std::vector<int> meas_assign_info(dets_list.size(), 255);

    // 贪心算法
    LA_solver->greedy_solver(cost_matrix, trace_assign_info, meas_assign_info);
    lap_main(cost_matrix, trace_list, dets_list);

    // trace management
    for(uint8_t trk_idx=0; trk_idx<trace_list.size(); trk_idx++){
        simple_tracker &sub_trace = trace_list.at(trk_idx);
        if(trace_assign_info.at(trk_idx) == 255){
            sub_trace.track_manage.continue_assigned_count = 0;
            sub_trace.track_manage.unassigned_count++;
            if(sub_trace.track_manage.unassigned_count > 5)
            {
                sub_trace.track_manage.track_status = TRK_Delete;
            }
        }
        else
        {
            if(sub_trace.track_manage.continue_assigned_count < 2){
                sub_trace.track_manage.continue_assigned_count++;
            }
            else{
               sub_trace.track_manage.track_status = TRK_Confirmed;
            }

            //关联到量测，update
            dets_t &sub_det = dets_list.at(trace_assign_info.at(trk_idx));
            Eigen::VectorXd Z_ = Eigen::VectorXd(Z_DIM);
            Z_ << sub_det.x, sub_det.y, sub_det.z, sub_det.heading, sub_det.length, sub_det.width, sub_det.height;
            sub_trace.trace_updateWithDet(Z_);

//            std::cout << "subtrace" << std::endl;
        }
    }

    std::vector<simple_tracker> trace_list_temp;
    for(auto sub_trace : trace_list)
    {
        if(sub_trace.track_manage.track_status != TRK_Delete)
        {
           trace_list_temp.push_back(sub_trace);
        }
    }
    trace_list.clear();
    trace_list = trace_list_temp;

    // new trace
    for(uint8_t meas_idx=0; meas_idx<dets_list.size(); meas_idx++) {
        if(meas_assign_info.at(meas_idx) == 255){
            dets_t &sub_det = dets_list.at(meas_idx);

            Eigen::VectorXd X = Eigen::VectorXd::Zero(X_DIM);

            X << sub_det.x, sub_det.y, sub_det.z, \
                sub_det.heading, sub_det.length, sub_det.width, sub_det.height, \
                0.0, 0.0, 0.0;

            Eigen::MatrixXd P = Eigen::MatrixXd::Identity(X_DIM, X_DIM) * 2.0;
            simple_tracker *new_trace = new simple_tracker(X, P);

            trace_list.push_back(*new_trace);
        }
    }
}

void MainWindow::lap_main(std::vector<std::vector<cost_pair>> cost_matrix,
              std::vector<simple_tracker> trace_list,
              std::vector<dets_t> dets_list)
{
    distMatrix_t distMatrix;
    std::vector<int> assignment, assignment2;

    uint max_num_ = (trace_list.size() > dets_list.size())? trace_list.size() : dets_list.size();
    double **cost_ptr = new double *[sizeof(double *) * max_num_];
    double **cost_ptr2 = new double *[sizeof(double *) * max_num_];

    for (uint i = 0; i < max_num_; i++)
    {
        cost_ptr[i] = new double[sizeof(double) * max_num_];
        cost_ptr2[i] = new double[sizeof(double) * max_num_];
    }

    for (uint i1 = 0; i1 < max_num_; i1++)
    {
        for (uint i2 = 0; i2 < max_num_; i2++)
        {
            if( (i1 < trace_list.size()) && (i2 < dets_list.size()))
            {
                cost_ptr[i1][i2] = (cost_matrix.at(i1).at(i2).second + 1.0) * -1.0;
                distMatrix.push_back((cost_matrix.at(i1).at(i2).second + 1.0) * -1.0);
                cost_ptr2[i1][i2] = (cost_matrix.at(i1).at(i2).second + 1.0);
            }
            else
            {
                cost_ptr[i1][i2] = 1e3;
                cost_ptr2[i1][i2] = -1.0*1e3;
//                distMatrix.push_back(1e3);
            }
        }
    }

    int trace_assign_c[max_num_];
    int det_assign_c[max_num_];
    struct timeval timestamp, timestamp2,timestamp3,timestamp4;

    if((trace_list.size() > 0) && (dets_list.size() > 0))
    {
        gettimeofday(&timestamp, NULL);
        LA_solver->Hungarian_solver(distMatrix, dets_list.size(), trace_list.size(), assignment, LA_solver->optimal);

        gettimeofday(&timestamp2, NULL);
        LA_solver->lapjv_internal(max_num_, cost_ptr, &trace_assign_c[0], &det_assign_c[0]);

        gettimeofday(&timestamp3, NULL);
        LA_solver->auction(max_num_, cost_ptr2, assignment2);

        gettimeofday(&timestamp4, NULL);

        std::cout << timestamp2.tv_usec - timestamp.tv_usec  << "  " \
                  << timestamp3.tv_usec - timestamp2.tv_usec << " " \
                  << timestamp4.tv_usec - timestamp3.tv_usec << " " <<  std::endl;
    }

    for(uint i=0; i<max_num_; i++)
    {
        delete[] cost_ptr[i];
        delete[] cost_ptr2[i];
    }

    delete[] cost_ptr;
    delete[] cost_ptr2;

    std::cout << "xx" << std::endl;
}

void bipartite_matcher(const std::vector<dets_t> dets_list,
                       const std::vector<Eigen::VectorXd> trace_list,
                       const std::string assign_mode)
{
//    if( (assign_mode == "iou") || (assign_mode == "giou") )
//    {
//        compute_iou_distance(dets_list, trace_list, assign_mode);
//    }
}


void compute_iou_distance(const std::vector<dets_t> dets_list,
                          const std::vector<Eigen::VectorXd> trace_list,
                          const std::string assign_mode,
                          std::vector<std::vector<cost_pair>> &cost_matrix)
{
    for(uint32_t trk_idx=0; trk_idx<trace_list.size(); trk_idx++){
        const Eigen::VectorXd &sub_trace = trace_list.at(trk_idx);

        std::vector<cost_pair> sub_matrix;
        cost_pair cost_pair_;
       for(uint32_t meas_idx=0; meas_idx<dets_list.size(); meas_idx++) {
            const dets_t &sub_det = dets_list.at(meas_idx);

            cost_pair_.first.first = trk_idx;
            cost_pair_.first.second = meas_idx;

            // 计算航迹与量测的 giou3d
            rect_basic_struct rect_1, rect_2;

            rect_1.center_pos[0]= sub_trace(0);
            rect_1.center_pos[1] = sub_trace(1);
            rect_1.center_pos[2] = sub_trace(2);
            rect_1.heading = sub_trace(3);
            rect_1.box_len = sub_trace(4);
            rect_1.box_wid = sub_trace(5);
            rect_1.box_height = sub_trace(6);

            rect_2.center_pos[0]= sub_det.x;
            rect_2.center_pos[1] = sub_det.y;
            rect_2.center_pos[2] = sub_det.z;
            rect_2.heading = sub_det.heading;
            rect_2.box_len = sub_det.length;
            rect_2.box_wid = sub_det.width;
            rect_2.box_height = sub_det.height;

            double value_;
            if(assign_mode == "iou")
            {
                value_ = IOU_3D(rect_1, rect_2);
            }
            else
            {
                value_ = GIOU_3D(rect_1, rect_2);
            }

            cost_pair_.second = value_;
            sub_matrix.push_back(cost_pair_);
       }

       cost_matrix.push_back(sub_matrix);
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


    std::vector<std::vector<cost_pair>> cost_matrix;

    for(uint8_t trk_idx=0; trk_idx<10; trk_idx++)
    {
       std::vector<cost_pair> sub_matrix;
       cost_pair cost_pair_;
       for(uint8_t meas_idx=0; meas_idx<10; meas_idx++)
       {
            cost_pair_.first.first = trk_idx;
            cost_pair_.first.second = meas_idx;
            cost_pair_.second = cost_c[trk_idx][meas_idx];
            sub_matrix.push_back(cost_pair_);
       }

       cost_matrix.push_back(sub_matrix);
    }

    std::vector<int> trace_assign_info(10, 255);
    std::vector<int> meas_assign_info(10, 255);
    LA_solver->greedy_solver(cost_matrix, trace_assign_info, meas_assign_info);
}


void MainWindow::mySleep(uint32_t ms)
{
    QTime dieTime = QTime::currentTime().addMSecs(ms);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

