#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qcustomplot.h"


#include "./simple_tracker/simple_tracker.h"
#include "common_lib/association.h"
#include "common_lib/iou.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void iou2d_test(void);

    void GreedyTest_Main(void);
    void simple_trace_proc(std::vector<dets_t> &dets_list);

private slots:
    void on_pushButton_released();

    void on_pushButton_2_released();

private:
    Ui::MainWindow *ui;

    Linear_Assigment *LA_solver;

    void mySleep(uint32_t ms);

    void lap_main(std::vector<std::vector<cost_pair>> cost_matrix,
                  std::vector<simple_tracker> trace_list,
                  std::vector<dets_t> dets_list);

    void simple_demo();
    void simple_demo_get_dets(const QString filePath, std::vector<dets_t> &dets_list);
    void simple_demo_plot(const std::vector<dets_t> &dets_list, const std::vector<simple_tracker> &trace_list);
    void simple_demo_plot_add_det_rect(const rect_basic_struct &rect, QString plot_obj);
};


void compute_iou_distance(const std::vector<dets_t> dets_list,
                          const std::vector<Eigen::VectorXd> trace_list,
                          const std::string assign_mode,
                          std::vector<std::vector<cost_pair> > &cost_matrix);







#endif // MAINWINDOW_H
