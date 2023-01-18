#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qcustomplot.h"


#include "./simple_tracker/simple_tracker.h"

typedef std::pair<std::pair<uint, uint>, double> cost_type;

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
};







void compute_iou_distance(const std::vector<dets_t> dets_list, const std::vector<Eigen::VectorXd> trace_list, const std::string assign_mode);
void greedy_test(std::vector<std::vector<cost_type>> &cost_matrix);
bool myTestFunc(cost_type c1, cost_type c2);







#endif // MAINWINDOW_H
