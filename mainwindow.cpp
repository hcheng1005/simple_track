#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFile>
#include <QDebug>

#include "iostream"
#include <QTextStream>


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


typedef struct
{
    double x;
    double y;
    double z;
    double heading;
    double length;
    double width;
    double height;
    double score;
}dets_t;

void MainWindow::on_pushButton_released()
{
   //read files
    QFile file("/home/charles/simple_track/test.csv");
    file.open(QIODevice::ReadOnly);
    QTextStream read22(&file);
    QStringList data = read22.readAll().split("\n", QString::SkipEmptyParts);
    QRegExp mySeparator = QRegExp(":| |,");

    std::vector<dets_t> dets_list;

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
        det_.score = tempData[pos].toDouble();pos += 2;

        dets_list.emplace_back(det_);
    }

    std::cout << dets_list.size() << std::endl;
}

