#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDialog>
#include <QGraphicsObject>
#include <QtCore>
#include <QtGui>

/************* SUPER FUCKING IMPORTANT *************/
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
/************* SUPER FUCKING IMPORTANT *************/

using namespace cv  ;
using namespace std ;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    string file_name_cv;
    Mat image;
   // Mat image_gray;
    // mark 1 = central
    QGraphicsRectItem *mark1;

    // mark 2 = polegar , mark 3 = indicador... mark 6 = minimo
    QGraphicsRectItem *mark2;
    QGraphicsRectItem *mark3;
    QGraphicsRectItem *mark4;
    QGraphicsRectItem *mark5;
    QGraphicsRectItem *mark6;
    double coordinates[24];

    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;
};

#endif // MAINWINDOW_H
