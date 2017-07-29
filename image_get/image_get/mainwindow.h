#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDialog>
#include <QGraphicsObject>
#include <QtCore>
#include <QtGui>

#define SCALE_INTERVAL 2
#define SCALE_PREC 10 // Decimal com 1 casa
#define SCALE_EL 12

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

    // variables - put this as private later
    int intensity[3];
    double coordinates[24];
    double R_channel_interp[ ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ] ;
    double G_channel_interp[ ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ] ;
    double B_channel_interp[ ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ] ;
    double temperature_interp[ ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ] ;

    // my functions
    int getIntensityGray (const Mat &image, int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) ;
    void getIntensityBGR (const Mat &image, int topLeftX, int topLeftY, int bottomRightX, int bottomRightY, int intensity_aux[]) ;
    double get_y_interp_increment( int x0, int x1, int y0, int y1, double temperature_precision ) ;
    void fill_vec( int v1, double v[], int initial_pos, double increment ) ;
    void fill_Color_Channel( int temperature_vec[], int colorVec[], double channel_dest[] ) ;


    // markers
    // mark 1 = central
    QGraphicsRectItem *mark1;
    // mark 2 = polegar , mark 3 = indicador... mark 6 = minimo
    QGraphicsRectItem *mark2;
    QGraphicsRectItem *mark3;
    QGraphicsRectItem *mark4;
    QGraphicsRectItem *mark5;
    QGraphicsRectItem *mark6;
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;
};

#endif // MAINWINDOW_H
