#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDialog>
#include <QGraphicsObject>
#include <QtCore>
#include <QtGui>
#include <QSpinBox>
#include <QDir>
#include <QGraphicsPixmapItem>
#include <QImage>
#include <QtXml>
#include <QDebug>
#include <QFileInfo>
#include <algorithm>
#include <vector>

#include "squaremarker.h"
#include "patientfile.h"


#define SCALE_INTERVAL 2
#define SCALE_PREC 10 // Decimal com 1 casa
#define SCALE_EL 12
#define MARKERS_NUMBER 6
#define PICTURE_NUMBER 12

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
    string path_att_cv;
    Mat image;

    QStringList image_list;
    QString dir;
    QString path_att;
    int image_number ;

    // variables - put this as private later
    int intensity[3];
    double coordinates[24];
    double R_channel_interp[   ( ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ) + 1 ] ;
    double G_channel_interp[   ( ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ) + 1 ] ;
    double B_channel_interp[   ( ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ) + 1 ] ;
    double temperature_interp[ ( ( SCALE_EL - 1 ) * SCALE_PREC * SCALE_INTERVAL ) + 1 ] ;

    int x_markers [ (MARKERS_NUMBER * PICTURE_NUMBER) ];
    int y_markers [ (MARKERS_NUMBER * PICTURE_NUMBER) ];
    double temp_values [ (MARKERS_NUMBER * PICTURE_NUMBER) ];
    int size_bigger [ (MARKERS_NUMBER * PICTURE_NUMBER) ];
    int size_smaller [ (MARKERS_NUMBER * PICTURE_NUMBER) ];

    // my functions
    int getIntensityGray (const Mat &image, int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) ;
    void getIntensityBGR (const Mat &image, int topLeftX, int topLeftY, int bottomRightX, int bottomRightY, int intensity_aux[]) ;
    double get_y_interp_increment( int x0, int x1, int y0, int y1, double temperature_precision ) ;
    void fill_vec( int v1, double v[], int initial_pos, double increment ) ;
    void fill_Color_Channel( int temperature_vec[], int colorVec[], double channel_dest[] ) ;

    bool fileExists(QString path);
    QStringList ListElement(QDomElement root, QString tagname, QString attribute);

    void ReadXML_temperature();
    void UpdateXML_temperature();

     qreal xc, yc, wc, hc ;
     qreal xf, yf, wf, hf ;

    ~MainWindow();

private slots:

    void on_spinBox_valueChanged(int arg1);
    void on_spinBox_2_valueChanged(int arg1);

    void on_anterior_clicked();
    void on_proximo_clicked();
    void on_diretorio_clicked();
    void on_temperature_clicked();
    void on_save_clicked();
    void on_editarFicha_clicked();

    void on_processingMethod_currentIndexChanged(const QString &arg1);

    void on_pushButton_clicked();

    void on_actionEscolher_novo_diretorio_triggered();

    void on_actionSalvar_triggered();

    void on_actionComo_usar_o_programa_triggered();

    void on_actionSobre_triggered();

private:
    Ui::MainWindow *ui;
    QGraphicsScene *scene;
    QString Filename ;


    // markers
    // mark 1 = central
    squaremarker *mark1;
    // mark 2 = polegar , mark 3 = indicador... mark 6 = minimo
    squaremarker *mark2;
    squaremarker *mark3;
    squaremarker *mark4;
    squaremarker *mark5;
    squaremarker *mark6;
};

#endif // MAINWINDOW_H
