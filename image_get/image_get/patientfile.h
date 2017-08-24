#ifndef PATIENTFILE_H
#define PATIENTFILE_H

#include <QtCore>
#include <QtXml>
#include <QDebug>
#include <QFileInfo>
#include <QDialog>

namespace Ui {
class patientFile;
}

class patientFile : public QDialog
{
    Q_OBJECT

public:
    explicit patientFile(QWidget *parent = 0);


    void ReadXML_patient();
    void UpdateXML_patient();


    QString get_dir_patient();
    void set_dir_patient(QString dir_new);

    bool fileExists(QString path);
    void ListElement(QDomElement root, QString tagname, QString attribute);

    ~patientFile();

private slots:
    void on_save_clicked();

    void on_cancel_clicked();

private:
    Ui::patientFile *ui;
    QString dir_patient;

};

#endif // PATIENTFILE_H
