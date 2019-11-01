#ifndef MainWindow_h__
#define MainWindow_h__

#include <QMainWindow>
#include <OSGWidget/OSGWidget.h>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = 0, Qt::WindowFlags flags = 0);
    ~MainWindow();

private slots:
    void onLoadPCD();
    void onLoadOOCO_Solid();
    void onLoadOOCO_Coord_X();
    void onLoadOOCO_Coord_Y();
    void onLoadOOCO_Coord_Z();
    void onLoadOOCO_Intensity();
    void onLoadOOCO_RGB();

private:
    OSGWidget* mOSGWidget;

};

#endif
