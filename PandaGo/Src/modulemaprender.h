#pragma once

#include <QDebug>
#include <QGLWidget>
#include <QStringList>
#include <QVector>
#include <cmath>
#include "infrastructure.h"
#include "../Planner/plsdk.h"

struct Spd4Render
{
    double spd_ins;
    double spd_pl;
};

class ModuleMapRender : public QGLWidget
{
    Q_OBJECT
public:
   explicit ModuleMapRender(QWidget *parent = 0);
   ~ModuleMapRender();

protected:
    void initializeGL();
    void paintGL();

protected:
    GLuint blockFrame;
    GLuint lidar64Frame;
	GLuint lidarHDFrame;

private:
	DataPool * dp;
    PLContext * data;
    int cur;
    //整个画布的大小
    int height;
    int width;
    //翻译表
    QStringList statuTranslator;
    QStringList TPTranslator_a;
    QStringList TPTranslator_b;
    //数据列表
    QStringList datalist;
    QVector<Spd4Render> spdvec;//NOTE: maintained in plotContolSystem
	//贴图变量
	QImage img;
	QImage buf;
	GLuint tex;
	QString im_path;

    void constructTranslator();
    void prepareDataList();
    void plotTrafficSignals();
    void plotBasicIO();
    void plotLidar64RawMap();
    void plotLidar64SimMap();
    void plotContolSystem();
	void plotLidarHD();
	void plotImage();

public:
    void renderMap(DataPool * _data);
};
