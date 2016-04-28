#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QSplitter>
#include <QPushButton>
#include <QLineEdit>
#include <QSpinBox>
#include <QTimer>
#include <time.h>

#include "ui_pandago.h"

#include "infrastructure.h"
#include "moduledataplatform.h"
#include "modulemaprender.h"
#include "moduleplsimulator.h"

namespace Ui {
	class PandaGoClass;
}

class PandaGo : public QMainWindow
{
	Q_OBJECT
public:
	explicit PandaGo(QWidget *parent = 0);
	~PandaGo();

private:
	Ui::PandaGoClass * ui;

	//数据池信息显示
	DataPool * dp;
	QLabel * datapool;

	//中央widget的容器
	QSplitter * centralSpliter;

	//规划模拟器
	ModulePlSimulator * plSim;

	//地图浏览器的view和model
	ModuleMapRender * mapGL;

	//调试控制器
	bool play;
	QPushButton * control;
	QLabel * info;
	QPushButton * step;
	QPushButton * back;
	QPushButton * stop;
	QPushButton * jump;
	QLineEdit * frame;
	QSpinBox * multiplier;
	QTimer * timer;
	int T;

	private slots:
	void on_actionLoad_triggered();
	void doControl();
	void doPlay();
	void doJump();
	void doStop();
	void doStep();
	void doBack();
	void playBack();
	void adjustMultiplier(int);

};
