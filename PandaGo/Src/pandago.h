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

	//���ݳ���Ϣ��ʾ
	DataPool * dp;
	QLabel * datapool;

	//����widget������
	QSplitter * centralSpliter;

	//�滮ģ����
	ModulePlSimulator * plSim;

	//��ͼ�������view��model
	ModuleMapRender * mapGL;

	//���Կ�����
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
