#include "pandago.h"
#include <Windows.h>

PandaGo::PandaGo(QWidget *parent) : QMainWindow(parent), ui(new Ui::PandaGoClass)
{
	ui->setupUi(this);
	//�������ݳض���������Ϣ
	dp = new DataPool;
	datapool = new QLabel(this);

	centralSpliter = new QSplitter(this);

	//on datapool
	ui->mainToolBar->setFixedHeight(30);
	datapool->setText("<b>DataPool:</b> <font color=\"red\">N/A</font> <b>with</b> <font color=\"blue\">" + QString::number(dp->end) + "</font> <b>frames</b>");
	datapool->setFixedWidth(350);
	ui->mainToolBar->addWidget(datapool);
	ui->mainToolBar->addSeparator();

	//on �滮ģ����
	plSim = new ModulePlSimulator;

	//on ��ͼ��Ⱦ��
	mapGL = new ModuleMapRender(this);
	centralSpliter->addWidget(mapGL);

	this->setCentralWidget(centralSpliter);

	//on player
	control = new QPushButton;
	control->setText("Play");
	control->setCheckable(true);
	control->setChecked(false);
	control->setShortcut(Qt::Key_Space);
	control->setStyleSheet("color:green");
	connect(control, SIGNAL(clicked()), this, SLOT(doControl()));
	info = new QLabel;
	info->setText("00000 / 00000");
	info->setFixedWidth(100);
	info->setAlignment(Qt::AlignCenter);
	step = new QPushButton;
	step->setText("Step");
	step->setShortcut(Qt::Key_Right);
	step->setStyleSheet("color:blue");
	connect(step, SIGNAL(clicked()), this, SLOT(doStep()));
	back = new QPushButton;
	back->setText("Back");
	back->setShortcut(Qt::Key_Left);
	back->setStyleSheet("color:blue");
	connect(back, SIGNAL(clicked()), this, SLOT(doBack()));
	stop = new QPushButton;
	stop->setText("Stop");
	stop->setStyleSheet("color:red");
	stop->setShortcut(Qt::Key_Home);
	connect(stop, SIGNAL(clicked()), this, SLOT(doStop()));
	jump = new QPushButton;
	jump->setText("Jump");
	jump->setStyleSheet("color:yellow");
	connect(jump, SIGNAL(clicked()), this, SLOT(doJump()));
	frame = new QLineEdit;
	frame->setText("00000");
	frame->setFixedWidth(100);
	frame->setAlignment(Qt::AlignCenter);
	multiplier = new QSpinBox;
	multiplier->setRange(10, 1000);
	multiplier->setSingleStep(10);
	multiplier->setSuffix(" ms");
	multiplier->setValue(100);
	multiplier->setFixedWidth(80);
	multiplier->setAlignment(Qt::AlignCenter);
	connect(multiplier, SIGNAL(valueChanged(int)), this, SLOT(adjustMultiplier(int)));
	ui->mainToolBar->addWidget(control);
	ui->mainToolBar->addWidget(multiplier);
	ui->mainToolBar->addWidget(back);
	ui->mainToolBar->addWidget(info);
	ui->mainToolBar->addWidget(step);
	ui->mainToolBar->addWidget(stop);
	ui->mainToolBar->addWidget(frame);
	ui->mainToolBar->addWidget(jump);
	timer = new QTimer;
	connect(timer, SIGNAL(timeout()), this, SLOT(doPlay()));
	play = false;
	T = multiplier->value();
	centralSpliter->setHandleWidth(3);
}

PandaGo::~PandaGo()
{
	delete dp;
	delete datapool;
	delete ui;
	delete mapGL;
	delete centralSpliter;
	delete control;
	delete info;
	delete step;
	delete back;
	delete stop;
	delete jump;
	delete frame;
	delete multiplier;
	delete timer;
}

void PandaGo::on_actionLoad_triggered()
{
	ModuleDataPlatform mDP(this, dp);
	mDP.exec();
	if (dp->status != DataPoolEmpty)
	{
		datapool->setText("<b>DataPool:</b> <font color=\"red\">" + dp->name + "</font>" + \
			" <b>with</b><font color=\"blue\"> " + QString::number(dp->end) + "</font> <b>frames</b>");
	}
}

void PandaGo::doControl()
{
	//���ݳطǿյ�����£������ü���������֡�ţ����򱨴�
	if (dp->status != DataPoolEmpty)
	{
		if (play == true)
		{
			play = false;
			timer->stop();
			control->setText("Play");
			control->setChecked(false);
		}
		else
		{
			play = true;
			timer->start(T);
			control->setText("Pause");
			control->setChecked(true);
		}
	}
	else
	{
		doStep();
	}
}

void PandaGo::doPlay()
{
	//���ݳطǿյ�����£����Ե���֡��
	if (dp->status != DataPoolEmpty)
	{
		dp->cur++;
		if (dp->cur >= dp->end)
		{
			//������Խ�������curά����ĩβ���������ݳص�״̬��Ϊ����ĩβ��ֹͣ��ʱ��
			doControl();
			dp->cur = dp->end - 1;
			dp->status = DataPoolAtEnd;
		}
		else if (dp->cur < 0)
		{
			//������Խ�������cur��ת��0������ʾ�������ݳصĿ�ʼ����������
			dp->cur = 0;
		}
	}
	playBack();
	return;
}

void PandaGo::doJump()
{
	//���ݳطǿյ�����£�������ת֡��
	if (dp->status != DataPoolEmpty)
	{
		int tarFrame = frame->text().toInt();
		if (tarFrame >= dp->end || tarFrame < 0)
		{
			//��������Խ��������curά�ֲ��䣬����ʾ����Ƿ�
			QMessageBox::warning(this, "Warning", "UNVALID frame id!", QMessageBox::Yes);
		}
		else
		{
			dp->cur = tarFrame;
			dp->status = DataPoolValid;
		}
	}
	playBack();
	return;
}

void PandaGo::doStep()
{
	//���ݳطǿյ�����£����Ե���֡��
	if (dp->status != DataPoolEmpty)
	{
		dp->cur++;
		if (dp->cur >= dp->end)
		{
			//������Խ�������curά����ĩβ���������ݳص�״̬��Ϊ����ĩβ
			dp->cur = dp->end - 1;
			dp->status = DataPoolAtEnd;
		}
		else if (dp->cur < 0)
		{
			//������Խ�������cur��ת��0
			dp->cur = 0;
		}
	}
	playBack();
	return;
}

void PandaGo::doBack()
{
	//���ݳطǿյ�����£����Եݼ�֡��
	if (dp->status != DataPoolEmpty)
	{
		dp->cur--;
		if (dp->cur < 0)
		{
			//������Խ�������curά����0������ʾ�������ݳصĿ�ʼ
			dp->cur = 0;
			QMessageBox::warning(this, "Warning", "\"DataPool\" reach to the Begin!", QMessageBox::Yes);
		}
		else if (dp->cur >= dp->end)
		{
			//������Խ�������curά����ĩβ���������ݳص�״̬��Ϊ����ĩβ
			dp->cur = dp->end - 1;
			dp->status = DataPoolAtEnd;
		}
	}
	playBack();
	return;
}

void PandaGo::doStop()
{
	//���ݳطǿյ�����£������������ݳ�
	if (dp->status != DataPoolEmpty)
	{
		dp->reset();
		timer->stop();
	}
	playBack();
	return;
}

void PandaGo::playBack()
{
	//ִ�в����߼������ŵ�ǰ֡
	//�����߼�������֡�ű仯�����ݳ�״̬�л���ֻ�������ݳر������ط�����
	switch (dp->status)
	{
	case DataPoolEmpty:
		QMessageBox::warning(this, "Warning", "No valid data in \"DataPool\"!", QMessageBox::Yes);
		info->setText(QString("00000 / 00000"));
		frame->setText(QString::number(0));
		break;

	case DataPoolAtEnd:
		QMessageBox::warning(this, "Warning", "\"DataPool\" reach to the END!", QMessageBox::Yes);
		info->setText(QString("%1 / %2").arg(QString::number(dp->cur)).arg(QString::number(dp->end - 1)));
		frame->setText(QString::number(dp->cur));
		break;

	default:
		info->setText(QString("%1 / %2").arg(QString::number(dp->cur)).arg(QString::number(dp->end - 1)));
		frame->setText(QString::number(dp->cur));
		LARGE_INTEGER tick_freq;
		QueryPerformanceFrequency(&tick_freq);
		LARGE_INTEGER tick_start;
		QueryPerformanceCounter(&tick_start);
		plSim->call(dp->sysFlow, dp->cur);
		LARGE_INTEGER tick_end;
		QueryPerformanceCounter(&tick_end);
		(dp->sysFlow + dp->cur)->clock = (tick_end.QuadPart - tick_start.QuadPart) / (double)(tick_freq.QuadPart) * 1000.0;
		mapGL->renderMap(dp);
		break;
	}
	return;
}

void PandaGo::adjustMultiplier(int t)
{
	T = t;
	if (timer->isActive())
	{
		timer->stop();
		timer->start(T);
	}
}
