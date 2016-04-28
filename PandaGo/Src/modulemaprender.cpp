#include "modulemaprender.h"

ModuleMapRender::ModuleMapRender(QWidget *parent) :
QGLWidget(parent)
{
	data = nullptr;
	cur = 0;
	//绘图范围大小
	height = 780;
	width = 1600;
	//预置翻译表
	constructTranslator();
	spdvec.clear();
}

ModuleMapRender::~ModuleMapRender()
{
	glDeleteLists(1, 3);
	glDeleteTextures(1, &tex);
}

void ModuleMapRender::initializeGL()
{
	setFixedHeight(height);
	setFixedWidth(width);
	glShadeModel(GL_SMOOTH);
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glColor4d(0.0, 1.0, 0.0, 0.0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	//定义显示列表:雷达图外线框
	blockFrame = glGenLists(1);
	glNewList(blockFrame, GL_COMPILE);
	glLineWidth(1.0);
	glColor4d(0.0, 128.0 / 255.0, 0.0, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex2d(-1.0 + 1e-3, 1.0 - 1e-3);
	glVertex2d(-1.0 + 1e-3, -1.0 + 1e-3);
	glVertex2d(1.0 - 1e-3, -1.0 + 1e-3);
	glVertex2d(1.0 - 1e-3, 1.0 - 1e-3);
	glEnd();
	glEndList();

	//定义显示列表:160*320雷达图内线框
	lidar64Frame = glGenLists(1);
	glNewList(lidar64Frame, GL_COMPILE);
	glLineWidth(1.0);
	glColor4d(1.0, 1.0, 1.0, 0.1);
	glBegin(GL_LINES);
	for (int c = 1; c < 8; ++c)
	{
		glVertex2d(-1.0 + c*(2.0 / 16.0), -1.0);
		glVertex2d(-1.0 + c*(2.0 / 16.0), 1.0);
	}
	for (int c = 9; c < 16; ++c)
	{
		glVertex2d(-1.0 + c*(2.0 / 16.0), -1.0);
		glVertex2d(-1.0 + c*(2.0 / 16.0), 1.0);
	}
	for (int c = 1; c < 8; ++c)
	{
		glVertex2d(-1.0, -1.0 + c*(2.0 / 32.0));
		glVertex2d(1.0, -1.0 + c*(2.0 / 32.0));
	}
	for (int c = 9; c < 32; ++c)
	{
		glVertex2d(-1.0, -1.0 + c*(2.0 / 32.0));
		glVertex2d(1.0, -1.0 + c*(2.0 / 32.0));
	}
	glColor4d(1.0, 1.0, 1.0, 0.2);
	glVertex2d(-1.0 + 8 * (2.0 / 16.0), -1.0);
	glVertex2d(-1.0 + 8 * (2.0 / 16.0), 1.0);
	glVertex2d(-1.0, -1.0 + 8 * (2.0 / 32.0));
	glVertex2d(1.0, -1.0 + 8 * (2.0 / 32.0));
	glEnd();
	glBegin(GL_LINES);
	
	
	glEnd();
	/* 绘制车辆（2.0m*5.0m）
	 * 车辆规格：每个栅格边长0.25m，车大小2.0m*5.0m
	 * 车辆位置：x=0/pbWidth，y=0/pbHeight*/
	glColor3d(1.0, 0.0, 0.0);
	glLineWidth(1.0);
	glBegin(GL_LINE_LOOP);
	glVertex2d(-1.0 / 20.0, -0.5 + 3.5 / 40.0);
	glVertex2d(-1.0 / 20.0, -0.5 - 1.5 / 40.0);
	glVertex2d(1.0 / 20.0, -0.5 - 1.5 / 40.0);
	glVertex2d(1.0 / 20.0, -0.5 + 3.5 / 40.0);
	glEnd();
	glEndList();

	//定义显示列表:240*240雷达图内线框
	lidarHDFrame = glGenLists(1);
	glNewList(lidarHDFrame, GL_COMPILE);
	glLineWidth(1.0);
	glColor4d(1.0, 1.0, 1.0, 0.1);
	glBegin(GL_LINES);
	for (int c = 1; c < 12; ++c)
	{
		glVertex2d(-1.0 + c*(2.0 / 24.0), -1.0);
		glVertex2d(-1.0 + c*(2.0 / 24.0), 1.0);
	}
	for (int c = 12; c < 24; ++c)
	{
		glVertex2d(-1.0 + c*(2.0 / 24.0), -1.0);
		glVertex2d(-1.0 + c*(2.0 / 24.0), 1.0);
	}
	for (int c = 1; c < 24; ++c)
	{
		glVertex2d(-1.0, -1.0 + c*(2.0 / 24.0));
		glVertex2d(1.0, -1.0 + c*(2.0 / 24.0));
	}
	glColor4d(1.0, 1.0, 1.0, 0.2);
	glVertex2d(-1.0 + 12 * (2.0 / 24.0), -1.0);
	glVertex2d(-1.0 + 12 * (2.0 / 24.0), 1.0);
	glEnd();
	glColor3d(1.0, 0.0, 0.0);
	glLineWidth(1.0);
	glBegin(GL_LINE_LOOP);
	glVertex2d(-1.0 / 15.0, 3.5 / 15.0 -1.0);
	glVertex2d(-1.0 / 15.0, -1.0);
	glVertex2d(1.0 / 15.0, -1.0);
	glVertex2d(1.0 / 15.0, 3.5 / 15.0 - 1.0);
	glEnd();
	glEndList();

	//绑定纹理:此处仅仅使用了一个纹理，所以可以直接绑定一次
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
}

void ModuleMapRender::paintGL()
{
	prepareDataList();
	plotBasicIO();
	plotLidar64RawMap();
	plotLidar64SimMap();
	plotContolSystem();
	plotLidarHD();
	plotImage();
}

void ModuleMapRender::constructTranslator()
{
	statuTranslator.clear();
	statuTranslator << QString("S_INVALID");
	statuTranslator << QString("S_WAIT");
	statuTranslator << QString("S_WAIT_LIGHT");
	statuTranslator << QString("S_ROAD_NAV");
	statuTranslator << QString("S_CROSS_UND");
	statuTranslator << QString("S_STRAIGHT");
	statuTranslator << QString("S_LEFT");
	statuTranslator << QString("S_RIGHT");
	statuTranslator << QString("S_UTURN");
	statuTranslator << QString("S_PARKING");
	statuTranslator << QString("S_FREE_RUN");
	statuTranslator << QString("S_TASK_OVER");
	statuTranslator << QString("NUM_STATUS");

	TPTranslator_a.clear();
	TPTranslator_a << QString("0-START   ");
	TPTranslator_a << QString("1-CROS_IN ");
	TPTranslator_a << QString("2-CROS_OUT");
	TPTranslator_a << QString("3-GUIDE   ");
	TPTranslator_a << QString("4-STOP_IN ");
	TPTranslator_a << QString("5-STOP_OUT");
	TPTranslator_a << QString("6-STOP_POT");
	TPTranslator_a << QString("7-END     ");
	TPTranslator_a << QString("8-ERROR1  ");

	TPTranslator_b.clear();
	TPTranslator_b << QString("0-UNKOWN ");
	TPTranslator_b << QString("1-FORWARD");
	TPTranslator_b << QString("2-TURN_R ");
	TPTranslator_b << QString("3-TURN_L ");
	TPTranslator_b << QString("4-UTURN  ");
	TPTranslator_b << QString("5-SIGN   ");
	TPTranslator_b << QString("6-ERROR2 ");
	TPTranslator_b << QString("6-ERROR2 ");
	TPTranslator_b << QString("6-ERROR2 ");
	TPTranslator_b << QString("6-ERROR2 ");
	TPTranslator_b << QString("6-ERROR2 ");
	TPTranslator_b << QString("6-ERROR2 ");
	return;
}

void ModuleMapRender::prepareDataList()
{
	datalist.clear();
	STATE & state = data->pl_func_input.state;
	GP_INFO & gp = data->pl_func_input.gp_info;
	CS_PL_DATA & cspl = data->pl_func_input.cs_pl_data;
	if (data != nullptr)
	{
		//TODO: 重复载入数据，此处会有异常(dataplatform 增加重载数据信号，各模块更新data指针)
		datalist << QString("FID @ sn   = %1").arg(QString::number(data->sn));
		datalist << QString(" ");
		datalist << QString("CLK @ %1").arg(QString::number(data->clock));
		datalist << QString(" ");
		datalist << QString("POS @  Mod = WGS84  DY");
		datalist << QString("    @ imuX = %1").arg(QString::number(state.pos.ins_coord.x));
		datalist << QString("    @ imuY = %1").arg(QString::number(state.pos.ins_coord.y));
		datalist << QString("    @ gpsX = %1").arg(QString::number(state.pos.com_coord.x));
		datalist << QString("    @ gpsY = %1").arg(QString::number(state.pos.com_coord.y));
		datalist << QString("    @  Yaw = %1").arg(QString::number(state.pos.yaw*1e-8*180.0 / 3.1415926535898, 'f', 2));
		datalist << QString("    @  Spd = %1 km/h").arg(QString::number(state.pos.spd * 3600 * 1e-5, 'f', 2));
		datalist << QString(" ");
		datalist << QString("STU @ %1").arg(statuTranslator.at((int)state.status));
		datalist << QString(" ");
		datalist << QString("TSK [0] %1").arg(QString::number(gp.tps[0].id));
		datalist << QString("        %1 %2").arg(TPTranslator_a.at(gp.tps[0].type)).arg(TPTranslator_b.at(gp.tps[0].direction));
		datalist << QString("    [1] %1").arg(QString::number(gp.tps[1].id));
		datalist << QString("        %1 %2").arg(TPTranslator_a.at(gp.tps[1].type)).arg(TPTranslator_b.at(gp.tps[1].direction));
		datalist << QString("    [2] %1").arg(QString::number(gp.tps[2].id));
		datalist << QString("        %1 %2").arg(TPTranslator_a.at(gp.tps[2].type)).arg(TPTranslator_b.at(gp.tps[2].direction));
		datalist << QString("    [3] %1").arg(QString::number(gp.tps[3].id));
		datalist << QString("        %1 %2").arg(TPTranslator_a.at(gp.tps[3].type)).arg(TPTranslator_b.at(gp.tps[3].direction));
		datalist << QString("CS  @  Str = %1").arg(QString::number(cspl.real_angle*1e-2, 'f', 2));
	}
	else
	{
		datalist << QString("FID @ sn     = N/A");
		datalist << QString("FID @ fu     = N/A");
		datalist << QString("FID @ gp     = N/A");
		datalist << QString("FID @ prm_pl = N/A");
		datalist << QString("FID @ pl_prm = N/A");
		datalist << QString(" ");
		datalist << QString("CLK @ N/A");
		datalist << QString(" ");
		datalist << QString("POS @  Mod = N/A");
		datalist << QString("    @ imuX = N/A");
		datalist << QString("    @ imuY = N/A");
		datalist << QString("    @ gpsX = N/A");
		datalist << QString("    @ gpsY = N/A");
		datalist << QString("    @  Yaw = N/A");
		datalist << QString("    @  Spd = N/A");
		datalist << QString(" ");
		datalist << QString("STU @ N/A");
		datalist << QString(" ");
		datalist << QString("TSK [0] N/A");
		datalist << QString("        N/A        N/A      ");
		datalist << QString("    [1] N/A");
		datalist << QString("        N/A        N/A      ");
		datalist << QString("    [2] N/A");
		datalist << QString("        N/A        N/A      ");
		datalist << QString("    [3] N/A");
		datalist << QString("        N/A        N/A      ");
		datalist << QString("CS  @  Str = N/A");
	}
	return;
}

void ModuleMapRender::plotBasicIO()
{
	glEnable(GL_SCISSOR_TEST);
	glScissor(5, 135, 240, 640);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_SCISSOR_TEST);
	glMatrixMode(GL_PROJECTION);
	glViewport(5, 135, 240, 640);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/* 绘制基本输入输出区域边框 */
	glCallList(blockFrame);

	//填充数据
	glColor4d(0.0, 1.0, 0.0, 1.0);
	QFont aFont("Consolas", 10, QFont::Light);
	for (int i = 0; i != datalist.size(); ++i)
		renderText(5 + 10, 5 + 16 * (i + 1), datalist.at(i), aFont);
}

void ModuleMapRender::plotLidar64RawMap()
{
	glEnable(GL_SCISSOR_TEST);
	glScissor(250, 135, 320, 640);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_SCISSOR_TEST);
	glMatrixMode(GL_PROJECTION);
	glViewport(250, 135, 320, 640);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glCallList(lidar64Frame);
	glCallList(blockFrame);
	if (data == nullptr) return;
	plsdk::Map & MAP = plsdk::Map::GetInstance();

	//填充栅格障碍
	const float * MAP_64_RAW = MAP.map_64_raw;
	for (unsigned int cnt = 0; cnt != MAP_64_CELL_NUM; ++cnt)
	{
		if (255.0f == MAP_64_RAW[cnt])
		{
			GLdouble lb_x0(-1.0 + (cnt % 160 + 0) / 80.0);
			GLdouble lb_y0(-1.0 + (cnt / 160 + 0) / 160.0);
			GLdouble rt_x0(-1.0 + (cnt % 160 + 1) / 80.0);
			GLdouble rt_y0(-1.0 + (cnt / 160 + 1) / 160.0);
			glColor4d(1.0, 0.0, 0.0, 1.0);
			glRectd(lb_x0, lb_y0, rt_x0, rt_y0);
		}
	}
	
	//绘制gis数据库数据
	std::list<plsdk::Target> gis_points;
	plsdk::Gis & gis = plsdk::Gis::GetInstance();
	glColor4d(1.0, 0.0, 0.0, 0.8);
	glPointSize(3.0);
	glEnable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);
	for (const plsdk::Target & t : gis.gisRef)
	{
		glVertex2f(t.x / 2000.0f, t.y / 4000.0f - 0.5f);
	}
	glEnd();
	glDisable(GL_POINT_SMOOTH);

	plsdk::Target & ref = gis.gisCtrl; bool valid = gis.isGisCtrlValid;
	if (valid)
	{
		glColor4d(0.0, 1.0, 0.0, 0.8);
		glPointSize(3.0);
		glEnable(GL_POINT_SMOOTH);
		glBegin(GL_POINTS);
		glVertex2f(ref.x / 2000.0f, ref.y / 4000.0f - 0.5f);
		glEnd();
	}
	
	//绘制自然道边数据
	glColor4d(0.0, 1.0, 1.0, 0.8);
	glLineWidth(2.0);
	glEnable(GL_LINE_SMOOTH);
	glBegin(GL_LINE_STRIP);
	for (int cnt = 0; cnt != MAP.n_l_num; ++cnt)
	{
		glVertex2f(MAP.n_l[cnt].x / 2000.0f, -0.5f + MAP.n_l[cnt].y / 4000.0f);
	}
	glEnd();
	glLineWidth(2.0);
	glBegin(GL_LINE_STRIP);
	for (int cnt = 0; cnt != MAP.n_r_num; ++cnt)
	{
		glVertex2f(MAP.n_r[cnt].x / 2000.0f, -0.5f + MAP.n_r[cnt].y / 4000.0f);
	}
	glEnd();
	glDisable(GL_LINE_SMOOTH);

	//绘制行道线数据
	for (int cnt = 0; cnt != MAP.road_lines->valid_num_in_l_wing; cnt++)
	{
		if (MAP.road_lines->l_edges[cnt].line_color == 1)
			glColor4d(1.0, 1.0, 0.0, 0.5);
		else
			glColor4d(1.0, 1.0, 1.0, 0.5);
		glLineWidth(1.0);
		glBegin(GL_LINE_STRIP);
		for (int idx = 0; idx != MAP.road_lines->l_edges[cnt].valid_num_points; ++idx)
			glVertex2f(MAP.road_lines->l_edges[cnt].line[idx].x / 2000.0f, -0.5f + MAP.road_lines->l_edges[cnt].line[idx].y / 4000.0f);
		glEnd();
	}
	for (int cnt = 0; cnt != MAP.road_lines->valid_num_in_r_wing; cnt++)
	{
		if (MAP.road_lines->r_edges[cnt].line_color == 1)
			glColor4d(1.0, 1.0, 0.0, 0.5);
		else
			glColor4d(1.0, 1.0, 1.0, 0.5);
		glLineWidth(1.0);
		glBegin(GL_LINE_STRIP);
		for (int idx = 0; idx != MAP.road_lines->r_edges[cnt].valid_num_points; ++idx)
			glVertex2f(MAP.road_lines->r_edges[cnt].line[idx].x / 2000.0f, -0.5f + MAP.road_lines->r_edges[cnt].line[idx].y / 4000.0f);
		glEnd();
	}

	//绘制数据库中的规划线
	glColor4d(1.0, 1.0, 1.0, 1.0);
	glLineWidth(1.0);
	glBegin(GL_LINE_STRIP);
	for (int cnt = 0; cnt < data->pl_cs_data_sim.number_of_effective_points; ++cnt)
	{
		glVertex2d(data->pl_cs_data_sim.path[cnt].x / 2000.0f, -0.5f + data->pl_cs_data_sim.path[cnt].y / 4000.0f);
	}
	glEnd();

	glDisable(GL_BLEND);
	return;
}

void ModuleMapRender::plotLidar64SimMap()
{
	glEnable(GL_SCISSOR_TEST);
	glScissor(575, 135, 320, 640);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_SCISSOR_TEST);
	glMatrixMode(GL_PROJECTION);
	glViewport(575, 135, 320, 640);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glCallList(lidar64Frame);
	glCallList(blockFrame);

	//空数据不做处理
	if (data == nullptr) return;
	plsdk::Map & MAP = plsdk::Map::GetInstance();

	//填充栅格障碍
	//填充gis光栅数据
	plsdk::Gis & gis = plsdk::Gis::GetInstance();
	for (const plsdk::Pos & p : gis.gisRaster)
	{
		GLdouble lb_x0(-1.0 + (p.x + 20 + 0) / 80.0);
		GLdouble lb_y0(-1.0 + (p.y + 80 + 0) / 160.0);
		GLdouble rt_x0(-1.0 + (p.x + 20 + 1) / 80.0);
		GLdouble rt_y0(-1.0 + (p.y + 80 + 1) / 160.0);
		glColor4d(1.0, 0.0, 0.0, 128.0 / 255.0f);
		glRectd(lb_x0, lb_y0, rt_x0, rt_y0);
	}
	const float * MAP_64 = MAP.map_64;
	for (unsigned int cnt = 0; cnt != MAP_64_CELL_NUM; ++cnt)
	{
		if (0.0f != MAP_64[cnt])
		{
			GLdouble lb_x0(-1.0 + (cnt % 160 + 0) / 80.0);
			GLdouble lb_y0(-1.0 + (cnt / 160 + 0) / 160.0);
			GLdouble rt_x0(-1.0 + (cnt % 160 + 1) / 80.0);
			GLdouble rt_y0(-1.0 + (cnt / 160 + 1) / 160.0);
			glColor4d(0.0, 1.0, 0.0, MAP_64[cnt]/255.0f);
			glRectd(lb_x0, lb_y0, rt_x0, rt_y0);
		}
	}
	

	//绘制自然道边数据
	glColor4d(0.0, 1.0, 1.0, 0.8);
	glLineWidth(2.0);
	glEnable(GL_LINE_SMOOTH);
	glBegin(GL_LINE_STRIP);
	for (int cnt = 0; cnt != MAP.n_l_num; ++cnt)
	{
		glVertex2f(MAP.n_l[cnt].x / 2000.0f, -0.5f + MAP.n_l[cnt].y / 4000.0f);
	}
	glEnd();
	glLineWidth(2.0);
	glBegin(GL_LINE_STRIP);
	for (int cnt = 0; cnt != MAP.n_r_num; ++cnt)
	{
		glVertex2f(MAP.n_r[cnt].x / 2000.0f, -0.5f + MAP.n_r[cnt].y / 4000.0f);
	}
	glEnd();
	glDisable(GL_LINE_SMOOTH);

	//绘制行道线数据
	glColor4d(1.0, 1.0, 0.0, 0.5);
	glLineWidth(1.0);
	glEnable(GL_LINE_SMOOTH);
	glBegin(GL_LINE_STRIP);
	glEnd();
	glDisable(GL_LINE_SMOOTH);

	//填充规划路径
	glColor4d(1.0, 1.0, 1.0, 1.0);
	glLineWidth(1.0);
	glBegin(GL_LINE_STRIP);
	for (int cnt = 0; cnt < data->pl_cs_data_sim.number_of_effective_points; ++cnt)
		glVertex2d(data->pl_cs_data_sim.path[cnt].x / 2000.0, -0.5 + data->pl_cs_data_sim.path[cnt].y / 4000.0);
	glEnd();

	//绘制gis数据库数据
	glColor4d(1.0, 0.0, 0.0, 0.8);
	glPointSize(3.0);
	glEnable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);
	for (const plsdk::Target & t : gis.gisRef)
	{
		glVertex2f(t.x / 2000.0, t.y / 4000.0 - 0.5);
	}
	glEnd();
	glDisable(GL_POINT_SMOOTH);

	if (gis.isGisCtrlValid)
	{
		glColor4d(0.0, 1.0, 0.0, 0.8);
		glPointSize(3.0);
		glEnable(GL_POINT_SMOOTH);
		glBegin(GL_POINTS);
		glVertex2f(gis.gisCtrl.x / 2000.0, gis.gisCtrl.y / 4000.0 - 0.5);
		glEnd();
	}

	//绘制fmt数据库数据
	plsdk::Fmt & fmt = plsdk::Fmt::GetInstance();
	glColor4d(0.0, 0.0, 1.0, 0.8);
	glPointSize(5.0);
	glEnable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);
	for (const plsdk::Target & t : fmt.fmtLocal)
	{
		glVertex2f(t.x / 2000.0, t.y / 4000.0 - 0.5);
	}
	glEnd();
	glDisable(GL_POINT_SMOOTH);

	glDisable(GL_BLEND);
	return;
}

void ModuleMapRender::plotContolSystem()
{
	glEnable(GL_SCISSOR_TEST);
	glScissor(5, 5, 890, 125);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_SCISSOR_TEST);
	glMatrixMode(GL_PROJECTION);
	glViewport(5, 5, 890, 125);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/* 绘制综合数据区边框 */
	glCallList(blockFrame);

	glLineWidth(1.0);
	glColor4d(0.0, 0.0, 64.0 / 255.0, 1.0);
	glLineStipple(1, 0x0F0F);
	glEnable(GL_LINE_STIPPLE);
	glBegin(GL_LINES);//每5km/h一条线,共45km/h
	for (int i = 1; i < 9; ++i)
	{
		glVertex2d(-1.0 + 2 * 245.0 / 810.0, 1.0 - 2 * i*5.0 / 45.0);
		glVertex2d(1.0 - 1e-3, 1.0 - 2 * i*5.0 / 45.0);
	}
	glVertex2d(-1.0 + 2 * 245.0 / 810.0, 1.0);
	glVertex2d(-1.0 + 2 * 245.0 / 810.0, -1.0);
	glEnd();
	glDisable(GL_LINE_STIPPLE);
	//标定坐标
	glColor4d(0.0, 1.0, 0.0, 1.0);
	renderText(235, 780 - 15, QString("05"), QFont("Consolas", 7, QFont::Light));
	renderText(235, 780 - 29, QString("10"), QFont("Consolas", 7, QFont::Light));
	renderText(235, 780 - 43, QString("15"), QFont("Consolas", 7, QFont::Light));
	renderText(235, 780 - 57, QString("20"), QFont("Consolas", 7, QFont::Light));
	renderText(235, 780 - 71, QString("25"), QFont("Consolas", 7, QFont::Light));
	renderText(235, 780 - 85, QString("30"), QFont("Consolas", 7, QFont::Light));
	renderText(235, 780 - 99, QString("35"), QFont("Consolas", 7, QFont::Light));
	renderText(235, 780 - 113, QString("40"), QFont("Consolas", 7, QFont::Light));
	//Frameid
	renderText(15, 780 - 110, QString("Frameid @ "), QFont("Consolas", 10, QFont::Light));
	//ISOK信号
	renderText(15, 780 - 85, QString("  Is_ok @ "), QFont("Consolas", 10, QFont::Light));
	//左转信号轮廓
	renderText(15, 780 - 60, QString("Sys_cmd"), QFont("Consolas", 10, QFont::Light));
	glColor4d(0.25, 0.25, 0.25, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex2d(0.0 - 650.0 / 810.0, 15.0 / 125.0 - 10.0 / 125);
	glVertex2d(-15.0 / 810.0 - 650.0 / 810.0, 0.0 - 10.0 / 125);
	glVertex2d(0.0 - 650.0 / 810.0, -15.0 / 125.0 - 10.0 / 125);
	glVertex2d(0.0 - 650.0 / 810.0, -7.5 / 125.0 - 10.0 / 125);
	glVertex2d(15.0 / 810.0 - 650.0 / 810.0, -7.5 / 125.0 - 10.0 / 125);
	glVertex2d(15.0 / 810.0 - 650.0 / 810.0, 7.5 / 125.0 - 10.0 / 125);
	glVertex2d(0.0 - 650.0 / 810.0, 7.5 / 125.0 - 10.0 / 125);
	glEnd();
	//右转信号轮廓
	glColor4d(0.25, 0.25, 0.25, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex2d(0.0 - 600.0 / 810.0, 15.0 / 125.0 - 10.0 / 125);
	glVertex2d(15.0 / 810.0 - 600.0 / 810.0, 0.0 - 10.0 / 125);
	glVertex2d(0.0 - 600.0 / 810.0, -15.0 / 125.0 - 10.0 / 125);
	glVertex2d(0.0 - 600.0 / 810.0, -7.5 / 125.0 - 10.0 / 125);
	glVertex2d(-15.0 / 810.0 - 600.0 / 810.0, -7.5 / 125.0 - 10.0 / 125);
	glVertex2d(-15.0 / 810.0 - 600.0 / 810.0, 7.5 / 125.0 - 10.0 / 125);
	glVertex2d(0.0 - 600.0 / 810.0, 7.5 / 125.0 - 10.0 / 125);
	glEnd();
	//倒车信号轮廓
	glColor4d(0.25, 0.25, 0.25, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex2d(-15.0 / 810.0 - 550.0 / 810.0, 0.0 - 10.0 / 125);
	glVertex2d(0.0 - 550.0 / 810.0, -15.0 / 125.0 - 10.0 / 125);
	glVertex2d(15.0 / 810.0 - 550.0 / 810.0, 0.0 - 10.0 / 125);
	glVertex2d(7.5 / 810.0 - 550.0 / 810.0, 0.0 - 10.0 / 125);
	glVertex2d(7.5 / 810.0 - 550.0 / 810.0, 15.0 / 125.0 - 10.0 / 125);
	glVertex2d(-7.5 / 810.0 - 550.0 / 810.0, 15.0 / 125.0 - 10.0 / 125);
	glVertex2d(-7.5 / 810.0 - 550.0 / 810.0, 0.0 - 10.0 / 125);
	glEnd();
	//停车信号轮廓
	glColor4d(0.25, 0.25, 0.25, 1.0);
	glBegin(GL_LINE_LOOP);
	glVertex2d(11.5 / 810.0 - 500.0 / 810.0, 11.5 / 125.0 - 10.0 / 125.0);
	glVertex2d(-11.5 / 810.0 - 500.0 / 810.0, 11.5 / 125.0 - 10.0 / 125.0);
	glVertex2d(-11.5 / 810.0 - 500.0 / 810.0, -11.5 / 125.0 - 10.0 / 125.0);
	glVertex2d(11.5 / 810.0 - 500.0 / 810.0, -11.5 / 125.0 - 10.0 / 125.0);
	glEnd();
	//档位轮廓
	glColor4d(0.0, 1.0, 0.0, 1.0);
	renderText(15, 780 - 35, QString("GearPos"), QFont("Consolas", 10, QFont::Light));
	glColor4d(0.25, 0.25, 0.25, 1.0);
	renderText(80, 780 - 35, QString("P"), QFont("Consolas", 12, QFont::Bold));
	renderText(105, 780 - 35, QString("R"), QFont("Consolas", 12, QFont::Bold));
	renderText(130, 780 - 35, QString("N"), QFont("Consolas", 12, QFont::Bold));
	renderText(155, 780 - 35, QString("D"), QFont("Consolas", 12, QFont::Bold));
	//规划速度
	glColor4d(0.0, 1.0, 0.0, 1.0);
	renderText(250, 780 - 118, QString(" pl  at "), QFont("Consolas", 9, QFont::Light));
	renderText(250, 780 - 103, QString(" sim at "), QFont("Consolas", 9, QFont::Light));
	renderText(250, 780 -  88, QString(" cs  at "), QFont("Consolas", 9, QFont::Light));

	/* 绘制速度曲线数据
	 * 共有41个速度值记录 */
	if (data == nullptr) return;
	//组织速度数据
	spdvec.clear();
	for (int i = 0; cur - i >= 0 && i < 41; ++i)
		spdvec.push_back(Spd4Render{ (data - i)->pl_func_input.state.pos.spd*3600.0*1e-5, data->pl_cs_data.speed*3600.0*1e-5 });
	//计算显示步宽：记录41个速度值
	GLdouble step = (2.0 - 2 * 245.0 / 810.0) / 40.0;
	//绘制速度曲线图
	for (int i = 0; i < spdvec.length() - 1; ++i)
	{
		glColor4d(0.0, 16.0 / 255.0, 0.0, 1.0);
		glBegin(GL_POLYGON);
		glVertex2d(1.0 - i*step, -1.0 + 2 * spdvec.at(i).spd_ins / 45.0);
		glVertex2d(1.0 - (i + 1)*step, -1.0 + 2 * spdvec.at(i + 1).spd_ins / 45.0);
		glVertex2d(1.0 - (i + 1)*step, -1.0 + 1e-2);
		glVertex2d(1.0 - i*step, -1.0 + 1e-2);
		glEnd();
		glLineWidth(1.0);
		glColor4d(0.0, 1.0, 0.0, 1.0);
		glBegin(GL_LINES);
		glVertex2d(1.0 - i*step, -1.0 + 1e-2 + 2 * spdvec.at(i).spd_ins / 45.0);
		glVertex2d(1.0 - (i + 1)*step, -1.0 + 1e-2 + 2 * spdvec.at(i + 1).spd_ins / 45.0);
		glEnd();
	}

	/* 绘制底层命令
	 * 内容包括：Frameid
	 *         is_ok
	 *         system_comman
	 *         gear_pos*/
	//Frameid
	renderText(15, 780 - 110, QString("Frameid @ %1").arg(QString::number(data->pl_cs_data.id)), QFont("Consolas", 10, QFont::Light));
	//ISOK信号
	if (data->pl_cs_data.isok == 0)
	{
		glColor4d(1.0, 0.0, 0.0, 1.0);
		renderText(15, 780 - 85, QString("  Is_ok @ NAY"), QFont("Consolas", 10, QFont::Light));
	}
	else
	{
		glColor4d(0.0, 1.0, 0.0, 1.0);
		renderText(15, 780 - 85, QString("  Is_ok @ YEA"), QFont("Consolas", 10, QFont::Light));
	}

	//左转信号
	if ((data->pl_cs_data.sys_command & 0x04) == 0x00 || (data->pl_cs_data.sys_command & 0x10) == 0x00)
		glColor4d(0.25, 0.25, 0.25, 1.0);
	else
		glColor4d(0.0, 1.0, 0.0, 1.0);
	glBegin(GL_POLYGON);
	glVertex2d(0.0 - 650.0 / 810.0, 15.0 / 125.0 - 10.0 / 125);
	glVertex2d(-15.0 / 810.0 - 650.0 / 810.0, 0.0 - 10.0 / 125);
	glVertex2d(0.0 - 650.0 / 810.0, -15.0 / 125.0 - 10.0 / 125);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex2d(0.0 - 650.0 / 810.0, -7.5 / 125.0 - 10.0 / 125);
	glVertex2d(15.0 / 810.0 - 650.0 / 810.0, -7.5 / 125.0 - 10.0 / 125);
	glVertex2d(15.0 / 810.0 - 650.0 / 810.0, 7.5 / 125.0 - 10.0 / 125);
	glVertex2d(0.0 - 650.0 / 810.0, 7.5 / 125.0 - 10.0 / 125);
	glEnd();
	//右转信号
	if ((data->pl_cs_data.sys_command & 0x08) == 0x00 || (data->pl_cs_data.sys_command & 0x10) == 0x00)
		glColor4d(0.25, 0.25, 0.25, 1.0);
	else
		glColor4d(0.0, 1.0, 0.0, 1.0);
	glBegin(GL_POLYGON);
	glVertex2d(0.0 - 600.0 / 810.0, 15.0 / 125.0 - 10.0 / 125);
	glVertex2d(15.0 / 810.0 - 600.0 / 810.0, 0.0 - 10.0 / 125);
	glVertex2d(0.0 - 600.0 / 810.0, -15.0 / 125.0 - 10.0 / 125);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex2d(0.0 - 600.0 / 810.0, -7.5 / 125.0 - 10.0 / 125);
	glVertex2d(-15.0 / 810.0 - 600.0 / 810.0, -7.5 / 125.0 - 10.0 / 125);
	glVertex2d(-15.0 / 810.0 - 600.0 / 810.0, 7.5 / 125.0 - 10.0 / 125);
	glVertex2d(0.0 - 600.0 / 810.0, 7.5 / 125.0 - 10.0 / 125);
	glEnd();
	//倒车信号
	if ((data->pl_cs_data.sys_command & 0x01) == 0x00)
		glColor4d(0.25, 0.25, 0.25, 1.0);
	else
		glColor4d(0.0, 1.0, 0.0, 1.0);
	glBegin(GL_POLYGON);
	glVertex2d(-15.0 / 810.0 - 550.0 / 810.0, 0.0 - 10.0 / 125);
	glVertex2d(0.0 - 550.0 / 810.0, -15.0 / 125.0 - 10.0 / 125);
	glVertex2d(15.0 / 810.0 - 550.0 / 810.0, 0.0 - 10.0 / 125);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex2d(7.5 / 810.0 - 550.0 / 810.0, 0.0 - 10.0 / 125);
	glVertex2d(7.5 / 810.0 - 550.0 / 810.0, 15.0 / 125.0 - 10.0 / 125);
	glVertex2d(-7.5 / 810.0 - 550.0 / 810.0, 15.0 / 125.0 - 10.0 / 125);
	glVertex2d(-7.5 / 810.0 - 550.0 / 810.0, 0.0 - 10.0 / 125);
	glEnd();
	//停车信号
	if ((data->pl_cs_data.sys_command & 0x02) == 0x00)
		glColor4d(0.25, 0.25, 0.25, 1.0);
	else
		glColor4d(1.0, 0.0, 0.0, 1.0);
	glBegin(GL_POLYGON);
	glVertex2d(11.5 / 810.0 - 500.0 / 810.0, 11.5 / 125.0 - 10.0 / 125.0);
	glVertex2d(-11.5 / 810.0 - 500.0 / 810.0, 11.5 / 125.0 - 10.0 / 125.0);
	glVertex2d(-11.5 / 810.0 - 500.0 / 810.0, -11.5 / 125.0 - 10.0 / 125.0);
	glVertex2d(11.5 / 810.0 - 500.0 / 810.0, -11.5 / 125.0 - 10.0 / 125.0);
	glEnd();

	//档位信号
	glColor4d(0.25, 0.25, 0.25, 1.0);
	if ((data->pl_cs_data.sys_command & 0x02) == 0x00)
		glColor4d(0.25, 0.25, 0.25, 1.0);
	else
		glColor4d(0.0, 1.0, 0.0, 1.0);
	renderText(80, 780 - 35, QString("P"), QFont("Consolas", 12, QFont::Bold));
	if ((data->pl_cs_data.sys_command & 0x01) == 0x00)
		glColor4d(0.25, 0.25, 0.25, 1.0);
	else
		glColor4d(0.0, 1.0, 0.0, 1.0);
	renderText(105, 780 - 35, QString("R"), QFont("Consolas", 12, QFont::Bold));
	if ((data->pl_cs_data.sys_command & 0x00) == 0x00)
		glColor4d(0.25, 0.25, 0.25, 1.0);
	else
		glColor4d(0.0, 1.0, 0.0, 1.0);
	renderText(130, 780 - 35, QString("N"), QFont("Consolas", 12, QFont::Bold));
	if ((data->pl_cs_data.sys_command & 0x0F) != 0x00)
		glColor4d(0.25, 0.25, 0.25, 1.0);
	else
		glColor4d(0.0, 1.0, 0.0, 1.0);
	renderText(155, 780 - 35, QString("D"), QFont("Consolas", 12, QFont::Bold));

	//规划速度
	renderText(250, 780 - 118, QString(" pl  at %1 km/h").arg(QString::number(data->pl_cs_data.speed * 3600 * 1e-5, 'f', 2)), QFont("Consolas", 9, QFont::Light));
	renderText(250, 780 - 103, QString(" sim at %1 km/h").arg(QString::number(data->pl_cs_data_sim.speed * 3600 * 1e-5, 'f', 2)), QFont("Consolas", 9, QFont::Light));
	renderText(250, 780 -  88, QString(" cs  at %1 km/h").arg(QString::number(data->pl_func_input.state.pos.spd*3600.0*1e-5, 'f', 2)), QFont("Consolas", 9, QFont::Light));
	return;
}

void ModuleMapRender::plotLidarHD()
{
	//绘制高清图
	glEnable(GL_SCISSOR_TEST);
	glScissor(900, 295, 480, 480);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_SCISSOR_TEST);
	glMatrixMode(GL_PROJECTION);
	glViewport(900, 295, 480, 480);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glCallList(lidarHDFrame);
	glCallList(blockFrame);
	if (data == nullptr) return;
	plsdk::Map & MAP = plsdk::Map::GetInstance();

	const float * MAP_HD = MAP.map_hd;
	for (unsigned int cnt = 0; cnt != MAP_HD_CELL_NUM; ++cnt)
	{
		GLdouble lb_x0(-1.0 + (cnt % 240 + 0) / 120.0);
		GLdouble lb_y0(-1.0 + (cnt / 240 + 0) / 120.0);
		GLdouble rt_x0(-1.0 + (cnt % 240 + 1) / 120.0);
		GLdouble rt_y0(-1.0 + (cnt / 240 + 1) / 120.0);
		if (255.0f == MAP_HD[cnt])
		{
			glColor4d(1.0, 0.0, 0.0, 1.0);
			glRectd(lb_x0, lb_y0, rt_x0, rt_y0);
		}
	}

	glCallList(blockFrame);
	glDisable(GL_BLEND);
	return;
}

void ModuleMapRender::plotImage()
{
	//首先读取图片
	glEnable(GL_SCISSOR_TEST);
	glScissor(900, 5, 480, 285);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_SCISSOR_TEST);
	glMatrixMode(GL_PROJECTION);
	glViewport(900, 5, 480, 285);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glCallList(blockFrame);
	if (data == nullptr) return;
	glEnable(GL_TEXTURE_2D);
	/*
	if (img.load(QString("%1/image/ImageL%2.jpg").arg(dp->path).arg(data->id_mv_n, 8, 10, QChar('0'))))
	{
		buf = convertToGLFormat(img);
		glTexImage2D(GL_TEXTURE_2D, 0, 3, buf.width(), buf.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, buf.bits());
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
		glBegin(GL_QUADS);
		glTexCoord2f(0.0, 0.0); glVertex2f(-1.0, -1.0);
		glTexCoord2f(1.0, 0.0); glVertex2f(1.0, -1.0);
		glTexCoord2f(1.0, 1.0); glVertex2f(1.0, 1.0);
		glTexCoord2f(0.0, 1.0); glVertex2f(-1.0, 1.0);
		glEnd();
	}
	*/
	glDisable(GL_TEXTURE_2D);
	
	return;
}

void ModuleMapRender::renderMap(DataPool * _data)
{
	//TODO using the copy of pointer, Mind MemoryLeak!!!
	dp = _data;
	cur = _data->cur;
	data = _data->sysFlow + _data->cur;
	im_path = _data->path;
	updateGL();
	return;
}
