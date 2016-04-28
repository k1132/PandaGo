#include "plsdk.h"

#include <string>
#include "./zgcc/trace_road.h"

typedef int KERNEL_RET;
#define RET_OK		0
#define RET_ERR		1

enum Action
{
	EXECU_STOP = 0,
	EXECU_REVERSE = 1,
	EXECU_LEFT_LANE = 2,
	EXECU_RIGHT_LANE = 3,

	TRACE_DUMMY_ROAD = 10,
	TRACE_NATURAL_ROAD = 11,
	TRACE_URBEN_ROAD = 12,
};

static Action REG_ACTION;
static float REG_SPEED;
static bool REG_IS_NATURAL_ROAD_BLOCKED;

extern KERNEL_RET Odometry2Speed(const ODO_INFO * odo, float * speed);//��ODO��Ϣ�м��㵱ǰ����
extern KERNEL_RET SetRegistry(const std::string & key, void * val);//�ı��ں�ע����ָ��������
extern KERNEL_RET GetRegistry(const std::string & key, void * val);//����ں�ע����ָ��������
extern KERNEL_RET GenerateVirtualNaturalRoad();//��һ��GPS��������һ����Ȼ����(��������)�����������ڵ����ڲ�

extern KERNEL_RET ZGCC_TraceNaturalRoad(PL_FUNC_INPUT * pl_func_input);
extern KERNEL_RET ZGCC_CutNaturalEdge(NATURAL_BOUNDARY & boundary);//��ȡ��Ȼ���߱���
extern KERNEL_RET ZGCC_FitNaturalEdge(NATURAL_BOUNDARY & boundary);//����������Ȼ���ߵı���
extern KERNEL_RET ZGCC_FixNaturalEdge(NATURAL_BOUNDARY & boundary);//�������һ����Ȼ����(��������)����դ��ͼ��������ײ������Ȼ����

KERNEL_RET Odometry2Speed(const ODO_INFO * odo, float * speed)
{
	static ODO_INFO odo_pre = *odo;
	static unsigned short odo_win[5] = { 0, 0, 0, 0, 0 };
	static unsigned short odo_buf[5] = { 0, 0, 0, 0, 0 };

	unsigned short odo_l_dif = 0u;
	unsigned short odo_r_dif = 0u;
	//���������ֵ�����������ʱ��ֵС�ھ�ֵ����Ҫ�Լ���������
	(odo->left < odo_pre.left) ? (odo_l_dif = odo->left + (0xFFFF - odo_pre.left)) : (odo_l_dif = odo->left - odo_pre.left);
	(odo->right < odo_pre.right) ? (odo_r_dif = odo->right + (0xFFFF - odo_pre.right)) : (odo_l_dif = odo->right - odo_pre.right);

	//�ƶ�����������ǰ����ֵ���뵽������
	memmove(odo_win + 1, odo_win, sizeof(unsigned short) * 4);
	odo_win[0] = odo_l_dif + odo_r_dif;

	//���»����������
	odo_pre = *odo;

	//������ֵ�˲������޳�������С��Ұֵ
	memcpy(odo_buf, odo_win, sizeof(unsigned short) * 5);
	register int odo_mid = 0;
	for (unsigned int i = 0; i != 3; ++i)//��ʵ��ֻ��Ҫ������±�Ϊ2��ֵ����
	{
		odo_mid = odo_buf[i];
		for (unsigned int j = i + 1; j != 5; ++j)
		{
			if (odo_buf[i] >= odo_buf[j])
			{
				odo_mid = odo_buf[j];
				odo_buf[i] = odo_mid;
			}
		}
	}

	//ͨ���궨��������������̼Ƶ������������ɾ��룬�Ա����� 0.05028  Ѳ�󽢲��� 0.0513149  ÿ��λ����ľ��� ��λm
	*speed = odo_mid * 0.5f * 51.3149f;
	if (1 == odo->revers) *speed = -*speed;

	return RET_OK;
}

KERNEL_RET SetRegistry(const std::string & key, void * val)
{
	if ("REG_ACTION" == key)
	{
		Action * p = static_cast<Action *>(val);
		REG_ACTION = *p;
		return RET_OK;
	}
	else if ("REG_SPEED" == key)
	{
		float * p = static_cast<float *>(val);
		REG_SPEED = *p;
		return RET_OK;
	}
	else
	{
		return RET_ERR;
	}
}

KERNEL_RET GetRegistry(const std::string & key, void * val)
{
	if ("REG_ACTION" == key)
	{
		Action * p = static_cast<Action *>(val);
		*p = REG_ACTION;
		return RET_OK;
	}
	else if ("REG_SPEED" == key)
	{
		float * p = static_cast<float *>(val);
		*p = REG_SPEED;
		return RET_OK;
	}
	else
	{
		val = nullptr;
		return RET_ERR;
	}
}

KERNEL_RET GenerateVirtualNaturalRoad(ROAD_NATURAL_BOUNDARY * natural, plsdk::Obs * obs)
{
	return RET_OK;
}

KERNEL_RET ZGCC_TraceNaturalRoad(PL_FUNC_INPUT * pl_input)
{
	static NATURAL_BOUNDARY g_natural_boundary;
	static int g_l_num, g_r_num;
	memset(&g_natural_boundary, 0, sizeof(NATURAL_BOUNDARY));

	//1 ��ȡ���޼���Ȼ��������,��һ���ֵĴ�����Դ��zgcc/trace_road.cpp:keep_natural_road
	//1.1 ��Э�����ݽṹ����ȡ����Ȼ���ߵ�����
	ROAD_NATURAL_BOUNDARY & _natural = pl_input->fu_pl_data.boundary;
	for (unsigned int cnt = 0; cnt != NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE; ++cnt)
	{
		if (_natural.l_boundary[cnt].x != 0 && _natural.l_boundary[cnt].y != 0)
		{
			g_natural_boundary.l_boundary[g_natural_boundary.l_nums++] = \
				COOR2{ (INT32)_natural.l_boundary[cnt].x, (INT32)_natural.l_boundary[cnt].y };
		}
		if (_natural.r_boundary[cnt].x != 0 && _natural.r_boundary[cnt].y != 0)
		{
			g_natural_boundary.r_boundary[g_natural_boundary.l_nums++] = \
				COOR2{ (INT32)_natural.r_boundary[cnt].x, (INT32)_natural.r_boundary[cnt].y };
		}
	}
	g_l_num = g_natural_boundary.l_nums;
	g_r_num = g_natural_boundary.r_nums;

	//1.2 �޼�������Э�����ݣ�ʹ��y���������4m��30m��Լ��Ҫ��,�����������²�ֵ��20����
	//��ȡ��Ȼ���߱���
	ZGCC_CutNaturalEdge(g_natural_boundary);
	//����������������
	ZGCC_FitNaturalEdge(g_natural_boundary);
	/* Sealed!!!!! zgcc�ϵ��߼�������ʹ�����µ��߼�
	unsigned int ptr_l = 0, ptr_r = 0, flag_l = 0, flag_r = 0, num_l = 0, num_r = 0;
	for (unsigned int cnt = 0; cnt != g_natural_boundary.l_nums; ++cnt)
	{
		//�ڲ���ͷģʽ�£�Ѱ�ҵ�һ���������4m�����ĵ�
		if (0 == flag_l && g_natural_boundary.l_boundary[cnt].y >= 400){ ptr_l = cnt; flag_l = 1; num_l = 1; }
		if (0 == flag_r && g_natural_boundary.r_boundary[cnt].y >= 400){ ptr_r = cnt; flag_r = 1; num_r = 1; }
		//�ڲ���βģʽ�£�Ϊ����С��30m�����
		if (1 == flag_l && g_natural_boundary.l_boundary[cnt].y < 3000){ ++num_l; }
		if (1 == flag_r && g_natural_boundary.r_boundary[cnt].y < 3000){ ++num_r; }
	}
	for (unsigned int cnt; cnt)
	if (num_l >= 2 && num_r >= 2)
	{
		memmove(g_natural_boundary.l_boundary, g_natural_boundary.l_boundary + ptr_l, sizeof(COOR2)*num_l);
		g_natural_boundary.l_nums = num_l;
		memmove(g_natural_boundary.r_boundary, g_natural_boundary.r_boundary + ptr_r, sizeof(COOR2)*num_r);
		g_natural_boundary.r_nums = num_r;
		
	}
	else
	{
		//û����Ч�����ݣ������Ȼ���ߵ�����
		memset(g_natural_boundary.l_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		g_natural_boundary.l_nums = 0;
		memset(&g_natural_boundary.r_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		g_natural_boundary.r_nums = 0;
	}
	*/
	g_l_num = g_natural_boundary.l_nums;
	g_r_num = g_natural_boundary.r_nums;

	//2 ������Ȼ���ߣ�ʹ����Ȼ�������е�������һ�����������򣬴�����Դ��zgcc/trace_road.cpp:check_the_lane
	static COOR2 l_edge_buf[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	static COOR2 r_edge_buf[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	//2.1 ���������ݿ�������ʱ��·�߽���
	memcpy(l_edge_buf, g_natural_boundary.l_boundary, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE*sizeof(COOR2));
	memcpy(r_edge_buf, g_natural_boundary.r_boundary, NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE*sizeof(COOR2));
	//2.2 ����·������������ϰ���ȡ��

	return RET_OK;
}

KERNEL_RET ZGCC_CutNaturalEdge(NATURAL_BOUNDARY & boundary)
{
	//ASSERT!!!!!
	if (boundary.l_nums < 2 || boundary.r_nums < 2)
	{
		memset(boundary.l_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		memset(boundary.r_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		boundary.l_nums = 0;
		boundary.r_nums = 0;
		return RET_ERR;
	}
	bool isValid_l = true, isValid_r = true;//���ߺϷ���־,Ԥ��Ϊ�Ϸ�

	//1 ���߶���
	//1.1 ���߶��룬��߽��߲���
	if (boundary.l_boundary[0].y > 400)//��һ�����Ѿ�������400������ߴ��ӳ�������һ����
	{
		COOR2 * _p = boundary.l_boundary;
		COOR2 base_point;
		unsigned int move_num = 0;//�����ƶ���
		//�жϵ�ǰ���ߵ��Ƿ��Ѿ���ռ��
		(boundary.l_nums < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE) ?
			//����δռ����ͳһ����ƶ�һ��λ�ã��ٲ���һ����ֵ���ܵ�����һ
			(move_num = boundary.l_nums++) :
			//������ռ����ֻ�ƶ�ǰ20-1�����ٲ�һ����ֵ���ܵ�������
			(move_num = boundary.l_nums-1);
		memmove(boundary.l_boundary + 1, boundary.l_boundary, sizeof(COOR2)*move_num);//Ų����
		//�������
		base_point.x = INT32(_p->x + (_p->x - (_p + 1)->x) * float(400 - _p->y) / float(_p->y - (_p + 1)->y));
		base_point.y = 400;
		boundary.l_boundary[0] = base_point;
	}
	else if (boundary.l_boundary[0].y < 400)//��һ����û�г���400����������Ѱ�ҵ���Խ���ߵ�һ�Ե�
	{
		//��λ������ǰһ����
		int p_pre = 0;
		for (int cnt = 0; cnt != boundary.l_nums; ++cnt)
		{
			if (boundary.l_boundary[cnt].y < 400)
				p_pre = cnt;
			else
				break;
		}
		if (p_pre + 1 < boundary.l_nums)//��λ�㲻�����һ���㣬���޸ĸö�λ�㣬ʹ��Ϊ��λ�����һ������y=400�Ľ�ȡ��
		{
			COOR2 * _p = boundary.l_boundary + p_pre;
			COOR2 base_point;
			base_point.x = INT32(_p->x + (_p->x - (_p + 1)->x) * float(400 - _p->y) / float(_p->y - (_p + 1)->y));
			base_point.y = 400;
			boundary.l_boundary[p_pre] = base_point;
			boundary.l_nums -= p_pre;//������Ч���ݵ����
			memmove(boundary.l_boundary, boundary.l_boundary + p_pre, sizeof(COOR2)*boundary.l_nums);
		}
		else//��λ�������һ���㣬�ñ����޷���ȡ�����õ�����
		{
			isValid_l = false;
		}
	}
	//1.2 ���߶��룬�ұ߽��߲���
	if (boundary.r_boundary[0].y > 400)//��һ�����Ѿ�������400������ߴ��ӳ�������һ����
	{
		COOR2 * _p = boundary.r_boundary;
		COOR2 base_point;
		unsigned int move_num = 0;//�����ƶ���
		//�жϵ�ǰ���ߵ��Ƿ��Ѿ���ռ��
		(boundary.r_nums < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE) ?
			//����δռ����ͳһ����ƶ�һ��λ�ã��ٲ���һ����ֵ���ܵ�����һ
			(move_num = boundary.r_nums++) :
			//������ռ����ֻ�ƶ�ǰ19�����ٲ�һ����ֵ���ܵ�������
			(move_num = boundary.r_nums-1);
		memmove(boundary.r_boundary + 1, boundary.r_boundary, sizeof(COOR2)*move_num);//Ų����
		//�����µõ��Ļ���
		base_point.x = INT32(_p->x + (_p->x - (_p + 1)->x) * float(400 - _p->y) / float(_p->y - (_p + 1)->y));
		base_point.y = 400;
		boundary.r_boundary[0] = base_point;
	}
	else if (boundary.r_boundary[0].y < 400)//��һ����û�г���400����������Ѱ�ҵ���Խ���ߵ�һ�Ե�
	{
		//��λ������ǰһ����
		int p_pre = 0;
		for (int cnt = 0; cnt != boundary.r_nums; ++cnt)
		{
			if (boundary.r_boundary[cnt].y < 400)
				p_pre = cnt;
			else
				break;
		}
		if (p_pre + 1 < boundary.r_nums)//��λ�㲻�����һ���㣬���޸ĸö�λ���ֵ��ʹ��Ϊ��λ�����һ������y=400�Ľ�ȡ��
		{
			COOR2 * _p = boundary.r_boundary + p_pre;
			COOR2 base_point;
			base_point.x = INT32(_p->x + (_p->x - (_p + 1)->x) * float(400 - _p->y) / float(_p->y - (_p + 1)->y));
			base_point.y = 400;
			boundary.r_boundary[p_pre] = base_point;
			boundary.r_nums -= p_pre;//������Ч�����ݵ����
			memmove(boundary.r_boundary, boundary.r_boundary + p_pre, sizeof(COOR2)*boundary.r_nums);
		}
		else//��λ�������һ���㣬�ñ����޷���ȡ�����õ�����
		{
			isValid_r = false;
		}
	}

	//�����޷���ȡ����Ч�����ݣ���Ŀ�������գ����ش���
	if (isValid_l && isValid_r)
	{
		//û����Ч�����ݣ������Ȼ���ߵ�����
		memset(boundary.l_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		boundary.l_nums = 0;
		memset(boundary.r_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);
		boundary.r_nums = 0;
		return RET_ERR;
	}

	//2 ���߶���
	//2.1 ���ҳ������ߵ����yֵ, �Խϳ����߲�ȡ��ȡ�ķ�����ʹ����y��������ֵ����һ�µ�
	int l_max_y = boundary.l_boundary[boundary.l_nums - 1].y;
	int	r_max_y = boundary.r_boundary[boundary.r_nums - 1].y;
	if (l_max_y > r_max_y)//��ȡ����ߣ�ʹ����ߵ����y���ұ���һ��
	{
		//��λ������ǰһ����
		int p_pre = 0;
		for (int cnt = 0; cnt != boundary.l_nums; ++cnt)
		{
			if (boundary.l_boundary[cnt].y < r_max_y)
				p_pre = cnt;
			else
				break;
		}
		//��ȡָ��yˮƽ�ϵĵ�
		COOR2 * _p = boundary.l_boundary + p_pre;
		COOR2 top_point;
		top_point.x = INT32(_p->x + (_p->x - (_p + 1)->x) * float(r_max_y - _p->y) / float(_p->y - (_p + 1)->y));
		top_point.y = r_max_y;
		memset(_p + 1, 0, sizeof(COOR2)*(NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - p_pre - 1));
		boundary.l_boundary[p_pre + 1] = top_point;
		boundary.l_nums = p_pre + 2;
	}
	else if (l_max_y < r_max_y)//��ȡ�ұ��ߣ�ʹ����ߵ����y���ұ���һ��
	{
		//��λ������ǰһ����
		int p_pre = 0;
		for (int cnt = 0; cnt != boundary.r_nums; ++cnt)
		{
			if (boundary.r_boundary[cnt].y < l_max_y)
				p_pre = cnt;
			else
				break;
		}
		//��ȡָ��yˮƽ�ϵĵ�
		COOR2 * _p = boundary.r_boundary + p_pre;
		COOR2 top_point;
		top_point.x = INT32(_p->x + (_p->x - (_p + 1)->x) * float(l_max_y - _p->y) / float(_p->y - (_p + 1)->y));
		top_point.y = l_max_y;
		memset(_p + 1, 0, sizeof(COOR2)*(NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - p_pre - 1));
		boundary.r_boundary[p_pre + 1] = top_point;
		boundary.r_nums = p_pre + 2;
	}
	return RET_OK;
}

KERNEL_RET ZGCC_FitNaturalEdge(NATURAL_BOUNDARY & boundary)
{
	//ASSERT!!!!!
	if (boundary.l_nums < 2 || boundary.r_nums < 2) return RET_ERR;

	static COOR2 edge[NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE];
	int edge_len = 0;
	float step_y = 0, yy = 0;//y�����Ĳ�����ʵ��ֵ

	//1 ���������
	memcpy(edge, boundary.l_boundary, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);//����߿���
	edge_len = boundary.l_nums;//��¼���ߵ�ĸ���
	step_y = (boundary.l_boundary[boundary.l_nums - 1].y - boundary.l_boundary[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);//����y����
	memset(boundary.l_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);//���ԭ�б�����Ϣ
	boundary.l_nums = 0;//���ԭ�б��ߵĵ����
	//��y�Ⱦ������ϲ�ֵ
	yy = (float)edge[0].y;
	for (int cnt = 0; cnt < edge_len - 1; ++cnt)
	{
		COOR2 & td1 = edge[cnt];
		COOR2 & td2 = edge[cnt + 1];
		//������֮����ˮƽ��ϵ��������������
		if (td1.y == td2.y) continue;
		//������֮��Ѱ��һ��y����Ҫ��ĵ�
		while (yy < td2.y && boundary.l_nums < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
		{
			boundary.l_boundary[boundary.l_nums].y = (INT32)yy;
			boundary.l_boundary[boundary.l_nums].x = INT32((float(td1.x - td2.x) / float(td1.y - td2.y) * (yy - td1.y)) + td1.x + 0.5);
			boundary.l_nums++;
			yy += step_y;
		}
		//���һ���㼴ʹ������y�ļ��Ҫ���ڱ���û����������£����Խ���ֱ�ӵ������һ����
		if (yy >= edge[edge_len - 1].y && boundary.l_nums < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
		{
			boundary.l_boundary[boundary.l_nums++] = edge[edge_len - 1];
			break;
		}
	}
	//�����ұ���
	memcpy(edge, boundary.r_boundary, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);//����߿���
	edge_len = boundary.r_nums;//��¼���ߵ�ĸ���
	step_y = (boundary.r_boundary[boundary.r_nums - 1].y - boundary.r_boundary[0].y) / (NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE - 1);//����y����
	memset(boundary.r_boundary, 0, sizeof(COOR2)*NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE);//���ԭ�б�����Ϣ
	boundary.r_nums = 0;//���ԭ�б��ߵĵ����
	//��y�Ⱦ������ϲ�ֵ
	yy = (float)edge[0].y;
	for (int cnt = 0; cnt < edge_len - 1; ++cnt)
	{
		COOR2 & td1 = edge[cnt];
		COOR2 & td2 = edge[cnt + 1];
		//������֮����ˮƽ��ϵ��������������
		if (td1.y == td2.y) continue;
		//������֮��Ѱ��һ��y����Ҫ��ĵ�
		while (yy < td2.y && boundary.r_nums < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
		{
			boundary.r_boundary[boundary.r_nums].y = (INT32)yy;
			boundary.r_boundary[boundary.r_nums].x = INT32((float(td1.x - td2.x) / float(td1.y - td2.y) * (yy - td1.y)) + td1.x + 0.5);
			boundary.r_nums++;
			yy += step_y;
		}
		//���һ���㼴ʹ������y�ļ��Ҫ���ڱ���û����������£����Խ���ֱ�ӵ������һ����
		if (yy >= edge[edge_len - 1].y && boundary.r_nums < NUM_OF_MAX_2D_POINTS_IN_EDGE_LINE)
		{
			boundary.r_boundary[boundary.r_nums++] = edge[edge_len - 1];
			break;
		}
	}
	return RET_OK;
}

KERNEL_RET ZGCC_FixNaturalEdge(NATURAL_BOUNDARY & boundary, const float * lidar64_map)
{
	static int raster_l[MAP_64_CELL_VER];//��Сָ��Ϊ64���״�դ��ĸ߶ȣ���֤��դ���㹻��
	static int raster_r[MAP_64_CELL_VER];//��Сָ��Ϊ64���״�դ��ĸ߶ȣ���֤��դ���㹻��
	memset(raster_l, 0, sizeof(int)*MAP_ROI_CELL_VER);
	memset(raster_r, 0, sizeof(int)*MAP_ROI_CELL_VER);
	const int base_line_offset = (2000 + 400) / 25;//��ʼդ��raster_l[0]�ڴ�դ���ͼ�е�����yƫ��
	const int base_line_y = 400;//դ����ʼΪ����λ�ã���λ����Լ���ĳ�ǰ4m
	const int raster_num = int((boundary.l_boundary[boundary.l_nums - 1].y - base_line_y) / MAP_ROI_CELL_SIZ);//��Ч��դ������

	//ASSERT!!!!! 
	if (raster_num < 1) return RET_ERR;

	//1  ���������߹�դ������¼����Ӧ�Ĺ�դλ����
	//1.1 ������ߵ����ݣ��������raster_l�ṹ
	int max_x_l= -2000;//Ĭ�ϲ�������-2000
	for (int cnt = 0; cnt != raster_num; ++cnt)
	{
		float y1 = base_line_y + cnt*25.0f;
		float y2 = y1 + 25.0f;;
		//ȷ����y1~y2������x,�����Ӧ�Ĺ�դ�ṹ��
		//��y1���߶εĽ��㣬��λĿ���λ��
		for (int idx = 0; idx != boundary.l_nums - 1; ++idx)
		{
			COOR2 & td1 = boundary.l_boundary[idx];
			COOR2 & td2 = boundary.l_boundary[idx + 1];
			//�ҵ�����y1�����ĵ�
			if (td1.y <= y1 && td2.y > y1)
				max_x_l = td1.x + (td1.x - td2.x) * (y1 - td1.y) / (td1.y - td2.y);
			else
				break;
		}
		//��y1��y2֮��ĵ㣬�����¶�λĿ���λ��
		for (int idx = 0; idx != boundary.l_nums; ++idx)
		{
			COOR2 & td = boundary.l_boundary[idx];
			if (td.y >= y1 && td.y < y2 && td.x > max_x_l)
				max_x_l = td.x;
		}
		//��y2���߶εĽ��㣬��λĿ���λ��
		for (int idx = 0; idx != boundary.l_nums - 1; ++idx)
		{
			COOR2 & td1 = boundary.l_boundary[idx];
			COOR2 & td2 = boundary.l_boundary[idx + 1];
			if (td1.y <= y2 && td2.y > y2)
			{
				float xx = td1.x + (td1.x - td2.x)*(y1 - td1.y) / (td1.y - td2.y);
				if (xx > max_x_l) max_x_l = xx;
			}
		}
		//����դ��λ���ϵ�xê����㲢��դ�������ʾ
		max_x_l += 2000.0f;//ת����դ������
		raster_l[cnt] = int((max_x_l + 2000.0f) / MAP_ROI_CELL_SIZ);
	}
	//1.2 �����ұߵ����ݣ��������raster_r�ṹ
	int max_x_r = 2000;//Ĭ�ϲ��ô���2000
	for (int cnt = 0; cnt != raster_num; ++cnt)
	{
		float y1 = base_line_y + cnt*25.0f;
		float y2 = y1 + 25.0f;;
		//ȷ����y1~y2������x,�����Ӧ�Ĺ�դ�ṹ��
		//��y1���߶εĽ��㣬��λĿ���λ��
		for (int idx = 0; idx != boundary.r_nums - 1; ++idx)
		{
			COOR2 & td1 = boundary.r_boundary[idx];
			COOR2 & td2 = boundary.r_boundary[idx + 1];
			//�ҵ�����y1�����ĵ�
			if (td1.y <= y1 && td2.y > y1)
				max_x_r = td1.x + (td1.x - td2.x) * (y1 - td1.y) / (td1.y - td2.y);
			else
				break;
		}
		//��y1��y2֮��ĵ㣬�����¶�λĿ���λ��
		for (int idx = 0; idx != boundary.r_nums; ++idx)
		{
			COOR2 & td = boundary.r_boundary[idx];
			if (td.y >= y1 && td.y < y2 && td.x < max_x_r)
				max_x_r = td.x;
		}
		//��y2���߶εĽ��㣬��λĿ���λ��
		for (int idx = 0; idx != boundary.r_nums - 1; ++idx)
		{
			COOR2 & td1 = boundary.r_boundary[idx];
			COOR2 & td2 = boundary.r_boundary[idx + 1];
			if (td1.y <= y2 && td2.y > y2)
			{
				float xx = td1.x + (td1.x - td2.x)*(y1 - td1.y) / (td1.y - td2.y);
				if (xx < max_x_r) max_x_r = xx;
			}
		}
		//����դ��λ���ϵ�xê����㲢��դ�������ʾ
		max_x_r += 2000.0f;//ת����դ������
		raster_r[cnt] = int((max_x_r + 2000.0f) / MAP_ROI_CELL_SIZ);
	}

	//2 ������դ���߽磬�������ҹ�߲���
	//2.1 ����߽��й�߲��������߾��ϰ�����ͺ󣩵ĺ������Ϊ0,1,2ʱ����ΪӦ�ñ����
	for (int cnt = 0; cnt != raster_num; ++cnt)
	{
		int cur = 1;
		while (cur < 4)
		{
			if (raster_l[cnt] + cur >= MAP_64_CELL_HOR) break;//��һ������Խ�磬ֹͣ����
			int lidar_pos = (base_line_offset + cnt)*MAP_64_CELL_HOR + raster_l[cnt];
			//�̳���һ�β���
			if (lidar64_map[lidar_pos + cur] > 0.0f)
			{
				raster_l[cnt] += cur;//���������ϰ�����£������ò����α�
				cur = 1;
			}
			else
			{
				cur++;//����û�����У������������α�
			}
		}
	}
	//2.2 �ұ��߽��й�߲����������ϰ�����ͺ󣩵ĺ������Ϊ0,1,2ʱ����ΪӦ�ñ����
	for (int cnt = 0; cnt != raster_num; ++cnt)
	{
		int cur = 1;
		while (cur < 4)
		{
			if (raster_r[cnt] - cur < 0) break;//��һ������Խ�磬ֹͣ����
			int lidar_pos = (base_line_offset + cnt)*MAP_64_CELL_HOR + raster_r[cnt];
			//�̳���һ�β���
			if (lidar64_map[lidar_pos - cur] > 0.0f)
			{
				raster_l[cnt] -= cur;//���������ϰ�����£������ò����α�
				cur = 1;
			}
			else
			{
				cur++;//����û�����У������������α�
			}
		}
	}

	//3 �Թ�դ�����߽����˲��Ż���ʹ��⻬
	static int cov_k[25] = {-7, -7, -6, -5, -4, -3, -2, -2, -1, -1, 0, 0, 0, -1, -1, -2, -2. -3, -4, -5, -6, -7, -7};
	
	return RET_OK;
}