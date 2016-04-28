#include "moduleplsimulator.h"


ModulePlSimulator::ModulePlSimulator()
{
	init();
}

ModulePlSimulator::~ModulePlSimulator()
{

}

void ModulePlSimulator::init()
{
	return;
}

void ModulePlSimulator::call(PLContext * _data, int _cur)
{
	//调用数据转换和分析程序
	PLContext * data = _data + _cur;

	UGV_PL::Process_Plan(&data->pl_func_input, &data->pl_cs_data_sim, &data->pl_prm_data, &data->pl_local_data);

	return;
}

void ModulePlSimulator::quit()
{

}

