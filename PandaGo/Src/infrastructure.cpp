#include "infrastructure.h"

DataPool::DataPool()
{
    status = DataPoolEmpty;
    cur = 0;
    end = 0;
    sysFlow = nullptr;
    name = "nullptr";
	path = "nullptr";
}

DataPool::~DataPool()
{
    DataPool::cur = 0;
    DataPool::end = 0;
    delete[] DataPool::sysFlow;
    DataPool::sysFlow = nullptr;
    DataPool::status = DataPoolEmpty;
    DataPool::name = "nullptr";
	DataPool::path = "nullptr";
}

void DataPool::clear()
{
    DataPool::cur = 0;
    DataPool::end = 0;
    delete[] DataPool::sysFlow;
    DataPool::sysFlow = nullptr;
    DataPool::status = DataPoolEmpty;
    DataPool::name = "nullptr";
	DataPool::path = "nullptr";
}

void DataPool::reset()
{
    DataPool::cur = 0;
    DataPool::status = DataPoolValid;
}
