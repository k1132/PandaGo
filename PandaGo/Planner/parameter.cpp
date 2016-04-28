#include "plsdk.h"
#include "tinyxml2.h"

namespace plsdk
{
	using namespace std;
	using namespace tinyxml2;

	Parameter & Parameter::GetInstance()
	{
		static Parameter inst;
		return inst;
	}

	Parameter::Parameter()
	{
		XMLDocument p;
		p.LoadFile(CFG_FILE);
		XMLElement * root = p.RootElement();
		XMLElement * ele(nullptr);

		ele = root->FirstChildElement("speed_max");
		speed_max = ele->FloatAttribute("val");
		ele = root->FirstChildElement("speed_gis_lim");
		speed_gis_lim = ele->FloatAttribute("val");
		ele = root->FirstChildElement("planner");
		planner = ele->IntAttribute("val");
		ele = root->FirstChildElement("mode");
		mode = ele->IntAttribute("val");
	}

	Parameter::~Parameter()
	{

	}
}