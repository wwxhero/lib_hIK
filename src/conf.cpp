#include "pch.h"
#include "conf.h"
#include "tinyxml.h"
#include "handle_helper.hpp"

HCONF load_conf(const wchar_t* path_src)
{
	std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
	std::string path_c = converter.to_bytes(path_src);
	TiXmlDocument* doc = new TiXmlDocument(path_c);
	bool loadOkay = doc->LoadFile();
	if (loadOkay)
		return CAST_2HCONF(doc);
	else
	{
		delete doc;
		return H_INVALID;
	}
}

void unload_conf(HCONF conf)
{
	if (VALID_HANDLE(conf))
	{
		TiXmlDocument* doc = CAST_2PCONF(conf);
		delete doc;
	}
}
