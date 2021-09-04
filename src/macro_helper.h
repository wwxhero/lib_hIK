#pragma once

typedef struct
{
	short value;
	const char* text;
} FlagText;

typedef FlagText EnumText;

#define DECLARE_ENUM_STR(type)\
		static type to_##type(const char* str);\
		static const char* from_##type(type);\
		static EnumText s_##type##_val_str[];


#define BEGIN_ENUM_STR(host, type)\
	EnumText host##::s_##type##_val_str[] = {

#define ENUM_ITEM(entry)\
	{entry, #entry},

#define END_ENUM_STR(host, type)\
	};\
	host##::##type host##::to_##type(const char* str)\
	{\
		int n = sizeof(s_##type##_val_str)/sizeof(EnumText);\
		for (int i = 0; i < n; i ++)\
		{\
			if (0 == strcmp(str, s_##type##_val_str[i].text))\
				return (host##::##type)s_##type##_val_str[i].value;\
		}\
		return (host##::##type)s_##type##_val_str[n-1].value;\
	}\
	const char* host##::from_##type(host##::##type algor)\
	{\
		for (int i = 0; i < sizeof(s_##type##_val_str)/sizeof(EnumText); i ++)\
		{\
			if (algor == s_##type##_val_str[i].value)\
				return s_##type##_val_str[i].text;\
		}\
		return NULL;\
	}
