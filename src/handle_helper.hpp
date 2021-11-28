#include "pch.h"

template <typename F, typename T>
T* CAST_2P(F h)
{
#ifdef _DEBUG
	return reinterpret_cast<T*>(h.p);
#else
	return reinterpret_cast<T*>(h);
#endif
}

template <typename F, typename T>
T CAST_2H(F* p)
{
#ifdef _DEBUG
	return {p};
#else
	return p;
#endif
}

#define CAST_2PBVH(hBVH)\
	CAST_2P<HBVH, CArtiBodyFile>(hBVH)

#define CAST_2HBVH(pBVH)\
	CAST_2H<CArtiBodyFile, HBVH>(pBVH)

#define CAST_2PBODY(hBody)\
	CAST_2P<HBODY, CArtiBodyNode>(hBody)

#define CAST_2HBODY(pBody)\
	CAST_2H<CArtiBodyNode, HBODY>(pBody)

#define CAST_2HMONODE(pMoNode)\
	CAST_2H<CMoNode, HMOTIONNODE>(pMoNode)

#define CAST_2PMONODE(hMoNode)\
	CAST_2P<HMOTIONNODE, CMoNode>(hMoNode)

#define CAST_2HCONF(pDOC)\
	CAST_2H<TiXmlDocument, HCONF>(pDOC)

#define CAST_2PCONF(hConf)\
	CAST_2P<HCONF, TiXmlDocument>(hConf)

#define CAST_2HCONFFKRC(pConfFKRC)\
	CAST_2H<CConfMoPipe, HCONFMOPIPE>(pConfFKRC)

#define CAST_2PCONFFKRC(hConfFKRC)\
	CAST_2P<HCONFMOPIPE, CConfMoPipe>(hConfFKRC)

#define CAST_2PPG(hPG)\
	CAST_2P<HPG, CPG>(hPG)

#define CAST_2HPG(pPG)\
	CAST_2H<CPG, HPG>(pPG)