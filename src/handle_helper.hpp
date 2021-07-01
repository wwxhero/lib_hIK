#include "pch.h"

template <typename F, typename T>
T* CAST_2P(F h)
{
	return reinterpret_cast<T*>(h.p);
}

template <typename F, typename T>
T CAST_2H(F* p)
{
	return {p};
}

#define VALID_HANDLE(h)\
	(h.p != NULL)

#define CAST_2PBVH(hBVH)\
	CAST_2P<HBVH, bvh11::BvhObject>(hBVH)

#define CAST_2HBVH(pBVH)\
	CAST_2H<bvh11::BvhObject, HBVH>(pBVH)

#define CAST_2PBODY(hBody)\
	CAST_2P<HBODY, CArtiBodyNode>(hBody)

#define CAST_2HBODY(pBody)\
	CAST_2H<CArtiBodyNode, HBODY>(pBody)

#define CAST_2HMONODE(pMoNode)\
	CAST_2H<CMoNode, HMOTIONNODE>(pMoNode)

#define CAST_2PMONODE(hMoNode)\
	CAST_2P<HMOTIONNODE, CMoNode>(hMoNode)

#define CAST_2HBVH(pBVH)\
	CAST_2H<bvh11::BvhObject, HBVH>(pBVH)

#define CAST_2PBVH(hBVH)\
	CAST_2P<HBVH, bvh11::BvhObject>(hBVH)