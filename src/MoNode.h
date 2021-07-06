#pragma once
#include <vector>
#include <map>
#include "ArtiBody.h"
#include "ik_logger.h"
typedef CArtiBodyNode CJoint; //CJoint is yet to be developed

class CMoNode : public TreeNode<CMoNode>
{
public:
	enum TM_TYPE { homo = 0, cross, identity };
	static const char* TM_TYPE_STR[];
private:
	struct JointPair
	{
		CJoint* j_from;
		CJoint* j_to;
		CTransform from2to;
		CTransform to2from;
	};

	inline bool InitJointPair(JointPair* pair, const CArtiBodyNode* artiPair[2], TM_TYPE tm_type, Real mf2t_w[3][4])
	{
		// static Real s_f2t[3][4] = {
		// 	{1,	0,	0,	0},
		// 	{0,	0,	1,	0},
		// 	{0,	1,	0,	0},
		// };
		CTransform f2t_w = (NULL == mf2t_w
							? CTransform()
							: CTransform(mf2t_w));
		// CTransform s = CTransform::Scale(6);
		// f2t_w = f2t_w * s;

		pair->j_from = (CJoint *)artiPair[0]; // todo: need a real joint
		pair->j_to = (CJoint *)artiPair[1];
		bool has_parent_0 = (NULL != artiPair[0]->GetParent());
		bool has_parent_1 = (NULL != artiPair[1]->GetParent());
		bool has_parent = (has_parent_0 || has_parent_1);
		bool ok = true; // (has_parent_0 == has_parent_1);

		if (ok)
		{
			if (cross == tm_type)
			{
				CTransform from2world, world2to;
				artiPair[0]->GetTransformLocal2World(from2world);
				artiPair[1]->GetTransformWorld2Local(world2to);
				pair->from2to = world2to * f2t_w * from2world;
				// if (has_parent)
				if (has_parent)
					pair->from2to.SetTT(0, 0, 0);
				pair->to2from = pair->from2to.inverse();
			}
			else if(homo == tm_type)
			{
				CTransform bound_from, bound_to_inv;
				artiPair[0]->GetTransformLocal2Parent(bound_from);
				artiPair[1]->GetTransformParent2Local(bound_to_inv);
				pair->from2to = bound_to_inv * bound_from;
				// pair->to2from = pair->from2to.inverse(); to2from is not used for homo transformation
				if (has_parent_1)
					ok = !pair->from2to.HasTT();
				else
					pair->from2to.SetTT(0, 0, 0);
			}
		}

		// homo == tm_type -> !pair->from2to.HasTT()
		bool homo_has_no_tt = (homo != tm_type || !pair->from2to.HasTT());
		assert(homo_has_no_tt);

		// (cross == tm_type && has_parent_1) -> !pair->from2to.HasTT()
		bool cross_had_no_tt = (!(cross == tm_type && has_parent) || !pair->from2to.HasTT());
		assert(cross_had_no_tt);

		std::stringstream logInfo;
		logInfo << "Binding:" << TM_TYPE_STR[m_tmType] << "\t"
				<< pair->j_from->GetName_c() << " => " << pair->j_to->GetName_c() << ":"
				<< pair->from2to.ToString().c_str();
		LOGIK(logInfo.str().c_str());
		return ok;
	}
	inline void UnInitJointPair(JointPair* pair)
	{
		pair->j_from = NULL;
		pair->j_to = NULL;
	}
public:
	CMoNode(CArtiBodyNode* body);
	~CMoNode();


	bool MoCNN_Initialize(TM_TYPE tm_type, Real p2c_w[3][4]);
	template<typename CHAR, typename LAMBDA_name>
	bool MoCNN_Initialize(TM_TYPE tm_type, const CHAR* name_pairs[][2], int n_pairs, LAMBDA_name GetName, Real f2t_w[3][4])
	{
		assert(n_pairs > 0);
		m_tmType = tm_type;
		typedef std::basic_string<CHAR> STR;
		std::map<STR, const CArtiBodyNode*>	bodies_from;
		std::map<STR, CArtiBodyNode*>		bodies_to;
		assert(NULL != m_parent
			&& NULL != m_parent->m_hostee); //root motion node is not supposed to run this function
		CArtiBodyNode* body_from = (m_parent->m_hostee);
		for (CArtiBodyNode* kina : body_from->m_kinalst)
			bodies_from[GetName(kina)] = kina;

		CArtiBodyNode* body_to = m_hostee;
		for (CArtiBodyNode* kina : body_to->m_kinalst)
			bodies_to[GetName(kina)] = kina;

		bool ok = true;
		m_jointPairs.resize(n_pairs, NULL);
		int i_pair = 0;
		for (
			; i_pair < n_pairs && ok
			; i_pair ++)
		{
			STR name_from(name_pairs[i_pair][0]);
			STR name_to(name_pairs[i_pair][1]);

			auto it_j_from = bodies_from.find(name_from);
			auto it_j_to = bodies_to.find(name_to);

			ok = (bodies_from.end() != it_j_from
				&& bodies_to.end() != it_j_to);

			if (ok)
			{
				JointPair* pair = (m_jointPairs[i_pair] = new JointPair);
				const CArtiBodyNode* artiPair[] = {
					it_j_from->second,
					it_j_to->second
				};
				ok = InitJointPair(pair, artiPair, tm_type, f2t_w);
			}
		}
		if (!ok)
		{
			int n_pair_est = i_pair-1;
			for (int i_pair = 0; i_pair < n_pair_est; i_pair ++)
			{
				UnInitJointPair(m_jointPairs[i_pair]);
			}
			m_jointPairs.clear();
		}
		assert(ok && "the given pair should be consistant with the given arti body");
		return ok;
	}

	void Motion_sync()
	{
		for (JointPair* pair : m_jointPairs)
		{
			CJoint* j_from = pair->j_from;
			CJoint* j_to = pair->j_to;
			CTransform delta_from;
			j_from->GetJointTransform(delta_from);
			CTransform delta_to;
			switch (m_tmType)
			{
			 	case cross:
			 		delta_to = pair->from2to * delta_from * pair->to2from;
			 		break;
			 	case homo:
			 		delta_to = pair->from2to * delta_from;
			 		break;
			}
			j_to->SetJointTransform(delta_to);

			std::stringstream logInfo;
			logInfo << "\n\t" << TM_TYPE_STR[m_tmType] << j_from->GetName_c() <<":" << delta_from.ToString().c_str() << "\n"
					<<   "\t" << TM_TYPE_STR[m_tmType] << j_to->GetName_c() << ":" <<  delta_to.ToString().c_str() << "\n";
			LOGIK(logInfo.str().c_str());
		}
		CArtiBodyTree::FK_Update(m_hostee);
	}
private:
	std::vector<JointPair*> m_jointPairs;
	TM_TYPE m_tmType;
	CArtiBodyNode* m_hostee;

};

class CMoTree : public Tree<CMoNode>
{
public:
	static bool Connect_cross(CMoNode* from, CMoNode* to, CNN cnn_type, const char* pairs[][2], int n_pairs, Real p2c_w[3][4]);
	static bool Connect_cross(CMoNode* from, CMoNode* to, CNN cnn_type, const wchar_t* pairs[][2], int n_pairs, Real p2c_w[3][4]);
	static bool Connect_homo(CMoNode* from, CMoNode* to, CNN cnn_type);
	static void Motion_sync(CMoNode* root);
};