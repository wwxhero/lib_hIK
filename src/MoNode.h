#pragma once
#include <vector>
#include <map>
#include "ArtiBody.h"
#include "ik_logger.h"
#include "matrix3.h"

class CMoNode : public TreeNode<CMoNode>
{
public:
	enum TM_TYPE { homo = 0, cross, unknown };
	static const char* TM_TYPE_STR[];
	static TM_TYPE to_TM_TYPE(const char* type_str);
private:
	struct JointPair
	{
		const IJoint* j_from;
		IJoint* j_to;
		Matrix3 from2to;
		Matrix3 to2from;
	};

	inline bool InitJointPair(JointPair* pair, const CArtiBodyNode* artiPair[2], TM_TYPE tm_type, const Real mf2t_w[3][3])
	{
		// static Real s_f2t[3][3] = {
		// 	{1,	0,	0,	0},
		// 	{0,	0,	1,	0},
		// 	{0,	1,	0,	0},
		// };
		// Transform_TRS s = Transform_TRS::Scale(6);
		// f2t_w = f2t_w * s;

		Matrix3 f2t_w;
		if (NULL == mf2t_w)
			f2t_w.setIdentity();
		else
			f2t_w.setData(mf2t_w);

		pair->j_from = artiPair[0]->GetJoint();
		pair->j_to = const_cast<CArtiBodyNode*>(artiPair[1])->GetJoint();
		bool has_parent_0 = (NULL != artiPair[0]->GetParent());
		bool has_parent_1 = (NULL != artiPair[1]->GetParent());
		bool has_parent = (has_parent_0 || has_parent_1);
		bool ok = true; // (has_parent_0 == has_parent_1);

		if (ok)
		{
			if (cross == tm_type)
			{
				const Transform* from2world = artiPair[0]->GetTransformLocal2World();
				const Transform* world2to = artiPair[1]->GetTransformWorld2Local();
				pair->from2to = world2to->getLinear() * f2t_w * from2world->getLinear();
				pair->to2from = pair->from2to.inverse();
			}
			else if(homo == tm_type)
			{
				const Transform* bound_from = artiPair[0]->GetTransformLocal2Parent0();
				const Transform* bound_to_inv = artiPair[1]->GetTransformParent2Local0();
				pair->from2to = bound_to_inv->getLinear() * bound_from->getLinear();
				// pair->to2from = pair->from2to.inverse(); to2from is not used for homo transformation
			}
		}

		// homo == tm_type -> !pair->from2to.HasTT()
		// (cross == tm_type && has_parent_1) -> !pair->from2to.HasTT()

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


	bool MoCNN_Initialize(TM_TYPE tm_type, const Real p2c_w[3][3]);
	template<typename CHAR, typename LAMBDA_name>
	bool MoCNN_Initialize(TM_TYPE tm_type, const CHAR* name_pairs[][2], int n_pairs, LAMBDA_name GetName, const Real f2t_w[3][3])
	{
		assert(n_pairs > 0);
		m_tmType = tm_type;
		typedef std::basic_string<CHAR> STR;
		std::map<STR, const CArtiBodyNode*>	bodies_from;
		std::map<STR, CArtiBodyNode*>		bodies_to;

		CMoNode* parent = GetParent();
		assert(NULL != parent
			&& NULL != parent->m_hostee); //root motion node is not supposed to run this function
		CArtiBodyNode* body_from = (parent->m_hostee);
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
			else
			{
				std::stringstream ss;
				ss << "can not find pair: [" << name_from.c_str() << ", " << name_to.c_str() << "]";
				LOGIK(ss.str().c_str());
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
			const IJoint* j_from = pair->j_from;
			IJoint* j_to = pair->j_to;
			const Transform* delta_from = j_from->GetTransform();
			Eigen::Matrix3r linear;
			Eigen::Vector3r tt;
			switch (m_tmType)
			{
				case cross:
				{
					linear =  pair->from2to
							* delta_from->getLinear()
							* pair->to2from;
					tt = pair->from2to * delta_from->getTranslation();
					break;
				}
				case homo:
					// delta_to = pair->from2to * delta_from;
					linear = pair->from2to * delta_from->getLinear();
					tt = pair->from2to * delta_from->getTranslation();
					break;
			}
			j_to->SetLinear(linear);
			j_to->SetTranslation(tt);

			std::stringstream logInfo;
			logInfo << "\n\t" << TM_TYPE_STR[m_tmType] << ":" << j_from->GetName_c() << ":" << delta_from->ToString().c_str()
				<< "\n\t" << pair->from2to.ToString().c_str()
				<< "\n\t" << TM_TYPE_STR[m_tmType] << ":" << j_to->GetName_c() << ":" << j_to->GetTransform()->ToString().c_str()
				<< "\n\t" << "Local2world: " << j_to->GetTransformLocal2World()->ToString().c_str();

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
	static bool Connect_cross(CMoNode* from, CMoNode* to, CNN cnn_type, const char* pairs[][2], int n_pairs, const Real p2c_w[3][3]);
	static bool Connect_cross(CMoNode* from, CMoNode* to, CNN cnn_type, const wchar_t* pairs[][2], int n_pairs, const Real p2c_w[3][3]);
	static bool Connect_homo(CMoNode* from, CMoNode* to, CNN cnn_type);
	static void Motion_sync(CMoNode* root);
	static void Destroy(CMoNode* root);
};