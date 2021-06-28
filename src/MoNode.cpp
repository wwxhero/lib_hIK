#include "pch.h"
#include "MoNode.h"
#include "leak_check_crt.h"

enum TM_TYPE { homo = 0, cross, identity };

const char* CMoNode::TM_TYPE_STR[] = {"homo", "cross", "identity"};

CMoNode::CMoNode(CArtiBodyNode* body)
	: m_tmType(identity)
	, m_hostee(body)
{
}

CMoNode::~CMoNode()
{
	for (auto jointPair : m_jointPairs)
	{
		UnInitJointPair(jointPair);
		delete jointPair;
	}
}



bool CMoNode::MoCNN_Initialize(TM_TYPE tm_type)
{
	m_tmType = tm_type;
	assert(NULL != m_parent
		&& NULL != m_parent->m_hostee); //root motion node is not supposed to run this function
	const auto& bodies_from = m_parent->m_hostee->m_kinalst;
	const auto& bodies_to = m_hostee->m_kinalst;
	std::size_t n_pairs = bodies_from.size();
	bool ok = (n_pairs == bodies_to.size());
	if (ok)
	{
		m_jointPairs.resize(n_pairs, NULL);
		std::size_t i_pair = 0;
		auto it_body_from = bodies_from.begin();
		auto it_body_to = bodies_to.begin();
		for(
			;    ok
			  && it_body_from != bodies_from.end()
			  && it_body_to != bodies_to.end()
			; it_body_from ++
			  , it_body_to ++
			)
		{
			std::string name_from((*it_body_from)->GetName_c());
			std::string name_to((*it_body_to)->GetName_c());
			ok = (name_from == name_to);
			if (ok)
			{
				const CArtiBodyNode* bodyPair[] = {*it_body_from, *it_body_to};
				JointPair* pair = (m_jointPairs[i_pair ++] = new JointPair);
				ok = InitJointPair(pair, bodyPair, tm_type);
			}
		}
	}
	return ok;
}



bool CMoNode::MoCNN_Initialize(TM_TYPE tm_type, const char* name_pairs[][2], int n_pairs)
{
	assert(n_pairs > 0);
	m_tmType = tm_type;
	std::map<std::string, const CArtiBodyNode*>	bodies_from;
	std::map<std::string, CArtiBodyNode*>		bodies_to;
	assert(NULL != m_parent
		&& NULL != m_parent->m_hostee); //root motion node is not supposed to run this function
	CArtiBodyNode* body_from = (m_parent->m_hostee);
	for (CArtiBodyNode* kina : body_from->m_kinalst)
		bodies_from[kina->GetName_c()] = kina;

	CArtiBodyNode* body_to = m_hostee;
	for (CArtiBodyNode* kina : body_to->m_kinalst)
		bodies_to[kina->GetName_c()] = kina;

	bool ok = true;
	m_jointPairs.resize(n_pairs, NULL);
	for (int i_pair = 0
		; i_pair < n_pairs && ok
		; i_pair ++)
	{
		std::string name_from(name_pairs[i_pair][0]);
		std::string name_to(name_pairs[i_pair][1]);

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
			ok = InitJointPair(pair, artiPair, tm_type);
		}
	}
	assert(ok && "the given pair should be consistant with the given arti body");
	return ok;
}

bool CMoTree::Connect(CMoNode* parent, CMoNode* child, CNN cnn_type, CMoNode::TM_TYPE tm_type, const char* pairs[][2], int n_pairs)
{
	Tree<CMoNode>::Connect(parent, child, cnn_type);
	bool connected = false;
	if (n_pairs > 0)
		connected = child->MoCNN_Initialize(tm_type, pairs, n_pairs);
	else
		connected = child->MoCNN_Initialize(tm_type);
	return connected;
}

bool CMoTree::Connect(CMoNode* parent, CMoNode* child, CNN cnn_type, CMoNode::TM_TYPE tm_type)
{
	Tree<CMoNode>::Connect(parent, child, cnn_type);
	bool connected = child->MoCNN_Initialize(tm_type);
	return connected;
}

void CMoTree::Motion_sync(CMoNode* root)
{
	auto onEnterNode = [] (CMoNode* n_this)
					{
						for (CMoNode* n_child = n_this->GetFirstChild()
							; NULL != n_child
							; n_child = n_child->GetNextSibling())
						{
							n_child->Motion_sync();
						}

					};
	auto onLeaveNode = [] (CMoNode* n_this)
					{

					};
	TraverseDFS_botree_nonrecur(root, onEnterNode, onLeaveNode);
}