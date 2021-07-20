#include "pch.h"
#include "MoNode.h"



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



bool CMoNode::MoCNN_Initialize(TM_TYPE tm_type, Real p2c_w[3][3])
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
				ok = InitJointPair(pair, bodyPair, tm_type, p2c_w);
			}
		}
	}
	return ok;
}



bool CMoTree::Connect_cross(CMoNode* from, CMoNode* to, CNN cnn_type, const wchar_t* pairs[][2], int n_pairs, Real p2c_w[3][3])
{
	Tree<CMoNode>::Connect(from, to, cnn_type);
	bool connected = false;
	if (n_pairs > 0)
	{
		auto GetBodyNodeName = [](CArtiBodyNode* node) -> const wchar_t*
							{
								return node->GetName_w();
							};
		connected = to->MoCNN_Initialize(CMoNode::cross, pairs, n_pairs, GetBodyNodeName, p2c_w);
	}
	else
		connected = to->MoCNN_Initialize(CMoNode::cross, p2c_w);
	return connected;
}

bool CMoTree::Connect_cross(CMoNode* from, CMoNode* to, CNN cnn_type, const char* pairs[][2], int n_pairs, Real p2c_w[3][3])
{
	Tree<CMoNode>::Connect(from, to, cnn_type);
	bool connected = false;
	if (n_pairs > 0)
	{
		auto GetBodyNodeName = [](CArtiBodyNode* node) -> const char*
						{
							return node->GetName_c();
						};
		connected = to->MoCNN_Initialize(CMoNode::cross, pairs, n_pairs, GetBodyNodeName, p2c_w);
	}
	else
		connected = to->MoCNN_Initialize(CMoNode::cross, p2c_w);
	return connected;
}

bool CMoTree::Connect_homo(CMoNode* from, CMoNode* to, CNN cnn_type)
{
	Tree<CMoNode>::Connect(from, to, cnn_type);
	bool connected = to->MoCNN_Initialize(CMoNode::homo, NULL);
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

void CMoTree::Destroy(CMoNode* root)
{
	auto onEnterNode = [] (CMoNode* n_this)
					{

					};
	auto onLeaveNode = [] (CMoNode* n_this)
					{
						delete n_this;
					};
	TraverseDFS_botree_nonrecur(root, onEnterNode, onLeaveNode);
}