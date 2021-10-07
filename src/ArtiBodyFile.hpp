#include "bvh11_helper.hpp"
#include <queue>
#include "loggerfast.h"
#include "ik_logger.h"
class CArtiBodyFile : public bvh11::BvhObject
{
public:
	CArtiBodyFile(const CArtiBodyNode* root_src, int n_frames);
	void UpdateMotion(int i_frame);

	template<typename STREAM>
	static void OutputHeader(CArtiBodyFile& bf, STREAM &logger)
	{
		//to be done
		logger << "HIERARCHY" << "\n";
		bf.WriteJointSubHierarchy<STREAM>(logger, bf.root_joint_, 0);
		// Motion
		logger << "MOTION" << "\n";
		logger << "Frames: " << bf.frames_ << "\n";
		logger << "Frame Time: " << bf.frame_time_ << "\n";
	}

	template<typename STREAM>
	static void OutputMotion(CArtiBodyFile& bf, int i_frame, STREAM& logger)
	{
		//to be done
		auto n_rows = bf.motion_.rows();
		auto n_cols = bf.channels_.size();
		auto n_stride = n_rows; // eigen is a column major matrix storage
		IKAssert(i_frame < n_rows);
		const double* p_data_start = bf.motion_.row(i_frame).data();
		const double* p_data_end = p_data_start + n_cols * n_stride;
		const double* p_data = p_data_start;
		logger << *p_data;
		for (p_data += n_stride; p_data < p_data_end; p_data += n_stride)
		  	logger << " " << *p_data;
		logger << "\n";
	}

private:
	void SetJointChannel(std::shared_ptr<bvh11::Joint> joint);
	typedef std::shared_ptr<const bvh11::Joint> Joint_bvh_ptr;
	typedef std::pair<Joint_bvh_ptr, const CArtiBodyNode*> Bound;

	template<typename LAMaccessEnter, typename LAMaccessLeave>
	inline void TraverseBFS_boundtree_norecur(Bound root, LAMaccessEnter OnEnterBound, LAMaccessLeave OnLeaveBound)
	{
		std::queue<Bound> queBFS;
		queBFS.push(root);
		OnEnterBound(root);
		while (!queBFS.empty())
		{
			auto b_this = queBFS.front();
			auto joint_bvh = b_this.first;
			const CArtiBodyNode* body_hik = b_this.second;
			auto& children_bvh = joint_bvh->children();
			auto it_bvh_child = children_bvh.begin();
			auto body_child = body_hik->GetFirstChild();
			for (
				; it_bvh_child != children_bvh.end()
				&& nullptr != body_child
				; it_bvh_child++,
				body_child = body_child->GetNextSibling())
			{
				auto b_child = std::make_pair(*it_bvh_child, body_child);
				queBFS.push(b_child);
				OnEnterBound(b_child);
			}
			queBFS.pop();
			OnLeaveBound(b_this);
		}
	}
private:
	const CArtiBodyNode* m_bodyRoot;
};


class CBodyLogger
{
public:
	CBodyLogger(const CArtiBodyNode* root, const char* path) throw(...);
	~CBodyLogger();
	void LogHeader();
	void LogMotion();
private:
	CArtiBodyFile m_bodyFile;
	LoggerFast m_logger;
};