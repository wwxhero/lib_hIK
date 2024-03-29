#ifndef BVH11_HELPER_HPP_
#define BVH11_HELPER_HPP_
#include "pch.h"
#include <vector>
#include <list>
#include <string>
#include <memory>
#include <iostream>
#include <stdlib.h>
#include "Math.hpp"
#include "macro_helper.h"

class CArtiBodyNode;

namespace bvh11
{
	struct Channel;
	class Joint
	{
	public:
		Joint(const std::string& name, std::shared_ptr<Joint> parent) : name_(name), parent_(parent)
		{

		}
		virtual ~Joint()
		{


		}

		const Eigen::Vector3d& offset() const { return offset_; }
		Eigen::Vector3d& offset() { return offset_; }

		bool has_end_site() const { return has_end_site_; }
		bool& has_end_site() { return has_end_site_; }

		const Eigen::Vector3d& end_site() const { return end_site_; }
		Eigen::Vector3d& end_site() { return end_site_; }

		const std::string& name() const { return name_; }

		const std::list<std::shared_ptr<Joint>>& children() const { return children_; }
		const std::list<int>& associated_channels_indices() const { return associated_channels_indices_; }

		std::shared_ptr<Joint> parent() const { return parent_.lock(); }

		void AddChild(std::shared_ptr<Joint> child) { children_.push_back(child); }
		void AssociateChannel(int channel_index) { associated_channels_indices_.push_back(channel_index); }

	private:
		const std::string            name_;
		const std::weak_ptr<Joint> parent_;

		bool                              has_end_site_ = false;
		Eigen::Vector3d                   end_site_;
		Eigen::Vector3d                   offset_;
		std::list<std::shared_ptr<Joint>> children_;
		std::list<int>                    associated_channels_indices_;
	};

	class BvhObject
	{
	public:
		/// \param file_path Path to the input BVH file.
		BvhObject(const std::string& file_path, const double scale = 1.0) throw(...)
		{
			ReadBvhFile(file_path, scale);
		}

		BvhObject() {} // constructor for derived class

		BvhObject(const BvhObject& src) throw(...);

		virtual ~BvhObject()
		{

		}

		void Dump();
		int    frames()     const { return frames_;     }
		double frame_time() const { return frame_time_; }

		const std::vector<Channel>& channels() const { return channels_; }
		const Eigen::MatrixXd&      motion()   const { return motion_;   }

		std::shared_ptr<const Joint> root_joint() const { return root_joint_; }

		/// \brief Return a list of all the joints.
		/// \return List of the joints sorted always in the same order.
		std::vector<std::shared_ptr<const Joint>> GetJointList() const;

		/// \param frame Frame. This value must be between 0 and frames() - 1.
		Eigen::Affine3d GetTransformationRelativeToParent(std::shared_ptr<const Joint> joint, int frame) const;

		/// \param frame Frame. This value must be between 0 and frames() - 1.
		Eigen::Affine3d GetTransformation(std::shared_ptr<const Joint> joint, int frame) const;

		Eigen::Affine3d GetLocalDeltaTM(std::shared_ptr<const Joint> joint, int frame) const;

		/// \param frame Frame. This value must be between 0 and frames() - 1.
		Eigen::Affine3d GetRootTransformation(int frame) const
		{
			return GetTransformation(root_joint_, frame);
		}

		void UpdateMotion(std::shared_ptr<const Joint> joint, const Eigen::Affine3d& tm_l, int i_frame);

		void PrintJointHierarchy() const { PrintJointSubHierarchy(root_joint_, 0); }

		/// \param file_path Path to the output BVH file.
		void WriteBvhFile(const std::string& file_path) const;

		/// \brief Change the number of the frames.
		/// \details When the specified number is larger than the current number, it adds new frames at the end
		///          that are uninitialized. When the specified number is smaller than the current frame,
		///          it deletes the last frames. When the specified number is the same as the current number,
		///          it does not change anything.
		/// \param num_new_frames Number of the new frames.
		void ResizeFrames(int num_new_frames);

	protected:
		int                  frames_;
		double               frame_time_;

		std::vector<Channel> channels_;
		Eigen::MatrixXd      motion_;

		std::shared_ptr<const Joint> root_joint_;

		void ReadBvhFile(const std::string& file_path, const double scale = 1.0) throw(...);

		void PrintJointSubHierarchy(std::shared_ptr<const Joint> joint, int depth) const;

		template<typename STREAM>
		void WriteJointSubHierarchy(STREAM& ofs, std::shared_ptr<const Joint> joint, int depth) const
		{
			auto indent_creation = [](int depth) -> std::string
			{
				std::string tabs = "";
				for (int i = 0; i < depth; ++ i) { tabs += "\t"; }
				return tabs;
			};

			ofs << indent_creation(depth);
			ofs << (joint->parent() == nullptr ? "ROOT" : "JOINT");
			ofs << " " << joint->name() << "\n";

			ofs << indent_creation(depth);
			ofs << "{" << "\n";

			ofs << indent_creation(depth + 1);
			ofs << "OFFSET" << " ";
			ofs << joint->offset()(0) << " " << joint->offset()(1) << " " << joint->offset()(2);
			ofs << "\n";

			const auto associated_channels_indices = joint->associated_channels_indices();
			ofs << indent_creation(depth + 1);
			ofs << "CHANNELS" << " " << associated_channels_indices.size();
			for (auto i : associated_channels_indices)
			{
				ofs << " " << Channel::from_Type(channels()[i].type);
			}
			ofs << "\n";

			if (joint->has_end_site())
			{
				ofs << indent_creation(depth + 1);
				ofs << "End Site" << "\n";
				ofs << indent_creation(depth + 1);
				ofs << "{" << "\n";
				ofs << indent_creation(depth + 2);
				ofs << "OFFSET" << " ";
				ofs << joint->end_site()(0) << " " << joint->end_site()(1) << " " << joint->end_site()(2);
				ofs << "\n";
				ofs << indent_creation(depth + 1);
				ofs << "}" << "\n";
			}

			for (auto child : joint->children())
			{
				WriteJointSubHierarchy(ofs, child, depth + 1);
			}

			ofs << indent_creation(depth);
			ofs << "}" << "\n";
		}
	};

	struct Channel
	{
		enum Type
		{
			Xposition, Yposition, Zposition,
			Zrotation, Xrotation, Yrotation
		};

		DECLARE_ENUM_STR(Type)

		const Type type;
		const std::shared_ptr<Joint> target_joint;
	};

	std::ostream& operator<<(std::ostream& os, const Channel::Type& type);

}

#endif
