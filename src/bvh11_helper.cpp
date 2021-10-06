#include "pch.h"
#include <regex>
#include <cassert>
#include <fstream>
#include <queue>
#include <Windows.h>
#include <map>
#include "math.hpp"
#include "bvh11_helper.hpp"
#include "ArtiBody.hpp"

namespace bvh11
{
	namespace internal
	{
		class FastStream
		{
		public:
			FastStream(const std::string& path)
			{
				//open a file, and map it to memory
				if (!LoadFile(path.c_str(), &m_mem))
				{
					memset(&m_mem, 0, sizeof(m_mem));
					m_streamPtr = NULL;
				}
				else
					m_streamPtr = m_mem.p;
			}

			~FastStream()
			{
				//close the file
				if (NULL != m_streamPtr)
					UnLoad(&m_mem);
				m_streamPtr = NULL;
			}

			bool is_open() const
			{
				//to check if the file is open and mapped to memor correctly
				return (m_mem.size > 0);
			}


			bool getline(std::string& line)
			{
				const unsigned char* p_end = m_mem.p + m_mem.size;
				while (m_streamPtr < p_end && is_on_line_delim(m_streamPtr, p_end))
					m_streamPtr ++;
				const unsigned char* p_line_start = m_streamPtr;
				while (m_streamPtr < p_end && !is_on_line_delim(m_streamPtr, p_end))
					m_streamPtr ++;
				const unsigned char* p_line_end = m_streamPtr;
				std::size_t len_line = p_line_end - p_line_start;
				std::size_t cap_line = len_line + 1;
				if (line.size() < cap_line)
					line.resize(cap_line, '\0');
				line.assign((const char*)p_line_start, len_line);
				return p_line_start < p_line_end;
			}
		private:
			bool is_on_line_delim(const unsigned char* p, const unsigned char* p_end)
			{
				const unsigned char* pp = p + 1;
				return (*p == '\n')
					|| ((*p == '\r') && (pp < p_end) && (*pp == '\n'));
			}

			struct MemSrc
			{
				const unsigned char* p;
				unsigned int size;
				HANDLE hFile;
				HANDLE hMapFile;
			};
			inline bool LoadFile(const char* filePath, MemSrc* mem)
			{
				mem->hFile = CreateFileA(filePath,               // file name
			                       GENERIC_READ,          // open for reading
			                       0,                     // do not share
			                       NULL,                  // default security
			                       OPEN_EXISTING,         // existing file only
			                       FILE_ATTRIBUTE_NORMAL, // normal file
			                       NULL);                 // no template

			   	bool ok = (INVALID_HANDLE_VALUE != mem->hFile);
			   	if (ok)
			   	{
			   		mem->size = GetFileSize(mem->hFile, 0);
			   		mem->hMapFile = CreateFileMapping(
			                 mem->hFile,    				// use paging file
			                 NULL,                  // default security
			                 PAGE_READONLY,        // read/write access
			                 0,						// maximum object size (high-order DWORD)
			                 mem->size,						// maximum object size (low-order DWORD)
			                 NULL);                 // name of mapping object
			   		ok = (mem->hMapFile != NULL);
			   	}

			   	if (ok)
			   	{
			   		mem->p = (const unsigned char*) MapViewOfFile(mem->hMapFile,   // handle to map object
			                    	    				FILE_MAP_READ, // read/write permission
			                    	    				0,
			                    	    				0,
			                    	    				mem->size);
			   	}
			   	else
			   	{
			   		mem->p = NULL;
			   		mem->size = 0;
			   		mem->hFile = INVALID_HANDLE_VALUE;
			   		mem->hMapFile = NULL;
			   	}
			   	return ok;
			}
			void UnLoad(MemSrc* mem)
			{
				UnmapViewOfFile(mem->p);

			   	CloseHandle(mem->hMapFile);
			   	CloseHandle(mem->hFile);
			   	mem->size = 0;

			}

			MemSrc m_mem;

			const unsigned char* m_streamPtr;
		};

		inline std::vector<std::string> split(const std::string& sequence, const std::string& pattern)
		{
			const std::regex regex(pattern);
			const std::sregex_token_iterator first{ sequence.begin(), sequence.end(), regex, -1 };
			const std::sregex_token_iterator last {};
			return std::vector<std::string>{ first->str().empty() ? std::next(first) : first, last };
		}



		inline void split(const std::string& sequence, std::vector<std::string>& tokens)
		{
			auto is_a_break = [](char c) -> bool
						{
							return ' ' == c
								|| '\t' == c;
						};
			auto is_a_token = [](char c) -> bool
						{
							return ' ' != c
								&& '\t' != c
								&& '\0' != c;
						};
			auto is_a_termi = [](char c) -> bool
						{
							return '\0' == c;
						};

			assert(!sequence.empty());

			std::size_t i_token = 0;
			std::size_t n_tokens = tokens.size();
			enum State {is_on_break = 0, is_on_token, is_on_end};
			State s = is_on_break;
			const char* p = sequence.c_str();
			const char* p_token_start = p;
			while(s != is_on_end
				&& i_token < n_tokens)
			{
				State s_p = s;
				assert(*p != '\r' && *p != '\n');
				if (is_a_break(*p))
					s_p = is_on_break;
				else if (is_a_token(*p))
					s_p = is_on_token;
				else if (is_a_termi(*p))
					s_p = is_on_end;

				if (s == is_on_break
					&& s_p == is_on_token)
				{
					p_token_start = p;
				}

				else if (s == is_on_token
					&& (s_p == is_on_end || s_p == is_on_break))
				{
					std::size_t len = (p-p_token_start);
					std::string& token_i = tokens[i_token];
					token_i.assign(p_token_start, len);
					token_i[len] = '\0';
					i_token ++;
				}
				s = s_p;
				p ++;
			}
		}

		inline std::vector<std::string> tokenize_next_line(std::ifstream& ifs)
		{
			std::string line;
			if (std::getline(ifs, line))
			{
				return internal::split(line, R"([\t\s]+)");
			}
			else
			{
				assert(false && "Failed to read the next line");
				return {};
			}
		}

		inline std::vector<std::string> tokenize_next_line(internal::FastStream& ifs)
		{
			std::string line;
			if (ifs.getline(line))
			{
				return internal::split(line, R"([\t\s]+)");
			}
			else
			{
				assert(false && "Failed to read the next line");
				return {};
			}
		}


		inline void tokenize_next_line(internal::FastStream& ifs, std::string& line, std::vector<std::string>& tokens)
		{
			if (ifs.getline(line))
			{
				internal::split(line, tokens);
			}
			else
			{
				assert(false && "Failed to read the next line");
			}
		}



		inline Eigen::Vector3d read_offset(const std::vector<std::string>& tokens)
		{
			assert(tokens.size() == 4 && tokens[0] == "OFFSET");
			const double offset_x = std::stod(tokens[1]);
			const double offset_y = std::stod(tokens[2]);
			const double offset_z = std::stod(tokens[3]);
			return Eigen::Vector3d(offset_x, offset_y, offset_z);
		}
	}

	BEGIN_ENUM_STR(Channel, Type)
		ENUM_ITEM(x_position)
		ENUM_ITEM(y_position)
		ENUM_ITEM(z_position)
		ENUM_ITEM(z_rotation)
		ENUM_ITEM(x_rotation)
		ENUM_ITEM(y_rotation)
	END_ENUM_STR(Channel, Type)

	BvhObject::BvhObject(const CArtiBodyNode* root_src, int n_frames)
		: frame_time_(0.0083333)
		, frames_(n_frames) // throw(...)
	{
		auto Offset = [](const CArtiBodyNode* body) -> Eigen::Vector3d
			{
				const CArtiBodyNode* body_p = body->GetParent();
				if (nullptr == body_p)
					return Eigen::Vector3d::Zero();
				else
				{
					Eigen::Vector3r offset_r = body->GetTransformLocal2World()->getTranslation()
												- body_p->GetTransformLocal2World()->getTranslation();
					return Eigen::Vector3d(offset_r.x(), offset_r.y(), offset_r.z());
				}
			};

		auto root_dst = std::shared_ptr<Joint>(new Joint(root_src->GetName_c(), nullptr));
		root_joint_ = root_dst;
		struct Bound
		{
			const CArtiBodyNode* src;
			std::shared_ptr<Joint> dst;
		};
		Bound rootBnd = { root_src, root_dst };
		std::queue<Bound> bfs_que;
		bfs_que.push(rootBnd);
		while (!bfs_que.empty())
		{
			Bound node_b = bfs_que.front();
			auto body_src = node_b.src;
			auto joint_dst = node_b.dst;
			joint_dst->offset() = Offset(body_src);
			const CArtiBodyNode* body_src_child = body_src->GetFirstChild();
			if (nullptr == body_src_child)
			{
				joint_dst->has_end_site() = true;
				joint_dst->end_site() = Eigen::Vector3d::Zero();
			}
			for (
				; nullptr != body_src_child
				; body_src_child = body_src_child->GetNextSibling())
			{
				auto joint_dst_child = std::shared_ptr<Joint>(new Joint(body_src_child->GetName_c(), joint_dst));
				Bound node_b_child = { body_src_child, joint_dst_child };
				bfs_que.push(node_b_child);
				joint_dst->AddChild(joint_dst_child);
			}
			bfs_que.pop();
		}

		SetJointChannel(root_dst);

		motion_.resize(frames_, channels_.size());
	}

	BvhObject::BvhObject(const BvhObject& src)
		: frame_time_(src.frame_time_)
		, frames_(src.frames_)
		, motion_(src.motion_)
	{
		std::map<std::string, std::shared_ptr<Joint>> name2joint;
		auto root_dst = std::shared_ptr<Joint>(new Joint(src.root_joint_->name(), nullptr));
		root_joint_ = root_dst;
		struct Bound
		{
			const std::shared_ptr<const Joint> src;
			std::shared_ptr<Joint> dst;
		};
		Bound rootBnd = { src.root_joint_, root_dst };
		std::queue<Bound> bfs_que;
		bfs_que.push(rootBnd);
		while (!bfs_que.empty())
		{
			Bound node_b = bfs_que.front();
			auto joint_src = node_b.src;
			auto joint_dst = node_b.dst;
			joint_dst->offset() = joint_src->offset();
			if (joint_src->children().empty())
			{
				joint_dst->has_end_site() = joint_src->has_end_site();
				joint_dst->end_site() = joint_src->end_site();
			}
			for (auto joint_src_child : joint_src->children())
			{
				auto joint_dst_child = std::shared_ptr<Joint>(new Joint(joint_src_child->name(), joint_dst));
				Bound node_b_child = { joint_src_child, joint_dst_child };
				bfs_que.push(node_b_child);
				joint_dst->AddChild(joint_dst_child);
			}
			name2joint[joint_dst->name()] = joint_dst;
			bfs_que.pop();
		}

		for (auto channel_src_i : src.channels_)
		{
			auto joint_i = name2joint[channel_src_i.target_joint->name()];
			Channel channel_dst_i = { channel_src_i.type, joint_i };
			joint_i->AssociateChannel((int)channels_.size());
			channels_.push_back(channel_dst_i);
		}

	}

	void BvhObject::Dump()
	{
		for (auto channel_i : channels_)
		{
			std::cout << Channel::from_Type(channel_i.type) << ", " << channel_i.target_joint->name() << std::endl;
		}
	}

	void BvhObject::SetJointChannel(std::shared_ptr<Joint> joint)
	{
		const std::vector<Channel::Type> channels_root = { Channel::Type::x_position
														, Channel::Type::y_position
														, Channel::Type::z_position
														, Channel::Type::z_rotation
														, Channel::Type::y_rotation
														, Channel::Type::x_rotation };
		const std::vector<Channel::Type> channels_leaf = { Channel::Type::z_rotation
														, Channel::Type::y_rotation
														, Channel::Type::x_rotation };
		bool is_root = (nullptr == joint->parent());
		const auto & channels_joint = (is_root ? channels_root : channels_leaf);

		for (auto channel : channels_joint)
		{
			joint->AssociateChannel((int)channels_.size());
			channels_.push_back({ channel, joint });
		}

		for (auto joint_child : joint->children())
			SetJointChannel(joint_child);
	};

	std::vector<std::shared_ptr<const Joint>> BvhObject::GetJointList() const
	{
		std::vector<std::shared_ptr<const Joint>> joint_list;
		std::function<void(std::shared_ptr<const Joint>)> add_joint = [&](std::shared_ptr<const Joint> joint)
		{
			joint_list.push_back(joint);
			for (auto child : joint->children()) { add_joint(child); }
		};
		add_joint(root_joint_);
		return joint_list;
	}

	template<typename LAMaccessEnter, typename LAMaccessLeave>
	inline void TraverseBFS_boundtree_norecur(BvhObject::Bound root, LAMaccessEnter OnEnterBound, LAMaccessLeave OnLeaveBound)
	{
		std::queue<BvhObject::Bound> queBFS;
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

	void BvhObject::SetMotion(const CArtiBodyNode* root, int i_frame)
	{
		auto& bvh = *this;
		auto onEnterBound_pose = [&bvh, i_frame](Bound b_this)
		{
			Joint_bvh_ptr joint_bvh = b_this.first;
			const IJoint* joint_hik = b_this.second->GetJoint();
			_TRANSFORM delta_l_tm = {0};
			joint_hik->GetTransform()->CopyTo(delta_l_tm);

			Eigen::Affine3d delta_l;
			Eigen::Vector3d tt(delta_l_tm.tt.x,
				delta_l_tm.tt.y,
				delta_l_tm.tt.z);
			Eigen::Quaterniond r(delta_l_tm.r.w,
				delta_l_tm.r.x,
				delta_l_tm.r.y,
				delta_l_tm.r.z);
			Eigen::Vector3d s(delta_l_tm.s.x,
				delta_l_tm.s.y,
				delta_l_tm.s.z);
			delta_l.fromPositionOrientationScale(tt, r, s);
			bvh.UpdateMotion(joint_bvh, delta_l, i_frame);

		};
		auto onLeaveBound_pose = [](Bound b_this) {};

		Bound root_b = std::make_pair(root_joint_, root);
		TraverseBFS_boundtree_norecur(root_b, onEnterBound_pose, onLeaveBound_pose);
	}

	void BvhObject::UpdateMotion(std::shared_ptr<const Joint> joint, const Eigen::Affine3d& tm_delta_l, int i_frame)
	{
		Eigen::Index order2rot_i[3] = { 0 };
		int rot_i2order[3] = { 0 };
		int order = 0;
		auto & idx_jc = joint->associated_channels_indices();
		for (auto it_i_c = idx_jc.begin()
			; order < 3 && it_i_c != idx_jc.end()
			; it_i_c ++)
		{
			int i_channel = *it_i_c;
			const bvh11::Channel& channel = channels()[i_channel];
			bool is_a_rotation = ( bvh11::Channel::Type::x_rotation == channel.type
								|| bvh11::Channel::Type::y_rotation == channel.type
								|| bvh11::Channel::Type::z_rotation == channel.type);
			if (is_a_rotation)
			{
				Eigen::Index rot_i = -1;
				switch (channel.type)
				{
					case bvh11::Channel::Type::x_rotation:
						rot_i = 0;
						break;
					case bvh11::Channel::Type::y_rotation:
						rot_i = 1;
						break;
					case bvh11::Channel::Type::z_rotation:
						rot_i = 2;
						break;
				}

				order2rot_i[order] = rot_i;
				rot_i2order[rot_i] = order;
				order ++;
			}
		}
		assert(3 == order
			&& order2rot_i[0] != order2rot_i[1]
			&& order2rot_i[0] != order2rot_i[2]
			&& order2rot_i[1] != order2rot_i[2]
			&& "make sure the order2rot_i is correctly recording the order of rotation");
		Eigen::Vector3d rots_euler = tm_delta_l.rotation().eulerAngles(order2rot_i[0], order2rot_i[1], order2rot_i[2]);
		const auto& data_euler = rots_euler.data();
		double value_rots[] = {
								data_euler[rot_i2order[0]], // rotate about x
								data_euler[rot_i2order[1]], // rotate about y
								data_euler[rot_i2order[2]], // rotate about z
							};

		const double c_rad2deg = 180.0/M_PI;

		const Eigen::Vector3d& value_tt = tm_delta_l.translation();
		bool has_translation_channel = false;
		for (int channel_index : joint->associated_channels_indices())
		{
			const bvh11::Channel& channel = channels()[channel_index];
			double&          value = const_cast<double&>(motion()(i_frame, channel_index));

			switch (channel.type)
			{
			case bvh11::Channel::Type::x_position:
				value = value_tt.x();
				has_translation_channel = true;
				break;
			case bvh11::Channel::Type::y_position:
				value = value_tt.y();
				has_translation_channel = true;
				break;
			case bvh11::Channel::Type::z_position:
				value = value_tt.z();
				has_translation_channel = true;
				break;
			case bvh11::Channel::Type::x_rotation:
				value = value_rots[0] * c_rad2deg;
				break;
			case bvh11::Channel::Type::y_rotation:
				value = value_rots[1] * c_rad2deg;
				break;
			case bvh11::Channel::Type::z_rotation:
				value = value_rots[2] * c_rad2deg;
				break;
			}
		}
		double epsilon = 1e-3;
		bool has_translation_in_delta_tm = (value_tt.norm() > epsilon);
		assert((!has_translation_in_delta_tm || has_translation_channel)
			&& "has_translation_in_delta_tm -> has_translation_channel");
	}



	Eigen::Affine3d BvhObject::GetLocalDeltaTM(std::shared_ptr<const Joint> joint, int frame) const
	{
		if (frame < 0)
			return Eigen::Affine3d::Identity();
		bool has_a_rotation = false;
		bool has_a_translation = false;
		Eigen::Affine3d tm_delta_l = Eigen::Affine3d::Identity();
		for (int channel_index : joint->associated_channels_indices())
		{
			const bvh11::Channel& channel = channels()[channel_index];
			const double          value   = motion()(frame, channel_index);

			switch (channel.type)
			{
				case bvh11::Channel::Type::x_position:
					tm_delta_l *= Eigen::Translation3d(Eigen::Vector3d(value, 0.0, 0.0));
					has_a_translation = true;
					break;
				case bvh11::Channel::Type::y_position:
					tm_delta_l *= Eigen::Translation3d(Eigen::Vector3d(0.0, value, 0.0));
					has_a_translation = true;
					break;
				case bvh11::Channel::Type::z_position:
					tm_delta_l *= Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, value));
					has_a_translation = true;
					break;
				case bvh11::Channel::Type::x_rotation:
					tm_delta_l *= Eigen::AngleAxisd(value * M_PI / 180.0, Eigen::Vector3d::UnitX());
					has_a_rotation = true;
					break;
				case bvh11::Channel::Type::y_rotation:
					tm_delta_l *= Eigen::AngleAxisd(value * M_PI / 180.0, Eigen::Vector3d::UnitY());
					has_a_rotation = true;
					break;
				case bvh11::Channel::Type::z_rotation:
					tm_delta_l *= Eigen::AngleAxisd(value * M_PI / 180.0, Eigen::Vector3d::UnitZ());
					has_a_rotation = true;
					break;
			}
		}

		assert((joint->associated_channels_indices().size() != 3 || (has_a_rotation && !has_a_translation)) && "|channels| == 3 -> (rotation && not translation)"
			&& (joint->associated_channels_indices().size() != 6 || (has_a_rotation && has_a_translation)) && "|channels| == 6 -> (rotation && translation)");

		return tm_delta_l;
	}

	Eigen::Affine3d BvhObject::GetTransformationRelativeToParent(std::shared_ptr<const Joint> joint, int frame) const
	{
		assert(frame < frames() && "Invalid frame is specified.");
		assert(joint->associated_channels_indices().size() == 3 || joint->associated_channels_indices().size() == 6);

		Eigen::Affine3d tm_l2p_0(Eigen::Translation3d(joint->offset()));
		// having:
		//		tm_l2p_0.linear() == Eigen::Matrix3d::Identity();
		//		tm_l2p_0.translation() == joint->offset();
		bool rest_posture = (frame < 0);
		if (!rest_posture)
		{
			Eigen::Affine3d tm_delta_l = GetLocalDeltaTM(joint, frame);
			return tm_l2p_0 * tm_delta_l;
		}
		else
			return tm_l2p_0;
	}

	Eigen::Affine3d BvhObject::GetTransformation(std::shared_ptr<const Joint> joint, int frame) const
	{
		Eigen::Affine3d transform = GetTransformationRelativeToParent(joint, frame);

		while (joint->parent().get() != nullptr)
		{
			joint = joint->parent();

			transform = GetTransformationRelativeToParent(joint, frame) * transform;
		}

		return transform;
	}

	void BvhObject::ReadBvhFile(const std::string& file_path, const double scale) throw(...)
	{
		// Open the input file
		internal::FastStream ifs(file_path);
		if (!ifs.is_open())
		{
			std::stringstream expInfo;
			expInfo << "File: " << file_path.c_str() << " can not be open!!!";
			throw expInfo.str();
		}
		// assert(ifs.is_open() && "Failed to open the input file.");

		// Read the HIERARCHY part
		[&]() -> void
		{
			std::string line;
			std::vector<std::shared_ptr<Joint>> stack;
			while(ifs.getline(line))
			{
				// Split the line into tokens
				const std::vector<std::string> tokens = internal::split(line, R"([\t\s]+)");

				// Ignore empty lines
				if (tokens.empty())
				{
					continue;
				}
				// Ignore a declaration of hierarchy section
				else if (tokens[0] == "HIERARCHY")
				{
					continue;
				}
				// Start to create a new joint
				else if (tokens[0] == "ROOT" || tokens[0] == "JOINT")
				{
					assert(tokens.size() == 2 && "Failed to find a joint name");

					// Read the joint name
					const std::string& joint_name = tokens[1];

					// Get a pointer for the parent if this is not a root joint
					const std::shared_ptr<Joint> parent = stack.empty() ? nullptr : stack.back();

					// Instantiate a new joint

					std::shared_ptr<Joint> new_joint(new Joint(joint_name, parent));

					// Register it to the parent's children list
					if (parent) { parent->AddChild(new_joint); }
					else { root_joint_ = new_joint; }

					// Add the new joint to the stack
					stack.push_back(new_joint);

					// Read the next line, which should be "{"
					const std::vector<std::string> tokens_begin_block = internal::tokenize_next_line(ifs);
					assert(tokens_begin_block.size() == 1 && "Found two or more tokens");
					assert(tokens_begin_block[0] == "{" && "Could not find an expected '{'");
				}
				// Read an offset value
				else if (tokens[0] == "OFFSET")
				{
					assert(tokens.size() == 4);
					const std::shared_ptr<Joint> current_joint = stack.back();
					current_joint->offset() = scale * internal::read_offset(tokens);
				}
				// Read a channel list
				else if (tokens[0] == "CHANNELS")
				{
					assert(tokens.size() >= 2);
					const int num_channels = std::stoi(tokens[1]);

					assert(tokens.size() == num_channels + 2);
					for (int i = 0; i < num_channels; ++ i)
					{
						const std::shared_ptr<Joint> target_joint = stack.back();
						const Channel::Type type = [](const std::string& channel_type)
						{
							if (channel_type == "Xposition") { return Channel::Type::x_position; }
							if (channel_type == "Yposition") { return Channel::Type::y_position; }
							if (channel_type == "Zposition") { return Channel::Type::z_position; }
							if (channel_type == "Zrotation") { return Channel::Type::z_rotation; }
							if (channel_type == "Xrotation") { return Channel::Type::x_rotation; }
							if (channel_type == "Yrotation") { return Channel::Type::y_rotation; }

							assert(false && "Could not find a valid channel type");
							return Channel::Type();
						}(tokens[i + 2]);

						channels_.push_back(Channel{ type, target_joint });

						const int channel_index = static_cast<int>(channels_.size() - 1);
						target_joint->AssociateChannel(channel_index);
					}
				}
				// Read an end site
				else if (tokens[0] == "End")
				{
					assert(tokens.size() == 2 && tokens[1] == "Site");

					const std::shared_ptr<Joint> current_joint = stack.back();
					current_joint->has_end_site() = true;

					// Read the next line, which should be "{"
					const std::vector<std::string> tokens_begin_block = internal::tokenize_next_line(ifs);
					assert(tokens_begin_block.size() == 1 && "Found two or more tokens");
					assert(tokens_begin_block[0] == "{" && "Could not find an expected '{'");

					// Read the next line, which should state an offset
					const std::vector<std::string> tokens_offset = internal::tokenize_next_line(ifs);
					current_joint->end_site() = scale * internal::read_offset(tokens_offset);

					// Read the next line, which should be "{"
					const std::vector<std::string> tokens_end_block = internal::tokenize_next_line(ifs);
					assert(tokens_end_block.size() == 1 && "Found two or more tokens");
					assert(tokens_end_block[0] == "}" && "Could not find an expected '}'");
				}
				// Finish to create a joint
				else if (tokens[0] == "}")
				{
					assert(!stack.empty());
					stack.pop_back();
				}
				// Stop this iteration and go to the motion section
				else if (tokens[0] == "MOTION")
				{
					return;
				}
			}
			assert(false && "Could not find the MOTION part");
		}();

		// Read the MOTION part
		[&]() -> void
		{
			// Read the number of frames
			const std::vector<std::string> tokens_frames = internal::tokenize_next_line(ifs);
			assert(tokens_frames.size() == 2);
			assert(tokens_frames[0] == "Frames:");
			frames_ = std::stoi(tokens_frames[1]);

			// Read the frame time
			const std::vector<std::string> tokens_frame_time = internal::tokenize_next_line(ifs);
			assert(tokens_frame_time.size() == 3);
			assert(tokens_frame_time[0] == "Frame" && tokens_frame_time[1] == "Time:");
			frame_time_ = std::stod(tokens_frame_time[2]);

			// Allocate memory for storing motion data
			motion_.resize(frames_, channels_.size());

			// Read each frame
			std::vector<std::string> tokens;
			tokens.resize(channels_.size());
			const std::size_t double_token_len = 256;
			for (auto& token : tokens)
				token.resize(double_token_len, '\0');
			std::string line;
			for (int frame_index = 0; frame_index < frames_; ++ frame_index)
			{
				internal::tokenize_next_line(ifs, line, tokens);
				assert(tokens.size() == channels_.size() && "Found invalid motion data");

				for (int channel_index = 0; channel_index < channels_.size(); ++ channel_index)
				{
					motion_(frame_index, channel_index) = std::stod(tokens[channel_index]);
				}
			}

			// Scale translations
			for (int channel_index = 0; channel_index < channels_.size(); ++ channel_index)
			{
				const Channel::Type& type = channels_[channel_index].type;
				if (type == Channel::Type::x_position || type == Channel::Type::y_position || type == Channel::Type::z_position)
				{
					motion_.col(channel_index) = scale * motion_.col(channel_index);
				}
			}
		}();
	}

	void BvhObject::PrintJointSubHierarchy(std::shared_ptr<const Joint> joint, int depth) const
	{
		for (int i = 0; i < depth; ++ i) { std::cout << "  "; }
		std::cout << joint->name() << std::endl;

		for (auto child : joint->children()) { PrintJointSubHierarchy(child, depth + 1); }
	}

	void BvhObject::WriteJointSubHierarchy(std::ofstream& ofs, std::shared_ptr<const Joint> joint, int depth) const
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
			ofs << " " << channels()[i].type;
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

	void BvhObject::WriteBvhFile(const std::string& file_path) const
	{
		// Eigen format
		const Eigen::IOFormat motion_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", "", "", "\n", "", "");

		// Open the input file
		std::ofstream ofs(file_path);
		assert(ofs.is_open() && "Failed to open the output file.");

		// Hierarch
		ofs << "HIERARCHY" << "\n";
		WriteJointSubHierarchy(ofs, root_joint_, 0);

		// Motion
		ofs << "MOTION" << "\n";
		ofs << "Frames: " << frames_ << "\n";
		ofs << "Frame Time: " << frame_time_ << "\n";
		ofs << motion_.format(motion_format);
	}

	void BvhObject::ResizeFrames(int num_new_frames)
	{
		assert(num_new_frames >= 0 && "Received an invalid frame number.");

		motion_.conservativeResize(num_new_frames, Eigen::NoChange);
		frames_ = num_new_frames;
		return;
	}

	std::ostream& operator<<(std::ostream& os, const Channel::Type& type)
	{
		switch (type) {
			case Channel::Type::x_position:
				os << "Xposition";
				break;
			case Channel::Type::y_position:
				os << "Yposition";
				break;
			case Channel::Type::z_position:
				os << "Zposition";
				break;
			case Channel::Type::x_rotation:
				os << "Xrotation";
				break;
			case Channel::Type::y_rotation:
				os << "Yrotation";
				break;
			case Channel::Type::z_rotation:
				os << "Zrotation";
				break;
		}
		return os;
	}
}
