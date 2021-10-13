#include <set>
#include <map>
#include <algorithm>
#include <cctype>
#include <string>
#include <filesystem>

namespace fs = std::experimental::filesystem;

inline std::string Norm(std::string path)
{
	std::transform(path.begin()
				, path.end()
				, path.begin()
				, [](unsigned char c)
					{
						return std::tolower(c);
					});
	return std::move(path);
}

template<typename LAMBDA_onext>
void TraverseDirTree(const std::string& dirPath, LAMBDA_onext onbvh, const std::string& ext) //  throw (std::string)
{
	WIN32_FIND_DATA ffd;
	LARGE_INTEGER filesize;
	std::string filter = dirPath + "\\*";
	HANDLE hFind = INVALID_HANDLE_VALUE;
	DWORD dwError=0;

	// Find the first file in the directory.
	hFind = FindFirstFile(filter.c_str(), &ffd);
	if (INVALID_HANDLE_VALUE == hFind)
	{
		throw std::string("FindFirstFile: ") + dirPath;
	}
	// List all the files in the directory with some info about them.
	std::set<std::string> trivial_dir = {".", ".."};
	bool traversing = true;
	do
	{
		if (ffd.dwFileAttributes & FILE_ATTRIBUTE_HIDDEN)
			continue;
		else if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
		{
			bool trivial = (trivial_dir.end() != trivial_dir.find(ffd.cFileName));

			if (!trivial)
			{
				try
				{
					fs::path dirPath_prime = fs::path(dirPath)/ffd.cFileName;
					TraverseDirTree(dirPath_prime.u8string(), onbvh, ext);
				}
				catch (std::string& info)
				{
					FindClose(hFind);
					traversing = false;
					throw info;
				}
			}
		}
		else
		{
			filesize.LowPart = ffd.nFileSizeLow;
			filesize.HighPart = ffd.nFileSizeHigh;

			if (fs::path(ffd.cFileName).extension().u8string() == ext)
			{
				fs::path filepath = fs::path(dirPath)/ffd.cFileName;
				traversing = onbvh(filepath.u8string().c_str());
			}

		}
	}
	while (traversing
		&& FindNextFile(hFind, &ffd) != 0);

	FindClose(hFind);
	dwError = GetLastError();
	if (dwError != ERROR_NO_MORE_FILES)
	{
		throw std::string("FindFirstFile");
	}
}

template<typename LAMBDA_onext>
void CopyDirTree(const std::string& dir_src, const std::string& dir_dst, LAMBDA_onext oncp, const std::string& ext ) // throw std::string
{
	namespace fs = std::experimental::filesystem;
	fs::create_directory(fs::path(dir_dst));
	WIN32_FIND_DATA ffd;
	LARGE_INTEGER filesize;
	std::string filter = dir_src + "\\*";
	HANDLE hFind = INVALID_HANDLE_VALUE;
	DWORD dwError=0;

	// Find the first file in the directory.
	hFind = FindFirstFile(filter.c_str(), &ffd);
	if (INVALID_HANDLE_VALUE == hFind)
	{
		throw std::string("FindFirstFile: ") + dir_src;
	}
	// List all the files in the directory with some info about them.
	std::set<std::string> trivial_dir = {".", ".."};
	bool traversing = true;
	do
	{
		if (ffd.dwFileAttributes & FILE_ATTRIBUTE_HIDDEN)
			continue;
		else if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
		{
			bool trivial = (trivial_dir.end() != trivial_dir.find(ffd.cFileName));

			if (!trivial)
			{
				try
				{
					fs::path dir_src_prime = fs::path(dir_src)/ffd.cFileName;
					fs::path dir_dst_prime = fs::path(dir_dst)/ffd.cFileName;
					CopyDirTree(dir_src_prime.u8string(), dir_dst_prime.u8string(), oncp, ext);
				}
				catch (std::string& info)
				{
					FindClose(hFind);
					traversing = false;
					throw info;
				}
			}
		}
		else
		{
			filesize.LowPart = ffd.nFileSizeLow;
			filesize.HighPart = ffd.nFileSizeHigh;

			if (fs::path(ffd.cFileName).extension().u8string() == ext)
			{
				fs::path filepath_src = fs::path(dir_src)/ffd.cFileName;
				fs::path filepath_dst = fs::path(dir_dst)/ffd.cFileName;
				traversing = oncp(filepath_src.u8string().c_str(), filepath_dst.u8string().c_str());
			}

		}
	}
	while (traversing
		&& FindNextFile(hFind, &ffd) != 0);

	FindClose(hFind);
	dwError = GetLastError();
	if (dwError != ERROR_NO_MORE_FILES)
	{
		throw std::string("FindFirstFile");
	}
}


template<typename LAMBDA_onext>
void CopyDirTree(const std::string& dir_src, const std::string& dir_dst, const std::vector<LAMBDA_onext>& oncps, const std::vector<std::string>& exts ) // throw std::string
{
	namespace fs = std::experimental::filesystem;
	fs::create_directory(fs::path(dir_dst));
	WIN32_FIND_DATA ffd;
	LARGE_INTEGER filesize;
	std::string filter = dir_src + "\\*";
	HANDLE hFind = INVALID_HANDLE_VALUE;
	DWORD dwError=0;

	std::map<std::string, int> ext2idx;
	int n_exts = (int)exts.size();
	for (int i_ext = 0; i_ext < n_exts; i_ext ++)
		ext2idx[exts[i_ext]] = i_ext;

	// Find the first file in the directory.
	hFind = FindFirstFile(filter.c_str(), &ffd);
	if (INVALID_HANDLE_VALUE == hFind)
	{
		throw std::string("FindFirstFile: ") + dir_src;
	}
	// List all the files in the directory with some info about them.
	std::set<std::string> trivial_dir = {".", ".."};
	bool traversing = true;
	do
	{
		if (ffd.dwFileAttributes & FILE_ATTRIBUTE_HIDDEN)
			continue;
		else if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
		{
			bool trivial = (trivial_dir.end() != trivial_dir.find(ffd.cFileName));

			if (!trivial)
			{
				try
				{
					fs::path dir_src_prime = fs::path(dir_src)/ffd.cFileName;
					fs::path dir_dst_prime = fs::path(dir_dst)/ffd.cFileName;
					CopyDirTree(dir_src_prime.u8string(), dir_dst_prime.u8string(), oncps, exts);
				}
				catch (std::string& info)
				{
					FindClose(hFind);
					traversing = false;
					throw info;
				}
			}
		}
		else
		{
			filesize.LowPart = ffd.nFileSizeLow;
			filesize.HighPart = ffd.nFileSizeHigh;
			auto it_idx = ext2idx.find(fs::path(ffd.cFileName).extension().u8string());
			if (ext2idx.end() != it_idx)
			{
				fs::path filepath_src = fs::path(dir_src)/ffd.cFileName;
				fs::path filepath_dst = fs::path(dir_dst)/ffd.cFileName;
				traversing = oncps[it_idx->second](filepath_src.u8string().c_str(), filepath_dst.u8string().c_str());
			}

		}
	}
	while (traversing
		&& FindNextFile(hFind, &ffd) != 0);

	FindClose(hFind);
	dwError = GetLastError();
	if (dwError != ERROR_NO_MORE_FILES)
	{
		throw std::string("FindFirstFile");
	}
}