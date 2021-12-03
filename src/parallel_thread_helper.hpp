#pragma once
#include <windows.h>

class CThread
{
public:
	virtual bool Create_main(unsigned int stk_size = 0) = 0;
	virtual void Run_worker() = 0;
	virtual void Execute_main() = 0;
	virtual unsigned int Dur_main() const = 0;
};

class CThread_W32 : public CThread
{
public:
	CThread_W32()
		: m_thread_h(NULL)
		, m_readiness_sema(NULL)
		, m_cmdExe_evt(NULL)
		, m_cmdQuit_evt(NULL)
		, m_durMilli(0)
	{
	}

	virtual bool Create_main(unsigned int stk_size = 0) final
	{
		m_readiness_sema = CreateSemaphore(NULL
										, 0	// initial count, nonsignaled
										, 1 // maximum count
										, NULL);

		if (NULL == m_readiness_sema)
			return false;

		HANDLE* cmd_evts[] = {&m_cmdExe_evt, &m_cmdQuit_evt};
		for (auto evt : cmd_evts)
		{
			*evt = CreateEvent(NULL
							, FALSE //manual-reset event
							, FALSE //initial state is nonsignaled
							, NULL );
			if (NULL == *evt)
				return false;
		}

		m_thread_h = CreateThread(NULL
								, stk_size
								, (LPTHREAD_START_ROUTINE)CThread_W32::ThreadProc_worker
								, this
								, 0
								, NULL);
		if (NULL == m_thread_h)
			return false;

		return true;
	}

	virtual ~CThread_W32()
	{
		CloseHandle(m_thread_h);
		CloseHandle(m_readiness_sema);
		CloseHandle(m_cmdExe_evt);
		CloseHandle(m_cmdQuit_evt);
	}

	operator HANDLE() const
	{
		return m_thread_h;
	}

	HANDLE Readiness_sema() const
	{
		return m_readiness_sema;
	}

	HANDLE Quit_evt() const
	{
		return m_cmdQuit_evt;
	}

protected:
	void Execute_main()
	{
		if (!SetEvent(m_cmdExe_evt))
    	{
        	printf("SetEvent failed (%d)\n", GetLastError());
        	return;
   		}
	}

	unsigned int Dur_main() const
	{
		return (unsigned int)m_durMilli;
	}


private:
	static DWORD WINAPI ThreadProc_worker(LPVOID lpParam)
	{
		CThread_W32* pThis = reinterpret_cast<CThread_W32*>(lpParam);
		HANDLE cmd_evts[] = {pThis->m_cmdExe_evt, pThis->m_cmdQuit_evt};
		DWORD n_evts = sizeof(cmd_evts)/sizeof(HANDLE);
		bool quit = false;
		DWORD err_code = 1;
		while( !quit)
		{
			if (!ReleaseSemaphore(pThis->m_readiness_sema, 1, NULL))
			{
				printf("ReleaseSemaphore error: %d\n", GetLastError());
				err_code = 0; //indicate an error happened
				break;
			}
			DWORD obj0_offset = ::WaitForMultipleObjects(n_evts,
														cmd_evts,
														FALSE,
														INFINITE);
			int cmd_i = int(obj0_offset - WAIT_OBJECT_0);
			const int CMD_EXE = 0;
			const int CMD_QUIT = 1;
			quit = (cmd_i == CMD_QUIT);
			bool exec = (CMD_EXE == cmd_i);
			if (exec)
			{
				auto tick_start = ::GetTickCount64();
				pThis->Run_worker();
				pThis->m_durMilli = ::GetTickCount64() - tick_start;
			}
		}
		return err_code;
	}
private:
	HANDLE m_thread_h;
	HANDLE m_readiness_sema;
	HANDLE m_cmdExe_evt;
	HANDLE m_cmdQuit_evt;
	volatile ULONGLONG m_durMilli;
};

template<typename Thread>
class CThreadPool_W32
{
public:
	template<typename LAMBDA_Initialize>
	bool Initialize_main(int n_threads, LAMBDA_Initialize Initialize)
	{
		m_threads.resize(n_threads, NULL);
		m_readiness_ref.resize(n_threads, NULL);
		m_cmdQuits_ref.resize(n_threads, NULL);
		bool all_created = true;
		for (int i_thread = 0
			; i_thread < n_threads
				&& all_created
			; i_thread ++)
		{
			Thread* thread_i = new Thread();
			Initialize(thread_i);
			all_created = thread_i->Create_main();
			m_threads[i_thread]  = thread_i;
			m_readiness_ref[i_thread] = thread_i->Readiness_sema();
			m_cmdQuits_ref[i_thread] = thread_i->Quit_evt();
		}
		return all_created;
	}

	~CThreadPool_W32()
	{
		for (auto cmd_quit : m_cmdQuits_ref)
			::SetEvent(cmd_quit);

		int n_threads = (int)m_threads.size();
		std::vector<HANDLE> threads_h;
		threads_h.resize(n_threads);
		for (int i_thread = 0; i_thread < n_threads; i_thread ++)
			threads_h[i_thread] = *(m_threads[i_thread]);
		::WaitForMultipleObjects((DWORD)n_threads
								, threads_h.data()
								, TRUE
								, INFINITE);
		for (auto thread : m_threads)
			delete thread;
	}

	Thread* WaitForAReadyThread_main(unsigned int millisec = 0)
	{
		DWORD obj0_offset = ::WaitForMultipleObjects((DWORD)m_readiness_ref.size(),
														m_readiness_ref.data(),
														FALSE,
														(DWORD)millisec);
		if (WAIT_TIMEOUT == obj0_offset)
			return NULL;
		int i_thread = (int)(obj0_offset - WAIT_OBJECT_0);
		return m_threads[i_thread];
	}

	std::vector<Thread*>& WaitForAllReadyThreads_main()
	{
		::WaitForMultipleObjects((DWORD)m_readiness_ref.size(),
								m_readiness_ref.data(),
								TRUE,
								INFINITE);
		return m_threads;
	}

private:
	std::vector<Thread*> m_threads;
	std::vector<HANDLE> m_readiness_ref; //semaphores
	std::vector<HANDLE> m_cmdQuits_ref;  //event
};