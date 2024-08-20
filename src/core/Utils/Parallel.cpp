#include "Utils/Parallel.h"

namespace RT
{
	void Barrier::wait() 
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		if (--m_count == 0)
		{
			// ���һ���̵߳�����������߳�
			m_cv.notify_all();
		}
		else
		{
			// �����߳�δ����ȴ������߳�
			m_cv.wait(lock, [this] { return m_count == 0; });
		}
	}
}