#include "Utils/Parallel.h"

namespace RT
{
	void Barrier::wait() 
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		if (--m_count == 0)
		{
			// 最后一个线程到达后唤醒所有线程
			m_cv.notify_all();
		}
		else
		{
			// 还有线程未到达等待其他线程
			m_cv.wait(lock, [this] { return m_count == 0; });
		}
	}
}