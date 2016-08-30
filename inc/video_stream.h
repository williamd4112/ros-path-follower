#ifndef _VIDEO_STREAM_H_
#define _VIDEO_STREAM_H_

#include <pthread.h>

#include <thread>
#include <mutex>
#include <queue>
#include <atomic>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

#include "config.h"

class videostream
{
public:
	videostream(int id);
	videostream() {}
	~videostream();
	
	void lock();
	void unlock();
	void read(videoframe_t & dst);
	void read_no_lock(videoframe_t & dst);
private:
	int m_id;
	bool m_terminate;
	std::queue<videoframe_t> m_buffer;
	std::thread m_thread;
	std::mutex m_mutex;

	void run();
};

#endif