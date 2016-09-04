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
	videostream(int id, size_t width, size_t height, int buffsize=4);
	videostream() {}
	~videostream();
	
	void lock();
	void unlock();
	void read(videoframe_t & dst);
	void read_no_lock(videoframe_t & dst);
private:
	int m_id;
	int m_buffsize;
	size_t m_width;
	size_t m_height;
	bool m_terminate;
	
	videosource_t cap;
	std::queue<videoframe_t> m_buffer;
	std::thread m_thread;
	std::mutex m_mutex;

	void run();
};

#endif
