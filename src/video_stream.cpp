#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <thread>
#include <mutex>
#include <queue>
#include <atomic>

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

#include "video_stream.h"

videostream::videostream(int id):
	m_id(id), m_terminate(false), m_thread(&videostream::run, this)
{
}

videostream::~videostream()
{
	m_terminate = true;
	m_thread.join();
}

void videostream::lock()
{
	m_mutex.lock();
}

void videostream::unlock()
{
	m_mutex.unlock();
}

void videostream::read(videoframe_t & dst)
{
	m_mutex.lock();
	if (m_buffer.size() > 0) {
    	m_buffer.front().copyTo(dst);   //get the oldest grabbed frame (queue=FIFO)
       	m_buffer.pop();		
	}
	m_mutex.unlock();
}

void videostream::read_no_lock(videoframe_t & dst)
{
	if (m_buffer.size() > 0) {
    	m_buffer.front().copyTo(dst);   //get the oldest grabbed frame (queue=FIFO)
       	m_buffer.pop();		
	}
}

void videostream::run()
{
	std::cout << "Videostream " << m_id << " running." << std::endl;

	videosource_t cap;
	videosource_init(cap, 0);
	videoframe_t tmp;

    while(!m_terminate) {
		fetch_frame(cap, tmp);
        if (tmp.empty()) {
			continue;
		}	
        m_mutex.lock();
		if (m_buffer.size() < 4) {
        	m_buffer.push(videoframe_t(tmp.size(), tmp.type()));
       		tmp.copyTo(m_buffer.back());
#ifdef DEBUG_PIPELINE
			cv::imshow("tmp", tmp);
			cv::waitKey(1);
#endif
		}
        m_mutex.unlock();
    }	
}
