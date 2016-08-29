#ifndef _CONCURRENT_BUFFER_H_
#define _CONCUREENT_BUFFER_H_

#include <cstdio>
#include <cstdlib>

#include <stdint.h>
#include <pthread.h>

#include <cassert>

template <typename T>
class concurrent_buffer
{
public:
	concurrent_buffer(size_t size=4);
	~concurrent_buffer();
	
	T * acquire();
	void release();	
private:
	size_t m_size;
	size_t m_index_in;
	size_t m_index_out;
	T * m_buffer;
	pthread_mutex_t * m_buffer_mutexes;

	void lock(size_t index);
	void unlock(size_t index);
};

template <typename T>
concurrent_buffer::concurrent_buffer(size_t size): 
	m_size(size), m_index_in(0), m_index_out(0)
{
	m_buffer = new T[size];
	m_buffer_mutexes = new pthread_mutex_t[size];
	for (int i = 0; i < size; i++) {
		m_buffer_mutexes[i] = PTHREAD_MUTEX_INITIALIZER;
	}

	assert(m_buffer != NULL);	
}

template <typename T>
concurrent_buffer::~concurrent_buffer()
{
	delete [] m_buffer;
}

template <typename T>
T * concurrent_buffer::acquire()
{
	lock(m_index_in);
	m_index_in++;
	m_index_in %= m_size;

	return &m_buffer[m_index_in];
}

#endif
