#ifndef _VIDEO_ADAPTER_HPP_
#define _VIDEO_ADAPTER_HPP_

typedef cv::Mat frame_t;

typedef enum videoadapter_source_type_t
{
	CAPTURE,
	STREAM
};

class videoadapter
{
public:
	videoadapter();
	~videoadapter();
	
	bool open(int id, videoadapter_source_type_t type);
	void read(frame_t & frame);
private:
		
};

#endif
