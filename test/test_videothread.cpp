#include <thread>
#include <map>
#include <mutex>
#include <queue>
#include <atomic>

#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <thread>
#include <mutex>
#include <queue>
#include <atomic>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

using namespace cv;

std::queue<Mat> buffer;
std::mutex mtxCam;
std::atomic<bool> grabOn; //this is lock free

void GrabThread(VideoCapture *cap)
{
    Mat tmp;

    //To know how many memory blocks will be allocated to store frames in the queue.
    //Even if you grab N frames and create N x Mat in the queue
    //only few real memory blocks will be allocated
    //thanks to std::queue and cv::Mat memory recycling
    std::map<unsigned char*, int> matMemoryCounter;
    uchar * frameMemoryAddr;

    while (grabOn.load() == true) //this is lock free
    {
        //grab will wait for cam FPS
        //keep grab out of lock so that 
        //idle time can be used by other threads
        *cap >> tmp; //this will wait for cam FPS

        if (tmp.empty()) continue;

        //get lock only when we have a frame
        mtxCam.lock();
        //buffer.push(tmp) stores item by reference than avoid
        //this will create a new cv::Mat for each grab
        buffer.push(Mat(tmp.size(), tmp.type()));
        tmp.copyTo(buffer.back());
        frameMemoryAddr = buffer.front().data;
        mtxCam.unlock();
        //count how many times this memory block has been used
        matMemoryCounter[frameMemoryAddr]++;  
    }
    std::cout << std::endl << "Number of Mat in memory: " << matMemoryCounter.size();
}

void ProcessFrame(const Mat &src)
{
    if(src.empty()) return;
    imshow("Image main", src);
}

int main() {
    Mat frame;
    VideoCapture cap;
    cap.open(0);
    if (!cap.isOpened()) //check if we succeeded
        return -1;

    grabOn.store(true);                //set the grabbing control variable
    std::thread t(GrabThread, &cap);          //start the grabbing thread
    int bufSize;
    while (true)
    {
		double t = (double)cv::getTickCount();  
		mtxCam.lock();                //lock memory for exclusive access
        bufSize = buffer.size();      //check how many frames are waiting 
        if (bufSize > 0)              //if some 
        {
            //reference to buffer.front() should be valid after 
            //pop because of Mat memory reference counting
            //but if content can change after unlock is better to keep a copy
            //an alternative is to unlock after processing (this will lock grabbing)
            buffer.front().copyTo(frame);   //get the oldest grabbed frame (queue=FIFO)
            buffer.pop();
			t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
			std::cout << "Time : " << t << std::endl;
		
			imshow("frame", frame);
			waitKey(1);
		}
		mtxCam.unlock();
	}

	return 0;
}
