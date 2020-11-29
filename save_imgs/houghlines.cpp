#include "opencv2/highgui/highgui.hpp"

#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    int m_file_mem;
    m_file_mem = open("/dev/mem", ( O_RDWR | O_SYNC ) );
    
    int cam_offset = 0;
    ifstream ifs;
    ifs.open("/home/root/fpga_index.txt", std::ifstream::in);
    char c[1];
    ifs.read(c, 1);
    ifs.close();
    if (!strcmp(c, "2")){
        cam_offset = 4;
    }
    cout << "\nCam offset " << cam_offset;
    
    uint32_t lw_base_address = 0xFF200000;
    void *virtual_lw_base;
    virtual_lw_base = mmap( NULL, 0x100000, ( PROT_READ | PROT_WRITE ), MAP_SHARED, m_file_mem, lw_base_address );
    uint64_t BTN_address = (uint64_t) virtual_lw_base + 0x4000;
    uint64_t LED_address = (uint64_t) virtual_lw_base + 0x3000;
    *(uint32_t *) LED_address = 0x20; // Light the LED to show the way
    volatile uint32_t* BTNs = (uint32_t *) BTN_address;
    
    for (int i = 0; i < 8; i++){
    	while ((*BTNs) == 0){
        	cout << "\nButton value " << hex << *BTNs;
        	usleep(500000);
    	}
    	if ((*BTNs) & 0x1) break;
    	*(uint32_t *) LED_address = 0x28; // Light the LED to show the way
    	usleep(5000000);
    	cout << "\nTaking photo";
    
	for (int base_choice = 0; base_choice < 4; base_choice++) {
	        uint32_t base_address;
	        if (base_choice == 0){
	            base_address = 0x30C00000;
	        } else     if (base_choice == 1){
	            base_address = 0x30800000;
	        } else     if (base_choice == 2){
	            base_address = 0x30400000;
	        } else     if (base_choice == 3){
	            base_address = 0x30000000;
	        } else     if (base_choice == 4){
	            base_address = 0x31000000;
	        } else     if (base_choice == 5){
	            base_address = 0x31400000;
	        } else     if (base_choice == 6){
	            base_address = 0x31800000;
	        } else     if (base_choice == 7){
	            base_address = 0x31C00000;
	        } else     {
	            cout << "\nBAD!";
	            return 0;
	        }    
	    
	        void *virtual_base;
	        virtual_base = mmap( NULL, 0x600000, ( PROT_READ | PROT_WRITE ), MAP_SHARED, m_file_mem, base_address );
	    
	        Mat image;
	        image = Mat(960, 960, CV_16UC1, virtual_base);
	        Mat color_image = Mat(image.size(), CV_8UC3);
	        if (base_choice < 4){
	            for (int y = 0; y < image.rows; y++){
	                for (int x = 0; x < image.cols; x++){
	                    uint16_t pix = image.at<ushort>(Point(x, y));
	                    color_image.at<Vec3b>(Point(x, y)) = Vec3b((pix >> 8) & 0xF8, (pix >> 3) & 0xFC, (pix << 3) & 0xF8);
	                }
	            }
	        }
	        stringstream filename;
	        filename << "/home/root/fbmount/saved_imgs/img_" << i << "_cam" << (base_choice + cam_offset) << ".png";
	        imwrite(filename.str(), color_image);
    	}
    	
    	usleep(5000000);
    	*(uint32_t *) LED_address = 0x20; // Light the LED to show the way
    	while (*BTNs){
        	cout << "\nButton value " << hex << *BTNs;
        	usleep(500000);
    	}
    	usleep(5000000);
    	
    }	
    
    *(uint32_t *) LED_address = 0x00; // Unlight the LED
}
