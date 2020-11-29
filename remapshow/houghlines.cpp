#include "opencv2/highgui/highgui.hpp"

#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/mman.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    	int m_file_mem;
	m_file_mem = open("/dev/mem", ( O_RDWR | O_SYNC ) );

	int base_choice = -1;
	cout << "\nBase choice? ";
	cin >> base_choice;
	uint32_t base_address;
	if (base_choice == 0){
		base_address = 0x30C00000;
	} else 	if (base_choice == 1){
		base_address = 0x30800000;
	} else 	if (base_choice == 2){
		base_address = 0x30400000;
	} else 	if (base_choice == 3){
		base_address = 0x30000000;
	} else 	if (base_choice == 4){
		base_address = 0x31000000;
	} else 	if (base_choice == 5){
		base_address = 0x31400000;
	} else 	if (base_choice == 6){
		base_address = 0x31800000;
	} else 	if (base_choice == 7){
		base_address = 0x31C00000;
	} else 	{
		cout << "\nBAD!";
		return 0;
	}	

	void *virtual_base;
	virtual_base = mmap( NULL, 0x600000, ( PROT_READ | PROT_WRITE ), MAP_SHARED, m_file_mem, base_address );

	//Color images are 960w x 1920h
	//Grey images are 720w x 480h
	
	Mat image;
	if (base_choice > 3){
		image = Mat(480, 768, CV_8UC1, virtual_base);
	} else {
		image = Mat(1920, 960, CV_16UC1, virtual_base);
	}
	
	Mat color_image = Mat(960, 480, CV_8UC3);

	Mat im_splits[4];
	split(image, im_splits);

	int* pixel_0 = (int *) virtual_base;
	cout << hex << "\nPixel 0: " << pixel_0[50000];
	cout << hex << "\nPixel 1: " << pixel_0[100000];
	cout << hex << "\nPixel 2: " << pixel_0[150000];
	
	int save_counter = 0;

	Mat image_color; //not rg or bg
	while (1){
		if (base_choice < 4){
			for (int y = 0; y < 960; y++){
				for (int x = 0; x < 480; x++){
					uint16_t pix = image.at<ushort>(Point(x*2, y*2));
					color_image.at<Vec3b>(Point(x, y)) = Vec3b((pix >> 8) & 0xF8, (pix >> 3) & 0xFC, (pix << 3) & 0xF8);
				}
			}
		}
			
		//cvtColor(image, image_color, COLOR_BayerGR2RGB);
		
		if (base_choice < 4){
			namedWindow("image", WINDOW_NORMAL);
			imshow("image", color_image);
			resizeWindow("image", 360, 720);
		} else {
			imshow("image", image);
		}
		
	        int c = waitKey(300);
	        if ((char) c == 27) {
	            break;
        	} else if ((char) c == 32) {
			string filename = "cam" + to_string(base_choice) + "_im" + to_string(save_counter) + ".bmp";
			imwrite(filename, image);
			cout << "\nWrote " << filename;
			cout << "\n";
			save_counter++;
		}
	}
}
