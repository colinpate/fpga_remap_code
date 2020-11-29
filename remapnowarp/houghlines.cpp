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


#define fr_cols 45
#define fr_rows 30

static void help()
{
    cout << "\nThis program demonstrates line finding with the Hough transform.\n"
            "Usage:\n"
            "./houghlines <image_name>, Default is pic1.png\n" << endl;
}

struct camera_matrix {
	float origin_pt[3];
	float rot_angle[3];
	float center[2];
	float k1;
	float k2;
	float k3;
};

int main(int argc, char** argv)
{
    	int m_file_mem;
	m_file_mem = open("/dev/mem", ( O_RDWR | O_SYNC ) );
	
	uint32_t lw_base_address = 0xFF200000;
	void *virtual_lw_base;
	virtual_lw_base = mmap( NULL, 0x100000, ( PROT_READ | PROT_WRITE ), MAP_SHARED, m_file_mem, lw_base_address );
    	uint64_t LED_address = (uint64_t) virtual_lw_base + 0x3000;
    	*(uint32_t *) LED_address = 0x80; // Activate FPGA reset

	void *virtual_base_2;
	
	
	camera_matrix cam_mat;
	cam_mat.rot_angle[0] = 90.0f;
	cam_mat.rot_angle[1] = 90.0f;
	cam_mat.rot_angle[2] = 67.5f;

	cam_mat.center[0] =  1024;// center_x[i];
	cam_mat.center[1] =  700;// center_y[i];

	cam_mat.k1 = 15520;
	cam_mat.k2 = -25144;
	cam_mat.k3 = 828;

	Vec3f this_pixel;

	Mat remap_mat = Mat(Size(fr_cols * 2 + 2, fr_rows + 1), CV_32FC2);


	int last_row[fr_cols];
	int last_blk_short = 0;
	for (int i = 0; i < fr_cols; i++) {
		last_row[i] = 0;
	}

	struct CoordBlock {
		int16_t val_array[10];
	};

	CoordBlock coord_blk_array[fr_rows][fr_cols];

	//Backwards pass
	for (int y = 0; y < fr_rows; y++) {
		for (int x = 0; x < fr_cols; x++) {
	                //Output frame 1920w x 960h
	                //Input frame 2304w x 1536h
	                //Scale by Y
	                //X start for read = (x * 32) * (1536 / 1440)
	                //Y start for write = (y * 32) * (1536 / 1440)
	        	//If write X is more than 511 it'll get subtracted by 480
	        	float offset = 300;
	        	float read_x = float(x) * (32.f * 1536.f / 960.f) + offset;
	        	if (read_x >= 2048){
	        		read_x = 2048;
	        	}
			float up_l_x = float(read_x) * 4;
			float up_l_y = float(y) * (32.f * 1536.f  * 4.f / 960.f);
	
			float up_r_x = (32.f * 1536.f  * 4.f / 960.f); //End of line relative to start (first row)
			float up_r_y = 0;
	
			float low_l_x = 0; //Start of line relative to start (last row)
			float low_l_y = (32.f * 1536.f  * 4.f / 960.f);
	
			float low_r_x = 0; //Change in end of line from first row to last
			float low_r_y = 0;
	
			int16_t* val_array;
	
			val_array = &coord_blk_array[last_row[x]][x].val_array[0];
			last_row[x] += 1;
	
			//start x
			val_array[0] = up_l_x;
			//start y
			val_array[1] = up_l_y;
			//end x
			val_array[2] = up_r_x;
			//end y
			val_array[3] = up_r_y;
			//start x inc
			val_array[4] = low_l_x;
			//start y inc
			val_array[5] = low_l_y;
			//end x inc
			val_array[6] = low_r_x;
			//end y inc
			val_array[7] = low_r_y;
	
			int write_x = x;
			if (x > 0){ // >= 32 rows
	    			write_x += 15; //Add 480 rows
			}
			val_array[8] = (int16_t)((y << 8) + (write_x * 4)); //Y is top 8 bits and is row / 32, x is bottom 8 and is col / 8
	
			val_array[9] = (int16_t)((1 << 3)); //Hella wide (Read 32 pixels)
			//if ((y > 14) && (y < 30)) val_array[9] |= 1; //Grayscale only
		}
	}

	//Say these are the last blocks
	for (int x = 0; x < fr_cols; x++) {
		coord_blk_array[last_row[x] - 1][x].val_array[9] |= 2;
	}
    
    	for (int i = 0; i < 4; i++){
    		virtual_base_2 = mmap( NULL, 0x20000, ( PROT_READ | PROT_WRITE ), MAP_SHARED, m_file_mem, 0x21000000 + (i * 0x400000) );
		for (int y = 0; y < fr_rows; y++) {
			for (int x = 0; x < fr_cols; x++) {
	                    uint64_t this_address = ((uint64_t) virtual_base_2 + (uint64_t) (x << 5) + (uint64_t) (y << 11));
	                    memcpy ((void*) this_address, (void*) &coord_blk_array[y][x].val_array[0], 20);
				//myfile.write((char*)coord_blk_array[y][x].val_array, 20);
			}
		}
	}	
	
	usleep(50000);
	*(uint32_t *) LED_address = 0x06; // Deactivate FPGA reset and light an LED
}
