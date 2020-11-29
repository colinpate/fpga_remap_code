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
#define fr_rows 60
#define frame_width_deg 135.f
#define frame_height_deg 180.f
#define block_width_deg float(frame_width_deg / fr_cols / 2)
#define block_height_deg float(frame_height_deg / fr_rows)

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
    virtual_base_2 = mmap( NULL, 0x20000, ( PROT_READ | PROT_WRITE ), MAP_SHARED, m_file_mem, 0x21000000 );
    
    camera_matrix cam_mat;
    cam_mat.rot_angle[0] = 0.0f; // now zero for horizontal sensor, was 90.0f;
    cam_mat.rot_angle[1] = 90.0f; // was 90 when working betters
    cam_mat.rot_angle[2] = 67.5f;

    cam_mat.center[0] =  1024;// center_x[i];
    cam_mat.center[1] =  700;// center_y[i];

    cam_mat.k1 = 15520;
    cam_mat.k2 = -25144;
    cam_mat.k3 = 828 * (1.4f / 2.2f);

    Vec3f this_pixel;

    Mat remap_mat = Mat(Size(fr_cols * 2 + 2, fr_rows + 1), CV_32FC2);

    for (int x = 0; x < fr_cols * 2 + 2; x++) { // Divide by 2 because horizontal is double res (to account for short blocks) plus 2 because end coordinates of last block must be known
        for (int y = 0; y < fr_rows + 1; y++) { // Plus one because end coordinates of the last block must be known
            //float phi = float(y) * 3; cuz 135 degree tall frame and 45 rows
            //float theta = float(x) * 1.5f; cuz 90 degree wide frame and 60 cols
            
            float phi = float(x) * block_width_deg;// / 2.f;  // Divided by 2 because horizontal is done with double resolution
            float theta = float(y) * block_height_deg;
            
            int closest_cam_idx = 0;

            //rotate about z
            float z_angle = -1.f * cam_mat.rot_angle[2];
            phi += z_angle;

            phi *= CV_PI / 180.f;
            theta *= CV_PI / 180.f;

            //convert to cartesian
            this_pixel[0] = sin(theta) * cos(phi);
            this_pixel[1] = sin(theta) * sin(phi);
            this_pixel[2] = cos(theta);

            //rotate about y
            float y_angle = -1.f * cam_mat.rot_angle[1] * CV_PI / 180.0f;
            float z2 = this_pixel[2];
            float x2 = this_pixel[0];
            this_pixel[2] = z2 * cos(y_angle) - x2 * sin(y_angle);
            this_pixel[0] = z2 * sin(y_angle) + x2 * cos(y_angle);

            //convert to polar
            float phi2 = atan2f(this_pixel[1], this_pixel[0]);
            float theta2 = atan2f(pow(pow(this_pixel[1], 2) + pow(this_pixel[0], 2), 0.5), this_pixel[2]);
            //rotate about z
            phi2 -= cam_mat.rot_angle[0] * CV_PI / 180.0f;

            //convert to x, y
            float k1a = cam_mat.k1 / (10.f * 1000.f);
            float k2a = cam_mat.k2 / (100.f * 1000.f);

            float r = cam_mat.k3 * (k1a * theta2 + k2a * pow(theta2, 3));
            float xa = r * -1.f * sin(phi2);
            float ya = r * cos(phi2);

            //xa *= (1.4f / 2.2f); Removed cuz now k3 is scaled
            //ya *= (1.4f / 2.2f);

            xa += cam_mat.center[0];
            ya += cam_mat.center[1];

            float x_read = max(0.f, xa);
            x_read = min(2048.f, x_read);
            float y_read = max(0.f, ya);
            y_read = min(1530.f, y_read);
            remap_mat.at<Vec2f>(Point(x, y)) = Vec2f(x_read, y_read);
        }
    }

    // Clear out the last row array. 
    // This tells us which row is the last one occupied in each column so the buffer_line_calc module
    // knows when each frame column is done
    int last_row[fr_cols];
    for (int i = 0; i < fr_cols; i++) {
        last_row[i] = 0;
    }

    struct CoordBlock {
        int16_t val_array[10];
    };

    /* New methodology:
    Need to fill out the full frame L-R with one rule: each block in each column cannot have a read
    coordinate that is above the block before it.
    
    Current methodology: 
    back then forwards from middle, consume as many as possible
    
    New methodology:
    Starting from vertical center, go R-L and then L-R from horizontal center
    sfsef
    
    */
    
    vector <CoordBlock> coord_blk_vector;

    for (int y = 10; y < fr_rows - 10; y++) {
        int x_pos = 0;
        //int back_pass = 1; // Start moving backwards from the center, and consume as much of the image as possible
                           // with the available number of block columns
        while (x_pos < (fr_cols * 2)) {
            //if ((x_pos < 100) && (x_pos > 20)){ // Just cut off these arbitrarily
                int start_x;
                int end_x;
                int blk_width = 2;

                start_x = x_pos;
                end_x = x_pos + 2;
                
                float up_r_ydiff = remap_mat.at<Vec2f>(Point(end_x, y))[1] - remap_mat.at<Vec2f>(Point(start_x, y))[1];
                float dn_r_ydiff = remap_mat.at<Vec2f>(Point(end_x, y + 1))[1] - remap_mat.at<Vec2f>(Point(start_x, y + 1))[1];
                if ((min(up_r_ydiff, dn_r_ydiff) < -15) || (max(up_r_ydiff, dn_r_ydiff) > 15)) {
                    blk_width = 1;
                    start_x = x_pos;
                    end_x = x_pos + 1;
                    cout << "\nHalfsy";
                }

                float up_l_x = remap_mat.at<Vec2f>(Point(start_x, y))[0] * 4;
                float up_l_y = remap_mat.at<Vec2f>(Point(start_x, y))[1] * 4;

                float up_r_x = remap_mat.at<Vec2f>(Point(end_x, y))[0] * 4 - up_l_x;
                float up_r_y = remap_mat.at<Vec2f>(Point(end_x, y))[1] * 4 - up_l_y;

                float low_l_x = remap_mat.at<Vec2f>(Point(start_x, y + 1))[0] * 4 - up_l_x;
                float low_l_y = remap_mat.at<Vec2f>(Point(start_x, y + 1))[1] * 4 - up_l_y;

                float low_r_x = remap_mat.at<Vec2f>(Point(end_x, y + 1))[0] * 4 - up_l_x - up_r_x - low_l_x;
                float low_r_y = remap_mat.at<Vec2f>(Point(end_x, y + 1))[1] * 4 - up_l_y - up_r_y - low_l_y;

                CoordBlock this_coord_block;

                /*val_array = &coord_blk_array[last_row[x]][x].val_array[0];
                last_row[x] += 1;*/

                //start x
                this_coord_block.val_array[0] = up_l_x;
                //start y
                this_coord_block.val_array[1] = up_l_y;
                //end x
                this_coord_block.val_array[2] = up_r_x;
                //end y
                this_coord_block.val_array[3] = up_r_y;
                //start x inc
                this_coord_block.val_array[4] = low_l_x;
                //start y inc
                this_coord_block.val_array[5] = low_l_y;
                //end x inc
                this_coord_block.val_array[6] = low_r_x;
                //end y inc
                this_coord_block.val_array[7] = low_r_y;

                this_coord_block.val_array[8] = (int16_t)((y << 8) + (start_x * 2));

                this_coord_block.val_array[9] = (int16_t)((blk_width << 2));
                
                // Blocks are 16 pixels wide, the frame is 135 degrees = 1440 pixels wide
                // and the gray region is the middle third of the frame so the gray blocks are:
                // 1440 / 3 / 16 = 30 start
                // 1440 / 3 / 16 * 2 = 60 end
                if ((start_x > 29) && (start_x < 60)) this_coord_block.val_array[9] |= 1;
                
                int blk_inserted = 0;
                /*for (int i = 0; i < coord_blk_vector.size(); i++){
                    if (this_coord_block.val_array[1] < coord_blk_vector[i].val_array[1]){
                        coord_blk_vector.insert(coord_blk_vector.begin() + i, this_coord_block);
                        blk_inserted = 1;
                        break;
                    }
                }*/
                if (blk_inserted == 0){
                    coord_blk_vector.push_back(this_coord_block);
                }
                    
                x_pos = end_x;
            //}
        }
        cout << dec << "\nEnd x pos " << x_pos << " y " << y;
    }
    
    cout << "\nNumber of blocks: " << coord_blk_vector.size();
    cout << "\nNumber of available slots: " << (fr_cols * fr_rows);
    
    CoordBlock coord_blk_array[fr_rows][fr_cols];

    // Fill out the array with the sorted vector
    int vector_index = 0;
    for (int y = 0; y < fr_rows; y++){
        for (int x = 0; x < fr_cols; x++){
            if (vector_index < coord_blk_vector.size()){
                coord_blk_array[y][x] = coord_blk_vector[vector_index];
                last_row[x] = y;
                vector_index++;
            }
        }
    }

    //Say these are the last blocks in the col by setting the flag bit
    for (int x = 0; x < fr_cols; x++) {
        coord_blk_array[last_row[x]][x].val_array[9] |= 2;
    }
    
    for (int y = 0; y < fr_rows; y++) {
        for (int x = 0; x < fr_cols; x++) {
            uint64_t this_address = ((uint64_t) virtual_base_2 + (uint64_t) (x << 5) + (uint64_t) (y << 11));
            memcpy ((void*) this_address, (void*) &coord_blk_array[y][x].val_array[0], 20);
            //myfile.write((char*)coord_blk_array[y][x].val_array, 20);
        }
    }
    
    usleep(500000);
    *(uint32_t *) LED_address = 0x02; // Deactivate FPGA reset and light an LED
}
