#include "opencv2/highgui/highgui.hpp"

#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <cstdlib>
#include <fstream>

using namespace cv;
using namespace std;

#define fr_cols 45
#define fr_rows 60
#define frame_width_deg 135.f
#define frame_height_deg 180.f
#define block_width_deg float(frame_width_deg / fr_cols / 2)
#define block_height_deg float(frame_height_deg / fr_rows)
#define param_scale float(1536.f / 960.f)
#define read_offset float(300)

static void help()
{
    cout << "\nThis program demonstrates line finding with the Hough transform.\n"
            "Usage:\n"
            "./houghlines <image_name>, Default is pic1.png\n" << endl;
}

struct camera_matrix {
    float origin_pt[3];
    vector <float> rot_angle;
    vector <float> center;
    float k1;
    float k2;
    float k3;
};

void load_camera_matrix(FileStorage &filestorage_in, camera_matrix &cam_mat_out, int cam_index);

int main(int argc, char** argv)
{
    int m_file_mem;
    m_file_mem = open("/dev/mem", ( O_RDWR | O_SYNC ) );
    
    uint32_t lw_base_address = 0xFF200000;
    void *virtual_lw_base;
    virtual_lw_base = mmap( NULL, 0x100000, ( PROT_READ | PROT_WRITE ), MAP_SHARED, m_file_mem, lw_base_address );
    uint64_t LED_address = (uint64_t) virtual_lw_base + 0x3000;
    *(uint32_t *) LED_address = 0x80; // Activate FPGA reset

    cout << "\nIntro";
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

    void *virtual_base_2;
    FileStorage cam_param_filestorage("/home/root/opencv/remaptest/cameras_out.ssf", FileStorage::READ);
    
    for (int i = 0; i < 4; i++){
        camera_matrix cam_mat;
        int cam_index = cam_offset + i;
        load_camera_matrix(cam_param_filestorage, cam_mat, cam_index);
        
        cam_mat.rot_angle.push_back(0.0f); // now zero for horizontal sensor, was 90.0f;
        cam_mat.rot_angle.push_back(90.0f); // was 90 when working betters
        cam_mat.rot_angle.push_back(-67.5f);

        /*cam_mat.center.push_back(1024);// center_x[i];
        cam_mat.center.push_back(700);// center_y[i];

        cam_mat.k1 = 15520;
        cam_mat.k2 = -25144;
        cam_mat.k3 = 828 * (1.4f / 2.2f);*/
        
        cout << "\nRotangles " << cam_mat.rot_angle[0] << " " << cam_mat.rot_angle[1] << " " << cam_mat.rot_angle[2];

        Vec3f this_pixel;

        Mat remap_mat = Mat(Size(fr_cols * 2 + 2, fr_rows + 1), CV_32FC2);
        
        /*float left_angle = cam_index * -45.f - 67.5f;
        float phi = 0 + left_angle;
        float z_angle = -1.f * cam_mat.rot_angle[2];
        phi += z_angle;
        cout << "\n";// Start phi: " << phi;*/

        for (int x = 0; x < fr_cols * 2 + 2; x++) { // Divide by 2 because horizontal is double res (to account for short blocks) plus 2 because end coordinates of last block must be known
            for (int y = 0; y < fr_rows + 1; y++) { // Plus one because end coordinates of the last block must be known
                float phi = -1.f * float(x) * block_width_deg;// + left_angle;// / 2.f;  // Divided by 2 because horizontal is done with double resolution
                float theta = float(y) * block_height_deg;
                
                int closest_cam_idx = 0;

                //rotate about z
                float z_angle = -1.f * cam_mat.rot_angle[2];
                phi += z_angle;
                if ((x == 0) && (y == 0)) cout << "\nStart phi " << phi;

                phi *= CV_PI / 180.f;
                theta *= CV_PI / 180.f;

                //convert to cartesian
                this_pixel[0] = sin(theta) * cos(phi);
                this_pixel[1] = sin(theta) * sin(phi);
                this_pixel[2] = cos(theta);
                
                /*if ((x == 0) && (y == (fr_rows / 2))) 
                cout << "\ntp l " << x_read
                 << "\ntp l " << y_read;*/

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
                //float ya = r * -1.f * cos(phi2); // Following the algorithm from the calibration_script.py script.
                float ya = r * 1.f * cos(phi2);

                xa += cam_mat.center[0];
                ya += cam_mat.center[1];
                

                float x_read = max(0.f, xa);
                x_read = min(2048.f, (x_read * param_scale) + read_offset); // Calibrated with half-sized images.
                //x_read = min(2048.f, x_read);
                float y_read = max(0.f, ya);
                y_read = min(1530.f, y_read * param_scale); // Calibrated with half-sized images.
                //y_read = min(1530.f, y_read); 
                remap_mat.at<Vec2f>(Point(x, y)) = Vec2f(x_read, y_read);
                if ((x == 0) && (y == (fr_rows / 2))) cout << "\nXA l " << x_read << "\nYA l " << y_read;
                if ((x == (fr_cols)) && (y == (fr_rows / 2))) cout << "\nXA center " << x_read << "\nYA center " << y_read;
                if ((x == (fr_cols * 2)) && (y == (fr_rows / 2))) cout << "\nXA r " << x_read << "\nYA r " << y_read;
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
                        //cout << "\nHalfsy";
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
            //cout << dec << "\nEnd x pos " << x_pos << " y " << y;
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
        
        virtual_base_2 = mmap( NULL, 0x20000, ( PROT_READ | PROT_WRITE ), MAP_SHARED, m_file_mem, 0x21000000 + ( 0x400000 * (3 - i) ) );
        for (int y = 0; y < fr_rows; y++) {
            for (int x = 0; x < fr_cols; x++) {
                uint64_t this_address = ((uint64_t) virtual_base_2 + (uint64_t) (x << 5) + (uint64_t) (y << 11));
                memcpy ((void*) this_address, (void*) &coord_blk_array[y][x].val_array[0], 20);
                //myfile.write((char*)coord_blk_array[y][x].val_array, 20);
            }
        }
    }
    
    usleep(500000);
    *(uint32_t *) LED_address = 0x02; // Deactivate FPGA reset and light an LED
}

void load_camera_matrix(FileStorage &filestorage_in, camera_matrix &cam_mat_out, int cam_index){
    char my_string[80];
    sprintf(my_string, "rot_angle_%d", cam_index);
    //filestorage_in[my_string] >> cam_mat_out.rot_angle;

    sprintf(my_string, "center_%d", cam_index);
    filestorage_in[my_string] >> cam_mat_out.center;
    //cam_mat.center[0] =  1024;// center_x[i];
    //cam_mat.center[1] =  700;// center_y[i];

    sprintf(my_string, "k1_%d", cam_index);
    filestorage_in[my_string] >> cam_mat_out.k1;
    sprintf(my_string, "k2_%d", cam_index);
    filestorage_in[my_string] >> cam_mat_out.k2;
    sprintf(my_string, "k3_%d", cam_index);
    filestorage_in[my_string] >> cam_mat_out.k3;
    /*cam_mat.k1 = 15520;
    cam_mat.k2 = -25144;
    cam_mat.k3 = 828 * (1.4f / 2.2f);*/
}