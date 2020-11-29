#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include "i2c2regs.h"
#include "ar0330.h"
#include <fcntl.h>
#include <iostream>
#include <sys/mman.h>
#include <linux/kernel.h>
#include <algorithm>

using namespace std;

#define DIV_ROUND_CLOSEST(x, divisor) (x+(divisor/2)) / divisor
#define DIV_ROUND_UP(x, divisor) (x + (divisor - 1)) / divisor

#define AR0330_ADDRESS 0x10
#define AR0330_MIN_EXT_CLK_HZ				6000000
#define AR0330_MAX_EXT_CLK_HZ				27000000
#define AR0330_MIN_VCO_CLK_HZ				384000000
#define AR0330_MAX_VCO_CLK_HZ				768000000
#define AR0330_MIN_PLL_MULT				32
#define AR0330_MAX_PLL_MULT				255
#define AR0330_MAX_PRE_PLL_CLK_DIV			64
/* pix array limits */
#define AR0330_PIXEL_ARRAY_WIDTH			2316
#define AR0330_PIXEL_ARRAY_HEIGHT			1555
#define AR0330_WINDOW_WIDTH_MIN				32
#define AR0330_WINDOW_WIDTH_DEF				2052//2304
#define AR0330_WINDOW_WIDTH_MAX				2304
#define AR0330_WINDOW_HEIGHT_MIN			32
#define AR0330_WINDOW_HEIGHT_DEF			1540//1544
#define AR0330_WINDOW_HEIGHT_MAX			1544
/* AR0330 Registers */
#define AR0330_CHIP_VERSION				0x3000
#define		AR0330_CHIP_VERSION_VALUE		0x2604
#define AR0330_Y_ADDR_START				0x3002
#define 	AR0330_Y_ADDR_START_MIN			6
#define AR0330_X_ADDR_START				0x3004
#define AR0330_X_ADDR_START_MIN				6
#define AR0330_Y_ADDR_END				0x3006
#define AR0330_Y_ADDR_END_MAX				1549
#define AR0330_X_ADDR_END				0x3008
#define AR0330_X_ADDR_END_MAX				2309
#define AR0330_FRAME_LENGTH_LINES			0x300a
#define AR0330_LINE_LENGTH_PCK				0x300c
#define AR0330_CHIP_REVISION				0x300e
#define		AR0330_CHIP_REVISION_1x			0x10
#define		AR0330_CHIP_REVISION_2x			0x20
#define AR0330_LOCK_CONTROL				0x3010
#define		AR0330_LOCK_CONTROL_UNLOCK		0xbeef
#define AR0330_COARSE_INTEGRATION_TIME			0x3012
#define AR0330_RESET					0x301a
#define		AR0330_RESET_SMIA_DIS			(1 << 12)
#define		AR0330_RESET_FORCE_PLL_ON		(1 << 11)
#define		AR0330_RESET_RESTART_BAD		(1 << 10)
#define		AR0330_RESET_MASK_BAD			(1 << 9)
#define		AR0330_RESET_GPI_EN			(1 << 8)
#define		AR0330_RESET_PARALLEL_EN		(1 << 7)
#define		AR0330_RESET_DRIVE_PINS			(1 << 6)
#define		AR0330_RESET_LOCK_REG			(1 << 3)
#define		AR0330_RESET_STREAM			(1 << 2)
#define		AR0330_RESET_RESTART			(1 << 1)
#define		AR0330_RESET_RESET			(1 << 0)
/* AR03303_MODE_SELECT is an alias for AR0330_RESET_STREAM */
#define AR0330_MODE_SELECT				0x301c
#define		AR0330_MODE_SELECT_STREAM		(1 << 0)
#define AR0330_VT_PIX_CLK_DIV				0x302a
#define AR0330_VT_SYS_CLK_DIV				0x302c
#define AR0330_PRE_PLL_CLK_DIV				0x302e
#define AR0330_PLL_MULTIPLIER				0x3030
#define AR0330_OP_PIX_CLK_DIV				0x3036
#define AR0330_OP_SYS_CLK_DIV				0x3038
#define AR0330_FRAME_COUNT				0x303a
#define AR0330_READ_MODE				0x3040
#define		AR0330_READ_MODE_VERT_FLIP		(1 << 15)
#define		AR0330_READ_MODE_HORIZ_MIRROR		(1 << 14)
#define		AR0330_READ_MODE_COL_BIN		(1 << 13)
#define		AR0330_READ_MODE_ROW_BIN		(1 << 12)
#define		AR0330_READ_MODE_COL_SF_BIN		(1 << 9)
#define		AR0330_READ_MODE_COL_SUM		(1 << 5)
#define	AR0330_EXTRA_DELAY				0x3042
#define AR0330_GREEN1_GAIN				0x3056
#define AR0330_BLUE_GAIN				0x3058
#define AR0330_RED_GAIN					0x305a
#define AR0330_GREEN2_GAIN				0x305c
#define AR0330_GLOBAL_GAIN				0x305e
#define		AR0330_GLOBAL_GAIN_MIN			0
#define		AR0330_GLOBAL_GAIN_DEF			1000
#define		AR0330_GLOBAL_GAIN_MAX			15992
#define AR0330_ANALOG_GAIN				0x3060
#define AR0330_SMIA_TEST				0x3064
#define		 AR0330_EMBEDDED_DATA			(1 << 8)
#define AR0330_DATAPATH_SELECT				0x306e
#define		AR0330_DATAPATH_SLEW_DOUT_MASK		(7 << 13)
#define		AR0330_DATAPATH_SLEW_DOUT_SHIFT		13
#define		AR0330_DATAPATH_SLEW_PCLK_MASK		(7 << 10)
#define		AR0330_DATAPATH_SLEW_PCLK_SHIFT		10
#define		AR0330_DATAPATH_HIGH_VCM		(1 << 9)
#define AR0330_TEST_PATTERN_MODE			0x3070
#define AR0330_TEST_DATA_RED				0x3072
#define AR0330_TEST_DATA_GREENR				0x3074
#define AR0330_TEST_DATA_BLUE				0x3076
#define AR0330_TEST_DATA_GREENB				0x3078
#define AR0330_SEQ_DATA_PORT				0x3086
#define AR0330_SEQ_CTRL_PORT				0x3088
#define AR0330_X_ODD_INC				0x30a2
#define AR0330_Y_ODD_INC				0x30a6
#define AR0330_X_ODD_INC				0x30a2
#define AR0330_DIGITAL_CTRL				0x30ba
#define		AR0330_DITHER_ENABLE			(1 << 5)
/* Note from Developer Guide Version E :
 * The original AR0330 Rev2.0 samples did not have R0x300E
 * programmed correctly
 * Therefore reading register 0x30F0 is the only sure method
 * to get chip revision : 0x1200 indicates Rev1 while 0x1208
 * indicates Rev2.X
 */
#define AR0330_RESERVED_CHIPREV				0x30F0
#define AR0330_DATA_FORMAT_BITS				0x31ac
#define AR0330_SERIAL_FORMAT				0x31ae
#define AR0330_FRAME_PREAMBLE				0x31b0
#define AR0330_LINE_PREAMBLE				0x31b2
#define AR0330_MIPI_TIMING_0				0x31b4
#define AR0330_MIPI_TIMING_1				0x31b6
#define AR0330_MIPI_TIMING_2				0x31b8
#define AR0330_MIPI_TIMING_3				0x31ba
#define AR0330_MIPI_TIMING_4				0x31bc
#define AR0330_MIPI_CONFIG_STATUS			0x31be
#define AR0330_COMPRESSION				0x31d0
#define		AR0330_COMPRESSION_ENABLE		(1 << 0)
#define AR0330_POLY_SC					0x3780
#define		AR0330_POLY_SC_ENABLE			(1 << 15)

//=========================================
//New stuff
#define AR0330_ANALOG_GAIN_MIN 100
#define AR0330_ANALOG_GAIN_MAX 800
#define AR0330_ANALOG_GAIN_DEF 100

int main();
void reset_camera();
void configure_pinmux();
void I2C0_Init();
static int __ar0330_read(uint16_t reg, int size);
static int __ar0330_write(uint16_t reg, uint16_t value, int size);
static inline int ar0330_read8(uint16_t reg);
static inline int ar0330_write8(uint16_t reg, uint8_t value);
static inline int ar0330_read16(uint16_t reg);
static inline int ar0330_write16(uint16_t reg, uint16_t value);
static inline int ar0330_set16(uint16_t reg, uint16_t value, uint16_t mask);
static int ar0330_probe(struct ar0330 *ar0330, int fps);
static int ar0330_calc_vt(struct ar0330 *ar0330);
static int ar0330_pll_init(struct ar0330 *ar0330);
static int ar0330_s_stream(struct ar0330 *ar0330, int enable);
static int ar0330_power_on(struct ar0330 *ar0330);
static int ar0330_pll_configure(struct ar0330 *ar0330);
static int ar0330_set_params(struct ar0330 *ar0330);
static int ar0330_s_frame_interval(struct ar0330 *ar0330, int fps);
static int ar0330_set_exposure(struct ar0330 *ar0330);
static int ar0330_set_digital_gains(struct ar0330 *ar0330);
static int ar0330_set_analog_gain(struct ar0330 *ar0330);
static int ar0330_analog_gain_value(uint16_t reg, int gain_val);
static int ar0330_digital_gain_value(uint16_t reg, int gain);

struct v4l2_rect {
	int   left;
	int   top;
	int   width;
	int   height;
};
struct v4l2_fract {
	uint32_t   numerator;
	uint32_t   denominator;
};
struct ar0330_pll {
	uint16_t pre_pll_clk_div;
	uint16_t pll_multiplier;
	uint16_t vt_sys_clk_div;
	uint16_t vt_pix_clk_div;
	uint16_t op_sys_clk_div;
	uint16_t op_pix_clk_div;
	uint32_t clk_pix;
};
struct ar0330 {
	struct ar0330_platform_data	pdata;
	struct ar0330_pll 		    pll;
	uint32_t 			        version;
	/* Sensor window */
	struct v4l2_rect 		    crop;
	struct v4l2_rect 		    video_timing;
	uint32_t 				    x_binning;
	uint32_t 				    y_binning;
	struct v4l2_rect 	        format;
	struct v4l2_fract		    frame_interval;
	bool				        streaming;
    uint32_t                    exposure;
    uint32_t                    gains[4];
    uint32_t                    global_gain;
    uint32_t                    analog_gain;
	/* lock to protect power_count */
	int 				        power_count;
	/* Registers cache */
	uint16_t 				    read_mode;
};

int m_file_mem;
void *virtual_base;
uint32_t *i2c_status;

int main(){
	int foo;
	cout << "\nInitializing memory";
	m_file_mem = open( "/dev/mem", ( O_RDWR | O_SYNC ) );
	if (m_file_mem != -1){
		virtual_base = mmap( NULL, MAPPED_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, m_file_mem, MAPPED_BASE );
        i2c_status = (uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_STATUS);
	}
	
	ar0330 this_cam;
	
	this_cam.pdata.clk_rate = 6000000;//27000000; //external clock = 27MHz
	this_cam.pdata.max_vco_rate = 768000000;//AR0330_MAX_VCO_CLK_HZ;
	this_cam.pdata.max_op_clk_rate = 48000000;//98000000; //max output clock = 98MHz
	this_cam.pdata.reset = 0; //reset-ness status
	this_cam.pdata.hw_bus = AR0330_HW_BUS_PARALLEL; //reset-ness status
	this_cam.version = 5;
    
    this_cam.exposure = 50 * 1000; //100ms
    this_cam.gains[0] = 3000;//AR0330_GLOBAL_GAIN_DEF;
    this_cam.gains[1] = 3000;//AR0330_GLOBAL_GAIN_DEF;
    this_cam.gains[2] = 3000;//AR0330_GLOBAL_GAIN_DEF;
    this_cam.gains[3] = 3000;//AR0330_GLOBAL_GAIN_DEF;
    this_cam.global_gain = 1000;//AR0330_GLOBAL_GAIN_DEF;
    this_cam.analog_gain = 400;//AR0330_ANALOG_GAIN_DEF;
    this_cam.frame_interval.numerator = 1;
    this_cam.frame_interval.denominator = 10; //10fps
	
	cout << "\nResetting camera";
	reset_camera();
	
    cout << "\nInitializing pinmux";
	configure_pinmux();
	
	cout << "\nIntializing I2C";
	I2C0_Init();
    

	cout << "\nDebug: reading that thing";
	ar0330_read16(AR0330_FRAME_COUNT);

    if (1){
	cout << "\n\nProbing camera"; 
	ar0330_probe(&this_cam, 10);
	
	cout << "\n\nStarting stream";
	ar0330_s_stream(&this_cam, 1);

    	cout << "\n\nSetting frame interval";
    	ar0330_s_frame_interval(&this_cam, 10);
    
    	cout << "\n\nSetting exposure";
    	ar0330_set_exposure(&this_cam);
    }
     if (1) {
    	cout << "\n\nSetting digital gains";
    	ar0330_set_digital_gains(&this_cam);
    
    	cout << "\n\nSetting analog gain";
    	ar0330_set_analog_gain(&this_cam);
    }
	
	cout << "\nDebug: reading that thing";
	ar0330_read16(AR0330_FRAME_COUNT);

	return 0;
}

void reset_camera(){
    	uint32_t lw_base_address = 0xFF200000;
	void *virtual_lw_base;
	virtual_lw_base = mmap( NULL, 0x100000, ( PROT_READ | PROT_WRITE ), MAP_SHARED, m_file_mem, lw_base_address );
    	uint64_t LED_address = (uint64_t) virtual_lw_base + 0x3000;
    	*(uint32_t *) LED_address = 0x40; // Activate camera reset
    	usleep(500000);
    	*(uint32_t *) LED_address = 0x00; // Deactivate camera reset
    	usleep(500000);
}

void configure_pinmux(){
	*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) SYSMGR_I2C2USEFPGA) = 1;
}

void I2C0_Init(){
	// Abort any ongoing transmits and disable I2C0.
	*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_ENABLE) = 2;
	
	// Wait until I2C0 is disabled
	while(((*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_ENABLE_STATUS))&0x1) == 1){}
	
	// Configure the config reg with the desired setting (act as 
	// a master, use 7bit addressing, normal mode (100kb/s)).
	*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_CON) = 0x65;
	
	// Set target address (disable special commands, use 7bit addressing)
	*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_TAR) = AR0330_ADDRESS;
	
	// Set SCL high/low counts (Assuming default 100MHZ clock input to I2C0 Controller).
	// The minimum SCL high period is 0.6us, and the minimum SCL low period is 1.3us,
	// However, the combined period must be 2.5us or greater, so add 0.3us to each.
	*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_FS_SCL_HCNT) = 1000;//240 + 120; // 2.4us + 1.2us // was 10000 when working
	*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_FS_SCL_LCNT) = 1000;//130 + 30; // 1.3us + 0.3us
	
	// Enable the controller
	*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_ENABLE) = 1;
	
	// Wait until controller is enabled
	while(((*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_ENABLE_STATUS))&0x1) == 0){}
	
}

static int __ar0330_read(uint16_t reg, int size)
{
	cout << "\n* Reading " << hex << size << " bytes from " << reg;	
	// Send reg address (+0x400 to send START signal)
	*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_DATA_CMD) = ((uint32_t) ((reg >> 8) & 0xFF)) + 0x400;
	*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_DATA_CMD) = (uint32_t) (reg & 0xFF);
	
	// Restart (auto) then send read signal and stop bit to get 16 bits
    *(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_DATA_CMD) = 0x100;
    *(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_DATA_CMD) = 0x300;
	
	uint32_t data[2];
	
	// Read the response (first wait until RX buffer contains data)  
	while (*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_RXFLR) == 0){usleep(1000);}
	data[1] = *(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_DATA_CMD);
    while (*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_RXFLR) == 0){usleep(1000);}
    data[0] = *(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_DATA_CMD);
	cout << " received " << data[1] << " " << data[0];
    return (data[1] << 8) | data[0];
}
static int __ar0330_write(uint16_t reg, uint16_t value, int size)
{
	uint8_t data[2];
	int ret;
	if (size == 2) {
		data[0] = value >> 8;
		data[1] = value & 0xff;
	} else {
		data[0] = value & 0xff;
	}
	cout << "\n* Writing " << hex << size << " bytes " << value << " to " << reg;
	// Send reg address (+0x400 to send START signal)
	*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_DATA_CMD) = ((uint32_t) ((reg >> 8) & 0xFF)) + 0x400;
	*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_DATA_CMD) = (uint32_t) reg & 0xFF;
	
	// Send value
	uint32_t stop_mask = 0;
	if (size == 1){
        stop_mask = 0x200;
	}
	*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_DATA_CMD) = (data[0] & 0xFF) + stop_mask;
	if (size == 2) {
		*(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_DATA_CMD) = (data[1] & 0xFF) + 0x200;
	}
	while ((*i2c_status & 0x4) == 0){ usleep(1000); }
	return 0;
}

static inline int ar0330_read8(uint16_t reg)
{
	return __ar0330_read(reg, 1);
}
static inline int ar0330_write8(uint16_t reg, uint8_t value)
{
	return __ar0330_write(reg, value, 1);
}
static inline int ar0330_read16(uint16_t reg)
{
	return __ar0330_read(reg, 2);
}
static inline int ar0330_write16(uint16_t reg, uint16_t value)
{
	return __ar0330_write(reg, value, 2);
}
static inline int ar0330_set16(uint16_t reg, uint16_t value,
				   uint16_t mask)
{
	cout << "\n* Setting " << reg << " to " << value << " with mask " << mask;
	int rval = ar0330_read16(reg);
	if (rval < 0)
		return rval;
	else
		return ar0330_write16(reg,
					 (rval & ~mask) | (value & mask));
}

static int ar0330_probe(struct ar0330 *ar0330, int fps)
{
	ar0330->read_mode = 0;
	ar0330->crop.width = AR0330_WINDOW_WIDTH_DEF;
	ar0330->crop.height = AR0330_WINDOW_HEIGHT_DEF;
	ar0330->crop.left = (AR0330_WINDOW_WIDTH_MAX - AR0330_WINDOW_WIDTH_DEF)
			  / 2;
	ar0330->crop.top = (AR0330_WINDOW_HEIGHT_MAX - AR0330_WINDOW_HEIGHT_DEF)
			 / 2;
	ar0330->format.width = AR0330_WINDOW_WIDTH_DEF;
	ar0330->format.height = AR0330_WINDOW_HEIGHT_DEF;
	/* FPS */
	ar0330->frame_interval.numerator =  1;
	ar0330->frame_interval.denominator = fps;//30; 20 for debug
	
    cout << "\nCalculating VT";
	ar0330_calc_vt(ar0330);
	int ret = ar0330_pll_init(ar0330);
	return ret;
}

static int ar0330_calc_vt(struct ar0330 *ar0330)
{
	struct v4l2_rect *vt = &ar0330->video_timing;
	struct v4l2_rect *crop = &ar0330->crop;
	struct v4l2_rect *fmt = &ar0330->format;
	uint32_t adc_readout = 1242;
	/* Row and column binning. */
	cout << "\nCrop wh: " << dec << crop->width << ", " << crop->height;
	cout << "\nFMT wh: " << fmt->width << ", " << fmt->height << hex;
	ar0330->x_binning = DIV_ROUND_CLOSEST(crop->width,
						fmt->width) * 2 - 1;
	ar0330->y_binning = DIV_ROUND_CLOSEST(crop->height,
						fmt->height) * 2 - 1;
	/* The line length can be limited by 3 factors :
	 * -Adc readout limitation :
	 *	The datasheet version D indicates 1204 for
	 *	ADC high-speed = 0 and 1116 else.
	 *	ADC high-speed seems to correspond to register
	 *	digital_ctrl which enables dithering after digital gain
	 *	Sequencer B has a default value of 1242 but nothing
	 *	is said about what it is when enabling ADC high speed
	 * - Digital readout limitation
	 *	1/3*(x_addr_end-x_addr_start)/((x_odd_inc +1)*0.5)
	 * - Output interface limitation
	 *	1/2*(x_add_end-x_addr_start)/((x_odd_inc+1)*0.5) + 96
	 *
	 * it is obvious that Digital readout limitation is always inferior to
	 * the output interface
	 */
	cout << "\nChip version: " << ar0330->version;
	if (ar0330->version < 2)
		adc_readout = 1204;
	else
		adc_readout = 1242;
	/* for config examples to fit in, width should replace
	 * x_addr_end-x_addr_start which is an odd number */
	vt->width = max(adc_readout,
			crop->width / (ar0330->x_binning + 1) + 96);
	vt->height = crop->height /
			((ar0330->y_binning + 1) / 2) + 12;
	cout << "\nCalculated VT";
	return 0;
}

static int ar0330_pll_init(struct ar0330 *ar0330)
{
	uint32_t ext_clk = ar0330->pdata.clk_rate;
	uint32_t max_vco = ar0330->pdata.max_vco_rate;
	uint32_t op_clk = ar0330->pdata.max_op_clk_rate;
	uint32_t vco_clk, i;
	uint16_t pre_pll_div, pll_mult, vt_sys_clk_div, vt_pix_clk_div;
	uint16_t op_pix_clk_div;
	if ((ext_clk < AR0330_MIN_EXT_CLK_HZ) ||
			(ext_clk > AR0330_MAX_EXT_CLK_HZ)) {
		//v4l2_err(&ar0330->subdev, "ext_clk = %u, out of range\n", ext_clk);
		return -EINVAL;
	}
	if ((max_vco < AR0330_MIN_VCO_CLK_HZ) ||
			(max_vco > AR0330_MAX_VCO_CLK_HZ)) {
		//v4l2_err(&ar0330->subdev, "max_vco = %u, out of range\n", max_vco);
		return -EINVAL;
	}
	if (op_clk == 0) {
		//v4l2_err(&ar0330->subdev, "the op_clk must be specified\n");
		return -EINVAL;
	}
		cout << "\nFinding divider combo";
	/* heuristics to find a divider combination that provides the highest
	 * vco_clk inferior to max_vco_rate with the specified ext_clk and
	 * the limits of the pll div and mult */
	for(vco_clk = max_vco; vco_clk >= AR0330_MIN_VCO_CLK_HZ; vco_clk--) {
		i = __gcd(vco_clk, ext_clk);
		pre_pll_div = ext_clk / i;
		pll_mult = vco_clk / i;
		if ((pll_mult <= AR0330_MAX_PLL_MULT) &&
			(pll_mult >= AR0330_MIN_PLL_MULT) &&
			(pre_pll_div <= AR0330_MAX_PRE_PLL_CLK_DIV)) {
			cout << "\nfound new vco_clk " << dec << vco_clk << hex;
			break;
		}
	}
	if (vco_clk <= AR0330_MIN_VCO_CLK_HZ) {
		cout << "\nno vco_clk found";
		return -EINVAL;
	}
	/* in Parallel mode, there is no need to use vt_sys_clk_div */
    vt_pix_clk_div = DIV_ROUND_UP(vco_clk, op_clk);
    vt_sys_clk_div = 1;
    op_clk = vco_clk / vt_pix_clk_div;
    op_pix_clk_div = vt_pix_clk_div;
	
	ar0330->pll.pre_pll_clk_div = pre_pll_div;
	ar0330->pll.pll_multiplier  = pll_mult;
	ar0330->pll.vt_sys_clk_div  = vt_sys_clk_div;
	ar0330->pll.vt_pix_clk_div  = vt_pix_clk_div;
	ar0330->pll.op_sys_clk_div  = 1; //constant
	ar0330->pll.op_pix_clk_div  = op_pix_clk_div;
	ar0330->pll.clk_pix = op_clk / 2;
	cout << "\nDivider: " << dec << pre_pll_div;
	cout << "\nMultiplier: " << pll_mult;
	cout << "\nPix clock: " << op_clk / 2 << hex;
	return 0;
}

static int ar0330_s_stream(struct ar0330 *ar0330, int enable)
{
	cout << "\nReading frame count";
	ar0330_read16( AR0330_FRAME_COUNT );
	usleep(100000);
	cout << "\nI2c status " << *(uint32_t *) ((uint64_t) virtual_base + (uint64_t) I2C0_STATUS);
	//ar0330_read16( AR0330_FRAME_COUNT);
    
	if (!enable) {
		cout << "\nDisabling Stream";
		ar0330->streaming = 0;
		ar0330_write8(AR0330_MODE_SELECT, 0);
		return 0;
	}
	cout << "\n\nCalculating VT";
	ar0330_calc_vt(ar0330);

	cout << "\n\nPowering on";
	ar0330_power_on(ar0330);
    
	cout << "\nApplying Patch";
	ar0330_write16(0x3ed2, 0x0146);
	ar0330_write16(0x3eda, 0x88bc);
	ar0330_write16(0x3edc, 0xaa63);
	ar0330_write16(0x305e, 0x00a0);
    
	cout << "\n\nConfiguring PLL";
	int ret = ar0330_pll_configure(ar0330);

	cout << "\n\nSetting parameters";
	ret = ar0330_set_params(ar0330);
	
	ar0330->streaming = 1;
	cout << "\nDebug: 0x4 is mode select, should be zero";
	ar0330_read16(AR0330_RESET);
	usleep(50000);
	
	ar0330_write8(AR0330_MODE_SELECT,
			0x1); // undow
	
	cout << "\nDebug: 0x4 is mode select, should now be one";
	ar0330_read16(AR0330_RESET);
	usleep(50000); //debug
    
	//cout << "\nDebug: i2c status " << hex << *i2c_status;
	//cout << "\nDebug: reading that thing";
	//ar0330_read16(AR0330_FRAME_COUNT);
    
	return 0;
}

static int ar0330_power_on(struct ar0330 *ar0330)
{
	int ret=0;
    
	cout << "\nAsserting reset";
	
	ret = ar0330_set16(AR0330_RESET, AR0330_RESET_RESET,
					   AR0330_RESET_RESET);
	usleep(200 * 1000); // wait at least 150000 EXTCLK periods, which at 6MHz is 25ms
	
	//Debug: trying this cuz the datasheet said to
	//ar0330_write16( 0x3152, 0xA114 );
	//ar0330_write16( 0x304A, 0x0070 );
	//usleep(100 * 1000); //150000 EXTCLK
    
	ret = ar0330_write16(AR0330_SERIAL_FORMAT, 0x0301);
	usleep(50 * 1000); //debug
   
	cout << "\nDebug: reading reset register";
	ar0330_read16(AR0330_RESET);
	usleep(50000);
	
	cout << "\nWriting reset stream";
	ret = ar0330_set16(AR0330_RESET, ~AR0330_RESET_STREAM,
			AR0330_RESET_STREAM);
	usleep(50000);
	
	cout << "\nWriting all the resets";
	ret = ar0330_set16(AR0330_RESET,
			AR0330_RESET_DRIVE_PINS |
			AR0330_RESET_PARALLEL_EN |
			~AR0330_RESET_GPI_EN |
			AR0330_RESET_SMIA_DIS,
			AR0330_RESET_DRIVE_PINS |
			AR0330_RESET_PARALLEL_EN |
			AR0330_RESET_GPI_EN |
			AR0330_RESET_SMIA_DIS);
	usleep(100000); //debug
	
	cout << "\nWriting embedded data";
	ret = ar0330_set16(AR0330_SMIA_TEST, AR0330_EMBEDDED_DATA,
			AR0330_EMBEDDED_DATA);
    
	return 0;
}

static int ar0330_pll_configure(struct ar0330 *ar0330)
{
	int ret;
	ret = ar0330_write16(AR0330_VT_PIX_CLK_DIV,
				 ar0330->pll.vt_pix_clk_div);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(AR0330_VT_SYS_CLK_DIV,
				 ar0330->pll.vt_sys_clk_div);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(AR0330_PRE_PLL_CLK_DIV,
				 ar0330->pll.pre_pll_clk_div);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(AR0330_PLL_MULTIPLIER,
				 ar0330->pll.pll_multiplier);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(AR0330_OP_PIX_CLK_DIV,
				 ar0330->pll.op_pix_clk_div);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(AR0330_OP_SYS_CLK_DIV,
				 ar0330->pll.op_sys_clk_div);
	if (ret < 0)
		return ret;
	usleep(1500);
	return 0;
}

static int ar0330_set_params(struct ar0330 *ar0330)
{
	const struct v4l2_rect *crop = &ar0330->crop;
	struct v4l2_rect *vt  = &ar0330->video_timing;
	int ret;
	/* In parallel mode, there is no need to configure timings */
	/* Windows position and size.
	 *
	 * TODO: Make sure the start coordinates and window size match the
	 * skipping and mirroring requirements.
	 */
	ret = ar0330_write16(AR0330_X_ADDR_START,
			crop->left);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(AR0330_Y_ADDR_START,
			crop->top);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(AR0330_X_ADDR_END,
			crop->left +
			crop->width - 1);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(AR0330_Y_ADDR_END,
			crop->top +
			crop->height - 1);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(AR0330_X_ODD_INC,
			ar0330->x_binning);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(AR0330_Y_ODD_INC,
			ar0330->y_binning);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(AR0330_LINE_LENGTH_PCK,
				 vt->width);
	if (ret < 0)
		return ret;
	ret = ar0330_write16(AR0330_FRAME_LENGTH_LINES,
				 vt->height);
	if (ret < 0)
		return ret;
	ar0330_set16(AR0330_DIGITAL_CTRL, ~AR0330_DITHER_ENABLE,
			 AR0330_DITHER_ENABLE);
	if (ret < 0)
		return ret;
	return ret;
}

static int ar0330_s_frame_interval(struct ar0330 *ar0330,
				     int fps)
{
	struct v4l2_rect  *vt       = &ar0330->video_timing;
	struct v4l2_rect  *c        = &ar0330->crop;
	struct v4l2_fract *cur_fi   = &ar0330->frame_interval;
	uint32_t min_vt_height;
	cur_fi->denominator = (uint32_t) fps;
	if (!ar0330->streaming)
		return 0;
	/* We are already streaming, so we try to adjust the vertical blanking
	 * in order to match the frame rate.
	 */
	vt->height = ((uint64_t) ar0330->pll.clk_pix * cur_fi->numerator) /
			(vt->width * cur_fi->denominator);
	/* In case min_vt_frame_blanking is not met, we adjust the frame rate */
	min_vt_height = c->height / ((ar0330->y_binning + 1) / 2) + 12;
	if (vt->height < min_vt_height) {
		vt->height = min_vt_height;
		/* Refresh FPS */
		cur_fi->denominator = ar0330->pll.clk_pix;
		cur_fi->numerator   = vt->width * vt->height;
	}
	ar0330_write16(AR0330_FRAME_LENGTH_LINES,
			vt->height);
	return 0;
}

static int ar0330_set_exposure(struct ar0330 *ar0330){
    struct v4l2_rect *vt = &ar0330->video_timing;
	uint32_t *exposure_us = &ar0330->exposure;
	uint32_t line_duration_ns, exposure_max_us;
	uint16_t coarse;
	int ret;
	/* If not streaming, just keep interval structures up-to-date */
	if (!ar0330->streaming){
        	cout << "\nNot streaming so exposure not setted";
		return 0;
	}
    /* Exposure is expressed in us */
    exposure_max_us = ((uint64_t) vt->width * vt->height *
            1000000) /
            ar0330->pll.clk_pix;
    if (*exposure_us > exposure_max_us) {
        cout << "requested exposure " << dec << *exposure_us << " is higher than exposure max " << exposure_max_us << hex;
        *exposure_us = exposure_max_us;
    }
    line_duration_ns = ((uint64_t) vt->width * 1000000000) /
            ar0330->pll.clk_pix;
    coarse = (*exposure_us * 1000) / line_duration_ns;
    return ar0330_write16(AR0330_COARSE_INTEGRATION_TIME,
            coarse);
}

static int ar0330_set_digital_gains(struct ar0330 *ar0330){
    static const uint16_t gains[4] = {
		AR0330_RED_GAIN, AR0330_GREEN1_GAIN,
		AR0330_GREEN2_GAIN, AR0330_BLUE_GAIN
	};
    int ret = 0;
    /* Update the gain controls. */
    for (int i = 0; i < 4; ++i) {
        uint32_t gain_val = ar0330->gains[i];
        ret = ar0330_digital_gain_value(gains[i],
                gain_val);
    }
    ar0330_digital_gain_value(AR0330_GLOBAL_GAIN,
            ar0330->global_gain);
    return 0;
}

static int ar0330_set_analog_gain(struct ar0330 *ar0330){
    return ar0330_analog_gain_value(AR0330_ANALOG_GAIN, ar0330->analog_gain);
}

static int ar0330_analog_gain_value(uint16_t reg, int gain_val)
{
	if (100 <= gain_val && gain_val < 103)
		return ar0330_write16(0x3060, 0x0000);
	else if (103 <= gain_val && gain_val < 107)
		return ar0330_write16(0x3060, 0x0001);
	else if (107 <= gain_val && gain_val < 110)
		return ar0330_write16(0x3060, 0x0002);
	else if (110 <= gain_val && gain_val < 114)
		return ar0330_write16(0x3060, 0x0003);
	else if (114 <= gain_val && gain_val < 119)
		return ar0330_write16(0x3060, 0x0004);
	else if (119 <= gain_val && gain_val < 123)
		return ar0330_write16(0x3060, 0x0005);
	else if (123 <= gain_val && gain_val < 128)
		return ar0330_write16(0x3060, 0x0006);
	else if (128 <= gain_val && gain_val < 133)
		return ar0330_write16(0x3060, 0x0007);
	else if (133 <= gain_val && gain_val < 139)
		return ar0330_write16(0x3060, 0x0008);
	else if (139 <= gain_val && gain_val < 145)
		return ar0330_write16(0x3060, 0x0009);
	else if (145 <= gain_val && gain_val < 152)
		return ar0330_write16(0x3060, 0x000a);
	else if (152 <= gain_val && gain_val < 160)
		return ar0330_write16(0x3060, 0x000b);
	else if (160 <= gain_val && gain_val < 168)
		return ar0330_write16(0x3060, 0x000c);
	else if (168 <= gain_val && gain_val < 178)
		return ar0330_write16(0x3060, 0x000d);
	else if (178 <= gain_val && gain_val < 188)
		return ar0330_write16(0x3060, 0x000e);
	else if (188 <= gain_val && gain_val < 200)
		return ar0330_write16(0x3060, 0x000f);
	else if (200 <= gain_val && gain_val < 213)
		return ar0330_write16(0x3060, 0x0010);
	else if (213 <= gain_val && gain_val < 229)
		return ar0330_write16(0x3060, 0x0012);
	else if (229 <= gain_val && gain_val < 246)
		return ar0330_write16(0x3060, 0x0014);
	else if (246 <= gain_val && gain_val < 267)
		return ar0330_write16(0x3060, 0x0016);
	else if (267 <= gain_val && gain_val < 291)
		return ar0330_write16(0x3060, 0x0018);
	else if (291 <= gain_val && gain_val < 320)
		return ar0330_write16(0x3060, 0x001a);
	else if (320 <= gain_val && gain_val < 356)
		return ar0330_write16(0x3060, 0x001c);
	else if (356 <= gain_val && gain_val < 400)
		return ar0330_write16(0x3060, 0x001e);
	else if (400 <= gain_val && gain_val < 457)
		return ar0330_write16(0x3060, 0x0020);
	else if (457 <= gain_val && gain_val < 533)
		return ar0330_write16(0x3060, 0x0024);
	else if (533 <= gain_val && gain_val < 640)
		return ar0330_write16(0x3060, 0x0028);
	else if (640 <= gain_val && gain_val < 800)
		return ar0330_write16(0x3060, 0x002c);
	else if (800 >= gain_val )
		return ar0330_write16(0x3060, 0x0030);
	return -EINVAL;
}

static int ar0330_digital_gain_value(uint16_t reg, int gain)
{
	/* From datasheet AR0330 page 36
	 * The format of each digital gain register
	 * is “xxxx.yyyyyyy” */
	int xxxx, yyyyyyy;
	xxxx = gain / 1000;
	yyyyyyy = gain % 1000;
	yyyyyyy = (yyyyyyy * 128 + 500) / 1000;
	return ar0330_write16(reg, (xxxx << 7) | yyyyyyy);
}
