//#ifndef AR0330_H
//#define AR0330_H

struct clk;
struct v4l2_subdev;

enum {
	AR0330_HW_BUS_PARALLEL,
	AR0330_HW_BUS_MIPI,
	AR0330_HW_BUS_HISPI,
};


struct ar0330_platform_data {
	struct clk     *clock;
	unsigned int		clk_rate;
	unsigned int		max_vco_rate;
	unsigned int		max_op_clk_rate;
	int (*set_power)(int on);
	unsigned int	reset;
	unsigned int		hw_bus;
};

#define AR0330_POWER_ON 1
#define ARO330_POWER_OFF 0

//#endif
