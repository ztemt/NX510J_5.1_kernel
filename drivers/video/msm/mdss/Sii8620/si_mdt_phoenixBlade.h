//#define PHOENIX_BLADE 				1
#define MDT_BUTTON_LEFT				1
#define MDT_BUTTON_RIGHT			2
#define MDT_BUTTON_MIDDLE			4

#define FIND_KEY_CONTROLLED_MOUSE_SIM_ON	1
#define FIND_KEY_CONTROLLED_MOUSE_SIM_OFF	0

#define TOUCH_SIMULATED				1
#define TOUCH_IGNORED				0
#define ONE_SECOND				1000

struct mdt_simulated_buttons_t {
	unsigned char left;
	unsigned char middle;
	unsigned char right;
};
/*
	unsigned char touchKey_home;	//These variables are exported into Cypress' driver
	unsigned char touchKey_menu;	//   for the native touch pad ( not tuch screen )
	unsigned char touchKey_back;	//The export enables PhoenixBlade to use the touch pad
	unsigned char touchKey_find;	//   as left, middle, and right mouse buttons
	unsigned char touchKey_changed;
	unsigned char oldTouchKey_find;
};
*/

enum double_touch_state_e {
	  MDT_PB_WAIT_FOR_TOUCH_RELEASE
	, MDT_PB_WAIT_FOR_1ST_TOUCH
	, MDT_PB_WAIT_FOR_1ST_TOUCH_RELEASE
	, MDT_PB_WAIT_FOR_2ND_TOUCH
	, MDT_PB_WAIT_FOR_2ND_TOUCH_RELEASE
};

enum phoenix_blade_demo_state_e {
	  MDT_PB_NATIVE_TOUCH_SCREEN
	, MDT_PB_SIMULATED_MOUSE_0
	, MDT_PB_SIMULATED_MOUSE_90
	, MDT_PB_SIMULATED_MOUSE_180
	, MDT_PB_SIMULATED_MOUSE_270
};


struct mdt_double_touch_t {
	enum double_touch_state_e	state;
	unsigned int			duration_release;
	unsigned int			duration_touch;
	unsigned int			last_release;
	unsigned int			last_touch;
};

unsigned char get_phoenix_blade_state(void);
unsigned char get_mdtdemo_virtual_btn(unsigned char button);
void set_mdtdemo_virtual_btn(unsigned char value, unsigned char do_sync);




