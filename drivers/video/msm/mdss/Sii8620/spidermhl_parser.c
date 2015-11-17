
#include "sii8338_drv_mdt_tx.h"
#include "si_mdt_inputdev.h"
#include "sii8338_driver_spider.h"

#ifdef MDT_SUPPORT
extern struct g_mdt_t g_mdt;
#endif

// [START] HELIXTECH: KT_SPIDER_FEATURE ====================================
#ifdef CONFIG_SPIDER_MHL
#ifdef MDT_SUPPORT
struct sii8338_spider_data *g_sii8338;

struct sii8338_spider_data *spider_get_sii8338_data(void)
#else
static struct sii8338_data *spider_get_sii8338_data(void)
#endif
{
	return g_sii8338;
}
/*
#ifdef MDT_SUPPORT
static struct spider_event *spider_get_queue(struct sii8338_spider_data *sii8338)
#else
static struct spider_event *spider_get_queue(struct sii8338_data *sii8338)
#endif
{
	struct spider_event *event;

	dbg_enter();

	if (sii8338->qtail == sii8338->qhead) {
		pr_info("spider: %s: queue empty\n", __func__);
		return NULL;
	}

	event = &sii8338->eventq[sii8338->qhead];
	sii8338->qhead++;
	sii8338->qhead %= MAX_EVENT_QUEUE;

	dbg_leave();

	return event;
}
*/
#ifdef MDT_SUPPORT
static void spider_put_queue(struct sii8338_spider_data *sii8338,
#else
static void spider_put_queue(struct sii8338_data *sii8338,
#endif
						struct spider_event *event)
{
	dbg_enter();

	if ((sii8338->qtail + 1) % MAX_EVENT_QUEUE == sii8338->qhead) {
		pr_warn("spider: %s: queue full\n", __func__);
		return;
	}

	memcpy(&sii8338->eventq[sii8338->qtail], event,
					sizeof(struct spider_event));

	sii8338->qtail++;
	sii8338->qtail %= MAX_EVENT_QUEUE;

	dbg_leave();
}


#ifdef MDT_SUPPORT
void spider_issue_event(struct sii8338_spider_data *sii8338,
#else
static void spider_issue_event(struct sii8338_data *sii8338,
#endif
					struct spider_event *event)
{
	dbg_enter();

	if (sii8338->sm_issued) {
		pr_debug("spider: %s: already issued\n", __func__);
		sii8338->sm_issued = false;
		return;
	}

	mutex_lock(&sii8338->spider_lock);

	spider_put_queue(sii8338, event);

	mutex_unlock(&sii8338->spider_lock);

	if (sii8338->isopened) {
		wake_up_interruptible(&sii8338->spider_wq);
		pr_info("spider: %s: event queued\n", __func__);
	}

	dbg_leave();
}

static BLOCKING_NOTIFIER_HEAD(spider_notifier_list);

void spider_register_notifier(struct notifier_block *nb)
{
	dbg_enter();

	blocking_notifier_chain_register(&spider_notifier_list, nb);

	dbg_leave();
}
//EXPORT_SYMBOL_GPL(spider_register_notifier);

void spider_unregister_notifier(struct notifier_block *nb)
{
	dbg_enter();

	blocking_notifier_chain_unregister(&spider_notifier_list, nb);

	dbg_leave();
}
//EXPORT_SYMBOL_GPL(spider_unregister_notifier);

#ifdef MDT_SUPPORT
void spider_mouse_event(struct sii8338_spider_data *sii8338,
#else
static void spider_mouse_event(struct sii8338_data *sii8338,
#endif
						struct spider_event *event)
{
	dbg_enter();

	blocking_notifier_call_chain(&spider_notifier_list, 0, event);

	dbg_leave();
}

#ifdef MDT_SUPPORT
static void spider_handle_new_state(struct sii8338_spider_data *sii8338,
#else
static void spider_handle_new_state(struct sii8338_data *sii8338,
#endif
						struct spider_event *event)
{
	int mhl_state;
	int mouse_state;
	int other_state;

	dbg_enter();

	mhl_state = SM_DEV_TYPE_MHL & event->dev_type;
	mouse_state = SM_DEV_TYPE_MOUSE & event->dev_type;
	other_state = SM_DEV_TYPE_NOT_MOUSE & event->dev_type;

#if KEYBD_PERF
	if ((SM_DEV_TYPE_KEYBOARD & event->dev_type)
		&& (SM_DEV_STATE_CONNECTED & event->event_state)) {
		pr_info("\n\n#$#$#$# spider: %s: Keyboard connected\n",
								__func__);
		sii8338->keycnt = 0;
	}
#endif

	/* handle mhl cable, keyboard state change */
	/* or keep alive message */
	if (other_state) {
		spider_issue_event(sii8338, event);

		/* prevent spider_handle_new_event issues this same event
									again */
		sii8338->sm_issued = true;

		if (mhl_state) {
			if (SM_DEV_STATE_CONNECTED & event->event_state) {
				pr_info("spider: %s: sm_connected\n", __func__);

				sii8338->sm_connected = true;
			} else {
				pr_info("spider: %s: sm_disconnected\n",
								__func__);

				sii8338->sm_connected = false;
				// [START][Samsung Kor-4G] KT_SPIDER_FEATURE
				sii8338->sm_discovery = false;
				// [END][Samsung Kor-4G] KT_SPIDER_FEATURE
			}
		}
	}

	/* handle mouse state change */
	if (mouse_state) {
		spider_mouse_event(sii8338, event);
	}

	dbg_leave();
}

#ifdef MDT_SUPPORT
struct spider_keyboard_packet {							// Experimentally determined keyboard packet structure
	unsigned char modifiers;
	unsigned char reserved_1;
	unsigned char key;
	unsigned char reserved_2;
};


#define SPIDER_KEY_MOVE_HOME		0x3A
#define SPIDER_KEY_SETTINGS		0x3C


#define SPIDER_KEY_PREVIOUS		0x40
#define SPIDER_KEY_PLAYPAUSE		0x41
#define SPIDER_KEY_NEXT			0x42
#define SPIDER_KEY_MUSIC		0x43
#define SPIDER_KEY_EXPLORER		0x44
#define SPIDER_KEY_ENVELOPE		0x45
#define SPIDER_KEY_SEARCH		0x49
#define SPIDER_KEY_BACK			0x29
//
#define SPIDER_KEY_TEL			0x46
#define SPIDER_KEY_CAPS			0x39
#define SPIDER_KEY_RECENT		0x3b
#define SPIDER_KEY_MESSAGE      0x48
#define SPIDER_KEY_LANGUAGE		0x90


#define USB_HID_KEYBOARD_HOME		0x4A
#define USB_HID_KEYBOARD_APP		0x65
#define MDT_HID_KEYBOARD_MENU		0xC9
#define MDT_HID_KEYBOARD_HOME		0xCA
#define MDT_HID_KEYBOARD_BACK		0xF1
#define MDT_HID_MEDIA_NEXT		163
#define MDT_HID_MEDIA_PREVIOUS		165
#define MDT_HID_MEDIA_PAUSE		201
#define MDT_HID_MEDIA_PLAYPAUSE		0xE8
#define MDT_HID_MEDIA_PREVIOUSSONG	0xEA
#define MDT_HID_MEDIA_NEXTSONG		0xEB
#define MDT_HID_SEARCH			0xC7
#define MDT_HID_ENVELOPE		155
#define MDT_HID_EXPLORER		154

///
#define MDT_HID_MUSIC			0xA0
#define MDT_HID_KEYBOARD_TEL 	0xA1
#define MDT_HID_KEYBOARD_CAPS	0xA3
#define MDT_HID_KEYBOARD_RECENT  0xA4  ////
#define MDT_HID_KEYBOARD_MESSAGE 0   /////
#define MDT_HID_KEYBOARD_LANGUAGE  368-256  ///



	
static void spider_keyboard_wrapper_for_mdt(struct spider_keyboard_packet *key_event)
{
	struct mdt_keyboard_event_t mdt_key_event;
	unsigned char left_modifiers		= ((unsigned char)(key_event->modifiers) & 0x0F);
	unsigned char right_modifiers		= (((key_event->modifiers) >> 4) & 0x0F);

	memset((unsigned char *)&mdt_key_event,0,7);
	mdt_key_event.header.isHID		= 1;
	mdt_key_event.header.isKeyboard		= 1;
	mdt_key_event.header.modifier_keys	= right_modifiers | left_modifiers;

	MHL_log_event(MDT_EVENT_PARSED, 0x20, mdt_key_event.header.modifier_keys);
	MHL_log_event(MDT_EVENT_PARSED, 0x21, key_event->key);
	

	switch (key_event->key) {						// Make corrections since some Spider keys don't aline with USB
		case SPIDER_KEY_MUSIC: 		mdt_key_event.body.keycodes_all[0]	= MDT_HID_MUSIC;		break;
		case SPIDER_KEY_EXPLORER: 	mdt_key_event.body.keycodes_all[0]	= MDT_HID_EXPLORER;		break;
		case SPIDER_KEY_ENVELOPE: 	mdt_key_event.body.keycodes_all[0]	= MDT_HID_ENVELOPE;		break;
		case SPIDER_KEY_SEARCH: 	mdt_key_event.body.keycodes_all[0]	= MDT_HID_SEARCH;		break;
		case SPIDER_KEY_PLAYPAUSE: 	mdt_key_event.body.keycodes_all[0]	= MDT_HID_MEDIA_PLAYPAUSE;	break;
		case SPIDER_KEY_PREVIOUS: 	mdt_key_event.body.keycodes_all[0]	= MDT_HID_MEDIA_PREVIOUSSONG;	break;
		case SPIDER_KEY_NEXT:	 	mdt_key_event.body.keycodes_all[0]	= MDT_HID_MEDIA_NEXTSONG;	break;

		case SPIDER_KEY_SETTINGS: 	mdt_key_event.body.keycodes_all[0]	= USB_HID_KEYBOARD_APP;		break;
		case SPIDER_KEY_MOVE_HOME: 	mdt_key_event.body.keycodes_all[0]	= MDT_HID_KEYBOARD_HOME;	break;
		case SPIDER_KEY_BACK:	 	mdt_key_event.body.keycodes_all[0]	= MDT_HID_KEYBOARD_BACK;	break;
		//
		case SPIDER_KEY_TEL:	 	mdt_key_event.body.keycodes_all[0]	= MDT_HID_KEYBOARD_TEL;	break;
		case SPIDER_KEY_CAPS:	 	mdt_key_event.body.keycodes_all[0]	= MDT_HID_KEYBOARD_CAPS;	break;
		case SPIDER_KEY_RECENT:	 	mdt_key_event.body.keycodes_all[0]	= MDT_HID_KEYBOARD_RECENT;	break;
		case SPIDER_KEY_MESSAGE:	mdt_key_event.body.keycodes_all[0]	= MDT_HID_KEYBOARD_MESSAGE;	break;
		case SPIDER_KEY_LANGUAGE:	mdt_key_event.body.keycodes_all[0]	= MDT_HID_KEYBOARD_LANGUAGE;	break;
		default:			mdt_key_event.body.keycodes_all[0]	= key_event->key;
	}


	mdt_generate_event_keyboard(&mdt_key_event);
}

static void spider_handle_new_event(struct sii8338_spider_data *sii8338,
#else
static void spider_handle_new_event(struct sii8338_data *sii8338,
#endif
						struct spider_event *event)
{
	int mouse_event;
	int kbd_event;

	dbg_enter();

	mouse_event = SM_DEV_EVENT_MOUSE & event->event_state;
	kbd_event = SM_DEV_EVENT_KEYBOARD & event->event_state;

	if (mouse_event) {
		//pr_debug("spider: %s: mouse event\n", __func__);

		spider_mouse_event(sii8338, event);
	}
	
	if (kbd_event) {
#ifdef MDT_SUPPORT								// Since Spider handles keyboard events in the Android
		spider_keyboard_wrapper_for_mdt(				//    framework and Samsung doesn't disclose that code.
			(struct spider_keyboard_packet *)(event->kbd_data));		// Do our own parsing here using MDT keyboard support.
#else
#if KEYBD_PERF
		pr_info("spider: %s: keyboard event %d\n", __func__,
							sii8338->keycnt++);
		pr_cont("%02X %02X %02X %02X\n", event->kbd_data[0],
				event->kbd_data[1], event->kbd_data[2],
				event->kbd_data[3]);
#endif
		pr_debug("spider: %s: keyboard event\n", __func__);

		event->event_state = SM_DEV_EVENT_KEYBOARD;
		spider_issue_event(sii8338, event);
#endif
	}

	dbg_leave();
}

#ifdef MDT_SUPPORT
void spider_handle_msg(struct sii8338_spider_data *sii8338, void *data,
#else
static void spider_handle_msg(struct sii8338_data *sii8338, void *data,
#endif
								int state)
{
	struct spider_event *event = (struct spider_event *)data;
	struct spider_event new_spider_event = {0, };

	int new_state;
	int new_event;

	dbg_enter();

	switch (state) {
	case SPIDER_WRITE_BURST_MSG:
		pr_debug("spider: %s: SPIDER_WRITE_BURST_MSG\n", __func__);

		new_state = SM_DEV_STATE_MASK & event->event_state;
		new_event = SM_DEV_EVENT_MASK & event->event_state;

		if (new_state) {
			pr_debug("spider: %s: SM_DEV_STATE_CHANGED\n", __func__);

			spider_handle_new_state(sii8338, event);
		}

		if (!sii8338->sm_connected) {
			printk("spider: %s: !sm_connected\n", __func__);
			break;
		}

		if (new_event) {
			pr_debug("spider: %s: SM_DEV_EVENT_RECEIVED\n",
								__func__);

			spider_handle_new_event(sii8338, event);
		}

		sii8338->sm_issued = false;
		break;

	case SPIDER_DISCONNECTED:
		pr_info("spider: %s: SPIDER_DISCONNECTED\n", __func__);

		if (!sii8338->sm_connected) {
			pr_warn("spider: %s: Not connected\n", __func__);
			break;
		}

		event = &new_spider_event;

		/* issue usb mouse disconnected */
		event->dev_type = SM_DEV_TYPE_MOUSE;
		event->event_state = SM_DEV_STATE_DISCONNECTED;
		spider_mouse_event(sii8338, event);

		/* issue mhl disconnected */
		event->dev_type = SM_DEV_TYPE_MHL;
		event->event_state = SM_DEV_STATE_DISCONNECTED;
		spider_issue_event(sii8338, event);

		sii8338->sm_connected = false;
		// [START][Samsung Kor-4G] KT_SPIDER_FEATURE
		sii8338->sm_discovery = false;
		// [END][Samsung Kor-4G] KT_SPIDER_FEATURE
		break;

	default:
		pr_err("spider: %s: unknown state %d\n", __func__, state);
		break;
	}

	dbg_leave();
}

/*
#ifdef MDT_SUPPORT
static void cbus_handle_msg(struct sii8338_spider_data *sii8338, int state)
#else
static void cbus_handle_msg(struct sii8338_data *sii8338, int state)
#endif
{
	int ret;
	u8 val[16] = {0, };

	dbg_enter();

	switch (state) {
	case SPIDER_WRITE_BURST_MSG:
		pr_debug("spider: %s: SPIDER_WRITE_BURST_MSG\n", __func__);

		ret = cbus_read_block(sii8338, MHL_SCRATCHPAD_REG_0,
								&val[0], 16);
		if (0 > ret) {
			pr_err("spider: %s: cbus_read_block failed(%d)\n",
								__func__, ret);
			break;
		}

#ifdef CONFIG_SPIDER_MHL_DEBUG
		pr_info("spider: %s: received write_burst data:\n", __func__);

		for (ret = 0; ret < 16; ret++) {
			pr_cont("%02X ", val[ret]);
			if (7 == ret)
				pr_cont("- ");
		}
		pr_cont("\n");
#endif

		spider_handle_msg(sii8338, (void *)&val[0],
							SPIDER_WRITE_BURST_MSG);
		break;

#if 0
	case SPIDER_MSC_MSG:
		pr_info("spider: %s: SPIDER_MSC_MSG\n", __func__);
		ret = cbus_read_reg(sii8338, CBUS_MSC_FIRST_DATA_IN,
								&val[0]);
		if (0 > ret) {
			pr_err("spider: %s: cbus_read_reg1 failed(%d)\n",
							__func__, ret);
		} else {
			pr_info("spider: %s 1: %#02x received\n",
							__func__, val[0]);
		}

		ret = cbus_read_reg(sii8338, CBUS_MSC_MSG_CMD_IN, &val[1]);
		if (0 > ret) {
			pr_err("spider: %s: cbus_read_reg2 failed(%d)\n",
							__func__, ret);
		} else {
			pr_info("spider: %s 2: %#02x received\n",
							__func__, val[1]);
		}

		ret = cbus_read_reg(sii8338, CBUS_MSC_MSG_DATA_IN, &val[2]);
		if (0 > ret) {
			pr_err("spider: %s: cbus_read_reg3 failed(%d)\n",
							__func__, ret);
		} else {
			pr_info("spider: %s 3: %#02x received\n",
							__func__, val[2]);
		}

		if ((SPIDER_EDID_LAPTOP_OLD == *(unsigned int *)&val[0]) ||
			(SPIDER_EDID_LAPTOP == *(unsigned int *)&val[0])) {
			pr_info("spider: %s: SPIDER_LAPTOP%s connected\n",
				__func__, (0xff == val[1]) ? "(NEW)" : "(OLD)");
			spider_handle_msg(sii8338, NULL, SPIDER_CONNECTED);
		}
		break;
#endif

	default:
		pr_err("spider: %s: can't reach here!\n", __func__);
		break;
	}

	dbg_leave();
}

static int spider_open(struct inode *inode, struct file *filp)
{
	#ifdef MDT_SUPPORT
	struct sii8338_spider_data *sii8338 = spider_get_sii8338_data();
	#else
	struct sii8338_data *sii8338 = spider_get_sii8338_data();
	#endif

	dbg_enter();

	if (sii8338->isopened) {
		pr_warn("sii8338: %s: already opened\n", __func__);
		return -EBUSY;
	}

	filp->private_data = sii8338;
	sii8338->isopened = true;

	dbg_leave();

	return 0;
}

static int spider_release(struct inode *inode, struct file *filp)
{
	#ifdef MDT_SUPPORT
	struct sii8338_spider_data *sii8338 = spider_get_sii8338_data();
	#else
	struct sii8338_data *sii8338 = spider_get_sii8338_data();
	#endif

	dbg_enter();

	spider_fasync(-1, filp, 0);

	sii8338->isopened = false;
	filp->private_data = NULL;

	dbg_leave();

	return 0;
}

static ssize_t spider_read(struct file *filp, char *buf, size_t count,
							loff_t *ppos)
{
	#ifdef MDT_SUPPORT
	struct sii8338_spider_data *sii8338 = filp->private_data;
	#else
	struct sii8338_data *sii8338 = filp->private_data;
	#endif
	struct spider_event *event;

	int ret = -1;

	dbg_enter();

	if (sizeof(struct spider_event) > count)
		return -EINVAL;

	if (O_NONBLOCK & filp->f_flags)
		return -EAGAIN;

	ret = wait_event_interruptible(sii8338->spider_wq,
				sii8338->qhead != sii8338->qtail);

	if (ret)
		return ret;

	mutex_lock(&sii8338->spider_lock);

	event = spider_get_queue(sii8338);

	mutex_unlock(&sii8338->spider_lock);

	if (copy_to_user(buf, event, count))
		return -EFAULT;

	dbg_leave();

	return count;
}

#ifndef MDT_SUPPORT	//don't support writes to the dock
static ssize_t spider_write(struct file *filp, const char *buf,
					size_t count, loff_t *ppos)
{
	#ifdef MDT_SUPPORT
	struct sii8338_spider_data *sii8338 = filp->private_data;
	#else
	struct sii8338_data *sii8338 = filp->private_data;
	#endif

	char event[16] = {0, };
	int ret;

	dbg_enter();

#if 0	// TEST TMDS_EN 12-05-08 pianist
{
	static bool enable = false;

	if (enable) {
		ret = mhl_tx_set_reg(sii8338, MHL_TX_TMDS_CCTRL, (1<<4));
	} else {
		ret = mhl_tx_clear_reg(sii8338, MHL_TX_TMDS_CCTRL, (1<<4));
	}

	pr_info("spider: %s: TMDS %sabled\n", __func__, enable ? "en" : "dis");
	enable = !enable;
}
#endif

	if ((ret = copy_from_user(event, buf, count))) {
		pr_err("spider: %s failed(%d)\n", __func__, ret);
		return -EFAULT;
	}

#ifdef DEBUG
	pr_info("spider: test: %s: buffers to send = \n", __func__);
	for (ret = 0; ret < count; ret++) {
		pr_cont("%02X ", event[ret]);
		if (7 == ret)
			pr_cont("- ");
	}
	pr_cont("\n");
#endif

	ret = cbus_send_wb(sii8338, event, count);

	if (0 > ret) {
		pr_err("spider: %s failed(%d)\n", __func__, ret);
		return -EIO;
	}


	dbg_leave();

	return count;
}
#endif
static unsigned int spider_poll(struct file *filp,
					struct poll_table_struct *wait)
{

	#ifdef MDT_SUPPORT
	struct sii8338_spider_data *sii8338 = filp->private_data;
	#else
	struct sii8338_data *sii8338 = filp->private_data;
	#endif

	dbg_enter();

	poll_wait(filp, &sii8338->spider_wq, wait);

	if (sii8338->qhead != sii8338->qtail)
		return POLLIN | POLLRDNORM;

	dbg_leave();

	return 0;
}

static long spider_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	dbg_enter();

	dbg_leave();

	return 0;
}

static int spider_fasync(int fd, struct file *filp, int on)
{
	#ifdef MDT_SUPPORT
	struct sii8338_spider_data *sii8338 = filp->private_data;
	#else
	struct sii8338_data *sii8338 = filp->private_data;
	#endif
	int ret;

	dbg_enter();

	ret = fasync_helper(fd, filp, on, &sii8338->spider_fa);
	if (0 > ret)
		return ret;

	dbg_leave();

	return 0;
}

static struct file_operations spider_fops = {
	.owner = THIS_MODULE,
	.read  = spider_read,
	#ifndef MDT_SUPPORT	//don't support writes to the dock
	.write = spider_write,
	#endif
	.unlocked_ioctl = spider_ioctl,
	.open  = spider_open,
	.release = spider_release,
	.poll  = spider_poll,
	.fasync = spider_fasync,
};

static struct miscdevice spider_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = MODULE_NAME,
	.fops  = &spider_fops,
};
*/
#ifdef MDT_SUPPORT
static struct spider_event events[MAX_EVENT_QUEUE];
int spider_init(struct sii8338_spider_data *sii8338)
#else
static int spider_init(struct sii8338_data *sii8338)
#endif
{
#ifndef MDT_SUPPORT
	int ret = -1;
#endif
	dbg_enter();

	if (NULL == sii8338) {
		pr_err("spider: %s: spider_init failed\n", __func__);
		return -EINVAL;
	}

	g_sii8338 = sii8338;

	sii8338->eventq = &events[0];

	memset(sii8338->eventq, 0, sizeof(struct spider_event)
							* MAX_EVENT_QUEUE);

	init_waitqueue_head(&sii8338->spider_wq);
	mutex_init(&sii8338->spider_lock);

	sii8338->qhead = sii8338->qtail = 0;
	sii8338->sm_connected = false;
	// [START][Samsung Kor-4G] KT_SPIDER_FEATURE
	sii8338->sm_discovery = false;
	// [END][Samsung Kor-4G] KT_SPIDER_FEATURE

	#ifndef MDT_SUPPORT
	ret = misc_register(&spider_dev);
	if (ret) {
		pr_err("spider: %s: misc_register failed(%d)\n",
							__func__, ret);

		spider_exit();

		return ret;
	}
	#endif

	dbg_leave();

	return 0;
}
/*
static void spider_exit(void)
{
	dbg_enter();

	#ifndef MDT_SUPPORT
	misc_deregister(&spider_dev);
	#endif

	dbg_leave();
}
*/
#endif	/* CONFIG_SPIDER_MHL */
// [END] HELIXTECH: KT_SPIDER_FEATURE ======================================

