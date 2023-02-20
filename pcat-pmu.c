// SPDX-License-Identifier: GPL-2.0-only
/*
 * Photonicat STM32 Multi-Function PMU
 *
 * Copyright (c) 2022 BigfootACA <bigfoot@classfun.cn>
 */

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/crc16.h>
#include <linux/device.h>
#include <linux/hwmon.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/rtc.h>
#include <linux/serdev.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/timer.h>

/* start global section */
enum pmu_cmd {
	PMU_CMD_HEARTBEAT = 0x1,
	PMU_CMD_HEARTBEAT_ACK = 0x2,
	PMU_CMD_PMU_HW_VERSION_GET = 0x3,
	PMU_CMD_PMU_HW_VERSION_GET_ACK = 0x4,
	PMU_CMD_PMU_FW_VERSION_GET = 0x5,
	PMU_CMD_PMU_FW_VERSION_GET_ACK = 0x6,
	PMU_CMD_STATUS_REPORT = 0x7,
	PMU_CMD_STATUS_REPORT_ACK = 0x8,
	PMU_CMD_DATE_TIME_SYNC = 0x9,
	PMU_CMD_DATE_TIME_SYNC_ACK = 0xA,
	PMU_CMD_SCHEDULE_STARTUP_TIME_SET = 0xB,
	PMU_CMD_SCHEDULE_STARTUP_TIME_SET_ACK = 0xC,
	PMU_CMD_PMU_REQUEST_SHUTDOWN = 0xD,
	PMU_CMD_PMU_REQUEST_SHUTDOWN_ACK = 0xE,
	PMU_CMD_HOST_REQUEST_SHUTDOWN = 0xF,
	PMU_CMD_HOST_REQUEST_SHUTDOWN_ACK = 0x10,
	PMU_CMD_PMU_REQUEST_FACTORY_RESET = 0x11,
	PMU_CMD_PMU_REQUEST_FACTORY_RESET_ACK = 0x12,
	PMU_CMD_WATCHDOG_TIMEOUT_SET = 0x13,
	PMU_CMD_WATCHDOG_TIMEOUT_SET_ACK = 0x14,
	PMU_CMD_CHARGER_ON_AUTO_START = 0x15,
	PMU_CMD_CHARGER_ON_AUTO_START_ACK = 0x16,
	PMU_CMD_VOLTAGE_THRESHOLD_SET = 0x17,
	PMU_CMD_VOLTAGE_THRESHOLD_SET_ACK = 0x18,
	PMU_CMD_NET_STATUS_LED_SETUP = 0x19,
	PMU_CMD_NET_STATUS_LED_SETUP_ACK = 0x1A,
	PMU_CMD_POWER_ON_EVENT_GET = 0x1B,
	PMU_CMD_POWER_ON_EVENT_GET_ACK = 0x1C,
};

#define PMU_MAGIC_HEAD 0xA5
#define PMU_MAGIC_END 0x5A
#define PMU_EP_HOST 0x01
#define PMU_EP_PMU 0x81

struct pmu_data_head {
	u8 magic_head;
	u8 source;
	u8 dest;
	u16 frame_id;
	u16 length;
	enum pmu_cmd command : 16;
} __packed;

struct pmu_data_foot {
	u8 need_ack;
	u16 crc16;
	u8 magic_end;
} __packed;

struct pmu_data_cmd_date_time {
	u16 year;
	u8 month;
	u8 day;
	u8 hour;
	u8 minute;
	u8 second;
} __packed;

struct pmu_data_cmd_status {
	u16 battery_volt;
	u16 charger_volt;
	u16 gpio_input;
	u16 gpio_output;
	struct pmu_data_cmd_date_time time;
	u16 __unknown0;
	u8 temp;
} __packed;

struct pmu_data_cmd_led_setup {
	u16 on_time;
	u16 down_time;
	u16 repeat;
} __packed;

struct pmu_data {
	struct pcat_pmu *pmu;
	struct pmu_data_head *head;
	struct pmu_data_foot *foot;
	void *data;
	size_t size;
};

struct pmu_request {
	struct pcat_pmu *pmu;
	u16 frame_id;
	struct completion received;
	struct pmu_request_request {
		enum pmu_cmd cmd;
		enum pmu_cmd want;
		void *data;
		size_t size;
	} request;
	struct pmu_request_reply {
		struct pmu_data_head head;
		struct pmu_data_foot foot;
		void *data;
		size_t size;
	} reply;
};

struct pcat_pmu {
	struct device *dev;
	struct serdev_device *serdev;
	struct pcat_pmu_rtc {
		struct rtc_device *dev;
		struct device_node *node;
	} rtc;
	struct pcat_pmu_charger {
		struct power_supply *dev;
		struct device_node *node;
		struct power_supply_desc desc;
		u32 max, min;
	} charger;
	struct pcat_pmu_battery {
		struct power_supply *dev;
		struct device_node *node;
		struct power_supply_desc desc;
		u32 charging_table[11];
		u32 discharge_table[11];
		u32 max, min;
	} battery;
	struct pcat_pmu_led {
		struct led_classdev dev;
		struct device_node *node;
	} led_status;
	struct pcat_pmu_hwmon {
		struct device *dev;
		struct device_node *node;
	} hwmon_board;
	struct pcat_pmu_input {
		struct input_dev *dev;
		struct device_node *node;
		struct pcat_pmu_input_event {
			struct device_node *node;
			u32 code;
		} reset, shutdown;
	} input;
	atomic_t frame;
	char buffer[8192];
	size_t length;
	struct pmu_request *reply;
	spinlock_t bus_lock;
	struct mutex reply_lock;
	struct mutex status_lock;
	struct timer_list heartbeat;
	struct pmu_data_cmd_status status;
	struct completion first_status;
};

struct cmd_hand {
	enum pmu_cmd cmd;
	int (*handler)(const struct pmu_data *);
};

static struct pcat_pmu *g_pmu = NULL;

static void lsnprintf(char *buff, size_t bufsiz, const char *fmt, ...)
{
	va_list args;
	size_t p = strnlen(buff, bufsiz);
	va_start(args, fmt);
	vsnprintf(buff + p, bufsiz - p, fmt, args);
	va_end(args);
}
/* end global section */

/* start data process section */
static inline bool pmu_is_charging(struct pcat_pmu *pmu)
{
	uint32_t volt;
	mutex_lock(&pmu->status_lock);
	volt = pmu->status.charger_volt * 1000;
	mutex_unlock(&pmu->status_lock);
	return volt >= pmu->charger.min && volt <= pmu->charger.max;
}

static inline bool pmu_have_battery(struct pcat_pmu *pmu)
{
	uint32_t volt;
	mutex_lock(&pmu->status_lock);
	volt = pmu->status.battery_volt * 1000;
	mutex_unlock(&pmu->status_lock);
	return volt >= pmu->battery.min && volt <= pmu->battery.max;
}

static int pmu_calc_table(u16 volt, u32 *table, u16 last)
{
	u16 i;
	int percent = 0;
	if (volt > table[0])
		percent = 100;
	else if (volt > table[last]) {
		for (i = 0; i < last; i++) {
			if (volt >= table[i + 1] && table[i + 1] > 0) {
				percent = (90 - last * i) +
					  ((int)volt - table[i + 1]) * last /
						  (table[i] - table[i + 1]);
				break;
			}
		}
	}
	return percent;
}

static int pmu_calc_percent(struct pcat_pmu *pmu)
{
	u32 *table;
	int percent;
	u16 volt, max, min;
	mutex_lock(&pmu->status_lock);
	volt = pmu->status.battery_volt;
	mutex_unlock(&pmu->status_lock);
	max = pmu->battery.max / 1000;
	min = pmu->battery.min / 1000;
	if (pmu_is_charging(pmu))
		table = pmu->battery.charging_table;
	else
		table = pmu->battery.discharge_table;
	if (table[0])
		percent = pmu_calc_table(volt, table, 10);
	else
		percent = (volt - min) * 100 / (max - min);
	return max(0, min(100, percent));
}

static void dump_data(struct pcat_pmu *pmu, enum pmu_cmd cmd, u8 *buffer,
		      size_t length)
{
	size_t i;
	char buff[768];
	memset(buff, 0, sizeof(buff));
	lsnprintf(buff, sizeof(buff), "write %02X command %zu bytes:", cmd,
		  length);
	for (i = 0; i < length; i++)
		lsnprintf(buff, sizeof(buff), " %02X", buffer[i]);
	dev_dbg_ratelimited(pmu->dev, "%s\n", buff);
}

static int pmu_write(struct pcat_pmu *pmu, uint16_t frame_id,
		     enum pmu_cmd command, bool need_ack, const void *data,
		     size_t len)
{
	u8 buffer[768];
	struct pmu_data_head *head = (struct pmu_data_head *)buffer;
	void *body = (void *)buffer + sizeof(struct pmu_data_head);
	struct pmu_data_foot *foot = body + len;
	size_t payload_size = len + sizeof(struct pmu_data_foot);
	size_t length = sizeof(struct pmu_data_head) + payload_size;
	if (length > sizeof(buffer)) {
		dev_warn(pmu->dev, "data too long, reject\n");
		return -ENOMEM;
	}
	head->magic_head = PMU_MAGIC_HEAD;
	head->source = PMU_EP_HOST;
	head->dest = PMU_EP_PMU;
	head->frame_id = frame_id;
	head->length = payload_size - 1;
	head->command = command;
	if (body != NULL && len > 0)
		memcpy(body, data, len);
	foot->need_ack = need_ack;
	foot->crc16 = crc16(0xFFFF, buffer + 1, payload_size + 5);
	foot->magic_end = PMU_MAGIC_END;
	dump_data(pmu, command, buffer, length);
	return serdev_device_write_buf(pmu->serdev, buffer, length);
}

static int pmu_send(struct pcat_pmu *pmu, enum pmu_cmd cmd, const void *data,
		    size_t len)
{
	u16 frame_id = atomic_inc_return(&pmu->frame);
	return pmu_write(pmu, frame_id, cmd, false, data, len);
}

static int pmu_exec(struct pmu_request *request)
{
	int ret = 0, retries = 0;
	unsigned long flags;
	struct pcat_pmu *pmu = request->pmu;
	struct pmu_request_request *req = &request->request;
	struct pmu_request_reply *reply = &request->reply;

	init_completion(&request->received);
	memset(reply, 0, sizeof(request->reply));
	mutex_lock(&pmu->reply_lock);
	if (request->frame_id == 0) {
		request->frame_id = atomic_inc_return(&pmu->frame);
	}
	pmu->reply = request;
	mutex_unlock(&pmu->reply_lock);

	dev_dbg(pmu->dev, "frame %04X start exec command %02X\n",
		request->frame_id, req->cmd);
retry:
	spin_lock_irqsave(&pmu->bus_lock, flags);
	ret = pmu_write(pmu, request->frame_id, req->cmd, true, req->data,
			req->size);
	spin_unlock_irqrestore(&pmu->bus_lock, flags);
	if (ret < 0) {
		dev_err(pmu->dev, "frame %04X write %02X command failed: %d\n",
			request->frame_id, req->cmd, ret);
		goto fail;
	}

	dev_dbg(pmu->dev, "frame %04X waiting response for %02X\n",
		request->frame_id, req->cmd);
	if (!wait_for_completion_timeout(&request->received, HZ)) {
		dev_warn(pmu->dev, "frame %04X command %02X timeout\n",
			 request->frame_id, req->cmd);
		if (retries < 3) {
			retries++;
			goto retry;
		} else {
			ret = -ETIMEDOUT;
			goto fail;
		}
	}
	dev_dbg(pmu->dev, "frame %04X got response %02X\n", request->frame_id,
		reply->head.command);

	return 0;
fail:
	mutex_lock(&pmu->reply_lock);
	pmu->reply = NULL;
	mutex_unlock(&pmu->reply_lock);
	dev_err(pmu->dev, "frame %04X failed: %d\n", request->frame_id, ret);
	return ret;
}

static void send_ack(const struct pmu_data *data, enum pmu_cmd cmd)
{
	unsigned long flags;
	if (!data->foot->need_ack)
		return;
	spin_lock_irqsave(&data->pmu->bus_lock, flags);
	pmu_write(data->pmu, data->head->frame_id, cmd, false, NULL, 0);
	spin_unlock_irqrestore(&data->pmu->bus_lock, flags);
}

static int proc_status(const struct pmu_data *data)
{
	static const size_t size = sizeof(struct pmu_data_cmd_status);
	struct pcat_pmu *pmu = data->pmu;
	if (data->size < 15) {
		dev_dbg_ratelimited(pmu->dev, "status data too small %zu\n",
				    data->size);
		return -EBADMSG;
	}
	mutex_lock(&pmu->status_lock);
	memset(&pmu->status, 0, size);
	memcpy(&pmu->status, data->data, min(size, data->size));
	mutex_unlock(&pmu->status_lock);
	complete(&pmu->first_status);
	send_ack(data, PMU_CMD_STATUS_REPORT_ACK);
	return 0;
}

static int proc_shutdown(const struct pmu_data *data)
{
	struct pcat_pmu *pmu = data->pmu;
	struct pcat_pmu_input *input = &pmu->input;
	dev_info(pmu->dev, "PMU request shutdown system\n");
#if IS_ENABLED(CONFIG_INPUT)
	if (input->dev && input->shutdown.code) {
		input_report_key(input->dev, input->shutdown.code, 1);
		input_sync(input->dev);
	}
#endif
	send_ack(data, PMU_CMD_PMU_REQUEST_SHUTDOWN_ACK);
	return 0;
}

static int proc_reset(const struct pmu_data *data)
{
	struct pcat_pmu *pmu = data->pmu;
	struct pcat_pmu_input *input = &pmu->input;
	dev_info(pmu->dev, "PMU request factory reset\n");
#if IS_ENABLED(CONFIG_INPUT)
	if (input->dev && input->reset.code) {
		input_report_key(input->dev, input->reset.code, 1);
		input_sync(input->dev);
	} else
#endif
	{
		kernel_power_off();
	}
	send_ack(data, PMU_CMD_PMU_REQUEST_FACTORY_RESET_ACK);
	return 0;
}

static bool proc_reply(struct pmu_data *p)
{
	bool processed = false;
	struct pcat_pmu *pmu = p->pmu;
	struct device *dev = pmu->dev;
	struct pmu_request *request;
	struct pmu_request_request *req;
	struct pmu_request_reply *reply;
	mutex_lock(&pmu->reply_lock);
	request = pmu->reply;
	if (!request)
		goto skip;
	req = &request->request;
	reply = &request->reply;
	if (request->frame_id != p->head->frame_id) {
		dev_dbg_ratelimited(dev, "skip mismatch frame %04X != %04X",
				    request->frame_id, p->head->frame_id);
		goto skip;
	}
	if (req->want == 0)
		req->want = req->cmd + 1;
	if (req->want != p->head->command) {
		dev_dbg_ratelimited(
			dev, "frame %04X skip mismatch command %02X != %02X",
			request->frame_id, req->want, p->head->command);
		goto skip;
	}
	if (completion_done(&request->received)) {
		dev_dbg_ratelimited(dev, "frame %04X skip done completion",
				    request->frame_id);
		goto skip;
	}
	memcpy(&reply->head, p->head, sizeof(struct pmu_data_head));
	memcpy(&reply->foot, p->foot, sizeof(struct pmu_data_head));
	if (p->data && p->size > 0) {
		reply->data = devm_kzalloc(pmu->dev, p->size + 1, GFP_KERNEL);
		if (pmu->reply->reply.data) {
			memcpy(reply->data, p->data, p->size);
			reply->size = p->size;
		}
	}
	complete(&request->received);
	pmu->reply = NULL;
	processed = true;
skip:
	mutex_unlock(&pmu->reply_lock);
	return processed;
}

static struct cmd_hand cmd_hands[] = {
	{ PMU_CMD_STATUS_REPORT, proc_status },
	{ PMU_CMD_PMU_REQUEST_SHUTDOWN, proc_shutdown },
	{ PMU_CMD_PMU_REQUEST_FACTORY_RESET, proc_reset },
	{ 0, NULL }
};

static int proc_data(struct pcat_pmu *pmu, const u8 *data, size_t len)
{
	int ret;
	u16 rchk;
	size_t i, data_size = 0;
	bool processed = false;
	struct pmu_data p = { .pmu = pmu };
	if (pmu == NULL || data == NULL || len <= 0) {
		return 0;
	}
	if (len < sizeof(struct pmu_data_head)) {
		dev_dbg_ratelimited(pmu->dev, "head too small %zu < %zu\n", len,
				    sizeof(struct pmu_data_head));
		return -EAGAIN;
	}
	p.head = (struct pmu_data_head *)data;
	if (p.head->magic_head != PMU_MAGIC_HEAD) {
		dev_dbg_ratelimited(pmu->dev, "magic mismatch %02X != %02X\n",
				    p.head->magic_head, PMU_MAGIC_HEAD);
		return -EBADMSG;
	}
	if (p.head->source != PMU_EP_PMU) {
		dev_dbg_ratelimited(pmu->dev, "unknown data source %02X\n",
				    p.head->source);
		return 0;
	}
	if (p.head->dest != PMU_EP_HOST) {
		dev_dbg_ratelimited(pmu->dev, "not data destination %02X\n",
				    p.head->dest);
		return 0;
	}
	if (p.head->length < sizeof(struct pmu_data_foot) ||
	    p.head->length >= 65532) {
		dev_dbg_ratelimited(pmu->dev, "invalid lengtgh %d\n",
				    p.head->length);
		return -EBADMSG;
	}
	data_size = sizeof(struct pmu_data_head) + p.head->length;
	if (data_size > len) {
		dev_dbg_ratelimited(pmu->dev, "data too small %zu > %zu\n",
				    data_size, len);
		return -EAGAIN;
	}
	p.data = (u8 *)data + sizeof(struct pmu_data_head);
	p.size = p.head->length + 1 - sizeof(struct pmu_data_foot);
	p.foot = (struct pmu_data_foot *)(data + p.size +
					  sizeof(struct pmu_data_head));
	if (p.foot->magic_end != PMU_MAGIC_END) {
		dev_dbg_ratelimited(pmu->dev, "magic mismatch %02X != %02X\n",
				    p.foot->magic_end, PMU_MAGIC_END);
		return -EBADMSG;
	}
	rchk = crc16(0xFFFF, data + 1, p.head->length + 6);
	if (p.foot->crc16 != rchk) {
		dev_warn_ratelimited(pmu->dev, "crc16 mismatch %04X != %04X\n",
				     p.foot->crc16, rchk);
		return -EBADMSG;
	}
	dev_dbg_ratelimited(pmu->dev, "got command %02X\n", p.head->command);
	processed = proc_reply(&p);
	for (i = 0; cmd_hands[i].handler; i++) {
		if (cmd_hands[i].cmd == p.head->command) {
			ret = cmd_hands[i].handler(&p);
			processed = true;
			break;
		}
	}
	if (!processed)
		dev_dbg_ratelimited(pmu->dev, "unknown command %02X\n",
				    p.head->command);
	return ret;
}

static inline int pmu_exec_data(struct pcat_pmu *pmu, enum pmu_cmd cmd,
				void *data, size_t size)
{
	int ret;
	struct pmu_request request = {
		.pmu = pmu,
		.request.cmd = cmd,
		.request.data = data,
		.request.size = size,
	};
	ret = pmu_exec(&request);
	if (request.reply.data)
		devm_kfree(pmu->dev, request.reply.data);
	return ret;
}

static inline int pmu_exec_get_str(struct pcat_pmu *pmu, enum pmu_cmd cmd,
				   char *s, size_t len)
{
	int ret;
	struct pmu_request request = { .pmu = pmu, .request.cmd = cmd };
	memset(s, 0, len);
	ret = pmu_exec(&request);
	if (request.reply.data) {
		memcpy(s, request.reply.data, min(len - 1, request.reply.size));
		devm_kfree(pmu->dev, request.reply.data);
	};
	return ret;
}

static inline int pmu_exec_u8(struct pcat_pmu *pmu, enum pmu_cmd cmd, u8 v)
{
	return pmu_exec_data(pmu, cmd, &v, sizeof(v));
}

static inline int pmu_exec_set_wdog_timeout(struct pcat_pmu *pmu, u8 sec)
{
	u8 t[3] = { 60, 60, sec };
	return pmu_exec_data(pmu, PMU_CMD_WATCHDOG_TIMEOUT_SET, &t, sizeof(t));
}
/* end data process section */

/* start rtc section */
#if IS_ENABLED(CONFIG_RTC_CLASS)
static int pcat_pmu_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	int ret = -EIO;
	struct pcat_pmu *pmu = dev_get_drvdata(dev);
	struct pmu_data_cmd_date_time *time;
	if (!pmu)
		return -EINVAL;
	mutex_lock(&pmu->status_lock);
	time = &pmu->status.time;
	memset(tm, 0, sizeof(struct rtc_time));
	if (time->second >= 60) {
		dev_dbg(dev, "invalid second %d\n", time->second);
		goto fail;
	}
	tm->tm_sec = time->second;
	if (time->minute >= 60) {
		dev_dbg(dev, "invalid minute %d\n", time->minute);
		goto fail;
	}
	tm->tm_min = time->minute;
	if (time->hour >= 24) {
		dev_dbg(dev, "invalid hour %d\n", time->hour);
		goto fail;
	}
	tm->tm_hour = time->hour;
	if (time->day <= 0 || time->day > 31) {
		dev_dbg(dev, "invalid day %d\n", time->day);
		goto fail;
	}
	tm->tm_mday = time->day;
	if (time->month <= 0 || time->month > 12) {
		dev_dbg(dev, "invalid month %d\n", time->month);
		goto fail;
	}
	tm->tm_mon = time->month - 1;
	if (time->year < 1900 || time->year > 9999) {
		dev_dbg(dev, "invalid year %d\n", time->year);
		goto fail;
	}
	tm->tm_year = time->year - 1900;
	tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, time->year);
	tm->tm_wday = ((time->year * (365 % 7)) + ((time->year - 1) / 4) -
		       ((time->year - 1) / 100) + ((time->year - 1) / 400) +
		       tm->tm_yday) % 7;
	ret = 0;
fail:
	mutex_unlock(&pmu->status_lock);
	return ret;
}

static int pcat_pmu_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	int ret = 0;
	struct pcat_pmu *pmu = dev_get_drvdata(dev);
	struct pmu_data_cmd_date_time time;
	struct pmu_request req = {
		.pmu = pmu,
		.request.cmd = PMU_CMD_DATE_TIME_SYNC,
		.request.want = PMU_CMD_DATE_TIME_SYNC_ACK,
		.request.data = &time,
		.request.size = sizeof(time),
	};
	if (!pmu)
		return -EINVAL;
	time.year = tm->tm_year + 1900;
	time.month = tm->tm_mon + 1;
	time.day = tm->tm_mday;
	time.hour = tm->tm_hour;
	time.minute = tm->tm_min;
	time.second = tm->tm_sec;
	ret = pmu_exec(&req);
	if (req.reply.data)
		devm_kfree(pmu->dev, req.reply.data);
	return ret;
}

static const struct rtc_class_ops pcat_pmu_rtc_ops = {
	.read_time = pcat_pmu_rtc_read_time,
	.set_time = pcat_pmu_rtc_set_time,
};

static void init_rtc(struct pcat_pmu *pmu)
{
	const char *name = NULL;
	struct device *dev = pmu->dev;
	struct device_node *node;
	struct rtc_device *rtc;
	u16 year;

	mutex_lock(&pmu->status_lock);
	year = pmu->status.time.year;
	mutex_unlock(&pmu->status_lock);
	node = of_get_child_by_name(dev->of_node, "pmu-rtc");
	if (!of_device_is_available(node))
		return;
	of_property_read_string(node, "label", &name);
	if (!name)
		name = "pmu-rtc";
	if (year == 0) {
		dev_info(dev, "no time date found, skip rtc\n");
		return;
	}

	rtc = devm_rtc_device_register(dev, name, &pcat_pmu_rtc_ops,
				       THIS_MODULE);
	if (IS_ERR(rtc)) {
		dev_warn(dev, "Failed to register rtc: %d\n",
			 (int)PTR_ERR(rtc));
		return;
	}

	pmu->rtc.node = node;
	pmu->rtc.dev = rtc;
	dev_info(dev, "register pmu rtc\n");
}
#else
#define init_rtc(pmu)
#endif
/* end rtc section */

/* start power supply section */
#if IS_ENABLED(CONFIG_POWER_SUPPLY)
/* start power supply charger section */
static inline void read_microvolt(struct device_node *node, const char *name,
				  u32 *val, u32 def)
{
	if (of_property_read_u32(node, name, val))
		*val = def;
}

static int pcat_pmu_get_charger_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct pcat_pmu *pmu = power_supply_get_drvdata(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pmu_is_charging(pmu);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mutex_lock(&pmu->status_lock);
		val->intval = pmu->status.charger_volt * 1000;
		mutex_unlock(&pmu->status_lock);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = pmu->charger.min;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = pmu->charger.max;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property pcat_pmu_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static void init_charger(struct pcat_pmu *pmu)
{
	int ret;
	const char *name = NULL;
	struct device *dev = pmu->dev;
	struct device_node *node;
	struct power_supply *supply;
	struct power_supply_config psy_cfg = {
		.drv_data = pmu,
	};

	node = of_get_child_by_name(dev->of_node, "supply-charger");
	if (!of_device_is_available(node))
		return;
	psy_cfg.of_node = node;
	of_property_read_string(node, "label", &name);
	if (!name)
		name = "charger";
	pmu->charger.desc.name = name;
	pmu->charger.desc.type = POWER_SUPPLY_TYPE_MAINS;
	pmu->charger.desc.properties = pcat_pmu_charger_props;
	pmu->charger.desc.num_properties = ARRAY_SIZE(pcat_pmu_charger_props);
	pmu->charger.desc.get_property = pcat_pmu_get_charger_property;

	supply = devm_power_supply_register(dev, &pmu->charger.desc, &psy_cfg);
	if (IS_ERR(supply)) {
		ret = PTR_ERR(supply);
		dev_warn(dev, "Failed to register charger: %d\n", ret);
		return;
	}

	pmu->charger.node = node;
	pmu->charger.dev = supply;
	read_microvolt(node, "max-microvolt", &pmu->charger.max, 4000000);
	read_microvolt(node, "min-microvolt", &pmu->charger.min, 6000000);
	dev_dbg(dev, "charger voltage %uuV - %uuV\n", pmu->charger.min,
		pmu->charger.max);
	dev_info(dev, "register charger power supply\n");
}
/* end power supply charger section */

/* start power supply battery section */
static inline bool check_table(u32 *table)
{
	size_t i;
	for (i = 0; i < 11; i++)
		if (i > 0 && table[i] > table[i - 1])
			return false;
	return true;
}

static void dump_table(struct device *dev, const char *name, u32 *table)
{
	size_t i;
	char buff[768];
	memset(buff, 0, sizeof(buff));
	lsnprintf(buff, sizeof(buff), "%s:", name);
	for (i = 0; i < 11; i++)
		lsnprintf(buff, sizeof(buff), " %uuV", table[i]);
	dev_dbg(dev, "%s\n", buff);
}

static void read_table(struct pcat_pmu *pmu, const char *name, u32 *table)
{
	size_t i;
	u32 tmp[11];
	if (!of_property_read_u32_array(pmu->battery.node, name, tmp, 11)) {
		dump_table(pmu->dev, name, tmp);
		if (!check_table(tmp)) {
			dev_warn(pmu->dev, "invalid table %s\n", name);
			return;
		}
		for (i = 0; i < 11; i++)
			table[i] = tmp[i] / 1000;
	} else
		dev_warn(pmu->dev, "%s not set\n", name);
}

static int pcat_pmu_get_battery_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct pcat_pmu *pmu = power_supply_get_drvdata(psy);
	int status = POWER_SUPPLY_STATUS_UNKNOWN, volt;
	int have_battery = pmu_have_battery(pmu);
	int is_charging = pmu_is_charging(pmu);
	int ccap = pmu_calc_percent(pmu);
	u32 full = 0;
	mutex_lock(&pmu->status_lock);
	volt = pmu->status.battery_volt * 1000;
	mutex_unlock(&pmu->status_lock);
	if (of_property_read_u32(pmu->battery.node, "energy", &full))
		full = 25000;
	if (have_battery) {
		if (is_charging) {
			if (ccap >= 100)
				status = POWER_SUPPLY_STATUS_FULL;
			else
				status = POWER_SUPPLY_STATUS_CHARGING;
		} else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = have_battery;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = volt;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = pmu->battery.min;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = pmu->battery.max;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = ccap;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		val->intval = full * 1000;
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		val->intval = full * ccap / 100 * 1000;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property pcat_pmu_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,      POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW, POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX, POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_ENERGY_FULL, POWER_SUPPLY_PROP_ENERGY_NOW,
};

static void init_battery(struct pcat_pmu *pmu)
{
	int ret;
	const char *name = NULL;
	struct device *dev = pmu->dev;
	struct device_node *node;
	struct power_supply *supply;
	struct power_supply_config psy_cfg = {
		.drv_data = pmu,
	};

	node = of_get_child_by_name(dev->of_node, "supply-battery");
	if (!of_device_is_available(node))
		return;
	psy_cfg.of_node = node;
	of_property_read_string(node, "label", &name);
	if (!name)
		name = "battery";
	pmu->battery.desc.name = name;
	pmu->battery.desc.type = POWER_SUPPLY_TYPE_BATTERY;
	pmu->battery.desc.properties = pcat_pmu_battery_props;
	pmu->battery.desc.num_properties = ARRAY_SIZE(pcat_pmu_battery_props);
	pmu->battery.desc.get_property = pcat_pmu_get_battery_property;

	supply = devm_power_supply_register(dev, &pmu->battery.desc, &psy_cfg);
	if (IS_ERR(supply)) {
		ret = PTR_ERR(supply);
		dev_warn(dev, "Failed to register battery: %d\n", ret);
		return;
	}

	pmu->battery.node = node;
	pmu->battery.dev = supply;
	read_microvolt(node, "max-microvolt", &pmu->battery.max, 3400000);
	read_microvolt(node, "min-microvolt", &pmu->battery.min, 4200000);
	read_table(pmu, "charging-table", pmu->battery.charging_table);
	read_table(pmu, "discharge-table", pmu->battery.discharge_table);
	dev_dbg(dev, "battery voltage %uuV - %uuV\n", pmu->battery.min,
		pmu->battery.max);
	dev_info(dev, "register battery power supply\n");
}
/* end power supply battery section */
#else
#define init_charger(pmu)
#define init_battery(pmu)
#endif
/* end power supply section */

/* start led section */
#if IS_ENABLED(CONFIG_LEDS_CLASS)
static void status_led_set(struct led_classdev *cdev, enum led_brightness value)
{
	unsigned long flags;
	struct pcat_pmu *pmu =
		container_of(cdev, struct pcat_pmu, led_status.dev);
	struct pmu_data_cmd_led_setup setup = { 0, 0, 0 };
	if (value)
		setup.on_time = 100;
	else
		setup.down_time = 100;
	spin_lock_irqsave(&pmu->bus_lock, flags);
	pmu_send(pmu, PMU_CMD_NET_STATUS_LED_SETUP, &setup, sizeof(setup));
	spin_unlock_irqrestore(&pmu->bus_lock, flags);
}

static void init_led_status(struct pcat_pmu *pmu)
{
	int ret;
	const char *name = NULL, *str;
	struct device *dev = pmu->dev;
	struct device_node *node;

	node = of_get_child_by_name(dev->of_node, "led-status");
	if (!of_device_is_available(node))
		return;
	of_property_read_string(node, "label", &name);
	if (!name)
		name = "status";
	pmu->led_status.dev.dev = dev;
	pmu->led_status.dev.name = name;
	pmu->led_status.dev.brightness_set = status_led_set;
	pmu->led_status.dev.max_brightness = 1;
	of_property_read_string(node, "linux,default-trigger",
				&pmu->led_status.dev.default_trigger);

	ret = devm_led_classdev_register(dev, &pmu->led_status.dev);
	if (ret < 0) {
		dev_warn(dev, "Failed to register led: %d\n", ret);
		return;
	}

	if (!of_property_read_string(node, "default-state", &str)) {
		if (!strcmp(str, "on"))
			status_led_set(&pmu->led_status.dev, 1);
		else if (!strcmp(str, "off"))
			status_led_set(&pmu->led_status.dev, 0);
	}
	pmu->led_status.node = node;
	dev_info(dev, "register net status led\n");
}
#else
#define init_led_status(pmu)
#endif
/* end led section */

/* start hwmon section */
#if IS_ENABLED(CONFIG_HWMON)
static ssize_t temp1_input_show(struct device *dev,
				struct device_attribute *devattr, char *buf)
{
	u16 temp;
	struct pcat_pmu *pmu = dev_get_drvdata(dev->parent);
	mutex_lock(&pmu->status_lock);
	temp = pmu->status.temp;
	mutex_unlock(&pmu->status_lock);
	return sprintf(buf, "%d\n", (temp - 40) * 1000);
}

static DEVICE_ATTR_RO(temp1_input);

static struct attribute *pmu_temp_attrs[] = {
	&dev_attr_temp1_input.attr,
	NULL
};

ATTRIBUTE_GROUPS(pmu_temp);

static void init_hwmon_board(struct pcat_pmu *pmu)
{
	int ret;
	u16 temp;
	const char *name = NULL;
	struct device *dev = pmu->dev;
	struct device *hwmon;
	struct device_node *node;

	mutex_lock(&pmu->status_lock);
	temp = pmu->status.temp;
	mutex_unlock(&pmu->status_lock);
	node = of_get_child_by_name(dev->of_node, "hwmon-board");
	if (!of_device_is_available(node))
		return;
	of_property_read_string(node, "label", &name);
	if (!name)
		name = "pcat_pmu_board";

	if (temp == 0) {
		dev_info(dev, "no board temp found, skip hwmon\n");
		return;
	}

	hwmon = devm_hwmon_device_register_with_groups(dev, name, NULL,
						       pmu_temp_groups);
	if (IS_ERR(hwmon)) {
		ret = PTR_ERR(hwmon);
		dev_warn(dev, "Failed to register hwmon: %d\n", ret);
		return;
	}

	pmu->hwmon_board.node = node;
	pmu->hwmon_board.dev = hwmon;
	dev_info(dev, "register board temp hwmon\n");
}
#else
#define init_hwmon_board(pmu)
#endif
/* end hwmon section */

/* start input section */
#if IS_ENABLED(CONFIG_INPUT)
static void init_input_event(struct pcat_pmu *pmu,
			     struct pcat_pmu_input_event *ev, const char *name,
			     u32 def)
{
	ev->node = of_get_child_by_name(pmu->input.node, name);
	if (!of_device_is_available(ev->node))
		return;
	if (of_property_read_u32(ev->node, "linux,code", &ev->code))
		ev->code = def;
	input_set_capability(pmu->input.dev, EV_KEY, ev->code);
}

static void init_input(struct pcat_pmu *pmu)
{
	int ret;
	const char *name = NULL;
	struct device *dev = pmu->dev;

	pmu->input.node = of_get_child_by_name(dev->of_node, "input");
	if (!of_device_is_available(pmu->input.node))
		return;
	of_property_read_string(pmu->input.node, "label", &name);
	if (!name)
		name = "pmu-events";

	pmu->input.dev = devm_input_allocate_device(dev);
	if (!pmu->input.dev) {
		dev_warn(dev, "Failed to allocate input\n");
		goto fail;
	}

	pmu->input.dev->name = name;
	pmu->input.dev->dev.parent = dev;
	pmu->input.dev->id.bustype = BUS_HOST;
	pmu->input.dev->evbit[0] = BIT_MASK(EV_KEY);

	init_input_event(pmu, &pmu->input.reset, "reset", KEY_CLEAR);
	init_input_event(pmu, &pmu->input.shutdown, "shutdown", KEY_POWER);

	ret = input_register_device(pmu->input.dev);
	if (ret < 0) {
		dev_err(dev, "Failed to register input device: %d\n", ret);
		goto fail;
	}
	dev_info(dev, "register events input\n");
	return;
fail:
	memset(&pmu->input, 0, sizeof(pmu->input));
}
#else
#define init_input(pmu)
#endif
/* end input section */

/* start serdev section */
static int proc_buffer(struct pcat_pmu *pmu, const unsigned char *buf,
		       size_t size)
{
	int ret;
	size_t proc = size;
	size_t new_len = pmu->length + size;
	if (!pmu || !buf || size <= 0)
		return 0;
	if (new_len > sizeof(pmu->buffer)) {
		new_len = sizeof(pmu->buffer);
		proc = new_len - pmu->length;
	}
	if (pmu->length)
		dev_dbg(pmu->dev, "got remaining message at %zu size %zu (%zu)",
			pmu->length, proc, new_len);
	memcpy(pmu->buffer + pmu->length, buf, proc);
	pmu->length = new_len;
	ret = proc_data(pmu, pmu->buffer, pmu->length);
	if (ret != -EAGAIN)
		pmu->length = 0;
	else
		dev_dbg(pmu->dev, "got partial message %zu", pmu->length);
	return proc;
}

static int pcat_pmu_receive_buf(struct serdev_device *serdev,
				const unsigned char *buf, size_t size)
{
	size_t i;
	char buff[768];
	struct device *dev = &serdev->dev;
	struct pcat_pmu *pmu = dev_get_drvdata(dev);
	if (!pmu)
		return 0;
	memset(buff, 0, sizeof(buff));
	lsnprintf(buff, sizeof(buff), "buffer %zu bytes:", size);
	for (i = 0; i < size; i++)
		lsnprintf(buff, sizeof(buff), " %02X", buf[i]);
	dev_dbg(dev, "%s\n", buff);
	return proc_buffer(pmu, buf, size);
}

static const struct serdev_device_ops pcat_pmu_serdev_device_ops = {
	.receive_buf = pcat_pmu_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};
/* end serdev section */

/* start initialize section */
static void on_heartbeat(struct timer_list *data)
{
	unsigned long flags;
	struct pcat_pmu *pmu = from_timer(pmu, data, heartbeat);
	if (!pmu->serdev)
		return;
	spin_lock_irqsave(&pmu->bus_lock, flags);
	pmu_send(pmu, PMU_CMD_HEARTBEAT, NULL, 0);
	spin_unlock_irqrestore(&pmu->bus_lock, flags);
	mod_timer(&pmu->heartbeat, jiffies + msecs_to_jiffies(1000));
}

static void pmu_power_off(void)
{
	struct pcat_pmu *pmu = g_pmu;
	if (!pmu)
		return;
	pmu_exec_set_wdog_timeout(pmu, 0);
	dev_info(pmu->dev, "Request PMU power off\n");
	pmu_exec_data(pmu, PMU_CMD_HOST_REQUEST_SHUTDOWN, NULL, 0);
	dev_warn(pmu->dev, "PMU power off command failed\n");
}

static void init_pmu(struct pcat_pmu *pmu)
{
	u8 timeout;
	char buff[256];
	if (of_property_read_u8(pmu->dev->of_node, "timeout-sec", &timeout))
		timeout = 10;
	pmu_exec_u8(pmu, PMU_CMD_CHARGER_ON_AUTO_START, 0);
	pmu_exec_get_str(pmu, PMU_CMD_PMU_FW_VERSION_GET, buff, sizeof(buff));
	if (buff[0])
		dev_info(pmu->dev, "PMU Firmware Version: %s\n", buff);
	pmu_exec_get_str(pmu, PMU_CMD_PMU_HW_VERSION_GET, buff, sizeof(buff));
	if (buff[0])
		dev_info(pmu->dev, "PMU Hardware Version: %s\n", buff);
	pmu_exec_data(pmu, PMU_CMD_POWER_ON_EVENT_GET, NULL, 0);
	pmu_exec_set_wdog_timeout(pmu, timeout);
	timer_setup(&pmu->heartbeat, on_heartbeat, 0);
	mod_timer(&pmu->heartbeat, jiffies + msecs_to_jiffies(1000));
}

static void wait_status(struct pcat_pmu *pmu)
{
	struct device *dev = pmu->dev;
	struct pmu_data_cmd_status *stat = &pmu->status;
	struct pmu_data_cmd_date_time *time = &stat->time;
	dev_dbg(dev, "waiting for first status report\n");
	if (!wait_for_completion_timeout(&pmu->first_status,
					 msecs_to_jiffies(5000))) {
		dev_warn(dev, "no status report received\n");
		return;
	}
	mutex_lock(&pmu->status_lock);
	dev_info(dev, "Charger Voltage: %umV\n", stat->charger_volt);
	dev_info(dev, "Battery Voltage: %umV\n", stat->battery_volt);
	if (time->year != 0)
		dev_info(dev, "RTC Time: %04d/%02d/%02d %02d:%02d:%02d\n",
			 time->year, time->month, time->day, time->hour,
			 time->minute, time->second);
	if (stat->temp != 0)
		dev_info(dev, "Board Tempature: %u°C\n", stat->temp - 40);
	mutex_unlock(&pmu->status_lock);
}

static int pcat_pmu_probe(struct serdev_device *serdev)
{
	int ret;
	u32 baud;
	struct pcat_pmu *pmu = NULL;
	struct device *dev = &serdev->dev;

	/* initialize */
	pmu = devm_kzalloc(dev, sizeof(struct pcat_pmu), GFP_KERNEL);
	if (!pmu)
		return -ENOMEM;
	pmu->dev = dev;
	pmu->serdev = serdev;
	spin_lock_init(&pmu->bus_lock);
	mutex_init(&pmu->reply_lock);
	init_completion(&pmu->first_status);

	/* open serial port */
	if (of_property_read_u32(dev->of_node, "current-speed", &baud))
		baud = 115200;
	dev_dbg(dev, "use baudrate %d\n", baud);

	serdev_device_set_client_ops(serdev, &pcat_pmu_serdev_device_ops);
	ret = devm_serdev_device_open(dev, serdev);
	if (ret < 0) {
		dev_err(dev, "Failed to open serdev: %d\n", ret);
		return ret;
	}

	serdev_device_set_baudrate(serdev, baud);
	serdev_device_set_flow_control(serdev, false);
	serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	dev_set_drvdata(dev, pmu);
	if (!g_pmu) {
		g_pmu = pmu;
		pm_power_off = pmu_power_off;
	}

	init_pmu(pmu);
	wait_status(pmu);
	init_rtc(pmu);
	init_charger(pmu);
	init_battery(pmu);
	init_led_status(pmu);
	init_hwmon_board(pmu);
	init_input(pmu);

	dev_info(dev, "pcat pmu probe done\n");
	return devm_of_platform_populate(dev);
}

static void pcat_pmu_remove(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct pcat_pmu *pmu = dev_get_drvdata(dev);
	if (!pmu)
		return;
	del_timer_sync(&pmu->heartbeat);
	pmu_exec_set_wdog_timeout(pmu, 0);
#if IS_ENABLED(CONFIG_LEDS_CLASS)
	if (pmu->led_status.node) {
		devm_led_classdev_unregister(dev, &pmu->led_status.dev);
		memset(&pmu->led_status, 0, sizeof(pmu->led_status));
	}
#endif
#if IS_ENABLED(CONFIG_INPUT)
	if (pmu->input.dev) {
		input_unregister_device(pmu->input.dev);
		memset(&pmu->input, 0, sizeof(pmu->input));
	}
#endif
	pmu->serdev = NULL;
	dev_set_drvdata(dev, NULL);
	if (g_pmu == pmu) {
		g_pmu = NULL;
		pm_power_off = NULL;
	}
	dev_info(dev, "pcat pmu remove done\n");
}
/* end initialize section */

static const struct of_device_id pcat_pmu_dt_ids[] = {
	{ .compatible = "photonicat,pmu" },
	{ /* sentinel */ }
};

static struct serdev_device_driver pcat_pmu_driver = {
	.driver = {
		.name = "pcat-pmu",
		.of_match_table = pcat_pmu_dt_ids,
	},
	.probe = pcat_pmu_probe,
	.remove = pcat_pmu_remove,
};
module_serdev_device_driver(pcat_pmu_driver);

MODULE_DEVICE_TABLE(of, pcat_pmu_dt_ids);
MODULE_AUTHOR("BigfootACA <bigfoot@classfun.cn>");
MODULE_DESCRIPTION("Photonicat STM32 PMU driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pcat-pmu");
