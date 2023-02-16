/*
 * Copyright (c) 2019-2022, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <xtensa/hal.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp32/rom/ets_sys.h>
#include "sdkconfig.h"
#include "queue_bss.h"
#include "zephyr/types.h"
#include "tools/util.h"
#include "config.h"
#include "adapter.h"
#include "adapter_debug.h"
#include "wired/wired.h"
#include "wireless/wireless.h"
#include "macro.h"

const uint32_t hat_to_ld_btns[16] = {
    BIT(PAD_LD_UP), BIT(PAD_LD_UP) | BIT(PAD_LD_RIGHT), BIT(PAD_LD_RIGHT), BIT(PAD_LD_DOWN) | BIT(PAD_LD_RIGHT),
    BIT(PAD_LD_DOWN), BIT(PAD_LD_DOWN) | BIT(PAD_LD_LEFT), BIT(PAD_LD_LEFT), BIT(PAD_LD_UP) | BIT(PAD_LD_LEFT),
};

const uint32_t generic_btns_mask[32] = {
    BIT(PAD_LX_LEFT), BIT(PAD_LX_RIGHT), BIT(PAD_LY_DOWN), BIT(PAD_LY_UP),
    BIT(PAD_RX_LEFT), BIT(PAD_RX_RIGHT), BIT(PAD_RY_DOWN), BIT(PAD_RY_UP),
    BIT(PAD_LD_LEFT), BIT(PAD_LD_RIGHT), BIT(PAD_LD_DOWN), BIT(PAD_LD_UP),
    BIT(PAD_RD_LEFT), BIT(PAD_RD_RIGHT), BIT(PAD_RD_DOWN), BIT(PAD_RD_UP),
    BIT(PAD_RB_LEFT), BIT(PAD_RB_RIGHT), BIT(PAD_RB_DOWN), BIT(PAD_RB_UP),
    BIT(PAD_MM), BIT(PAD_MS), BIT(PAD_MT), BIT(PAD_MQ),
    BIT(PAD_LM), BIT(PAD_LS), BIT(PAD_LT), BIT(PAD_LJ),
    BIT(PAD_RM), BIT(PAD_RS), BIT(PAD_RT), BIT(PAD_RJ),
};

struct generic_ctrl *ctrl_input;
struct generic_ctrl *ctrl_output;
struct generic_fb fb_input;
struct bt_adapter bt_adapter = {0};
struct wired_adapter wired_adapter = {0};

static uint32_t btn_id_to_btn_idx(uint8_t btn_id) {
    if (btn_id < 32) {
        return 0;
    }
    else if (btn_id >= 32 && btn_id < 64) {
        return 1;
    }
    else if (btn_id >= 64 && btn_id < 96) {
        return 2;
    }
    else {
        return 3;
    }
}

static uint32_t adapter_map_from_axis(struct map_cfg * map_cfg) {
    uint32_t out_mask = BIT(map_cfg->dst_id);
    struct generic_ctrl *out = &ctrl_output[map_cfg->dst_id];
    uint8_t src = map_cfg->src_btn;
    uint8_t dst = map_cfg->dst_btn;
    uint32_t dst_mask = BIT(dst & 0x1F);
    uint32_t dst_btn_idx = btn_id_to_btn_idx(dst);
    uint32_t src_axis_idx = btn_id_to_axis(src);
    uint32_t dst_axis_idx = btn_id_to_axis(dst);

    if (src_axis_idx == AXIS_NONE) {
        return 0;
    }

    /* Check if mapping dst exist in output */
    if (dst_mask & out->mask[dst_btn_idx]) {
        int32_t abs_src_value = abs(ctrl_input->axes[src_axis_idx].value);
        int32_t src_sign = btn_sign(ctrl_input->axes[src_axis_idx].meta->polarity, src);
        int32_t sign_check = src_sign * ctrl_input->axes[src_axis_idx].value;

        /* Check if the srv value sign match the src mapping sign */
        if (sign_check >= 0) {
            /* Check if dst is an axis */
            if (dst_mask & out->desc[dst_btn_idx]) {
                /* Keep track of source axis type */
                out->axes[dst_axis_idx].relative = ctrl_input->axes[src_axis_idx].meta->relative;
                /* Dst is an axis */
                int32_t deadzone = (int32_t)(((float)map_cfg->perc_deadzone/10000) * ctrl_input->axes[src_axis_idx].meta->abs_max) + ctrl_input->axes[src_axis_idx].meta->deadzone;
                /* Check if axis over deadzone */
                if (abs_src_value > deadzone) {
                    int32_t value = abs_src_value - deadzone;
                    int32_t dst_sign = btn_sign(out->axes[dst_axis_idx].meta->polarity, dst);
                    float scale, fvalue;
                    switch (map_cfg->algo & 0xF) {
                        case LINEAR:
                            scale = ((float)out->axes[dst_axis_idx].meta->abs_max / (ctrl_input->axes[src_axis_idx].meta->abs_max - deadzone)) * (((float)map_cfg->perc_max)/100);
                            break;
                        default:
                            scale = ((float)map_cfg->perc_max)/100;
                            break;

                    }
                    fvalue = dst_sign * value * scale;
                    value = (int32_t)fvalue;

                    if (abs(value) > abs(out->axes[dst_axis_idx].value)) {
                        out->axes[dst_axis_idx].value = value;
                        out->axes[dst_axis_idx].cnt_mask = map_cfg->turbo;
                    }
                }
            }
            else {
                /* Dst is a button */
                int32_t threshold = (int32_t)(((float)map_cfg->perc_threshold/100) * ctrl_input->axes[src_axis_idx].meta->abs_max);
                /* Check if axis over threshold */
                if (abs_src_value > threshold) {
                    out->btns[dst_btn_idx].value |= dst_mask;
                    out->btns[dst_btn_idx].cnt_mask[dst & 0x1F] = map_cfg->turbo;
                }
            }
        }
        /* Flag this dst for update */
        out->map_mask[dst_btn_idx] |= dst_mask;
    }

    return out_mask;
}

static uint32_t adapter_map_from_btn(struct map_cfg * map_cfg, uint32_t src_mask, uint32_t src_btn_idx) {
    uint32_t out_mask = BIT(map_cfg->dst_id);
    struct generic_ctrl *out = &ctrl_output[map_cfg->dst_id];
    uint8_t dst = map_cfg->dst_btn;
    uint32_t dst_mask = BIT(dst & 0x1F);
    uint32_t dst_btn_idx = btn_id_to_btn_idx(dst);

    /* Check if mapping dst exist in output */
    if (dst_mask & out->mask[dst_btn_idx]) {
        /* Check if button pressed */
        if (ctrl_input->btns[src_btn_idx].value & src_mask) {
            /* Check if dst is an axis */
            if (dst_mask & out->desc[dst_btn_idx]) {
                /* Dst is an axis */
                uint32_t axis_id = btn_id_to_axis(dst);

                if (axis_id == AXIS_NONE) {
                    return 0;
                }
                else {
                    float fvalue = out->axes[axis_id].meta->abs_max
                                * btn_sign(out->axes[axis_id].meta->polarity, dst)
                                * (((float)map_cfg->perc_max)/100);
                    int32_t value = (int32_t)fvalue;

                    if (abs(value) > abs(out->axes[axis_id].value)) {
                        out->axes[axis_id].value = value;
                        out->axes[axis_id].cnt_mask = map_cfg->turbo;
                    }
                }
            }
            else {
                /* Dst is a button */
                out->btns[dst_btn_idx].value |= dst_mask;
                out->btns[dst_btn_idx].cnt_mask[dst & 0x1F] = map_cfg->turbo;
            }
        }
        /* Flag this dst for update */
        out->map_mask[dst_btn_idx] |= dst_mask;
    }

    return out_mask;
}

static uint32_t adapter_mapping(struct in_cfg * in_cfg) {
    uint32_t out_mask = 0;

    for (uint32_t i = 0; i < in_cfg->map_size; i++) {
        uint8_t src = in_cfg->map_cfg[i].src_btn;
        uint32_t src_mask = BIT(src & 0x1F);
        uint32_t src_btn_idx = btn_id_to_btn_idx(src);

        /* Check if mapping src exist in input */
        if (src_mask & ctrl_input->mask[src_btn_idx]) {
            /* Check if src is an axis */
            if (src_mask & ctrl_input->desc[src_btn_idx]) {
                /* Src is an axis */
                out_mask |= adapter_map_from_axis(&in_cfg->map_cfg[i]);
            }
            else {
                /* Src is a button */
                out_mask |= adapter_map_from_btn(&in_cfg->map_cfg[i], src_mask, src_btn_idx);
            }
        }
    }
    return out_mask;
}

static void adapter_fb_stop_cb(void* arg) {
    struct raw_fb fb_data = {0};

    fb_data.header.wired_id = (uint8_t)(uintptr_t)arg;
    fb_data.header.type = FB_TYPE_RUMBLE;
    fb_data.header.data_len = 0;

    /* Send 0 byte data, system that require callback stop shall look for that */
    adapter_q_fb(&fb_data);
    adapter_fb_stop_timer_stop((uint8_t)(uintptr_t)arg);
}


void adapter_serial_init(struct generic_ctrl *ctrl_input,
		struct generic_ctrl *ctrl_output) {
	uint32_t *mask_tmp = (uint32_t*) malloc(sizeof(uint32_t) * 4);
	mask_tmp[0] = 0xFFFF0FFF;
	mask_tmp[1] = 0;
	mask_tmp[2] = 0;
	mask_tmp[3] = 0;
	ctrl_input->mask = (const uint32_t *) mask_tmp;
	uint32_t *desc_tmp = (uint32_t*) malloc(sizeof(uint32_t) * 4);
	desc_tmp[0] = 0x110000FF;
	desc_tmp[1] = 0;
	desc_tmp[2] = 0;
	desc_tmp[3] = 0;
	ctrl_input->desc = (const uint32_t *) desc_tmp;

	struct in_cfg *in_cfg = &config.in_cfg[0];
	in_cfg->map_size = 1;
	for (uint32_t i = 0; i < ADAPTER_MAPPING_MAX; i++) {
		in_cfg->map_cfg[i] = (struct map_cfg ) { i, i, 0, 100, 50, 135, 0, 0 };
	}

	ctrl_input->index = 0;
	for (int i = 0; i < ADAPTER_MAX_AXES; i++) {
		struct ctrl_meta *meta_tmp = (struct ctrl_meta*) malloc(
				sizeof(struct ctrl_meta));
		meta_tmp->neutral = 127;
		meta_tmp->deadzone = 0;
		meta_tmp->abs_btn_thrs = 0;
		meta_tmp->abs_max = 127;
		meta_tmp->sign = 0;
		meta_tmp->polarity = 0;
		meta_tmp->size_min = 0;
		meta_tmp->size_max = 0;
		meta_tmp->relative = 0;
		ctrl_input->axes[i].meta = meta_tmp;
		ctrl_input->axes[i].value = 0;
		ctrl_input->axes[i].relative = 0;
		ctrl_input->axes[i].cnt_mask = 0;
	}
	for (int i = 0; i < 4; i++) {
		ctrl_input->btns[i].value = 0;
		ctrl_input->map_mask[i] = 0;
		for (int i2 = 0; i2 < 32; i2++) {
			ctrl_input->btns[i].cnt_mask[i2] = 0;
		}
	}
	for (int i = 0; i < WIRED_MAX_DEV; i++) {
		ctrl_output[i].index = i;
	}
}


void print_cfg(struct in_cfg *in_cfg) {

	printf("inicio-----------------------------\n");
	printf("in_cfg:\n");
	printf("in_cfg->bt_dev_id: %d, ", in_cfg->bt_dev_id);
	printf("in_cfg->bt_subdev_id: %d, ", in_cfg->bt_subdev_id);
	printf("in_cfg->map_size: %d, ", in_cfg->map_size);
	printf("in_cfg->map_cfg:\n");
	for (int i = 0; i < ADAPTER_MAPPING_MAX; ++i) {
		printf(".\nin_cfg->map_cfg[%d]->src_btn: %d\n", i,
				in_cfg->map_cfg[i].src_btn);
		printf("in_cfg->map_cfg[%d]->dst_btn: %d\n", i,
				in_cfg->map_cfg[i].dst_btn);
		printf("in_cfg->map_cfg[%d]->dst_id: %d\n", i,
				in_cfg->map_cfg[i].dst_id);
		printf("in_cfg->map_cfg[%d]->perc_max: %d\n", i,
				in_cfg->map_cfg[i].perc_max);
		printf("in_cfg->map_cfg[%d]->perc_threshold: %d\n", i,
				in_cfg->map_cfg[i].perc_threshold);
		printf("in_cfg->map_cfg[%d]->perc_deadzone: %d\n", i,
				in_cfg->map_cfg[i].perc_deadzone);
		printf("in_cfg->map_cfg[%d]->turbo: %d\n", i, in_cfg->map_cfg[i].turbo);
		printf("in_cfg->map_cfg[%d]->algo: %d\n", i, in_cfg->map_cfg[i].algo);
	}
	printf("\nconfig.out_cfg[i]:\n");
	for (int i = 0; i < WIRED_MAX_DEV; ++i) {
		printf("config.out_cfg[%d].dev_mode: %d \n", i, config.out_cfg[i].dev_mode);
		printf("config.out_cfg[%d].acc_mode: %d \n", i, config.out_cfg[i].acc_mode);
	}
	printf("-----------------------------fim\n");
}

void print_ctrl(struct generic_ctrl *ctrl_input) {
	printf("inicio-----------------------------\n");
	printf("ctrl_input->index: %ld\n", ctrl_input->index);
	for (int i = 0; i < 4; ++i) {
		printf("ctrl_input->mask[%d]: %lX\n",i ,ctrl_input->mask[i]);
		printf("ctrl_input->desc[%d]: %lX\n",i ,ctrl_input->desc[i]);
	}
	printf(".\n\nctrl_input->map_mask: \n");
	for (int i = 0; i < 4; i++) {
		printf("ctrl_input->map_mask[%d]-> %ld, ", i, ctrl_input->map_mask[i]);
	}
	printf("\n\nctrl_input->btns: \n");
	for (int i = 0; i < 4; i++) {
		printf("\nctrl_input->btns[%d]->value %ld:\n", i,
				ctrl_input->btns[i].value);
		printf("ctrl_input->btns[%d]->cnt_mask: \n", i);
		for (int i2 = 0; i2 < 32; ++i2) {
			printf("ctrl_input->btns[%d]->cnt_mask[%d]: %ld, \n", i, i2,
					ctrl_input->btns[i].cnt_mask[i2]);
		}
	}
	printf("\naxes\n");
	for (int i = 0; i < 6; i++) {
		printf("\nctrl_input->axes[%d]->value: %ld, ", i,
				ctrl_input->axes[i].value);
		printf("ctrl_input->axes[%d]->relative: %ld, \n", i,
				ctrl_input->axes[i].relative);
		printf("ctrl_input->axes[%d]->meta\n", i);
		printf("ctrl_input->axes[%d]->meta->neutral: %ld \n", i,
				ctrl_input->axes[i].meta->neutral);
		printf("ctrl_input->axes[%d]->meta->deadzone: %ld \n", i,
				ctrl_input->axes[i].meta->deadzone);
		printf("ctrl_input->axes[%d]->meta->abs_btn_thrs: %ld \n", i,
				ctrl_input->axes[i].meta->abs_btn_thrs);
		printf("ctrl_input->axes[%d]->meta->abs_max: %ld \n", i,
				ctrl_input->axes[i].meta->abs_max);
		printf("ctrl_input->axes[%d]->meta->sign: %ld \n", i,
				ctrl_input->axes[i].meta->sign);
		printf("ctrl_input->axes[%d]->meta->polarity: %ld \n", i,
				ctrl_input->axes[i].meta->polarity);
		printf("ctrl_input->axes[%d]->meta->size_min: %ld \n", i,
				ctrl_input->axes[i].meta->size_min);
		printf("ctrl_input->axes[%d]->meta->size_max: %ld \n", i,
				ctrl_input->axes[i].meta->size_max);
		printf("ctrl_input->axes[%d]->meta->relative: %ld \n", i,
				ctrl_input->axes[i].meta->relative);
		printf("ctrl_input->axes[%d]->cnt_mask: %lu, \n", i,
				ctrl_input->axes[i].cnt_mask);
	}
	printf("-----------------------------fim\n");
}

int32_t btn_id_to_axis(uint8_t btn_id) {
    switch (btn_id) {
        case PAD_LX_LEFT:
        case PAD_LX_RIGHT:
        case MOUSE_WX_LEFT:
        case MOUSE_WX_RIGHT:
            return AXIS_LX;
        case PAD_LY_DOWN:
        case PAD_LY_UP:
        case MOUSE_WY_DOWN:
        case MOUSE_WY_UP:
            return AXIS_LY;
        case PAD_RX_LEFT:
        case PAD_RX_RIGHT:
        //case MOUSE_X_LEFT:
        //case MOUSE_X_RIGHT:
            return AXIS_RX;
        case PAD_RY_DOWN:
        case PAD_RY_UP:
        //case MOUSE_Y_DOWN:
        //case MOUSE_Y_UP:
            return AXIS_RY;
        case PAD_LM:
            return TRIG_L;
        case PAD_RM:
            return TRIG_R;
    }
    return AXIS_NONE;
}

uint32_t axis_to_btn_mask(uint8_t axis) {
    switch (axis) {
        case AXIS_LX:
            return BIT(PAD_LX_LEFT) | BIT(PAD_LX_RIGHT) | BIT(MOUSE_WX_LEFT) | BIT(MOUSE_WX_RIGHT);
        case AXIS_LY:
            return BIT(PAD_LY_DOWN) | BIT(PAD_LY_UP) | BIT(MOUSE_WY_DOWN) | BIT(MOUSE_WY_UP);
        case AXIS_RX:
            return BIT(PAD_RX_LEFT) | BIT(PAD_RX_RIGHT); /* BIT(MOUSE_X_LEFT) | BIT(MOUSE_X_RIGHT) */
        case AXIS_RY:
            return BIT(PAD_RY_DOWN) | BIT(PAD_RY_UP); /* BIT(MOUSE_Y_DOWN) | BIT(MOUSE_Y_UP) */
        case TRIG_L:
            return BIT(PAD_LM);
        case TRIG_R:
            return BIT(PAD_RM);
    }
    return 0x00000000;
}

uint32_t IRAM_ATTR axis_to_btn_id(uint8_t axis) {
    switch (axis) {
        case AXIS_LX:
            return PAD_LX_LEFT;
        case AXIS_LY:
            return PAD_LY_DOWN;
        case AXIS_RX:
            return PAD_RX_LEFT;
        case AXIS_RY:
            return PAD_RY_DOWN;
        case TRIG_L:
            return PAD_LM;
        case TRIG_R:
            return PAD_RM;
    }
    return 0;
}

int8_t btn_sign(uint32_t polarity, uint8_t btn_id) {
    switch (btn_id) {
        case PAD_LX_RIGHT:
        case PAD_LY_UP:
        case PAD_RX_RIGHT:
        case PAD_RY_UP:
        case PAD_LM:
        case PAD_RM:
        case MOUSE_WX_RIGHT:
        case MOUSE_WY_UP:
        //case MOUSE_X_RIGHT:
        //case MOUSE_Y_UP:
            return polarity ? -1 : 1;
        case PAD_LX_LEFT:
        case PAD_LY_DOWN:
        case PAD_RY_DOWN:
        case PAD_RX_LEFT:
        case MOUSE_WX_LEFT:
        case MOUSE_WY_DOWN:
        //case MOUSE_X_LEFT:
        //case MOUSE_Y_DOWN:
            return polarity ? 1 : -1;
    }
    return 1;
}

void IRAM_ATTR adapter_init_buffer(uint8_t wired_id) {
    if (wired_adapter.system_id != WIRED_AUTO) {
        wired_adapter.data[wired_id].index = wired_id;
        wired_init_buffer(config.out_cfg[wired_id].dev_mode, &wired_adapter.data[wired_id]);
    }
}

void serial_to_input(char cmd_char[8], struct generic_ctrl *ctrl_input) {
	ctrl_input->btns[0].value = 0;
	ctrl_input->axes[0].value = 0;
	ctrl_input->axes[1].value = 0;
	switch (cmd_char[0]) {
	case 'a': // left
		ctrl_input->btns[0].value = 0x100;
		break;
	case 's': // down
		ctrl_input->btns[0].value = 0x400;
		break;
	case 'd': // right
		ctrl_input->btns[0].value = 0x200;
		break;
	case 'w': // up
		ctrl_input->btns[0].value = 0x800;
		break;
	case 'z': // select
		ctrl_input->btns[0].value = 0x200000;
		break;
	case 'x': // start
		ctrl_input->btns[0].value = 0x100000;
		break;
	case 'k': // A
		ctrl_input->btns[0].value = 0x40000;
		break;
	case 'l': // B
		ctrl_input->btns[0].value = 0x20000;
		break;
	case 'i': // X
		ctrl_input->btns[0].value = 0x10000;
		break;
	case 'o': // Y
		ctrl_input->btns[0].value = 0x80000;
		break;
	case 'f': // axix-left
		ctrl_input->axes[0].value = 0xFFFFFF80;
		break;
	case 'g': // axix-down
		ctrl_input->axes[1].value = 0x0000007F;
		break;
	case 'h': // axix-right
		ctrl_input->axes[0].value = 0x0000007F;
		break;
	case 't': // axix-up
		ctrl_input->axes[1].value = 0xFFFFFF80;
		break;
	case 'q': // LB
		ctrl_input->btns[0].value = 0x2000000;
		break;
	case 'e': // RB
		ctrl_input->btns[0].value = 0x20000000;
		break;
	default:
		break;
	}
}

/**
 * steps to reproduce:
 * build (sdk hw1), flash, monitor
 * pair some BT gamepad, wait to sync
 * send bytes to serial port
 */
void serial_bridge(char bytes[8]) {
	if(bytes[0] == '0'){
		adapter_serial_init(ctrl_input, ctrl_output);
	}
	/*
	 *  serial_to_input changes:
	 *  ctrl_input->btns[0].value,
	 *  ctrl_input->axes[0].value and ctrl_input->axes[1].value
	 *  with some logic faking the BT gamepad that I have
	 */
	serial_to_input(bytes, ctrl_input);
//	print_ctrl(ctrl_input);
	adapter_debug_print(ctrl_input); // printf debug
	if (wired_meta_init(ctrl_output)) {
		/* Unsupported system */
		return;
	}
	adapter_mapping(&config.in_cfg[0]);
	adapter_debug_print(&ctrl_output[0]); // printf debug
	printf("--------------------\n");
	wired_from_generic(config.out_cfg[0].dev_mode, &ctrl_output[0], &wired_adapter.data[0]);
}

void adapter_bridge(struct bt_data *bt_data) {
    uint32_t out_mask = 0;

    if (bt_data->base.pids->type != BT_NONE) {
        if (wireless_to_generic(bt_data, ctrl_input)) {
            /* Unsupported report */
            return;
        }

        sys_macro_hdl(ctrl_input, &bt_data->base.flags[PAD]);

#ifdef CONFIG_BLUERETRO_ADAPTER_INPUT_DBG
        adapter_debug_print(ctrl_input);
#ifdef CONFIG_BLUERETRO_ADAPTER_RUMBLE_DBG
        if (ctrl_input->btns[0].value & BIT(PAD_RB_DOWN)) {
            uint8_t tmp = 0;
            adapter_q_fb(&tmp, 1);
        }
#endif
#else
        if (wired_adapter.system_id != WIRED_AUTO) {
            if (wired_meta_init(ctrl_output)) {
                /* Unsupported system */
                return;
            }

            print_cfg(&config.in_cfg[bt_data->base.pids->out_idx]);

            out_mask = adapter_mapping(&config.in_cfg[bt_data->base.pids->out_idx]);

#ifdef CONFIG_BLUERETRO_ADAPTER_INPUT_MAP_DBG
            adapter_debug_print(&ctrl_output[0]);
#else
            for (uint32_t i = 0; out_mask; i++, out_mask >>= 1) {
                if (out_mask & 0x1) {
                    ctrl_output[i].index = i;
                    wired_from_generic(config.out_cfg[i].dev_mode, &ctrl_output[i], &wired_adapter.data[i]);
                }
            }
#endif /* CONFIG_BLUERETRO_ADAPTER_INPUT_MAP_DBG */
        }
#endif /* CONFIG_BLUERETRO_ADAPTER_INPUT_DBG */
    }
}

void adapter_fb_stop_timer_start(uint8_t dev_id, uint64_t dur_us) {
    if (wired_adapter.data[dev_id].fb_timer_hdl == NULL) {
        const esp_timer_create_args_t fb_timer_args = {
            .callback = &adapter_fb_stop_cb,
            .arg = (void*)(uintptr_t)dev_id,
            .name = "fb_timer"
        };
        esp_timer_create(&fb_timer_args, (esp_timer_handle_t *)&wired_adapter.data[dev_id].fb_timer_hdl);
        esp_timer_start_once(wired_adapter.data[dev_id].fb_timer_hdl, dur_us);
    }
}

void adapter_fb_stop_timer_stop(uint8_t dev_id) {
    if (wired_adapter.data[dev_id].fb_timer_hdl) {
        esp_timer_delete(wired_adapter.data[dev_id].fb_timer_hdl);
        wired_adapter.data[dev_id].fb_timer_hdl = NULL;
    }
}

uint32_t adapter_bridge_fb(struct raw_fb *fb_data, struct bt_data *bt_data) {
    uint32_t ret = 0;
#ifndef CONFIG_BLUERETRO_ADAPTER_RUMBLE_DBG
    if (wired_adapter.system_id != WIRED_AUTO) {
        wired_fb_to_generic(config.out_cfg[bt_data->base.pids->id].dev_mode, fb_data, &fb_input);
#else
        fb_input.state ^= 0x01;
#endif
        if (bt_data->base.pids->type != BT_NONE) {
            wireless_fb_from_generic(&fb_input, bt_data);
            ret = 1;
        }
#ifndef CONFIG_BLUERETRO_ADAPTER_RUMBLE_DBG
    }
#endif
    return ret;
}

void IRAM_ATTR adapter_q_fb(struct raw_fb *fb_data) {
    /* Best efford only on fb */
    queue_bss_enqueue(wired_adapter.input_q_hdl, (uint8_t *)fb_data, sizeof(*fb_data));
}

void adapter_init(void) {
    wired_adapter.system_id = WIRED_AUTO;

    /* Save regular DRAM by allocating big sruct w/ only 32bits access in IRAM */
    ctrl_input = heap_caps_malloc(sizeof(struct generic_ctrl), MALLOC_CAP_32BIT);
    if (ctrl_input == NULL) {
        printf("# %s ctrl_input alloc fail\n", __FUNCTION__);
    }
    ctrl_output = heap_caps_malloc(sizeof(struct generic_ctrl) * WIRED_MAX_DEV, MALLOC_CAP_32BIT);
    if (ctrl_output == NULL) {
        printf("# %s ctrl_output alloc fail\n", __FUNCTION__);
    }
    for (uint32_t i = 0; i < BT_MAX_DEV; i++) {
        bt_adapter.data[i].raw_src_mappings = heap_caps_malloc(sizeof(struct raw_src_mapping) * REPORT_MAX, MALLOC_CAP_32BIT);
        if (bt_adapter.data[i].raw_src_mappings == NULL) {
            printf("# %s bt_adapter.data[%ld].raw_src_mappings alloc fail\n", __FUNCTION__, i);
        }
        bt_adapter.data[i].reports = heap_caps_malloc(sizeof(struct hid_report) * REPORT_MAX, MALLOC_CAP_32BIT);
        if (bt_adapter.data[i].reports == NULL) {
            printf("# %s bt_adapter.data[%ld].reports alloc fail\n", __FUNCTION__, i);
        }
    }

    wired_adapter.input_q_hdl = queue_bss_init(16, sizeof(struct raw_fb));
    if (wired_adapter.input_q_hdl == NULL) {
        ets_printf("# %s: Failed to create fb queue\n", __FUNCTION__);
    }

#ifdef CONFIG_BLUERETRO_BT_DISABLE
    adapter_serial_init(ctrl_input, ctrl_output);
#endif
}
