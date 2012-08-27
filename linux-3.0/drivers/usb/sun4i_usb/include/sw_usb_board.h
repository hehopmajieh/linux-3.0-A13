/*
*************************************************************************************
*                         			      Linux
*					           USB Host Controller Driver
*
*				        (c) Copyright 2006-2010, All winners Co,Ld.
*							       All Rights Reserved
*
* File Name 	: sw_usb_board.h
*
* Author 		: javen
*
* Description 	: �弶����
*
* Notes         :
*
* History 		:
*      <author>    		<time>       	<version >    		<desc>
*       javen     	  2010-12-20           1.0          create this file
*
*************************************************************************************
*/
#ifndef  __SW_USB_BOARD_H__
#define  __SW_USB_BOARD_H__

//----------------------------------------------------------
//
//----------------------------------------------------------
#define  SET_USB_PARA				"usb_para"
#define  SET_USB0					"usbc0"
#define  SET_USB1					"usbc1"
#define  SET_USB2					"usbc2"

#define  KEY_USB_GLOBAL_ENABLE		"usb_global_enable"
#define  KEY_USBC_NUM				"usbc_num"

#define  KEY_USB_ENABLE				"usb_used"
#define  KEY_USB_PORT_TYPE			"usb_port_type"
#define  KEY_USB_DETECT_TYPE		"usb_detect_type"
#define  KEY_USB_ID_GPIO			"usb_id_gpio"
#define  KEY_USB_DETVBUS_GPIO		"usb_det_vbus_gpio"
#define  KEY_USB_DRVVBUS_GPIO		"usb_drv_vbus_gpio"

#define  KEY_USB_HOST_INIT_STATE    "usb_host_init_state"

//---------------------------------------------------
//
//  USB  ������Ϣ
//
//---------------------------------------------------
enum usb_gpio_group_type{
    GPIO_GROUP_TYPE_PIO = 0,
    GPIO_GROUP_TYPE_POWER,
};

/* 0: device only; 1: host only; 2: otg */
enum usb_port_type{
    USB_PORT_TYPE_DEVICE = 0,
    USB_PORT_TYPE_HOST,
    USB_PORT_TYPE_OTG,
};

/* 0: dp/dm��⣬ 1: vbus/id��� */
enum usb_detect_type{
    USB_DETECT_TYPE_DP_DM = 0,
    USB_DETECT_TYPE_VBUS_ID,
};

/* pio��Ϣ */
typedef struct usb_gpio{
	__u32 valid;          	/* pio�Ƿ���á� 0:��Ч, !0:��Ч	*/

	__u32 group_type;		/* pio���� 							*/
	user_gpio_set_t gpio_set;
}usb_gpio_t;

typedef struct usb_port_info{
	__u32 enable;          				/* port�Ƿ����			*/

	__u32 port_no;						/* usb�˿ں�			*/
	enum usb_port_type port_type;    	/* usb�˿�����			*/
	enum usb_detect_type detect_type; 	/* usb��ⷽʽ			*/

	usb_gpio_t id;						/* usb id pin��Ϣ 		*/
	usb_gpio_t det_vbus;				/* usb vbus pin��Ϣ 	*/
	usb_gpio_t drv_vbus;				/* usb drv_vbus pin��Ϣ	*/
	__u32 host_init_state;				/* usb �������ĳ�ʼ��״̬��0 : ������. 1 : ���� */
}usb_port_info_t;

typedef struct usb_cfg{
	u32 usb_global_enable;
	u32 usbc_num;

	struct usb_port_info port[USBC_MAX_CTL_NUM];
}usb_cfg_t;

#endif   //__SW_USB_BOARD_H__
