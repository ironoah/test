/*
 * ata.c
 *
 * Copyright 2002, Minoru Murashima. All rights reserved.
 * Distributed under the terms of the BSD License.
 *
 * ATA device driver
 */


#include"config.h"
#include"types.h"
#include"lib.h"
#include"interrupt.h"
#include"proc.h"
#include"time.h"
#include"errno.h"
#include"mm.h"
#include"mp.h"
#include"sp.h"
#include"pci.h"
#include"fs.h"
#include"device.h"


enum{
	TIME_OUT=2000,			/* Time out ms */
	IDENTIFY_SIZE=512,		/* ATA ATAPI identify buffer size */
	ATA_SECTOR_SIZE=512,		/* ATA disk sector size */

	/* IO register */
	PRIM_BASE=0x1F0,
	SECN_BASE=0x170,

	/* status bit */
	BSY_BIT=0x80,
	DF_BIT=0x20,
	SRV_BIT=0x10,
	DRQ_BIT=0x8,
	ERR_BIT=0x1,
	CHK_BIT=0x1,

	/* Interrupt reason bit */
	CD_BIT=0x1,
	IO_BIT=0x2,
	REL_BIT=0x4,

	/* IRQ number */
	PRIM_IRQ=14,
	SECOND_IRQ=15,

	/* Set features subcommand code */
	SET_TRANSFER=0x3,	/* Set transfer mode */

	/* Transfer mode of set features subcommand */
	SUB_PIO_DEF=0x0,
	SUB_PIO_FLO=0x8,
	SUB_M_DMA=0x20,
	SUB_U_DMA=0x40,

	/* Transfer mode bit in identify infomation */
	PIO4=0x2,		/* PIO mode4 */
	PIO3=0x1,		/* PIO mode3 */
	M_DMA2=0x4,		/* Multi DMA mode2 */
	M_DMA1=0x2,		/* Multi DMA mode1 */
	M_DMA0=0x1,		/* Multi DMA mode0 */
	U_DMA6=0x40,	/* Ultra DMA mode6 */
	U_DMA5=0x20,	/* Ultra DMA mode5 */
	U_DMA4=0x10,	/* Ultra DMA mode4 */
	U_DMA3=0x8,		/* Ultra DMA mode3 */
	U_DMA2=0x4,		/* Ultra DMA mode2 */
	U_DMA1=0x2,		/* Ultra DMA mode1 */
	U_DMA0=0x1,		/* Ultra DMA mode0 */

	/* Packet command features flag */
	PACKET_DMA=0x1,		/* DMA transfer */
	PACKET_OVERLAP=0x2, /* Over lapped */

	/* Conect device infomation */
	ATA=1,
	ATAPI=2,
	PIO=1,		/* PIO transfer mode */
	M_DMA=2,	/* Multi DMA transfer mode */
	U_DMA=3,	/* Ultra DMA transfer mode */

	/* Interrupt mode */
	INTR_DISABLE=0,
	INTR_ENABLE=1,

	/* DMA mode values */
	READ_DMA=0,		/* Read from fd */
	WRITE_DMA=1,	/* Write to fd */

	/* Trasfer mode */
	READ=0,
	WRITE=1,

	LBA_BIT=0x40,			/* LBA bit in Device_head register */
	ATAPI_LBA_BIT=0x200, 	/* LBA enable bit in ATAPI identify infomation */

	/* IDE Bus Master IO register */
	PCI_CONF_BMBASE=0x20,	/* IDE Bus Master IO base address register in PCI Configration */
	IDE_BMIC=0x0,			/* Bus Master IDE Command register */
	IDE_BMIS=0x2,			/* Bus Master IDE Status register */
	IDE_BMIDTP=0x4,			/* Bus Master IDE Descriptor Table Pointer register(4byte) */
	IDE_BMIO_SECOND=0x8,	/* セカンダリーホストの場合にレジスター値にプラスする値 */
	PRD_EOT=0x1<<31,		/* PRD EOT bit */

	/* ATAPI function flag */
	PACK_OVL=0x2,			/* Packet feature overlappe flag */
	PACK_DMA=0x1,			/* Packet feature DMA flag */
	ATAPI_OVL=0x2000,		/* Overlappe flag in ATAPI function */

	/* Packet start stop unit flag */
	SSU_STOP=0,				/* Disk stop */
	SSU_START=0x1,			/* Disk start */
	SSU_EJECT=0x2,			/* Disk eject */
	SSU_STANBY=0x30,		/* Stanby */
};


/* ATA IO register structure */
typedef struct{
	int dtr;	/* ATA_DTR=0 */
	int err;	/* ATA_ERR=1 */
	int ftr;	/* ATA_FTR=1 */
    int scr;	/* ATA_SCR=2 */
    int irr;	/* ATA_IRR=2 */
    int snr;	/* ATA_SNR=3 */
    int clr;	/* ATA_CLR=4 */
    int blr;	/* ATA_BLR=4 */
    int chr;	/* ATA_CHR=5 */
    int bhr;	/* ATA_BHR=5 */
    int dhr;	/* ATA_DHR=6 */
    int str;	/* ATA_STR=7 */
    int cmr;	/* ATA_CMR=7 */
    int ctr;	/* ATA_CTR=0x206 */
    int astr;	/* ATA_ASTR=0x206 */
}ATA_REG;

/* ATA ATAPI identify infomation(ATA-5仕様) */
typedef struct{
	ushort	config;				/* 0 定義違いあり */
	ushort	n_cyl;				/* 1 ATA only */
	ushort	sub_command;		/* 2 */
	ushort	n_head;				/* 3 ATA only */
	ushort	vender_def1[2];		/* 4 */
	ushort	n_sect;				/* 6 ATA only */
	ushort	reserv1[3];			/* 7 */
	uchar	serial_number[20];	/* 10 */
	ushort	nodef1[3];			/* 20 */
	uchar	firm_ware[8];		/* 23 */
	uchar	model[40];			/* 27 */
	ushort	multi_intr;			/* 47 ATA only */
	ushort	reserv2;			/* 48 */
	ushort	iordy;				/* 49 定義違いあり */
	ushort	stby_timer;			/* 50 ATA only */
	ushort	nodef2[2];			/* 51 */
	ushort	word_enable;		/* 53 */
	ushort	current_n_cyl;		/* 54 ATA only */
	ushort	current_n_head;		/* 55 ATA only */
	ushort	current_n_sect;		/* 56 ATA only */
	ushort	all_sect[2];		/* 57 ATA only */
	ushort	multiple;			/* 59 ATA only */
	ushort	lba_all_sect[2];	/* 60 ATA only */
	ushort	nodef3;				/* 62 */
	ushort	multi_dma;			/* 63 */
	ushort	pio;				/* 64 */
	ushort	min_mdma_cycl;		/* 65 */
	ushort	rcm_mdma_cycl;		/* 66 */
	ushort	min_flw_pio_cycl;	/* 67 */
	ushort	min_nflw_pio_cycl;	/* 68 */
	ushort	reserv3[2];			/* 69 */
	ushort	pck_bus_release;	/* 71 ATAPI only */
	ushort	bsy_clear_time;		/* 72 ATAPI only */
	ushort	reserv4[2];			/* 73 */
	ushort	max_cue_size;		/* 75 */
	ushort	reserv5[4];			/* 76 */
	ushort	major_num;			/* 80 */
	ushort	minor_num;			/* 81 */
	ushort	cmd1;				/* 82 */
	ushort	cmd2;				/* 83 */
	ushort	cmd_ext;			/* 84 */
	ushort	cmd1_enable;		/* 85 */
	ushort	cmd2_enable;		/* 86 */
	ushort	cmd_ext_enable;		/* 87 */
	ushort	ultra_dma;			/* 88 */
	ushort	secu_erase_time;	/* 89 ATA only */
	ushort	esecu_erase_time;	/* 90 ATA only */
	ushort	pmm_value;			/* 91 ATA only */
	ushort	pass_rev_coad;		/* 92 ATA only */
	ushort	hard_reset_info;	/* 93 */
	ushort	reserv6[32];		/* 94 */
	ushort	atapi_byte_count;	/* 126 ATAPI only */
	ushort	remov_set;			/* 127 */
	ushort	secu_stat;			/* 128 */
	ushort	vender_def2[31];	/* 129 */
	ushort	cfa_pm;				/* 160 */
}ID_INFO;

/* ATA command parameters */
typedef struct{
	uchar device;
	uchar count;
	uchar sector;
	uchar cyl_low;
	uchar cyl_high;
}ATA_PARAM;

/* Packet command parameters */
typedef struct{
	void *buf;			/* Block read write buffer */
	int size;			/* Transfer bytes or Secter size */
	uchar feutures;		/* Packet command flag */
	uchar packet[14];	/* Packet command parameters and return packet */
}PACKET_PARAM;

/* Conect device */
typedef struct{
	int type;				/* ATA=1 or ATAPI=2 */
	int mode;				/* PIO=0,Multi DMA=1,Ultra DMA=2 */
	int sector_size;		/* Secter size */
	uint all_sectors;		/* LBA all sectors */
	int flag;				/* Function flag */
	int (*transfer)(int,int,int,void*,int,uint); /* Tranfer function */
}CONECT_DEV;

/* Physical Region Descriptor for IDE Busmaster */
typedef struct{
	void *phys_addr;	/* Physical address */
	uint count;			/* EOT bit(31)|Transfer count by bytes,Max 64kbyte */
}PRD;


/* ATA IO register */
static ATA_REG reg[2]={
	{
		PRIM_BASE+0,PRIM_BASE+1,PRIM_BASE+1,PRIM_BASE+2,PRIM_BASE+2,PRIM_BASE+3,PRIM_BASE+4,PRIM_BASE+4,
		PRIM_BASE+5,PRIM_BASE+5,PRIM_BASE+6,PRIM_BASE+7,PRIM_BASE+7,PRIM_BASE+0x206,PRIM_BASE+0x206
	},
	{
		SECN_BASE+0,SECN_BASE+1,SECN_BASE+1,SECN_BASE+2,SECN_BASE+2,SECN_BASE+3,SECN_BASE+4,SECN_BASE+4,
		SECN_BASE+5,SECN_BASE+5,SECN_BASE+6,SECN_BASE+7,SECN_BASE+7,SECN_BASE+0x206,SECN_BASE+0x206
	}
};
static CONECT_DEV conect_dev[2][2]={			/* Conect device infomation */
	{{0,0,0,0,0,NULL},{0,0,0,0,0,NULL}},
	{{0,0,0,0,0,NULL},{0,0,0,0,0,NULL}}
};
static int current_intr[2];						/* Current host interrupt mode,enable=1 or diable=0 */
static uint64 time_out;							/* Time out counts */
static WAIT_INTR wait_intr_queue[2]={			/* 割り込み待ち用 */
	{NULL,0},{NULL,0}
};
static WAIT_QUEUE wait_queue[2]={				/* 処理待ち用Wait queue */
	{NULL,(PROC*)&wait_queue[0],0,0},
	{NULL,(PROC*)&wait_queue[1],0,0}
};
static int ide_base[2];							/* IDE Bus Master IO base address */
static uchar irq_num[2]={PRIM_IRQ,SECOND_IRQ};	/* IRQ number */
static PRD prd[2];								/* Physical Region Descriptor */


static int check_busy(int);
static void set_intr(int,int);
static int prim_intr_handler();
static int second_intr_handler();
static int change_mode(int,int,int);
static int read_pio(int,int,void*,int,int);
static int write_pio(int,int,void*,int,int);
static int read_dma(int,int,void*,int,int);
static int write_dma(int,int,void*,int,int);
static int init_ide_busmaster(int,PCI_INFO*);
static int reset_host(int);
static char *cnv_idinfo_str(char*,int);
static int soft_reset();
static int device_select(int,int);
static int _transfer_ata(int,int,int,void*,int,uint);
static int reset_device(int,int);
static int identify_device(int,int,int,void*);
static int idle_immediate_device(int,int);
static int init_device_param(int,int,uchar,uchar);
static int set_features(int,int,uchar,uchar);
static int issue_packet_command(int,int,PACKET_PARAM*);
static int test_unit_ready(int,int);
static int request_sense(int,int);
static int start_stop_unit(int,int,uchar);
static int read_capacity(int,int);
static int _transfer_atapi(int,int,int,void*,int,uint);
static int transfer(int,int,int,void*,size_t,size_t);
static int test_atapi(int,int);
static int open_hda();
static int open_hdb();
static int open_hdc();
static int open_hdd();
static int read_hda(void*,size_t,size_t);
static int read_hdb(void*,size_t,size_t);
static int read_hdc(void*,size_t,size_t);
static int read_hdd(void*,size_t,size_t);
static int write_hda(void*,size_t,size_t);
static int write_hdb(void*,size_t,size_t);
static int write_hdc(void*,size_t,size_t);
static int write_hdd(void*,size_t,size_t);
static int ioctl_hda(int,void*);
static int ioctl_hdb(int,void*);
static int ioctl_hdc(int,void*);
static int ioctl_hdd(int,void*);


static DEV_INFO hd_info[2][2]={
	{{"hda",0,0,0,open_hda,read_hda,write_hda,ioctl_hda},{"hdb",0,0,0,open_hdb,read_hdb,write_hdb,ioctl_hdb}},
	{{"hdc",0,0,0,open_hdc,read_hdc,write_hdc,ioctl_hdc},{"hdd",0,0,0,open_hdd,read_hdd,write_hdd,ioctl_hdd}}
};


/*
 * Data transfer
 * parameters : Host number,Device number,Mode=READ or WRITE,buffer,Transfer blocks,begin block
 * return : Transfer size or Error number
 */
extern inline int transfer(int host,int dev,int mode,void *buf,size_t blocks,size_t begin)
{
	int error;
	int rest;


	if(blocks==0)return 0;
	if(begin+blocks>conect_dev[host][dev].all_sectors)return PRINT_ERR(EINVAL,"transfer");

	wait_proc(&wait_queue[host]);
	{
		/* SMPなら割り込みが同じcpuに発生するようにする */
		if(MFPS_addres)set_intr_cpu(irq_num[host],get_current_cpu());

		/* 転送開始 */
		if((error=conect_dev[host][dev].transfer(host,dev,mode,buf,blocks,begin))!=0)rest=error;
		else rest=blocks;
	}
	wake_proc(&wait_queue[host]);

	return rest;
}


/*
 * Check busy flag in status register
 * parameters : Stat register,Mask,Compare rest value
 * return : Status value
 */
int check_busy(int str)
{
	uchar in;
	uint64 count;


	count=rdtsc();
	while((in=inb(str))&BSY_BIT)
		if(rdtsc()-count>time_out)return in;

	return in;
}


/*
 * Set interrupt mode
 * parameters : Host,Interrupt mode
 */
void set_intr(int host,int flag)
{
	if(flag==current_intr[host])return;

	if(flag==INTR_DISABLE)
	{
		set_irq_mask(irq_num[host]);
		outb(reg[host].ctr,0x2);			/* Disable interrupt */
		mili_timer(5);						/* wait */
		current_intr[host]=INTR_DISABLE;
	}
	else
	{
		outb(reg[host].ctr,0);				/* Enable interrupt */
		release_irq_mask(irq_num[host]);
		mili_timer(5);						/* wait */
		current_intr[host]=INTR_ENABLE;
	}
}


/*
 * Primary ATA interrupt handler
 * return : Task switch on
 */
int prim_intr_handler()
{
/***************************************/
	printk("Interrupt IRQ14\n");
/***************************************/
	wake_intr(&wait_intr_queue[0]);

	return 1;
}


/*
 * Secondary ATA interrupt handler
 * return : Task switch on
 */
int second_intr_handler()
{
/***************************************/
	printk("Interrupt IRQ15\n");
/***************************************/
	wake_intr(&wait_intr_queue[1]);

	return 1;
}


/*
 * Change transfer mode
 * parameters : Host number,Device number,Transfer mode(PIO=1 or Multi DMA=2 or Ultra DMA=3)
 * return : 0 or Error number
 */
int change_mode(int host,int dev,int mode)
{
	uchar subcm;
	uint value;
	int error;
	ID_INFO *id_info;
	PCI_INFO ide;


	if(conect_dev[host][dev].mode==mode)return 0;

	if((id_info=(ID_INFO*)kmalloc(IDENTIFY_SIZE))==NULL)return PRINT_ERR(ENOMEM,"change_mode");

	/* Read identify infomation */
	if((error=identify_device(host,dev,conect_dev[host][dev].type,id_info))!=0)return error;

	if(mode==PIO)
	{
		if(id_info->pio&PIO4)subcm=SUB_PIO_FLO|4;
		else if(id_info->pio&PIO3)subcm=SUB_PIO_FLO|3;
		else subcm=SUB_PIO_DEF;
	}
	else if(mode==M_DMA)
	{
		/* Set ATA transfer mode */
		if(id_info->multi_dma&M_DMA2)subcm=SUB_M_DMA|2;
		else if(id_info->multi_dma&M_DMA1)subcm=SUB_M_DMA|1;
		else if(id_info->multi_dma&M_DMA0)subcm=SUB_M_DMA|0;
		else return PRINT_ERR(ENOSYS,"change_mode");

		/*
		 * ULTRA DMA対応のドライブについては、BIOSでIDEがULTRA DMAに
		 * 設定されているので、その設定を取り消す必要がある
		 */
		if((error=init_ide_busmaster(host,&ide))!=0)return error;

		switch(ide.vender)
		{
			case 0x24CB8086:		/* Intel ICH4 */
			case 0x248a8086:		/* Intel ICH3 mobile */
    		case 0x248b8086:		/* Intel ICH3 */
    		case 0x244a8086:		/* Intel ICH2 mobile */
    		case 0x244b8086:		/* Intel ICH2 */
    		case 0x24118086:		/* Intel ICH */
    		case 0x76018086:		/* Intel ICH */
    		case 0x24218086:		/* Intel ICH0 */
			case 0x71118086:		/* Intel PIIX4 */
    		case 0x84CA8086:		/* Intel PIIX4 */
    		case 0x71998086:		/* Intel PIIX4e */
    			value=read_pci_config(ide.bus,ide.dev,ide.func,0x48);
    			writedw_pci_config(ide.bus,ide.dev,ide.func,0x48,value&~(1<<(host*2+dev)));
    			break;
    		case 0x74411022:		/* AMD 768 */
    		case 0x74111022:		/* AMD 766 */
    		case 0x74091022:		/* AMD 756 */
    		case 0x05711106:		/* VIA 82C571, 82C586, 82C596, 82C686 , 8231, 8233 */
    		case 0x31471106:		/* VIA 8233a */
    		case 0x82311106:		/* VIA 8231 */
    		case 0x30741106:		/* VIA 8233 */
    		case 0x31091106:		/* VIA 8233c */
    		case 0x06861106:		/* VIA 82C686 82C686a 82C686b */
    		case 0x05961106:		/* VIA 82C596a 82C596b */
    		case 0x05861106:		/* VIA 82C586b */
    			value=read_pci_config(ide.bus,ide.dev,ide.func,0x50);
    			writedw_pci_config(ide.bus,ide.dev,ide.func,0x50,value&~(0x40000000>>(host*16+dev*8)));
    			/*writeb_pci_config(ide.bus,ide.dev,ide.func,0x4b-host*2-dev,0x31);*/
    			break;
    		case 0x55131039:		/* SiS 5591 */
    		case 0x06301039:		/* SiS 630 */
    		case 0x06331039:		/* SiS 633 */
    		case 0x06351039:		/* SiS 635 */
    		case 0x06401039:		/* SiS 640 */
    		case 0x06451039:		/* SiS 645 */
    		case 0x06501039:		/* SiS 650 */
    		case 0x07301039:		/* SiS 730 */
    		case 0x07331039:		/* SiS 733 */
    		case 0x07351039:		/* SiS 735 */
    		case 0x07401039:		/* SiS 740 */
    		case 0x07451039:		/* SiS 745 */
    		case 0x07501039:		/* SiS 750 */
    		case 0x05301039:		/* SiS 530 */
    		case 0x05401039:		/* SiS 540 */
    		case 0x06201039:		/* SiS 620 */
    			value=read_pci_config(ide.bus,ide.dev,ide.func,0x40+host*4);
    			writedw_pci_config(ide.bus,ide.dev,ide.func,0x40+host*4,value&~(0xf000<<dev*16));
    			break;
    	}
	}
	else if(mode==U_DMA)
	{
		if((id_info->ultra_dma&(U_DMA5|U_DMA4|U_DMA3|U_DMA2|U_DMA1|U_DMA0))==0)
			return PRINT_ERR(ENOSYS,"change_mode");

		/*
		 * ULTRA DMA対応のドライブについては、BIOSでIDEがULTRA DMAに
		 * 設定されているものとみなす
		 */
		if((error=init_ide_busmaster(host,&ide))!=0)return error;

		switch(ide.vender)
		{
			case 0x24CB8086:		/* Intel ICH4 */
			case 0x248a8086:		/* Intel ICH3 mobile */
    		case 0x248b8086:		/* Intel ICH3 */
    		case 0x244a8086:		/* Intel ICH2 mobile */
    		case 0x244b8086:		/* Intel ICH2 */
    			if(id_info->ultra_dma&U_DMA5)
    			{
					subcm=SUB_U_DMA|5;
					goto INTEL;
				}
			case 0x24118086:		/* Intel ICH */
    		case 0x76018086:		/* Intel ICH */
    			if(id_info->ultra_dma&U_DMA4)
    			{
					subcm=SUB_U_DMA|4;
					goto INTEL;
				}
				if(id_info->ultra_dma&U_DMA3)
				{
					subcm=SUB_U_DMA|3;
					goto INTEL;
				}
			case 0x24218086:		/* Intel ICH0 */
			case 0x71118086:		/* Intel PIIX4 */
    		case 0x84CA8086:		/* Intel PIIX4 */
    		case 0x71998086:		/* Intel PIIX4e */
    			if(id_info->ultra_dma&U_DMA2)
    			{
					subcm=SUB_U_DMA|2;
					goto INTEL;
				}
				if(id_info->ultra_dma&U_DMA1)
				{
					subcm=SUB_U_DMA|1;
					goto INTEL;
				}
				if(id_info->ultra_dma&U_DMA0)subcm=SUB_U_DMA|0;
INTEL:			value=read_pci_config(ide.bus,ide.dev,ide.func,0x48);
    			writedw_pci_config(ide.bus,ide.dev,ide.func,0x48,value|(1<<(host*2+dev)));
				break;
    		case 0x74411022:		/* AMD 768 */
    		case 0x74111022:		/* AMD 766 */
    			if(id_info->ultra_dma&U_DMA5)
    			{
					subcm=SUB_U_DMA|5;
					goto VIA;
				}
    		case 0x74091022:		/* AMD 756 */
    			if(id_info->ultra_dma&U_DMA4)
    			{
					subcm=SUB_U_DMA|4;
					goto VIA;
				}
				if(id_info->ultra_dma&U_DMA3)
				{
					subcm=SUB_U_DMA|3;
					goto VIA;
				}
				if(id_info->ultra_dma&U_DMA2)
    			{
					subcm=SUB_U_DMA|2;
					goto VIA;
				}
				if(id_info->ultra_dma&U_DMA1)
				{
					subcm=SUB_U_DMA|1;
					goto VIA;
				}
				if(id_info->ultra_dma&U_DMA0)
				{
					subcm=SUB_U_DMA|0;
					goto VIA;
				}
    		/*case 0x05711106:*/	/* VIA 82C571, 82C586, 82C596, 82C686 , 8231, 8233 */
    		case 0x31471106:		/* VIA 8233a */
    			if(id_info->ultra_dma&U_DMA6)
				{
					subcm=SUB_U_DMA|6;
					goto VIA;
				}
    		case 0x06861106:		/* VIA 82C686b */
    		case 0x82311106:		/* VIA 8231 */
    		case 0x30741106:		/* VIA 8233 */
    		case 0x31091106:		/* VIA 8233c */
    			if(id_info->ultra_dma&U_DMA5)
				{
					subcm=SUB_U_DMA|5;
					goto VIA;
				}
    		/*case 0x06861106:*/	/* VIA 82C686a */
    		case 0x05961106:		/* VIA 82C596b */
    			if(id_info->ultra_dma&U_DMA4)
				{
					subcm=SUB_U_DMA|4;
					goto VIA;
				}
				if(id_info->ultra_dma&U_DMA3)
				{
					subcm=SUB_U_DMA|3;
					goto VIA;
				}
    		/*case 0x06861106:*/	/* VIA 82C686 */
    		/*case 0x05961106:*/	/* VIA 82C596a */
    		case 0x05861106:		/* VIA 82C586b */
    			if(id_info->ultra_dma&U_DMA2)
				{
					subcm=SUB_U_DMA|2;
					goto VIA;
				}
				if(id_info->ultra_dma&U_DMA1)
				{
					subcm=SUB_U_DMA|1;
					goto VIA;
				}
				if(id_info->ultra_dma&U_DMA0)subcm=SUB_U_DMA|0;
VIA:   			value=read_pci_config(ide.bus,ide.dev,ide.func,0x50);
    			writedw_pci_config(ide.bus,ide.dev,ide.func,0x50,value|(0x40000000>>(host*16+dev*8)));
    			break;
    		case 0x55131039:		/* SiS 5591 */
    		case 0x06301039:		/* SiS 630 */
    		case 0x06331039:		/* SiS 633 */
    		case 0x06351039:		/* SiS 635 */
    		case 0x06401039:		/* SiS 640 */
    		case 0x06451039:		/* SiS 645 */
    		case 0x06501039:		/* SiS 650 */
    		case 0x07301039:		/* SiS 730 */
    		case 0x07331039:		/* SiS 733 */
    		case 0x07351039:		/* SiS 735 */
    		case 0x07401039:		/* SiS 740 */
    		case 0x07451039:		/* SiS 745 */
    		case 0x07501039:		/* SiS 750 */
    			if(id_info->ultra_dma&U_DMA5)
				{
					subcm=SUB_U_DMA|5;
					value=read_pci_config(ide.bus,ide.dev,ide.func,0x40+host*4);
    				writedw_pci_config(ide.bus,ide.dev,ide.func,0x40+host*4,value|(0x8000<<dev*16));
    				break;
				}
    			if(id_info->ultra_dma&U_DMA4)
				{
					subcm=SUB_U_DMA|4;
					value=read_pci_config(ide.bus,ide.dev,ide.func,0x40+host*4);
    				writedw_pci_config(ide.bus,ide.dev,ide.func,0x40+host*4,value|(0x9000<<dev*16));
    				break;
				}
				if(id_info->ultra_dma&U_DMA2)
				{
					subcm=SUB_U_DMA|2;
					value=read_pci_config(ide.bus,ide.dev,ide.func,0x40+host*4);
    				writedw_pci_config(ide.bus,ide.dev,ide.func,0x40+host*4,value|(0xb000<<dev*16));
    				break;
				}
    		case 0x05301039:		/* SiS 530 */
    		case 0x05401039:		/* SiS 540 */
    		case 0x06201039:		/* SiS 620 */
    			if(id_info->ultra_dma&U_DMA4)
				{
					subcm=SUB_U_DMA|4;
					value=read_pci_config(ide.bus,ide.dev,ide.func,0x40+host*4);
    				writedw_pci_config(ide.bus,ide.dev,ide.func,0x40+host*4,value|(0x9000<<dev*16));
    				break;
				}
				if(id_info->ultra_dma&U_DMA2)
				{
					subcm=SUB_U_DMA|2;
					value=read_pci_config(ide.bus,ide.dev,ide.func,0x40+host*4);
    				writedw_pci_config(ide.bus,ide.dev,ide.func,0x40+host*4,value|(0xa000<<dev*16));
    				break;
				}
			default:
				return PRINT_ERR(ENOSYS,"change_mode");
		}
	}
	else return PRINT_ERR(EINVAL,"change_mode");

	conect_dev[host][dev].mode=mode;

	kfree(id_info);

	return set_features(host,dev,SET_TRANSFER,subcm);
}


/*
 * Init IDE Bus Master
 * parameters : Host number,PCI_INFO buffer
 * return : 0 or Error number
 */
int init_ide_busmaster(int host,PCI_INFO *ide)
{
	ushort com;


	/* Serch IDE */
	if(search_pci_class(PCI_CLS_IDE,ide)==-1)return PRINT_ERR(ENODEV,"init_ide_busmaster");

	/* Test Bus Master enable bit on */
	com=read_pci_config(ide->bus,ide->dev,ide->func,PCI_CONF_COM);
	writew_pci_config(ide->bus,ide->dev,ide->func,PCI_CONF_COM,com|PCI_COM_BM_BIT);
	if((read_pci_config(ide->bus,ide->dev,ide->func,PCI_CONF_COM)&PCI_COM_BM_BIT)==0)return PRINT_ERR(ENOSYS,"init_ide_busmaster");

	/* Get IO base address */
	if((ide_base[0]=read_pci_config(ide->bus,ide->dev,ide->func,PCI_CONF_BMBASE))==0)return PRINT_ERR(ENOSYS,"init_ide_busmaster");
	ide_base[0]&=0xfff0;
	ide_base[1]=ide_base[0]+IDE_BMIO_SECOND;

	/* Reset Bus Master */
	outb(ide_base[host]+IDE_BMIC,0);

	return 0;
}


/*
 * PIO read data
 * parameters : Host number,Device number,buffer,Block size,Block count
 * return : Status coad
 */
int read_pio(int host,int dev,void *buf,int block,int count)
{
	int error;
	int dtr;
	int i,j,last;


	dtr=reg[host].dtr;
	last=block/2;
	for(i=0;i<count;++i)
	{
		if(((error=check_busy(reg[host].str))&(BSY_BIT|DRQ_BIT))!=DRQ_BIT)return error;
		for(j=0;j<last;++j)((short*)buf)[j]=inw(dtr);
		(uint)buf+=block;
	}

	return check_busy(reg[host].str);
}


/*
 * PIO write data
 * parameters : Host number,Device number,buffer,Block size,Block count
 * return : Status coad
 */
int write_pio(int host,int dev,void *buf,int block,int count)
{
	int error;
	int dtr;
	int i,j,last;


	dtr=reg[host].dtr;
	last=block/2;
	for(i=0;i<count;++i)
	{
		if(((error=check_busy(reg[host].str))&(BSY_BIT|DRQ_BIT))!=DRQ_BIT)return error;
		for(j=0;j<last;++j)outw(dtr,((short*)buf)[j]);
		(uint)buf+=block;
	}

	return check_busy(reg[host].str);
}


/*
 * DMA read data
 * parameters : Host number,Device number,buffer,Block size,Block count
 * return : Status coad
 */
int read_dma(int host,int dev,void *buf,int block,int count)
{
	/* Set PRD */
	prd[host].phys_addr=buf;
	prd[host].count=block*count|PRD_EOT;
	outdw(ide_base[host]+IDE_BMIDTP,(uint)&prd[host]);
	/*
	 * バスマスターステータスレジスタの割り込みフラグをクリアーしないと
	 * 割り込みが発生しないようだ
	 */
	outb(ide_base[host]+IDE_BMIS,0x6);			/* Clear interrupt bit and error bit */
	outb(ide_base[host]+IDE_BMIC,0x9);			/* Start read Bus Master */
	wait_intr(&wait_intr_queue[host],2000);		/* Wait interrupt */
	if(wait_intr_queue[host].flag==-1)return PRINT_ERR(ETIMEOUT,"read_dma");
	outb(ide_base[host]+IDE_BMIC,0);			/* Stop Bus Master */

	return inb(reg[host].str);
}


/*
 * DMA write data
 * parameters : Host number,Device number,buffer,Block size,Block count
 * return : Status coad
 */
int write_dma(int host,int dev,void *buf,int block,int count)
{
	/* Set PRD */
	prd[host].phys_addr=buf;
	prd[host].count=block*count|PRD_EOT;
	outdw(ide_base[host]+IDE_BMIDTP,(uint)&prd[host]);
	/*
	 * バスマスターステータスレジスタの割り込みフラグをクリアーしないと
	 * 割り込みが発生しないようだ
	 */
	outb(ide_base[host]+IDE_BMIS,0x6);		/* Clear interrupt bit and error bit */
	outb(ide_base[host]+IDE_BMIC,0x1);		/* Start write Bus Master */
	wait_intr(&wait_intr_queue[host],2000);		/* Wait interrupt */
	if(wait_intr_queue[host].flag==-1)return PRINT_ERR(ETIMEOUT,"write_dma");
	outb(ide_base[host]+IDE_BMIC,0);		/* Stop Bus Master */

	return inb(reg[host].str);
}


/*
 * reset host
 * parameters : host number
 * return : 0 or error number
 */
int reset_host(int host)
{
	int error;
	int i;


	/* soft reset */
	if((error=soft_reset(host))!=0)return error;

	/* set transfer mode */
	for(i=0;i<2;++i)change_mode(host,i,conect_dev[host][i].mode);

	return 0;
}


/************************************************************************************************
 *
 * ATA initialize
 *
 ************************************************************************************************/


/*
 * ATAの初期化
 * return : 0 or Error number
 */
int init_ata()
{
	uchar cl,ch;
	int error;
	ID_INFO *id_info;
	int i,j;


	/* タイムアウト値の代入 */
	time_out=clock_1m*TIME_OUT;

	if((id_info=(ID_INFO*)kmalloc(IDENTIFY_SIZE))==NULL)return PRINT_ERR(ENOMEM,"init_ata");

	/* 接続デバイスを判定する */
	for(i=0;i<2;++i)
	{
		/* ソフトリセット */
		if(soft_reset(i)!=0)continue;
		current_intr[i]=0;

		for(j=0;j<2;++j)
		{
			/*
			 * ソフトリセット後ATAならATA_CLR=0 ATA_CHR=0、ATAPIならATA_CLR=0x14 ATA_CHR=0xeb
			 * になる
			 */
			outb(reg[i].dhr,j<<4);
			mili_timer(5);
			cl=inb(reg[i].clr);
			ch=inb(reg[i].chr);

			/* ATA disk */
			if((cl==0)&&(ch==0))
			{
				id_info->model[0]=0xff;
				if(identify_device(i,j,ATA,id_info)!=0)continue;	/* Read identify infomation */
				if(id_info->model[0]==0xff)continue;				/* 読み出しているかをチェック */

				/* Print device infomation */
				cnv_idinfo_str(id_info->model,40);
				printk("%s : %s, %s\n",hd_info[i][j].name,id_info->model,"ATA DISK drive");

				/* LBA all sectors */
				conect_dev[i][j].all_sectors=(uint)id_info->lba_all_sect[1]<<16|(uint)id_info->lba_all_sect[0];
				if(conect_dev[i][j].all_sectors==0)
				{
					printk("This device is not support LBA. Stop initialize!");
					continue;
				}
				conect_dev[i][j].type=ATA;
				conect_dev[i][j].transfer=_transfer_ata;

				/* Init device parameters */
				init_device_param(i,j,(uchar)id_info->n_head,id_info->n_sect);

				hd_info[i][j].last_blk=conect_dev[i][j].all_sectors-1;
				hd_info[i][j].sector_size=ATA_SECTOR_SIZE;
			}

			/* ATAPI device */
			else if((cl==0x14)&&(ch==0xeb))
			{
				char *media;


				if(identify_device(i,j,ATAPI,id_info)!=0)continue;	/* Read identify infomation */

				/* Print device infomation */
				cnv_idinfo_str(id_info->model,40);
				switch((id_info->config>>8)&0x1f)		/* Medium name */
				{
					case 0x5:
						media="ATAPI CDROM drive";
						break;
					default:
						media="ATAPI OTHER drive";
						break;
				}
				printk("%s : %s, %s\n",hd_info[i][j].name,id_info->model,media);

				conect_dev[i][j].flag=id_info->iordy;
				conect_dev[i][j].type=ATAPI;
				conect_dev[i][j].transfer=_transfer_atapi;
			}
			else continue;

			/* Idle device */
			idle_immediate_device(i,j);

			/* Set PIO mode */
			if((error=change_mode(i,j,PIO))!=0)
				printk("Transfer mode set error : %x\n",error);

			/* Regster to dev filesystem */
			regist_device(&hd_info[i][j]);
		}
	}

	kfree(id_info);

	/*
	 * もう一度確認
	 * デバイスによっては、同一ホストが存在しない場合確認処理によって
	 * ビジー状態のままになってしまう。
	 */
	for(i=0;i<2;++i)
		for(j=0;j<2;++j)
		{
			outb(reg[i].dhr,j<<4);
			mili_timer(5);
			if((inb(reg[i].str)&BSY_BIT)==BSY_BIT)reset_host(i);
		}

	/*
	 * 割り込みの設定
	 * 8259PICの場合マスクしても割り込みは保持されているので、マスク解除の後
	 * タイマーを入れて、ハンドラ設定前に割り込みを発生させる。
	 */
	release_irq_mask(PRIM_IRQ);
	release_irq_mask(SECOND_IRQ);
	mili_timer(2);
	irq_entry[IRQ14]=prim_intr_handler;
	irq_entry[IRQ15]=second_intr_handler;

	return 0;
}


/*
 * ワードのビッグエンディアンをリトルエンディアンに変える
 * parameters : string address,string length
 * return : string address
 */
char *cnv_idinfo_str(char *str,int len)
{
	char c;
	int count;
	int i;


	/*
	 * ' 'が２つつづいたら'\0'を入れる
	 */
	count=0;
	for(i=0;i<len;++i)
	{
		c=str[i];
		str[i]=str[i+1];
		str[i+1]=c;

		if(str[i]==' ')++count;
		else count=0;
		if(count==2)break;
		if(str[++i]==' ')++count;
		else count=0;
		if(count==2)break;
	}
	str[--i]='\0';

	return str;
}


/*
 * ソフトウェアリセット
 * parameters : コントロールレジスタ
 * return : 0 or Error number
 */
int soft_reset(int host)
{
	outb(reg[host].ctr,0x4);	/* ソフトリセット */
	mili_timer(5);				/* 5ms wait */
	outb(reg[host].ctr,0x2);	/* リセット解除|割り込み禁止 */
	mili_timer(20);				/* 20ms wait */
	if((check_busy(reg[host].astr)&BSY_BIT)!=0)return PRINT_ERR(EDBUSY,"soft_reset");

	return 0;
}


/************************************************************************************************
 *
 * ATA command protocol
 *
 ************************************************************************************************/


/*
 * デバイスセレクション
 * parameters : Device number,Host number
 * return : 0 or Error number
 */
int device_select(int host,int dev)
{
	int error;


	if(((error=check_busy(reg[host].astr))&(DRQ_BIT|BSY_BIT))!=0)
	{
		if(error&DRQ_BIT)return PRINT_ERR(EDERRE,"device_select");
		if(error&BSY_BIT)return PRINT_ERR(EDBUSY,"device_select");
	}

	/* Device select */
	outb(reg[host].dhr,(uchar)dev|0xa0);
	micro_timer(1);					/* 400ns wait */

	if(((error=check_busy(reg[host].astr))&(DRQ_BIT|BSY_BIT))!=0)
	{
		if(error&DRQ_BIT)return PRINT_ERR(EDERRE,"device_select");
		if(error&BSY_BIT)return PRINT_ERR(EDBUSY,"device_select");
	}
	return 0;
}


/*
 * ATA data transfer protocol
 * parameters : Host number,Device number,Mode=READ or WRITE,buffer,sector count,begin sector
 * return : 0 or Error number
 */
int _transfer_ata(int host,int dev,int trans_mode,void *buf,int count,uint begin)
{
	int error;


	if(conect_dev[host][dev].mode==PIO)set_intr(host,INTR_DISABLE);
	else set_intr(host,INTR_ENABLE);

	if((error=device_select(host,begin>>24|(dev<<4)|LBA_BIT))!=0)return error;

	outb(reg[host].scr,(uchar)count);
	outb(reg[host].snr,(uchar)begin);
	outb(reg[host].clr,(uchar)(begin>>8));
	outb(reg[host].chr,(uchar)(begin>>16));

	/* PIO transfer */
	if(conect_dev[host][dev].mode==PIO)
	{
		if(trans_mode==READ)
		{
			outb(reg[host].cmr,0x20);
			error=read_pio(host,dev,buf,ATA_SECTOR_SIZE,count);
		}
		else
		{
			outb(reg[host].cmr,0x30);
			error=write_pio(host,dev,buf,ATA_SECTOR_SIZE,count);
		}
	}
	/* DMA transfer */
	else
	{
		if(trans_mode==READ)
		{
			outb(reg[host].cmr,0xc8);
			error=read_dma(host,dev,buf,ATA_SECTOR_SIZE,count);
		}
		else
		{
			outb(reg[host].cmr,0xca);
			error=write_dma(host,dev,buf,ATA_SECTOR_SIZE,count);
		}
	}

	if((error&(BSY_BIT|DRQ_BIT|ERR_BIT))!=0)
	{
		if(error&(DRQ_BIT|ERR_BIT))return PRINT_ERR(EDERRE,"_transfer_ata");
		if(error&BSY_BIT)return PRINT_ERR(EDBUSY,"_transfer_ata");
	}

	return 0;
}


/************************************************************************************************
 *
 * ATA command
 *
 ************************************************************************************************/


/*
 * Reset device
 * parameters : Host number,Device number
 * return : 0 or Error number
 */
int reset_device(int host,int dev)
{
	int error;


	set_intr(host,INTR_DISABLE);

	if((error=device_select(host,dev<<4))!=0)return error;

	outb(reg[host].cmr,0x8);
	micro_timer(1);			/* 400ns wait */

	if(((error=check_busy(reg[host].astr))&(BSY_BIT|ERR_BIT))!=0)
	{
		if(error&ERR_BIT)return PRINT_ERR(EDERRE,"reset_device");
		if(error&BSY_BIT)return PRINT_ERR(EDBUSY,"reset_device");
	}
	return 0;
}


/*
 * Identif device
 * parameters : Host number,Device number,ATA or ATAPI,buffer
 * return : 0 or Error number
 */
int identify_device(int host,int dev,int drv,void *buf)
{
	int error;


	set_intr(host,INTR_DISABLE);

	if((error=device_select(host,dev<<4))!=0)return error;

	outb(reg[host].cmr,(drv==ATA)?0xec:0xa1);
	micro_timer(1);					/* 400ns wait */

	/* Read data */
	if(((error=read_pio(host,dev,buf,IDENTIFY_SIZE,1))&(BSY_BIT|DRQ_BIT|ERR_BIT))!=0)
	{
		if(error&(DRQ_BIT|ERR_BIT))return PRINT_ERR(EDERRE,"identify_device");
		if(error&BSY_BIT)return PRINT_ERR(EDBUSY,"identify_device");
	}
	return 0;
}


/*
 * Idle device
 * parameters : Host number,Device number
 * return : 0 or Error number
 */
int idle_immediate_device(int host,int dev)
{
	int error;


	set_intr(host,INTR_DISABLE);

	if((error=device_select(host,dev<<4))!=0)return error;

	outb(reg[host].cmr,0xe1);
	micro_timer(1);			/* 400ns wait */

	if(((error=check_busy(reg[host].astr))&(BSY_BIT|ERR_BIT))!=0)
	{
		if(error&ERR_BIT)return PRINT_ERR(EDERRE,"idle_immediate_device");
		if(error&BSY_BIT)return PRINT_ERR(EDBUSY,"idle_immediate_device");
	}
	return 0;
}


/*
 * Initialize device parameters
 * parameters : Host number,Device number,Number of heads,Number of sectors
 * return : 0 or Error number
 */
int init_device_param(int host,int dev,uchar head,uchar sectors)
{
	int error;


	if(head>0xf)return EINVAL;

	set_intr(host,INTR_DISABLE);

	if((error=device_select(host,(dev<<4)|head))!=0)return error;

	outb(reg[host].scr,sectors);
	outb(reg[host].cmr,0x91);
	micro_timer(1);			/* 400ns wait */

	if(((error=check_busy(reg[host].astr))&(BSY_BIT|ERR_BIT))!=0)
	{
		if(error&ERR_BIT)return PRINT_ERR(EDERRE,"init_device_param");
		if(error&BSY_BIT)return PRINT_ERR(EDBUSY,"init_device_param");
	}
	return 0;
}


/*
 * デバイスの動作設定
 * parameters : Host number,Device number,Subcommand,Transfer mode or 0
 * return : 0 or Error number
 */
int set_features(int host,int dev,uchar subcommand,uchar trans_mode)
{
	int error;


	set_intr(host,INTR_DISABLE);

	if((error=device_select(host,dev<<4))!=0)return error;

	outb(reg[host].ftr,subcommand);
	outb(reg[host].scr,trans_mode);
	outb(reg[host].cmr,0xef);
	micro_timer(1);			/* 400ns wait */

	if(((error=check_busy(reg[host].astr))&(BSY_BIT|ERR_BIT))!=0)
	{
		if(error&ERR_BIT)return PRINT_ERR(EDERRE,"set_features");
		if(error&BSY_BIT)return PRINT_ERR(EDBUSY,"set_features");
	}
	return 0;
}


/************************************************************************************************
 *
 * Packet command
 *
 ************************************************************************************************/


/*
 * Issue packet command
 * parameters : host,device,Packet parameters
 * return : 0 or Error number
 */
int issue_packet_command(int host,int dev,PACKET_PARAM *param)
{
	int dtr;
	int error;
	int i;


	dtr=reg[host].dtr;

	/* デバイスセレクション */
	if((error=device_select(host,dev<<4))!=0)return error;

	/* Issue packet command  */
	outb(reg[host].ftr,param->feutures);
	outb(reg[host].scr,0);
	outb(reg[host].clr,0xff);
	outb(reg[host].chr,0xff);
	outb(reg[host].cmr,0xa0);
	if((check_busy(reg[host].str)&(DRQ_BIT|CHK_BIT))!=DRQ_BIT)return PRINT_ERR(EDERRE,"issue_packet_command");
	if((inb(reg[host].irr)&(CD_BIT|IO_BIT))!=CD_BIT)return PRINT_ERR(EDERRE,"issue_packet_command");

	/* DMA transfer */
	if(param->feutures&PACK_DMA)
	{
		set_intr(host,INTR_ENABLE);		/* enable interrupt */

		/* Send packet */
		for(i=0;i<6;++i)outw(dtr,((short*)param->packet)[i]);

		/* Overrapped */
		if(param->feutures&PACK_OVL)
		{
			wait_intr(&wait_intr_queue[host],2000);		/* Wait interrupt */
			if(wait_intr_queue[host].flag==-1)return PRINT_ERR(ETIMEOUT,"issue_packet_command");

			error=(int)inb(reg[host].str);
			if(error&ERR_BIT)return PRINT_ERR(EDERRE,"issue_packet_command");			/* Packet command Error */
			if((error&DRQ_BIT)==0)return 0;				/* Non data transfer */

			if((error=device_select(host,dev<<4))!=0)return error;
			wait_intr(&wait_intr_queue[host],2000);		/* Wait interrupt */
			if(wait_intr_queue[host].flag==-1)return PRINT_ERR(ETIMEOUT,"issue_packet_command");
			outb(reg[host].cmr,0xa2);					/* Issue service command */
			if((check_busy(reg[host].str)&(BSY_BIT|DRQ_BIT))!=DRQ_BIT)return PRINT_ERR(EDBUSY,"issue_packet_command");
		}

		/* Data transfer */
		if(param->packet[0]==0x28)
			error=read_dma(host,dev,param->buf,param->size,(uint)param->packet[7]<<8|(uint)param->packet[8]);
		else if(param->packet[0]==0x2a)
			error=write_dma(host,dev,param->buf,param->size,(uint)param->packet[7]<<8|(uint)param->packet[8]);
		else
			error=read_dma(host,dev,param->buf,param->size,1);
	}

	/* PIO transfer */
	else
	{
		set_intr(host,INTR_DISABLE);

		/* Send packet */
		for(i=0;i<6;++i)outw(dtr,((short*)param->packet)[i]);

		error=check_busy(reg[host].str);
		if(error&ERR_BIT)return PRINT_ERR(EDERRE,"issue_packet_command");		/* Packet command Error */
		if((error&DRQ_BIT)==0)return 0;			/* Non data transfer */

		/* Overrapped */
		if(param->feutures&PACK_OVL)
		{
			if((error=device_select(host,dev<<4))!=0)return error;
			wait_intr(&wait_intr_queue[host],2000);			/* Wait interrupt */
			if(wait_intr_queue[host].flag==-1)return PRINT_ERR(ETIMEOUT,"issue_packet_command");
			outb(reg[host].cmr,0xa2);						/* Issue service command */
			if((check_busy(reg[host].str)&(BSY_BIT|DRQ_BIT))!=DRQ_BIT)return PRINT_ERR(EDBUSY,"issue_packet_command");
		}

		/* Data transfer */
		if(param->packet[0]==0x28)
			error=read_pio(host,dev,param->buf,param->size,(uint)param->packet[7]<<8|(uint)param->packet[8]);
		else if(param->packet[0]==0x2a)
			error=write_pio(host,dev,param->buf,param->size,(uint)param->packet[7]<<8|(uint)param->packet[8]);
		else
			error=read_pio(host,dev,param->buf,param->size,1);
	}

	/* Last check */
	if((error&(BSY_BIT|DRQ_BIT|ERR_BIT))!=0)
	{
		if(error&(DRQ_BIT|ERR_BIT))return PRINT_ERR(EDERRE,"issue_packet_command");
		if(error&BSY_BIT)return PRINT_ERR(EDBUSY,"issue_packet_command");
	}
	return 0;
}


/*
 * Test unit ready
 * parameters : Host number,Device number
 * return : 0 or Error number
 */
int test_unit_ready(int host,int dev)
{
	PACKET_PARAM param;


	/* Set packet parameters */
	param.feutures=0;
	param.size=0;
	memset(param.packet,0,12);
	param.packet[0]=0;

	return issue_packet_command(host,dev,&param);
}


/*
 * Request sense
 * parameters : Host number,Device number
 * return : Ready=0 or Retry=1 or Error number
 */
int request_sense(int host,int dev)
{
	static char buf[14];
	PACKET_PARAM param;


	/* Set packet parameters */
	param.feutures=conect_dev[host][dev].mode>>1;
	param.size=14;
	param.buf=buf;
	memset(param.packet,0,12);
	param.packet[0]=0x3;
	param.packet[4]=14;

	if(issue_packet_command(host,dev,&param)!=0)return -1;
	switch((((uint)buf[2]&0xf)<<16)|((uint)buf[12]<<8)|((uint)buf[13]))
	{
		case 0x62800:return 0;			/* Not ready to ready change */
		case 0x62900:					/* Power on,Reset */
		case 0x20401:return 1;			/* Becoming ready */
		case 0x63a00:return PRINT_ERR(ENOMEDIUM,"request_sense");	/* Medium not present */
		default:return PRINT_ERR(EDERRE,"request_sense");
	}
}


/*
 * Start stop unit
 * parameters : Host number,Device number,Operation flag
 * return : 0 or Error number
 */
int start_stop_unit(int host,int dev,uchar ope)
{
	PACKET_PARAM param;


	/* Set packet parameters */
	param.feutures=(conect_dev[host][dev].flag&ATAPI_OVL)>>12;
	memset(param.packet,0,12);
	param.packet[0]=0x1b;
	param.packet[4]=ope;

	return issue_packet_command(host,dev,&param);
}


/*
 * Read capacity
 * parameters : Host number,Device number
 * return : 0 or Error number
 */
int read_capacity(int host,int dev)
{
	PACKET_PARAM param;
	char tmp,*buf;
	int error;
	int i,j;


	buf=(char*)&conect_dev[host][dev].sector_size;

	/* Set packet parameters */
	param.feutures=conect_dev[host][dev].mode>>1;
	param.size=8;
	param.buf=buf;
	memset(param.packet,0,12);
	param.packet[0]=0x25;

	if((error=issue_packet_command(host,dev,&param))!=0)return error;

	for(i=0,j=7;i<4;++i,--j)
	{
		tmp=buf[i];
		buf[i]=buf[j];
		buf[j]=tmp;
	}

	/*
	 * 512byte単位で切捨て
	 * ドライブによっては物理セクターサイズの場合がある
	 */
	conect_dev[host][dev].sector_size=ROUNDDOWN(conect_dev[host][dev].sector_size,512);

	return 0;
}


/*
 * Read data
 * parameters : Host number,Device number,Transfer mode,Buffer,Begin sector
 * rturn : 0 or Error number
 */
int _transfer_atapi(int host,int dev,int trans_mode,void *buf,int count,uint begin)
{
	PACKET_PARAM param;


	/* Set packet parameters */
	param.feutures=((conect_dev[host][dev].flag&ATAPI_OVL)>>12)|(conect_dev[host][dev].mode>>1);
	param.size=conect_dev[host][dev].sector_size;
	param.buf=buf;
	memset(param.packet,0,12);
	param.packet[0]=(trans_mode==READ)?0x28:0x2a;
	param.packet[2]=(uchar)begin>>24;
	param.packet[3]=(uchar)begin>>16;
	param.packet[4]=(uchar)begin>>8;
	param.packet[5]=(uchar)begin;
	param.packet[7]=(uchar)count>>8;
	param.packet[8]=(uchar)count;

	return issue_packet_command(host,dev,&param);
}


/************************************************************************************************
 *
 * System call interface
 *
 ************************************************************************************************/


/*
 * Test atapi and read capacity
 * parameters : Host number,Device number
 * return : Redy=0 or Error number
 */
int test_atapi(int host,int dev)
{
	int error;


	for(;test_unit_ready(host,dev)!=0;)
	{
		error=request_sense(host,dev);
		if(error==0)break;
		else if(error<0)return error;
	}

	/* Read sector size and max sector number */
	if((error=read_capacity(host,dev))!=0)return error;
	hd_info[host][dev].last_blk=conect_dev[host][dev].all_sectors-1;
	hd_info[host][dev].sector_size=conect_dev[host][dev].sector_size;

	return 0;
}


/*
 * File operation interface
 */
int open_hda()
{
	switch(conect_dev[0][0].type)
	{
		case ATA:return 0;
		case ATAPI:return test_atapi(0,0);
		default:return PRINT_ERR(ENODEV,"open_hda");
	}
}

int open_hdb()
{
	switch(conect_dev[0][1].type)
	{
		case ATA:return 0;
		case ATAPI:return test_atapi(0,1);
		default:return PRINT_ERR(ENODEV,"open_hdb");
	}
}

int open_hdc()
{
	switch(conect_dev[1][0].type)
	{
		case ATA:return 0;
		case ATAPI:return test_atapi(1,0);
		default:return PRINT_ERR(ENODEV,"open_hdc");
	}
}

int open_hdd()
{
	switch(conect_dev[1][1].type)
	{
		case ATA:return 0;
		case ATAPI:return test_atapi(1,1);
		default:return PRINT_ERR(ENODEV,"open_hdd");
	}
}

int read_hda(void *buf,size_t size,size_t begin)
{
	return transfer(0,0,READ,buf,size,begin);
}

int read_hdb(void *buf,size_t size,size_t begin)
{
	return transfer(0,1,READ,buf,size,begin);
}

int read_hdc(void *buf,size_t size,size_t begin)
{
	return transfer(1,0,READ,buf,size,begin);
}

int read_hdd(void *buf,size_t size,size_t begin)
{
	return transfer(1,1,READ,buf,size,begin);
}

int write_hda(void *buf,size_t size,size_t begin)
{
	return transfer(0,0,WRITE,buf,size,begin);
}

int write_hdb(void *buf,size_t size,size_t begin)
{
	return transfer(0,1,WRITE,buf,size,begin);
}

int write_hdc(void *buf,size_t size,size_t begin)
{
	return transfer(1,0,WRITE,buf,size,begin);
}

int write_hdd(void *buf,size_t size,size_t begin)
{
	return transfer(1,1,WRITE,buf,size,begin);
}

int ioctl_hda(int command,void *param)
{
	return 0;
}

int ioctl_hdb(int command,void *param)
{
	return 0;
}

int ioctl_hdc(int command,void *param)
{
	return 0;
}

int ioctl_hdd(int command,void *param)
{
	return 0;
}
/******************************************************************/
void test_hd()
{
	char *buf1=(char*)0x80000;
	char *buf2=(char*)0x90000;


	buf1[0]=0xaa;
	buf1[0x1000-1]=0xbb;

	if(change_mode(0,1,M_DMA)!=0)printk("Fail change_mode!\n");
	printk("transfer=%d\n",transfer(0,1,WRITE,buf1,8,0x3f+1600));
	printk("transfer=%d\n",transfer(0,1,READ,buf2,8,0x3f+1600));
	printk("buf2[0]=%x,buf2[0x10000-1]=%x\n",buf2[0],buf2[0x1000-1]);

/*	int *buf=(int*)0x90000;

	if(change_mode(1,0,M_DMA)!=0)printk("Fail change_mode!\n");

	if(test_atapi(1,0)!=0)
	{
		printk("test_atapi error\n");
		return;
	}

	printk("transfer=%d\n",transfer(1,0,READ,buf,0x1000,32));
	printk("transfer=%d\n",transfer(1,0,READ,buf,0x1000,60));
	printk("buf[0]=%x,buf[0xf20/4-1]=%x\n",buf[0],buf[0xf20/4-1]);
*/
}
