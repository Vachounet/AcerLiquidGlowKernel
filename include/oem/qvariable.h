#ifndef __QVARIABLE_H
#define __QVARIABLE_H

#define VAR_LEN		5

#define VAR_ROOT_STR		"root"
#define VAR_ROOT_OFFSET		0x100
#define VAR_ROOT_OnSTR		"R1oOt"

#define VAR_ADB_STR			"adb"
#define VAR_ADB_OFFSET		0x252
#define VAR_ADB_OnSTR		"qa2Db"

#define VAR_DIAG_STR		"diag"
#define VAR_DIAG_OFFSET		0x3C3
#define VAR_DIAG_OnSTR		"DIa3g"

#define VAR_FTD_STR			"ftd"
#define VAR_FTD_OFFSET		0x507
#define VAR_FTD_OnSTR		"ftdQ4"

#define VAR_UART_STR		"uart"
#define VAR_UART_OFFSET		0x66D
#define VAR_UART_OnSTR		"U5ARt"

#define VAR_FIXUSBID_STR	"fixusbid"
#define VAR_FIXUSBID_OFFSET	0x7C2
#define VAR_FIXUSBID_OnSTR	"Us6bQ"

#define VAR_DEBUGFS_STR		"debugfs"
#define VAR_DEBUGFS_OFFSET	0x917
#define VAR_DEBUGFS_OnSTR	"DbG7q"

#define VAR_LOGNOTIFY_STR	"lognotify"
#define VAR_LOGNOTIFY_OFFSET	0xA5B
#define VAR_LOGNOTIFY_OnSTR	"LoGN8"

#define VAR_NOOFFCHARGE_STR	"nooffcharge"
#define VAR_NOOFFCHARGE_OFFSET	0xBA7
#define VAR_NOOFFCHARGE_OnSTR	"9nChG"

#define VAR_NONFCUPGRADE_STR    "nonfcupgrade"
#define VAR_NONFCUPGRADE_OFFSET 0xC81
#define VAR_NONFCUPGRADE_OnSTR  "n0NFC"

static struct
{
    char Name[32];
    unsigned int Offset;
    unsigned int Length;
    unsigned char OnValue[16];
} const VarTbl[] =
{
    {VAR_ROOT_STR,          VAR_ROOT_OFFSET,        VAR_LEN, VAR_ROOT_OnSTR},
    {VAR_ADB_STR,           VAR_ADB_OFFSET,         VAR_LEN, VAR_ADB_OnSTR},
    {VAR_DIAG_STR,          VAR_DIAG_OFFSET,		VAR_LEN, VAR_DIAG_OnSTR},
    {VAR_FTD_STR,           VAR_FTD_OFFSET,         VAR_LEN, VAR_FTD_OnSTR},
    {VAR_UART_STR,          VAR_UART_OFFSET,        VAR_LEN, VAR_UART_OnSTR},
    {VAR_FIXUSBID_STR,      VAR_FIXUSBID_OFFSET,    VAR_LEN, VAR_FIXUSBID_OnSTR},
    {VAR_DEBUGFS_STR,       VAR_DEBUGFS_OFFSET,     VAR_LEN, VAR_DEBUGFS_OnSTR},
    {VAR_LOGNOTIFY_STR,	    VAR_LOGNOTIFY_OFFSET,   VAR_LEN, VAR_LOGNOTIFY_OnSTR},
    {VAR_NOOFFCHARGE_STR,   VAR_NOOFFCHARGE_OFFSET, VAR_LEN, VAR_NOOFFCHARGE_OnSTR},
    {VAR_NONFCUPGRADE_STR,  VAR_NONFCUPGRADE_OFFSET,VAR_LEN, VAR_NONFCUPGRADE_OnSTR},
    {"unlock",  0x4F45, 11, {0x4B, 0x31, 0x32, 0x43, 0x6F, 0x4C, 0x4E, 0x75, 0x4F, 0x65, 0x4D}},
};


#endif
