/*
 * Copyright (C) 2015
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/*
 * @ingroup auto_init_gnrc_netif
 * @{
 *
 * @file
 * @brief   Auto initialization for nx_cc2538rf network interfaces
 *
 * @author  Anon Mall <anon.mall@dai-labor.de>
 */

#ifdef MODULE_CC2538RF

#include "board.h"
#include "net/gnrc/nomac.h"
#include "net/gnrc.h"

#include "cc2538rf.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/**
 * @brief   Define stack parameters for the MAC layer thread
 * @{
 */



void auto_init_cc2538rf(void)
{

}
#else
typedef int dont_be_pedantic;
#endif /* MODULE_CC2538RF */

/** @} */
