/*******************************************************************************
 *                     SPIRI APS PROPRIETARY INFORMATION
 *
 * Property of Spiri ApS, Unauthorized reproduction and/or distribution
 * is strictly prohibited.  This product  is  protected  under  copyright  law
 * and  trade  secret law as an unpublished work.
 * (C) Copyright Spiri ApS.  All rights reserved.
 *
 * Description      :   Internal SPI communication layer between the  
 *                      microcontroller and the Computation Module in the 
 *                      Central Communication Unit (CCU).
 *
 * Author           :   Nikolaj Due Oesterbye
 * Date             :   September 14 2016
 *
 ******************************************************************************/

#ifndef _CCU_SPI_H_
#define _CCU_SPI_H_

/* ioctls */
//#define CCU_SPI_SYNC_READ      _IOW('F', 1, int)


struct ccu_spi_platform_data {
	unsigned rtr_gpio;
	unsigned cts_gpio;
};


#endif /* _CCU_SPI_H_ */
