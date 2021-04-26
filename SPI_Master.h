#ifndef _SPI_MASTER_H_

/***Configure SPI1 use any blow pin pack*******/
/***PIN pack 1********/
#define GPIOA_PIN_5 			5
#define GPIOA_PIN_6 			6
#define GPIOA_PIN_7 			7
/***PIN pack 2********/
#define GPIOB_PIN_3 			3
#define GPIOB_PIN_4 			4
#define GPIOB_PIN_5 			5

/***Configure SPI2 use any blow pin pack*******/
/***PIN pack 1********/
#define GPIOC_PIN_3 			3
#define GPIOC_PIN_2 			2
#define GPIOB_PIN_10 			10
/***PIN pack 2********/
#define GPIOB_PIN_13 			13
#define GPIOB_PIN_14 			14
#define GPIOB_PIN_15 			15

/***PIN pack 3********/
#define GPIOI_PIN_0 			0
#define GPIOI_PIN_2 			2
#define GPIOI_PIN_3 			3

/***Configure SPI3 use any blow pin pack*******/
/***PIN pack 1********/
#define GPIOB_PIN_3 			3
#define GPIOB_PIN_4 			4
#define GPIOB_PIN_5 			5
/***PIN pack 2********/
#define GPIOC_PIN_10 			10
#define GPIOC_PIN_11 			11
#define GPIOC_PIN_12 			12
/*****configure GPIO's for SPI functionality***************/
#define SPI_CLK_PIN		GPIOB_PIN_13

#define SPI_MISO_PIN  GPIOB_PIN_14

#define SPI_MOSI_PIN  GPIOB_PIN_15



enum SPI_NUM{
	SPI_1 =0,
	SPI_2,
	SPI_3,
	SPI_4
};



#endif //_SPI_MASTER_H_
