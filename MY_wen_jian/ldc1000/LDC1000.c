#include "LDC1000.h"
#include "stm32f10x.h"

u8 proximtyData[2];
u8 proximtyData1[3];
u16 Rp1;
u32 Fr,Rp2;

void LDC_SPI_Init(void)
{
	GPIO_InitTypeDef		GPIO_InitTypeStructure;
	SPI_InitTypeDef	SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1|RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitTypeStructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_6;
	GPIO_InitTypeStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitTypeStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitTypeStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_6);
	
	GPIO_InitTypeStructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_0;
	GPIO_InitTypeStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitTypeStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitTypeStructure);
	
	GPIO_InitTypeStructure.GPIO_Pin=GPIO_Pin_4;
	GPIO_InitTypeStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitTypeStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitTypeStructure);
	
	GPIO_SetBits(GPIOC,GPIO_Pin_5|GPIO_Pin_0);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//串行同步时钟的第一个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	SPI_Init(SPI1,&SPI_InitStructure);
	
	SPI_Cmd(SPI1, ENABLE); //使能SPI外设
	
}

u8 SPI1_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据					    
}

void spi_writeByte(u8 addr,u8 data)
{
	addr=addr&0x7F;
	
	NSS_RES;
	
	SPI1_ReadWriteByte(addr);
	SPI1_ReadWriteByte(data);
	
	NSS_SET;
}
void spi_readByte(u8 addr,u8 *data)
{
	addr|=0x80;
	
	NSS_RES;
	
	SPI1_ReadWriteByte(addr);
	*data=SPI1_ReadWriteByte(0xFF);
	
	NSS_SET;
}
void LDC1000_init(void)
{
	u8 iii;
	spi_readByte(LDC1000_CMD_PWRCONFIG, &iii);
	if(iii)
	spi_writeByte(LDC1000_CMD_PWRCONFIG,   0x00);
	spi_writeByte(LDC1000_CMD_RPMAX,       0x16);//TEST_RPMAX_INIT
  spi_writeByte(LDC1000_CMD_RPMIN,       0x3F);//TEST_RPMIN_INIT
	spi_writeByte(LDC1000_CMD_SENSORFREQ,  0x94);
	spi_writeByte(LDC1000_CMD_LDCCONFIG,   0x17);
	spi_writeByte(LDC1000_CMD_CLKCONFIG,   0x00);
	spi_writeByte(LDC1000_CMD_INTCONFIG,   0x02);

	spi_writeByte(LDC1000_CMD_THRESHILSB,  0x1C);//50   61
	spi_writeByte(LDC1000_CMD_THRESHIMSB,  0x25);//14   26
	spi_writeByte(LDC1000_CMD_THRESLOLSB,  0xC4);//C0   A8
	spi_writeByte(LDC1000_CMD_THRESLOMSB,  0x09);//12   48
	spi_writeByte(LDC1000_CMD_PWRCONFIG,   0x01);
}
void   LDC_RP(void)
{
	spi_readByte(LDC1000_CMD_PROXLSB,&proximtyData[0]);
	spi_readByte(LDC1000_CMD_PROXLSB+1,&proximtyData[1]);
	Rp1=proximtyData[0]+(u16)(proximtyData[1]<<8);
}
void readL(void)
{
	spi_readByte(LDC1000_CMD_FREQCTRLSB,&proximtyData1[0]);
	spi_readByte(LDC1000_CMD_FREQCTRMID,&proximtyData1[1]);
	spi_readByte(LDC1000_CMD_FREQCTRMSB,&proximtyData1[2]);
	Rp2=proximtyData[0]+(u32)(proximtyData[1]<<8)+(u32)(proximtyData1[2]<<16);
}


