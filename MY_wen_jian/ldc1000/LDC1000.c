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

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//����ͬ��ʱ�ӵĵ�һ�������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI1,&SPI_InitStructure);
	
	SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����
	
}

u8 SPI1_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI1, TxData); //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����					    
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


