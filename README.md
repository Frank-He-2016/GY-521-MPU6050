# GY-521-MPU6050
一、GY-521程序中的数据流方向
1.X的值=T_X，而T_X=[(BUF[1]<<8)|BUF[0]]/16.4
Y和Z的值与之同理
2.T的值为T_T，而T_T=((BUF[7]<<8)|BUF[6] + 13200)) / 280+35
3.DATA_printf这个函数是将一个大整数化成各位分别发送,+0x30可以由ASCII转化为数字
二、GY-521读I2C的原理
1.MPU3050的从地址是0xD0,
在文件开头进行了宏定义#define	MPU3050_Addr   0xD0
我猜测是不是与I2C_Init函数中的I2C_OwnAddress1是一个东西
2.读取I2C数据的原理
大致是先发送从器件地址，再发送要读数据的地址，再读取返回的数据
这些内容写在Single_Read这个函数之中，我们来具体看一下：
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{   
  char  test; 
  unsigned char REG_data;     	
  if(!I2C_Start())return FALSE;//检测I2C是否启动，没启动则返回FALSE
    I2C_SendByte(SlaveAddress);//发送从器件地址
    if(!I2C_WaitAck()){I2C_Stop();test=1; return FALSE;}//如果没有等待回应，也返回FALSE
    I2C_SendByte((u8) REG_Address);  //发送要读数据的地址   
    I2C_WaitAck();//等待回应
    I2C_Start();//启动I2C
    I2C_SendByte(SlaveAddress+1);//从器件地址后移一位
    I2C_WaitAck();//等待回应

	REG_data= I2C_RadeByte();//读取数据
    I2C_NoAck();//无回应
    I2C_Stop();//则停止
	return REG_data;//返回读取的数据

}
其中所有的地址都在开头进行了宏定义
然后存入缓存
BUF[0]=Single_Read(MPU3050_Addr,GX_L); 
BUF[1]=Single_Read(MPU3050_Addr,GX_H);
BUF[2]=Single_Read(MPU3050_Addr,GY_L);
BUF[3]=Single_Read(MPU3050_Addr,GY_H);
BUF[4]=Single_Read(MPU3050_Addr,GZ_L);
BUF[5]=Single_Read(MPU3050_Addr,GZ_H);
BUF[6]=Single_Read(MPU3050_Addr,TMP_L); 
BUF[7]=Single_Read(MPU3050_Addr,TMP_H);
