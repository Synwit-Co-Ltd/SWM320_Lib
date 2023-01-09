#include "SWM320.h"

#include "arm_math.h"
#include "math_helper.h"


#define TEST_LENGTH_SAMPLES  320
#define SNR_THRESHOLD_F32    140.0f
#define BLOCK_SIZE            32
#define NUM_TAPS              29


float32_t testInput_f32_1kHz_15kHz[320];	//Test input signal contains 1000Hz + 15000 Hz
float32_t refOutput[320];					//reference output (computed with MATLAB)

float32_t testOutput[TEST_LENGTH_SAMPLES];


static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];	//Declare State buffer of size (numTaps + blockSize - 1)


const float32_t firCoeffs32[NUM_TAPS] = {		//FIR Coefficients buffer generated using fir1() MATLAB function: fir1(28, 6/24)
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};


void SerialInit(void);

int main(void)
{
	uint32_t i;
	float32_t  snr;
	arm_fir_instance_f32 fir;
	
	SystemInit();
	
	SerialInit();
   	
	arm_fir_init_f32(&fir, NUM_TAPS, (float32_t *)firCoeffs32, firStateF32, BLOCK_SIZE);
	
	for(i=0; i < TEST_LENGTH_SAMPLES/BLOCK_SIZE; i++)	//Call the FIR process function for every blockSize samples
	{
		arm_fir_f32(&fir, testInput_f32_1kHz_15kHz + BLOCK_SIZE*i, testOutput + BLOCK_SIZE*i, BLOCK_SIZE);
	}
	
	snr = arm_snr_f32(refOutput, testOutput, TEST_LENGTH_SAMPLES);	//将计算结果与标准结果比较计算
	
	if(snr < SNR_THRESHOLD_F32)
	{
		printf("Fail!\r\n");
	}
	else
	{
		printf("Success!\r\n");
	}
	
	while(1==1)
	{
		printf("Hi, World\r\n");
		
		for(i = 0; i < 10000000; i++) __NOP();
	}
}

float32_t testInput_f32_1kHz_15kHz[320] = 
{
+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f, 
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f, 
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f, 
-0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f, 
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f, 
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f, 
+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f, 
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f, 
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f, 
+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f, 
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f, 
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f, 
+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f, 
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f, 
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f, 
+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f, 
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f, 
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f, 
-0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f, 
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f, 
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f, 
-0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f, 
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f, 
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f, 
+0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f, 
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f, 
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f, 
+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f, 
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f, 
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f, 
-0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f, 
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f, 
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f, 
+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f, 
-0.8660254038f, -0.4619397663f, -1.3194792169f, -1.1827865776f, -0.5000000000f, -1.1827865776f, -1.3194792169f, -0.4619397663f, 
-0.8660254038f, -1.2552931065f, -0.3535533906f, -0.4174197128f, -1.0000000000f, -0.1913417162f, +0.0947343455f, -0.5924659585f, 
-0.0000000000f, +0.5924659585f, -0.0947343455f, +0.1913417162f, +1.0000000000f, +0.4174197128f, +0.3535533906f, +1.2552931065f, 
+0.8660254038f, +0.4619397663f, +1.3194792169f, +1.1827865776f, +0.5000000000f, +1.1827865776f, +1.3194792169f, +0.4619397663f, 
+0.8660254038f, +1.2552931065f, +0.3535533906f, +0.4174197128f, +1.0000000000f, +0.1913417162f, -0.0947343455f, +0.5924659585f, 
+0.0000000000f, -0.5924659585f, +0.0947343455f, -0.1913417162f, -1.0000000000f, -0.4174197128f, -0.3535533906f, -1.2552931065f, 
};

float32_t refOutput[320] = 
{
+0.0000000000f, -0.0010797829f, -0.0007681386f, -0.0001982932f, +0.0000644313f, +0.0020854271f, +0.0036891871f, +0.0015855941f, 
-0.0026280805f, -0.0075907658f, -0.0119390538f, -0.0086665968f, +0.0088981202f, +0.0430539279f, +0.0974468742f, +0.1740405600f, 
+0.2681416601f, +0.3747720089f, +0.4893362230f, +0.6024154672f, +0.7058740791f, +0.7968348987f, +0.8715901940f, +0.9277881093f, 
+0.9682182661f, +0.9934674267f, +1.0012052245f, +0.9925859371f, +0.9681538347f, +0.9257026822f, +0.8679010068f, +0.7952493046f, 
+0.7085021596f, +0.6100062330f, +0.5012752767f, +0.3834386057f, +0.2592435399f, +0.1309866321f, -0.0000000000f, -0.1309866321f, 
-0.2592435399f, -0.3834386057f, -0.5012752767f, -0.6100062330f, -0.7085021596f, -0.7952493046f, -0.8679010068f, -0.9257026822f, 
-0.9681538347f, -0.9936657199f, -1.0019733630f, -0.9936657199f, -0.9681538347f, -0.9257026822f, -0.8679010068f, -0.7952493046f, 
-0.7085021596f, -0.6100062330f, -0.5012752767f, -0.3834386057f, -0.2592435399f, -0.1309866321f, +0.0000000000f, +0.1309866321f, 
+0.2592435399f, +0.3834386057f, +0.5012752767f, +0.6100062330f, +0.7085021596f, +0.7952493046f, +0.8679010068f, +0.9257026822f, 
+0.9681538347f, +0.9936657199f, +1.0019733630f, +0.9936657199f, +0.9681538347f, +0.9257026822f, +0.8679010068f, +0.7952493046f, 
+0.7085021596f, +0.6100062330f, +0.5012752767f, +0.3834386057f, +0.2592435399f, +0.1309866321f, -0.0000000000f, -0.1309866321f, 
-0.2592435399f, -0.3834386057f, -0.5012752767f, -0.6100062330f, -0.7085021596f, -0.7952493046f, -0.8679010068f, -0.9257026822f, 
-0.9681538347f, -0.9936657199f, -1.0019733630f, -0.9936657199f, -0.9681538347f, -0.9257026822f, -0.8679010068f, -0.7952493046f, 
-0.7085021596f, -0.6100062330f, -0.5012752767f, -0.3834386057f, -0.2592435399f, -0.1309866321f, +0.0000000000f, +0.1309866321f, 
+0.2592435399f, +0.3834386057f, +0.5012752767f, +0.6100062330f, +0.7085021596f, +0.7952493046f, +0.8679010068f, +0.9257026822f, 
+0.9681538347f, +0.9936657199f, +1.0019733630f, +0.9936657199f, +0.9681538347f, +0.9257026822f, +0.8679010068f, +0.7952493046f, 
+0.7085021596f, +0.6100062330f, +0.5012752767f, +0.3834386057f, +0.2592435399f, +0.1309866321f, -0.0000000000f, -0.1309866321f, 
-0.2592435399f, -0.3834386057f, -0.5012752767f, -0.6100062330f, -0.7085021596f, -0.7952493046f, -0.8679010068f, -0.9257026822f, 
-0.9681538347f, -0.9936657199f, -1.0019733630f, -0.9936657199f, -0.9681538347f, -0.9257026822f, -0.8679010068f, -0.7952493046f, 
-0.7085021596f, -0.6100062330f, -0.5012752767f, -0.3834386057f, -0.2592435399f, -0.1309866321f, +0.0000000000f, +0.1309866321f, 
+0.2592435399f, +0.3834386057f, +0.5012752767f, +0.6100062330f, +0.7085021596f, +0.7952493046f, +0.8679010068f, +0.9257026822f, 
+0.9681538347f, +0.9936657199f, +1.0019733630f, +0.9936657199f, +0.9681538347f, +0.9257026822f, +0.8679010068f, +0.7952493046f, 
+0.7085021596f, +0.6100062330f, +0.5012752767f, +0.3834386057f, +0.2592435399f, +0.1309866321f, +0.0000000000f, -0.1309866321f, 
-0.2592435399f, -0.3834386057f, -0.5012752767f, -0.6100062330f, -0.7085021596f, -0.7952493046f, -0.8679010068f, -0.9257026822f, 
-0.9681538347f, -0.9936657199f, -1.0019733630f, -0.9936657199f, -0.9681538347f, -0.9257026822f, -0.8679010068f, -0.7952493046f, 
-0.7085021596f, -0.6100062330f, -0.5012752767f, -0.3834386057f, -0.2592435399f, -0.1309866321f, +0.0000000000f, +0.1309866321f, 
+0.2592435399f, +0.3834386057f, +0.5012752767f, +0.6100062330f, +0.7085021596f, +0.7952493046f, +0.8679010068f, +0.9257026822f, 
+0.9681538347f, +0.9936657199f, +1.0019733630f, +0.9936657199f, +0.9681538347f, +0.9257026822f, +0.8679010068f, +0.7952493046f, 
+0.7085021596f, +0.6100062330f, +0.5012752767f, +0.3834386057f, +0.2592435399f, +0.1309866321f, +0.0000000000f, -0.1309866321f, 
-0.2592435399f, -0.3834386057f, -0.5012752767f, -0.6100062330f, -0.7085021596f, -0.7952493046f, -0.8679010068f, -0.9257026822f, 
-0.9681538347f, -0.9936657199f, -1.0019733630f, -0.9936657199f, -0.9681538347f, -0.9257026822f, -0.8679010068f, -0.7952493046f, 
-0.7085021596f, -0.6100062330f, -0.5012752767f, -0.3834386057f, -0.2592435399f, -0.1309866321f, -0.0000000000f, +0.1309866321f, 
+0.2592435399f, +0.3834386057f, +0.5012752767f, +0.6100062330f, +0.7085021596f, +0.7952493046f, +0.8679010068f, +0.9257026822f, 
+0.9681538347f, +0.9936657199f, +1.0019733630f, +0.9936657199f, +0.9681538347f, +0.9257026822f, +0.8679010068f, +0.7952493046f, 
+0.7085021596f, +0.6100062330f, +0.5012752767f, +0.3834386057f, +0.2592435399f, +0.1309866321f, +0.0000000000f, -0.1309866321f, 
-0.2592435399f, -0.3834386057f, -0.5012752767f, -0.6100062330f, -0.7085021596f, -0.7952493046f, -0.8679010068f, -0.9257026822f, 
-0.9681538347f, -0.9936657199f, -1.0019733630f, -0.9936657199f, -0.9681538347f, -0.9257026822f, -0.8679010068f, -0.7952493046f, 
-0.7085021596f, -0.6100062330f, -0.5012752767f, -0.3834386057f, -0.2592435399f, -0.1309866321f, +0.0000000000f, +0.1309866321f, 
+0.2592435399f, +0.3834386057f, +0.5012752767f, +0.6100062330f, +0.7085021596f, +0.7952493046f, +0.8679010068f, +0.9257026822f, 
+0.9681538347f, +0.9936657199f, +1.0019733630f, +0.9936657199f, +0.9681538347f, +0.9257026822f, +0.8679010068f, +0.7952493046f 
};


void SerialInit(void)
{
	UART_InitStructure UART_initStruct;
	
	PORT_Init(PORTA, PIN2, FUNMUX0_UART0_RXD, 1);	//GPIOA.2配置为UART0输入引脚
	PORT_Init(PORTA, PIN3, FUNMUX1_UART0_TXD, 0);	//GPIOA.3配置为UART0输出引脚
 	
 	UART_initStruct.Baudrate = 57600;
	UART_initStruct.DataBits = UART_DATA_8BIT;
	UART_initStruct.Parity = UART_PARITY_NONE;
	UART_initStruct.StopBits = UART_STOP_1BIT;
	UART_initStruct.RXThresholdIEn = 0;
	UART_initStruct.TXThresholdIEn = 0;
	UART_initStruct.TimeoutIEn = 0;
 	UART_Init(UART0, &UART_initStruct);
	UART_Open(UART0);
}

/****************************************************************************************************************************************** 
* 函数名称: fputc()
* 功能说明: printf()使用此函数完成实际的串口打印动作
* 输    入: int ch		要打印的字符
*			FILE *f		文件句柄
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
int fputc(int ch, FILE *f)
{
	UART_WriteByte(UART0, ch);
	
	while(UART_IsTXBusy(UART0));
 	
	return ch;
}
