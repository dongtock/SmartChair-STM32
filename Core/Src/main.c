#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "vl53l8cx_api.h"
#include "../VL53L0X_API(1D)/core/inc/vl53l0x_api.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

#define MUX_ADDR (0x70<<1)
#define MPU6050_ADDR (0x68<<1)
#define TOF_ADDR (0x29<<1)

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

VL53L8CX_Configuration vl53l8cx_dev[2];
VL53L0X_Dev_t vl53l0x_dev[4];
VL53L8CX_ResultsData tof3d_results;
int32_t hx711_weight[12]={0,},hx711_offset[12]={0,};
uint8_t rx_data,rx_index=0,oled_ok=0;
char rx_buffer[10]={0,};
volatile uint8_t stop_flag=0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);

#ifdef __GNUC__
int __io_putchar(int ch){ HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,0xFFFF); return ch; }
#endif

void OLED_Line(uint8_t line,const char *text){
  if(!oled_ok) return;
  ssd1306_SetCursor(0,line*16);
  char buf[19]; snprintf(buf,sizeof(buf),"%-18s",text);
  ssd1306_WriteString(buf,Font_7x10,White);
}
void OLED_Show(void){ if(oled_ok) ssd1306_UpdateScreen(); }
void OLED_Clear(void){ if(oled_ok) ssd1306_Fill(Black); }

void MUX_SelectChannel(uint8_t channel){
  if(channel>7) return;
  uint8_t reg=(1<<channel);
  HAL_I2C_Master_Transmit(&hi2c1,MUX_ADDR,&reg,1,20);
}

HAL_StatusTypeDef MPU6050_Init(void){
  uint8_t d;
  d=0x00; if(HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,0x6B,1,&d,1,100)!=HAL_OK) return HAL_ERROR;
  HAL_Delay(50);
  d=0x00; if(HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,0x1C,1,&d,1,100)!=HAL_OK) return HAL_ERROR;
  d=0x00; if(HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,0x1B,1,&d,1,100)!=HAL_OK) return HAL_ERROR;
  d=0x04; if(HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,0x1A,1,&d,1,100)!=HAL_OK) return HAL_ERROR;
  d=19;   if(HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,0x19,1,&d,1,100)!=HAL_OK) return HAL_ERROR;
  return HAL_OK;
}

HAL_StatusTypeDef VL53L8CX_Init(uint8_t ch){
  uint8_t is_alive=0,status=0;
  if(ch==0){
    vl53l8cx_dev[ch].platform.ncs_port=TOF3D_1_NCS_GPIO_Port;   vl53l8cx_dev[ch].platform.ncs_pin=TOF3D_1_NCS_Pin;
    vl53l8cx_dev[ch].platform.lpn_port=TOF3D_1_LPN_GPIO_Port;   vl53l8cx_dev[ch].platform.lpn_pin=TOF3D_1_LPN_Pin;
    vl53l8cx_dev[ch].platform.pwren_port=TOF3D_1_PWREN_GPIO_Port; vl53l8cx_dev[ch].platform.pwren_pin=TOF3D_1_PWREN_Pin;
  }else{
    vl53l8cx_dev[ch].platform.ncs_port=TOF3D_2_NCS_GPIO_Port;   vl53l8cx_dev[ch].platform.ncs_pin=TOF3D_2_NCS_Pin;
    vl53l8cx_dev[ch].platform.lpn_port=TOF3D_2_LPN_GPIO_Port;   vl53l8cx_dev[ch].platform.lpn_pin=TOF3D_2_LPN_Pin;
    vl53l8cx_dev[ch].platform.pwren_port=TOF3D_2_PWREN_GPIO_Port; vl53l8cx_dev[ch].platform.pwren_pin=TOF3D_2_PWREN_Pin;
  }
  HAL_GPIO_WritePin(vl53l8cx_dev[ch].platform.ncs_port,vl53l8cx_dev[ch].platform.ncs_pin,GPIO_PIN_SET);
  VL53L8CX_Reset_Sensor(&vl53l8cx_dev[ch].platform);
  status=vl53l8cx_is_alive(&vl53l8cx_dev[ch],&is_alive); if(status||!is_alive) return HAL_ERROR;
  status=vl53l8cx_init(&vl53l8cx_dev[ch]); if(status) return HAL_ERROR;
  status=vl53l8cx_set_resolution(&vl53l8cx_dev[ch],VL53L8CX_RESOLUTION_4X4); if(status) return HAL_ERROR;
  status=vl53l8cx_set_ranging_frequency_hz(&vl53l8cx_dev[ch],50); if(status) return HAL_ERROR;
  status=vl53l8cx_start_ranging(&vl53l8cx_dev[ch]); if(status) return HAL_ERROR;
  return HAL_OK;
}

HAL_StatusTypeDef VL53L0X_Init(uint8_t ch){
  VL53L0X_DEV dev=&vl53l0x_dev[ch];
  dev->I2cHandle=&hi2c1; dev->I2cDevAddr=TOF_ADDR;
  if(VL53L0X_DataInit(dev)) return HAL_ERROR;
  if(VL53L0X_StaticInit(dev)) return HAL_ERROR;
  uint32_t rsc; uint8_t ias;
  if(VL53L0X_PerformRefSpadManagement(dev,&rsc,&ias)) return HAL_ERROR;
  uint8_t vhv,pc;
  if(VL53L0X_PerformRefCalibration(dev,&vhv,&pc)) return HAL_ERROR;
  if(VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,20000)) return HAL_ERROR;
  if(VL53L0X_SetDeviceMode(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING)) return HAL_ERROR;
  if(VL53L0X_StartMeasurement(dev)) return HAL_ERROR;
  return HAL_OK;
}

uint8_t HX711_ReadAll_Raw(int32_t *data){
  uint32_t portA=GPIOA->IDR,portB=GPIOB->IDR;
  if(portA&((1<<1)|(1<<2)|(1<<3))) return 0;
  if(portA&((1<<8)|(1<<11)|(1<<12))) return 0;
  if(portB&((1<<15)|(1<<5)|(1<<8)|(1<<9))) return 0;
  if(portB&((1<<10)|(1<<12))) return 0;
  for(int i=0;i<12;i++) data[i]=0;
  __disable_irq();
  for(int i=0;i<24;i++){
    GPIOA->BSRR=GPIO_PIN_0;
    for(volatile int d=0;d<10;d++);
    portA=GPIOA->IDR; portB=GPIOB->IDR;
    if(portA&(1<<1))  data[0]  |= (1<<(23-i));
    if(portA&(1<<2))  data[1]  |= (1<<(23-i));
    if(portA&(1<<3))  data[2]  |= (1<<(23-i));
    if(portA&(1<<8))  data[3]  |= (1<<(23-i));
    if(portA&(1<<11)) data[4]  |= (1<<(23-i));
    if(portA&(1<<12)) data[5]  |= (1<<(23-i));
    if(portB&(1<<15)) data[6]  |= (1<<(23-i));
    if(portB&(1<<5))  data[7]  |= (1<<(23-i));
    if(portB&(1<<8))  data[8]  |= (1<<(23-i));
    if(portB&(1<<9))  data[9]  |= (1<<(23-i));
    if(portB&(1<<10)) data[10] |= (1<<(23-i));
    if(portB&(1<<12)) data[11] |= (1<<(23-i));
    GPIOA->BSRR=(uint32_t)GPIO_PIN_0<<16;
    for(volatile int d=0;d<10;d++);
  }
  GPIOA->BSRR=GPIO_PIN_0; for(volatile int d=0;d<10;d++);
  GPIOA->BSRR=(uint32_t)GPIO_PIN_0<<16; for(volatile int d=0;d<10;d++);
  __enable_irq();
  for(int i=0;i<12;i++) if(data[i]&0x00800000) data[i]|=0xFF000000;
  return 1;
}

void HX711_ReadAll_Blocking(int32_t *data){
  uint32_t timeout=HAL_GetTick();
  while(1){
    uint32_t portA=GPIOA->IDR,portB=GPIOB->IDR; uint8_t not_ready=0;
    if(portA&((1<<1)|(1<<2)|(1<<3))) not_ready=1;
    if(portA&((1<<8)|(1<<11)|(1<<12))) not_ready=1;
    if(portB&((1<<15)|(1<<5)|(1<<8)|(1<<9))) not_ready=1;
    if(portB&((1<<10)|(1<<12))) not_ready=1;
    if(not_ready==0) break;
    if(HAL_GetTick()-timeout>150) break;
  }
  for(int i=0;i<12;i++) data[i]=0;
  __disable_irq();
  for(int i=0;i<24;i++){
    GPIOA->BSRR=GPIO_PIN_0;
    for(volatile int d=0;d<10;d++);
    uint32_t portA=GPIOA->IDR,portB=GPIOB->IDR;
    if(portA&(1<<1))  data[0]  |= (1<<(23-i));
    if(portA&(1<<2))  data[1]  |= (1<<(23-i));
    if(portA&(1<<3))  data[2]  |= (1<<(23-i));
    if(portA&(1<<8))  data[3]  |= (1<<(23-i));
    if(portA&(1<<11)) data[4]  |= (1<<(23-i));
    if(portA&(1<<12)) data[5]  |= (1<<(23-i));
    if(portB&(1<<15)) data[6]  |= (1<<(23-i));
    if(portB&(1<<5))  data[7]  |= (1<<(23-i));
    if(portB&(1<<8))  data[8]  |= (1<<(23-i));
    if(portB&(1<<9))  data[9]  |= (1<<(23-i));
    if(portB&(1<<10)) data[10] |= (1<<(23-i));
    if(portB&(1<<12)) data[11] |= (1<<(23-i));
    GPIOA->BSRR=(uint32_t)GPIO_PIN_0<<16;
    for(volatile int d=0;d<10;d++);
  }
  GPIOA->BSRR=GPIO_PIN_0; for(volatile int d=0;d<10;d++);
  GPIOA->BSRR=(uint32_t)GPIO_PIN_0<<16; for(volatile int d=0;d<10;d++);
  __enable_irq();
  for(int i=0;i<12;i++) if(data[i]&0x00800000) data[i]|=0xFF000000;
}

void HX711_Init(void){
  int32_t tmp[12]={0,}; int64_t sum[12]={0,};
  HAL_Delay(500);
  for(int n=0;n<5;n++){ HX711_ReadAll_Blocking(tmp); HAL_Delay(20); }
  for(int n=0;n<20;n++){
    HX711_ReadAll_Blocking(tmp);
    for(int i=0;i<12;i++) sum[i]+=tmp[i];
    HAL_Delay(20);
  }
  for(int i=0;i<12;i++) hx711_offset[i]=(int32_t)(sum[i]/20);
}

typedef struct __attribute__((packed)){
  char header[4];
  int32_t hx711[12];
  uint16_t tof3d[32];
  uint16_t tof1d[4];
  int16_t mpu[2];
  uint8_t checksum;
} Main_Packet_t;

void Run_Calibration(void){
  Main_Packet_t pkt; memset(&pkt,0,sizeof(pkt)); memcpy(pkt.header,"CAL:",4);
  OLED_Clear(); OLED_Line(0,"Calibrating..."); OLED_Show();
  for(int cnt=0;cnt<500;cnt++){
    uint32_t t=HAL_GetTick(); uint8_t rdy=0;
    vl53l8cx_check_data_ready(&vl53l8cx_dev[0],&rdy);
    if(rdy){ vl53l8cx_get_ranging_data(&vl53l8cx_dev[0],&tof3d_results); for(int j=0;j<16;j++) pkt.tof3d[j]=(uint16_t)tof3d_results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*j]; }
    rdy=0; vl53l8cx_check_data_ready(&vl53l8cx_dev[1],&rdy);
    if(rdy){ vl53l8cx_get_ranging_data(&vl53l8cx_dev[1],&tof3d_results); for(int j=0;j<16;j++) pkt.tof3d[16+j]=(uint16_t)tof3d_results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*j]; }
    for(int i=0;i<4;i++){
      MUX_SelectChannel(i+2); uint8_t is=0;
      VL53L0X_RdByte(&vl53l0x_dev[i],0x13,&is);
      if(is&0x07){
        uint8_t buf[12];
        VL53L0X_ReadMulti(&vl53l0x_dev[i],0x14,buf,12);
        VL53L0X_WrByte(&vl53l0x_dev[i],0x0B,0x01);
        uint8_t st=buf[0]>>3;
        if(st==6||st==9||st==11){ uint16_t d=((uint16_t)buf[10]<<8)|buf[11]; if(d<1200) pkt.tof1d[i]=d; }
      }
    }
    for(int i=0;i<2;i++){
      MUX_SelectChannel(i+6); uint8_t b[6]; int16_t ax,az;
      if(HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,0x3B,1,b,6,20)==HAL_OK){
        ax=(b[0]<<8)|b[1]; az=(b[4]<<8)|b[5];
        pkt.mpu[i]=(int16_t)(atan2((float)(-ax),(float)az)*(180.0/3.141592));
      }else pkt.mpu[i]=0;
    }
    /* 메인 루프와 동일하게 110ms 간격 Blocking 읽기 */
    { static uint32_t last_hx_time=0;
      if(HAL_GetTick()-last_hx_time>=110){
        int32_t raw[12]={0,};
        HX711_ReadAll_Blocking(raw);
        for(int i=0;i<12;i++) pkt.hx711[i]=raw[i]-hx711_offset[i];
        last_hx_time=HAL_GetTick();
      }
    }
    uint8_t *p=(uint8_t*)&pkt,ck=0;
    for(int k=0;k<sizeof(Main_Packet_t)-1;k++) ck^=p[k];
    pkt.checksum=ck;
    HAL_UART_Transmit(&huart1,(uint8_t*)&pkt,sizeof(Main_Packet_t),50);
    { static uint32_t ot=0;
      if(oled_ok&&HAL_GetTick()-ot>=1000){ char ob[19]; snprintf(ob,sizeof(ob),"CAL: %d/%d",cnt,500); OLED_Line(1,ob); OLED_Show(); ot=HAL_GetTick(); }
    }
    while(HAL_GetTick()-t<20){}
  }
}

int main(void){
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();

  uint8_t i2c_ok=0;
  if(HAL_I2C_IsDeviceReady(&hi2c1,0x78,2,50)==HAL_OK){
    i2c_ok=1; ssd1306_Init(); oled_ok=1;
    OLED_Clear(); OLED_Line(0,"Booting..."); OLED_Show();
  }else if(HAL_I2C_IsDeviceReady(&hi2c1,MUX_ADDR,2,50)==HAL_OK) i2c_ok=1;

  HAL_GPIO_WritePin(HX_SCK_GPIO_Port,HX_SCK_Pin,GPIO_PIN_RESET);
  HAL_Delay(1000);

  uint8_t tof3d_ok[2]={0,0};
  for(uint8_t i=0;i<2;i++) if(VL53L8CX_Init(i)==HAL_OK) tof3d_ok[i]=1;
  { char ob[19]; snprintf(ob,sizeof(ob),"3D: %s %s",tof3d_ok[0]?"OK":"ER",tof3d_ok[1]?"OK":"ER"); OLED_Line(1,ob); OLED_Show(); }
  HAL_Delay(2000);

  uint8_t t1d_cnt=0,mpu_cnt=0;
  if(i2c_ok&&HAL_I2C_IsDeviceReady(&hi2c1,MUX_ADDR,5,100)==HAL_OK){
    for(uint8_t i=2;i<8;i++){
      MUX_SelectChannel(i); HAL_Delay(5);
      uint16_t addr=(i<6)?TOF_ADDR:MPU6050_ADDR;
      if(HAL_I2C_IsDeviceReady(&hi2c1,addr,3,100)==HAL_OK){
        if(i<6){ if(VL53L0X_Init(i-2)==HAL_OK) t1d_cnt++; }
        else{ if(MPU6050_Init()==HAL_OK) mpu_cnt++; }
      }
    }
  }
  { char ob[19]; snprintf(ob,sizeof(ob),"1D:%d/4 MPU:%d/2",t1d_cnt,mpu_cnt); OLED_Line(2,ob); OLED_Show(); }
  HAL_Delay(3000);

  OLED_Line(3,"HX711 Tare..."); OLED_Show();
  HX711_Init();
  OLED_Line(3,"HX711: OK"); OLED_Show();
  HAL_Delay(2000);

  OLED_Clear(); OLED_Line(0,"RPi5 waiting..."); OLED_Show();
  char rx_ack[16]={0}; uint8_t b=0,idx=0; uint16_t tx_ok=0,tx_fail=0;

  while(1){
    HAL_StatusTypeDef tx_result=HAL_UART_Transmit(&huart1,(uint8_t*)"READY\n",6,100);
    if(tx_result==HAL_OK) tx_ok++; else tx_fail++;
    { char ob[20]; snprintf(ob,sizeof(ob),"OK:%d FAIL:%d",tx_ok,tx_fail); OLED_Line(1,ob); OLED_Show(); }
    memset(rx_ack,0,sizeof(rx_ack)); idx=0;
    uint32_t start=HAL_GetTick();
    while(HAL_GetTick()-start<500){
      if(HAL_UART_Receive(&huart1,&b,1,10)==HAL_OK){
        if(idx<sizeof(rx_ack)-1){ rx_ack[idx++]=b; rx_ack[idx]='\0'; }
        if(strstr(rx_ack,"ACK")){ OLED_Line(2,"ACK received!"); OLED_Show(); goto handshake_done; }
      }
    }
    HAL_Delay(500);
  }

handshake_done:
  HAL_UART_Transmit(&huart1,(uint8_t*)"LINK_OK\n",8,100);
  OLED_Line(3,"LINK_OK sent!"); OLED_Show();
  HAL_Delay(1000);

  while(1){
    int8_t is_seated=0;
    OLED_Clear(); OLED_Line(0,"Wait CHK_SIT..."); OLED_Show();
    char rx_cmd[15]={0,}; uint8_t rx_idx=0;
    while(1){
      uint8_t b=0;
      if(HAL_UART_Receive(&huart1,&b,1,10)==HAL_OK){
        rx_cmd[rx_idx++]=b;
        if(strstr(rx_cmd,"CHK_SIT")) break;
        if(rx_idx>=14){ memset(rx_cmd,0,sizeof(rx_cmd)); rx_idx=0; }
      }
    }

    OLED_Line(0,"Wait SIT..."); OLED_Show();
    while(1){
      int32_t current_raw[12]={0,}; HX711_ReadAll_Blocking(current_raw);
      int32_t seat_weight_sum=0;
      for(int i=8;i<12;i++) seat_weight_sum+=(current_raw[i]-hx711_offset[i]);
      if(seat_weight_sum>225000){
        is_seated=1;
        HAL_UART_Transmit(&huart1,(uint8_t*)"SIT\n",4,100);
        OLED_Line(0,"SIT detected"); OLED_Show();
        break;
      }
      HAL_Delay(100);
    }

    OLED_Line(1,"Wait CAL/GO..."); OLED_Show();
    memset(rx_cmd,0,sizeof(rx_cmd)); rx_idx=0;
    while(1){
      uint8_t b=0;
      if(HAL_UART_Receive(&huart1,&b,1,10)==HAL_OK){
        rx_cmd[rx_idx++]=b;
        if(strstr(rx_cmd,"CHK_SIT")){
          HAL_UART_Transmit(&huart1,(uint8_t*)"SIT\n",4,100);
          OLED_Line(2,"SIT re-sent"); OLED_Show();
          memset(rx_cmd,0,sizeof(rx_cmd)); rx_idx=0; continue;
        }
        if(strstr(rx_cmd,"CAL")){
          Run_Calibration();
          HAL_Delay(100);
          HAL_UART_Transmit(&huart1,(uint8_t*)"CAL_DONE\n",9,100);
          is_seated=0; break;
        }
        if(strstr(rx_cmd,"GO")){
          OLED_Line(2,"GO received!"); OLED_Show();
          break;
        }
        if(rx_idx>=14){ memset(rx_cmd,0,sizeof(rx_cmd)); rx_idx=0; }
      }
    }

    if(is_seated==1){
      Main_Packet_t pkt; memset(&pkt,0,sizeof(pkt)); memcpy(pkt.header,"DAT:",4);
      OLED_Clear(); OLED_Line(0,"Measuring..."); OLED_Show();
      uint32_t empty_count=0,oled_tick=0,hz_count=0; uint8_t exit_reason=0;
      stop_flag=0; rx_index=0; memset(rx_buffer,0,sizeof(rx_buffer));
      HAL_UART_Receive_IT(&huart1,&rx_data,1);

      while(is_seated){
        uint32_t loop_start=HAL_GetTick(),t0=HAL_GetTick(); hz_count++;

        uint8_t rdy=0;
        vl53l8cx_check_data_ready(&vl53l8cx_dev[0],&rdy);
        if(rdy){ vl53l8cx_get_ranging_data(&vl53l8cx_dev[0],&tof3d_results); for(int j=0;j<16;j++) pkt.tof3d[j]=(uint16_t)tof3d_results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*j]; }
        rdy=0; vl53l8cx_check_data_ready(&vl53l8cx_dev[1],&rdy);
        if(rdy){ vl53l8cx_get_ranging_data(&vl53l8cx_dev[1],&tof3d_results); for(int j=0;j<16;j++) pkt.tof3d[16+j]=(uint16_t)tof3d_results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE*j]; }

        for(int i=0;i<4;i++){
          MUX_SelectChannel(i+2); uint8_t is=0;
          VL53L0X_RdByte(&vl53l0x_dev[i],0x13,&is);
          if(is&0x07){
            uint8_t buf[12];
            VL53L0X_ReadMulti(&vl53l0x_dev[i],0x14,buf,12);
            VL53L0X_WrByte(&vl53l0x_dev[i],0x0B,0x01);
            uint8_t st=buf[0]>>3;
            if(st==6||st==9||st==11){ uint16_t d=((uint16_t)buf[10]<<8)|buf[11]; if(d<1200) pkt.tof1d[i]=d; }
          }
        }

        for(int i=0;i<2;i++){
          MUX_SelectChannel(i+6); uint8_t b[6]; int16_t ax,az;
          if(HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,0x3B,1,b,6,20)==HAL_OK){
            ax=(b[0]<<8)|b[1]; az=(b[4]<<8)|b[5];
            pkt.mpu[i]=(int16_t)(atan2((float)(-ax),(float)az)*(180.0/3.141592));
          }else pkt.mpu[i]=0;
        }

        /* HX711 10SPS: 110ms 간격으로 Blocking 읽기하여 12채널 동기화.
           나머지 프레임은 이전 값을 재사용하므로 50Hz 루프 유지. */
        int32_t seat_sum=0,seat_raw_sum=0;
        { static uint32_t last_hx_time=0;
          if(HAL_GetTick()-last_hx_time>=110){
            int32_t raw[12]={0,};
            HX711_ReadAll_Blocking(raw);
            for(int i=0;i<12;i++){
              pkt.hx711[i]=raw[i]-hx711_offset[i];
            }
            last_hx_time=HAL_GetTick();
          }
        }
        for(int i=8;i<=11;i++){ seat_sum+=pkt.hx711[i]; }

        if(seat_sum<225000){
          empty_count++;
          if(empty_count>=250){
            HAL_UART_Transmit(&huart1,(uint8_t*)"STAND\n",6,10);
            is_seated=0; exit_reason=1; break;
          }
        }else empty_count=0;

        if(stop_flag==1){ is_seated=0; stop_flag=0; exit_reason=2; break; }

        uint8_t *pb=(uint8_t*)&pkt,ck=0;
        for(int k=0;k<sizeof(Main_Packet_t)-1;k++) ck^=pb[k];
        pkt.checksum=ck;
        HAL_UART_Transmit(&huart1,(uint8_t*)&pkt,sizeof(Main_Packet_t),50);
        uint32_t total_ms=HAL_GetTick()-t0;

        if(oled_ok&&HAL_GetTick()-oled_tick>=1000){
          char ob[19];
          OLED_Clear(); OLED_Line(0,"Measuring...");
          snprintf(ob,sizeof(ob),"S+TX:%lu ms",total_ms); OLED_Line(1,ob);
          snprintf(ob,sizeof(ob),"AVG:%lu Hz",hz_count); OLED_Line(2,ob);
          snprintf(ob,sizeof(ob),"RAW:%ld",(long)seat_raw_sum); OLED_Line(3,ob);
          OLED_Show(); hz_count=0; oled_tick=HAL_GetTick();
        }

        while(HAL_GetTick()-loop_start<20){}
      }

      HAL_UART_AbortReceive(&huart1);
      OLED_Clear();
      if(exit_reason==1) OLED_Line(0,"STAND detected");
      else if(exit_reason==2) OLED_Line(0,"STOP/QUIT recv");
      else OLED_Line(0,"Session ended");
      OLED_Show(); HAL_Delay(2000);
    }
  }
}

void SystemClock_Config(void){
  RCC_OscInitTypeDef o={0}; RCC_ClkInitTypeDef c={0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  o.OscillatorType=RCC_OSCILLATORTYPE_HSI; o.HSIState=RCC_HSI_ON;
  o.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;
  o.PLL.PLLState=RCC_PLL_ON; o.PLL.PLLSource=RCC_PLLSOURCE_HSI;
  o.PLL.PLLM=8; o.PLL.PLLN=100; o.PLL.PLLP=RCC_PLLP_DIV2; o.PLL.PLLQ=4;
  if(HAL_RCC_OscConfig(&o)!=HAL_OK) Error_Handler();
  c.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  c.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK; c.AHBCLKDivider=RCC_SYSCLK_DIV1;
  c.APB1CLKDivider=RCC_HCLK_DIV2; c.APB2CLKDivider=RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&c,FLASH_LATENCY_3)!=HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void){
  hi2c1.Instance=I2C1; hi2c1.Init.ClockSpeed=400000; hi2c1.Init.DutyCycle=I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1=0; hi2c1.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE; hi2c1.Init.OwnAddress2=0;
  hi2c1.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE; hi2c1.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
  if(HAL_I2C_Init(&hi2c1)!=HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void){
  hspi1.Instance=SPI1; hspi1.Init.Mode=SPI_MODE_MASTER; hspi1.Init.Direction=SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize=SPI_DATASIZE_8BIT; hspi1.Init.CLKPolarity=SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase=SPI_PHASE_2EDGE; hspi1.Init.NSS=SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_32; hspi1.Init.FirstBit=SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode=SPI_TIMODE_DISABLE; hspi1.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial=10;
  if(HAL_SPI_Init(&hspi1)!=HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void){
  huart1.Instance=USART1; huart1.Init.BaudRate=921600;
  huart1.Init.WordLength=UART_WORDLENGTH_8B; huart1.Init.StopBits=UART_STOPBITS_1;
  huart1.Init.Parity=UART_PARITY_NONE; huart1.Init.Mode=UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl=UART_HWCONTROL_NONE; huart1.Init.OverSampling=UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&huart1)!=HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void){
  GPIO_InitTypeDef g={0};
  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(HX_SCK_GPIO_Port,HX_SCK_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TOF3D_1_NCS_GPIO_Port,TOF3D_1_NCS_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,TOF3D_1_LPN_Pin|TOF3D_1_PWREN_Pin|TOF3D_2_LPN_Pin|TOF3D_2_PWREN_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TOF3D_2_NCS_GPIO_Port,TOF3D_2_NCS_Pin,GPIO_PIN_SET);

  g.Pin=HX_SCK_Pin|TOF3D_1_NCS_Pin; g.Mode=GPIO_MODE_OUTPUT_PP; g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA,&g);
  g.Pin=DOUT_1_Pin|DOUT_2_Pin|DOUT_3_Pin|DOUT_4_Pin|DOUT_5_Pin|DOUT_6_Pin; g.Mode=GPIO_MODE_INPUT; g.Pull=GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA,&g);
  g.Pin=TOF3D_1_LPN_Pin|TOF3D_1_PWREN_Pin|TOF3D_2_NCS_Pin|TOF3D_2_LPN_Pin|TOF3D_2_PWREN_Pin;
  g.Mode=GPIO_MODE_OUTPUT_PP; g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB,&g);
  g.Pin=DOUT_11_Pin|DOUT_12_Pin|DOUT_7_Pin|DOUT_8_Pin|DOUT_9_Pin|DOUT_10_Pin; g.Mode=GPIO_MODE_INPUT; g.Pull=GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB,&g);
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev,uint8_t idx,uint8_t *p,uint32_t c){ HAL_I2C_Mem_Write(Dev->I2cHandle,Dev->I2cDevAddr,idx,I2C_MEMADD_SIZE_8BIT,p,c,100); return 0; }
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev,uint8_t idx,uint8_t *p,uint32_t c){ HAL_I2C_Mem_Read(Dev->I2cHandle,Dev->I2cDevAddr,idx,I2C_MEMADD_SIZE_8BIT,p,c,100); return 0; }
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev,uint8_t idx,uint8_t d){ return VL53L0X_WriteMulti(Dev,idx,&d,1); }
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev,uint8_t idx,uint16_t d){ uint8_t b[2]={d>>8,d&0xFF}; return VL53L0X_WriteMulti(Dev,idx,b,2); }
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev,uint8_t idx,uint32_t d){ uint8_t b[4]={(d>>24)&0xFF,(d>>16)&0xFF,(d>>8)&0xFF,d&0xFF}; return VL53L0X_WriteMulti(Dev,idx,b,4); }
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev,uint8_t idx,uint8_t a,uint8_t o){ uint8_t d; VL53L0X_ReadMulti(Dev,idx,&d,1); d=(d&a)|o; return VL53L0X_WriteMulti(Dev,idx,&d,1); }
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev,uint8_t idx,uint8_t *d){ return VL53L0X_ReadMulti(Dev,idx,d,1); }
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev,uint8_t idx,uint16_t *d){ uint8_t b[2]; VL53L0X_ReadMulti(Dev,idx,b,2); *d=((uint16_t)b[0]<<8)|b[1]; return 0; }
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev,uint8_t idx,uint32_t *d){ uint8_t b[4]; VL53L0X_ReadMulti(Dev,idx,b,4); *d=((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3]; return 0; }
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev){ HAL_Delay(1); return 0; }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance==USART1){
    rx_buffer[rx_index++]=rx_data;
    if(rx_index>=4){
      rx_buffer[rx_index]='\0';
      if(strstr(rx_buffer,"STOP")||strstr(rx_buffer,"QUIT")){
        stop_flag=1; memset(rx_buffer,0,sizeof(rx_buffer)); rx_index=0;
      }
    }
    if(rx_index>=9){ memset(rx_buffer,0,sizeof(rx_buffer)); rx_index=0; }
    HAL_UART_Receive_IT(&huart1,&rx_data,1);
  }
}

void Error_Handler(void){ __disable_irq(); while(1){} }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file,uint32_t line){}
#endif
