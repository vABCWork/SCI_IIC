#include	<machine.h>
#include	 "iodefine.h"
#include	 "misratypes.h"
#include	"delay.h"
#include	"timer.h"
#include	"sci_iic.h"

void clear_module_stop(void);
void test_sci_iic_intr_aht25(void);
void test_sci_iic_intr_thermo_pile(void);


void main(void)
{
	
	clear_module_stop();	//  モジュールストップの解除
	
	initSCI_12();		// SCI12(簡易I2C) 初期設定
					
	delay_msec(100);	// センサ安定待ち (100msec 待つ) 
	
	Timer10msec_Set();      // タイマ(10msec)作成(CMT0)
     	Timer10msec_Start();    // タイマ(10msec)開始　
	

	test_sci_iic_intr_aht25();		//  温湿度センサ(AHT25)のテスト  (簡易I2C 割り込み使用)
	

//	test_sci_iic_intr_thermo_pile();		// サーモパイルのテスト(簡易I2C 割り込み使用)

}


// 温湿度センサ(AHT25)のテスト  (簡易I2C 割り込み使用)
//  
//    0        500msec   1000msec   1500msec      0        500msec   1000msec
//    +----------+----------+----------+----------+----------+----------+----------+
//  task0       task1      task2                task0       task1      task2
// 
//  task0: マスタ送信(測定開始コマンド)
//  task1: マスタ受信(温湿度データ読み出し) 
//  task2: 温湿度の計算
//
void test_sci_iic_intr_aht25(void)
{
	uint32_t flg_task0;
	uint32_t flg_task1;
	uint32_t flg_task2;
	
	flg_task0 = 0;
	flg_task1 = 0;
	flg_task2 = 0;
	
	
	sci_iic_slave_adrs = 0x38;    	//  スレーブアドレス = 0x3B (温湿度センサ AHT25)
	
	while(1)			
	{
	  
	  if ( timer_10msec_cnt == 0 ) {   // 最初 		
	     if ( flg_task0 == 0 ) {
		   
	       	   sci_wr_sensor_cmd();	//温湿度センサへの測定開始コマンド送信
		   
		   flg_task0 = 1;
	      }
	   }
		  
	   else if ( timer_10msec_cnt == 50 ) {    // 500[msec]経過
	       if ( flg_task1 == 0 ) {    
	  
		    sci_rd_sensor_humi_temp();	// 温湿度データの読み出し  
		    
		    flg_task1 = 1;   
	       }
	   }
	  
	   else if ( timer_10msec_cnt == 100 ) {    // 1000[msec]経過
	       if ( flg_task2 == 0 ) {    
	  
		    sci_cal_humidity_temperature();	//  湿度と温度を計算
		    
		    flg_task2 = 1;   
	       }
	   }
	  
	  else if ( timer_10msec_cnt == 150 ) {    // 1500[msec]経過
	  	
	  	flg_task0 = 0;
		flg_task1 = 0;
		flg_task2 = 0;
	  
	  }	       	  
		
	}
	
}



//　サーモパイルのテスト(簡易I2C 割り込み使用)
//  
//    0        500msec   1000msec   1500msec      0        500msec   1000msec   1500msec
//    +----------+----------+----------+----------+----------+----------+----------+----
//  task0       task1      task2     task3     task0       task1      task2      task3
// 
//  task0: マスタ送受信(周囲温度の読み出し)
//  task1:　周囲温度の計算 
//  task2: マスタ送受信(測定対象部温度の読み出し)
//  task3:  測定対象部温度の計算
//
void test_sci_iic_intr_thermo_pile(void)
{
	uint32_t flg_task0;
	uint32_t flg_task1;
	uint32_t flg_task2;
	uint32_t flg_task3;
	
	flg_task0 = 0;
	flg_task1 = 0;
	flg_task2 = 0;
	flg_task3 = 0;
	
	sci_iic_slave_adrs = 0x3d;    	//  スレーブアドレス = 0x3D (7bit; 011 1101)  (サーモパイル A3D01S)
	
	while(1)			
	{
	  
	  if ( timer_10msec_cnt == 0 ) {   // 最初 		
	     if ( flg_task0 == 0 ) {
		     
		   sci_rd_thermo_pile(0);          // センサの周囲温度(Self temperature)(TA)を読み出す
	       	   
		   flg_task0 = 1;
	      }
	   }
		  
	   else if ( timer_10msec_cnt == 50 ) {    // 500[msec]経過
	       if ( flg_task1 == 0 ) {  
		       
	          sci_iic_cal_crc_thermo_pile();	// PECによるデータチェック 
	
		  sci_iic_cal_Ta_To_temperature();	// 周囲温度の計算
		    
		  flg_task1 = 1;   
	       }
	   }
	  
	   else if ( timer_10msec_cnt == 100 ) {    // 1000[msec]経過
	       if ( flg_task2 == 0 ) {    
	  
		    sci_rd_thermo_pile(1);          // 測定対象物の温度(Objet temperature)(TO)を読み出す
		    
		    flg_task2 = 1;   
	       }
	   }
	   
	    else if ( timer_10msec_cnt == 150 ) {    // 1500[msec]経過
		if ( flg_task3 == 0 ) {   
	  	  
	          sci_iic_cal_crc_thermo_pile();	// PECによるデータチェック 
	
		  sci_iic_cal_Ta_To_temperature();	// 測定対象物の温度の計算
		 
		  flg_task3 = 1;
	        } 
	    }	       	  
	   
	   else if ( timer_10msec_cnt == 190 ) {    // 1900[msec]経過
	  	
	  	flg_task0 = 0;
		flg_task1 = 0;
		flg_task2 = 0;
		flg_task3 = 0;
	  
	  }	       	  
		
	}
}




// モジュールストップの解除
// コンペアマッチタイマ(CMT) ユニット0(CMT0, CMT1) 
//   シリアルコミュニケーションインタフェース12(SCI12)(簡易I2C通信)
//  CRC 演算器（CRC）(RIIC I2C通信用)
//
void clear_module_stop(void)
{
	SYSTEM.PRCR.WORD = 0xA50F;	// クロック発生、消費電力低減機能関連レジスタの書き込み許可	
	
	MSTP(CMT0) = 0;			// コンペアマッチタイマ(CMT) ユニット0(CMT0, CMT1) モジュールストップの解除
	MSTP(SCI12) = 0;		// SCI12  モジュールストップの解除
	MSTP(CRC) = 0;			// CRC モジュールストップの解除
	
	SYSTEM.PRCR.WORD = 0xA500;	// クロック発生、消費電力低減機能関連レジスタ書き込み禁止
}

