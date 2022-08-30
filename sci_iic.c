#include "iodefine.h"
#include "misratypes.h"
#include "sci_iic.h"



uint8_t sci_iic_slave_adrs;  // 簡易I2C スレーブアドレス  00: 7bitアドレス( 例:100 0000 = 0x40 )

volatile uint8_t sci_iic_rcv_data[16];   // IIC受信データ
volatile uint8_t sci_iic_sd_data[16];    // 送信データ
volatile uint8_t sci_iic_sd_pt;	    // 送信データ位置
volatile uint8_t sci_iic_rcv_pt;         // 受信データ位置

volatile uint8_t  sci_dummy_send_fg;    // ダミーデータ(0xff)の送信により受信データを得るためのフラグ

volatile uint8_t  sci_iic_sd_rcv_fg;	    // 0:送信のみまたは受信のみの場合,  1:マスタ送受信の場合(= リスタートがある場合)
volatile uint8_t  sci_iic_sd_num;	    // 送信データ数(スレーブアドレスを含む)
volatile uint8_t  sci_iic_rcv_num;      // 受信データ数
volatile uint8_t  sci_iic_com_over_fg;  // 1:STOPコンディションの検出時


uint8_t sci_smbus_crc_8;	        //  CRC演算対象データ: 最初のスレーブアドレスから、コマンド、スレーブアドレス(Read用)、受信データ(low側）、受信データ(high側)の5バイト文
uint8_t sci_smbus_crc_ng;          // 上記の5byteに、スレーブ側から送信された、PECを入れて、CRC計算した値が0ならば、異常なし。
			           // ( 32.2.3 CRC データ出力レジスタ（CRCDOR）「RX23E-Aグループ ユーザーズマニュアル　ハードウェア編」 (R01UH0801JJ0120 Rev.1.20) )  
			
		
uint8_t sci_iic_sensor_status;	   // センサのステータス
uint32_t sci_iic_sensor_humidity;	   // センサからの湿度データ
uint32_t sci_iic_sensor_temperature;  // センサからの温度データ
					

uint8_t  sci_crc_x8_x5_x4_1;	// 温湿度センサ用　CRC-8 (x8 + x5 + X4 + 1)
uint8_t  sci_iic_crc_ng;


uint16_t sci_ta_word;
float  sci_ta_celsius;		  // Self temperature (センサの周囲温度 Ta)[℃]

uint16_t sci_to_word;
float  sci_to_celsius;		// Object temperature (測定対象物の温度 To)[℃]


//
// 受信割り込み(SCI12)
//   1バイトの受信を完了した時点で発生する
// 
#pragma interrupt (Excep_SCI12_RXI12(vect=239))
void Excep_SCI12_RXI12(void)
{
	  sci_iic_rcv_data[sci_iic_rcv_pt] =  SCI12.RDR;	// 受信データ読み出し
	  
	  sci_iic_rcv_pt++;
	  
}


//
// 送信割り込み(TXI12)
//  1バイトの送信を完了し、ACKまたはNACK検出後に発生
//
#pragma interrupt (Excep_SCI12_TXI12(vect=240))

void Excep_SCI12_TXI12(void)
{
	if ( SCI12.SISR.BIT.IICACKR == 0 ) {	// ACK認識
          
	  if ( sci_iic_sd_rcv_fg == 1 ) { 	// マスタ送受信の場合
	   if( sci_dummy_send_fg == 0 ) {    // ダミー送信でない場合
	     if ( sci_iic_sd_pt == 1 ) {    //  ( スレーブアドレス + Wビット)送信完了の場合
	
		SCI12.TDR = sci_iic_sd_data[sci_iic_sd_pt];     // 送信データをライト　(コマンド(読み出しアドレス)の送信)
	     	sci_iic_sd_pt++;				// 送信データの格納場所のインクリメント
	     }
	     
	     else if ( sci_iic_sd_pt == 2 ) {	 // コマンド(読み出しアドレス)送信完了の場合、再開始条件(リスタートコンディション)の発行
		
	        SCI12.SIMR3.BIT.IICSTIF = 0;     // IICSTIFフラグを“0”にしてから、各条件生成を行う。
	        SCI12.SIMR3.BYTE = 0x52;	// 再開始条件(リスタートコンディション)の生成 (STI割り込み発生)
	     }
	     
	     else if ( sci_iic_sd_pt == 3 ) {   // ( スレーブアドレス + Rビット)送信完了の場合、　
		SCI12.SIMR2.BIT.IICACKT = 0;    // 以降はACK送信 (マスタ受信時には、スレーブへの ACK送信)
	        SCI12.SCR.BIT.RIE = 1;		// 受信データ割り込み(RXI)要求の許可	
	     
		sci_dummy_send_fg = 1;		// ダミー送信フラグのセット
		SCI12.TDR = 0xff;		// ダミーデータの送信
	     }
	   
	   }
	   else {				// ダミー送信の場合
	      if ( sci_iic_rcv_pt < 3 ) {       // 受信するデータが残っている場合
	          SCI12.TDR = 0xff;		// 再度、ダミーデータの送信
	      }
	      else {				// 全て受信済みの場合、停止条件(ストップコンディション発行)
	        sci_dummy_send_fg = 0;		// ダミー送信フラグのクリア
	        
	        SCI12.SIMR3.BIT.IICSTIF = 0;     // IICSTIFフラグを“0”にしてから、各条件生成を行う。
	        SCI12.SIMR3.BYTE = 0x54;	 // 停止条件(ストップコンディション)の生成 (STI割り込み発生)
	  
	      } // 全て受信済みの
	   }  // ダミー送信
	  }   // マスタ送受信の場合
	  
	  else {						// マスタ送信、マスタ受信の場合
	  	
		if ( (sci_iic_sd_data[0] & 0x01) == 1 ) {  // マスタ受信の場合
		
		   if( sci_dummy_send_fg == 0 ) {    // ダミー送信でない場合(スレーブアドレスとRビット 送信後)
		   
	        	SCI12.SCR.BIT.RIE = 1;		// 受信データ割り込み(RXI)要求の許可	
	     
		        if ( sci_iic_rcv_num == 1 ) {   // 受信データが1バイトの場合
			    
		           SCI12.SIMR2.BIT.IICACKT = 1;   // NACK送信の準備
			}
			else {
			   SCI12.SIMR2.BIT.IICACKT = 0;   // ACK送信の準備
			}
		        sci_dummy_send_fg = 1;		// ダミー送信フラグのセット
			SCI12.TDR = 0xff;		// ダミーデータの送信
		   }
		   else {				// ダミー送信中の場合
		   
		        if ( sci_iic_rcv_pt == (sci_iic_rcv_num - 1) ) {   //　次の受信が最終データの場合
			   	SCI12.SIMR2.BIT.IICACKT = 1;   // NACK送信の準備
			 }
			 
		         SCI12.TDR = 0xff;		// 再度、ダミーデータの送信
	            
		   }
		}     // マスタ受信
		
		else{					// マスタ送信の場合
		  if (  sci_iic_sd_pt < sci_iic_sd_num ) {
		 	SCI12.TDR = sci_iic_sd_data[sci_iic_sd_pt];     // 送信データをライト　(コマンド(読み出しアドレス)の送信)
	     		sci_iic_sd_pt++;				// 送信データの格納場所のインクリメント
		  }
		  else {			       // 全データの送信完了 
	              SCI12.SIMR3.BIT.IICSTIF = 0;     // IICSTIFフラグを“0”にしてから、各条件生成を行う。
	              SCI12.SIMR3.BYTE = 0x54;	       // 停止条件(ストップコンディション)の生成 (STI割り込み発生)
		  }
		}  // マスタ送信
		
	  }   //  マスタ送信、マスタ受信の場合
	  
	}     // ACK受信
	
	else {					// NACK認識 (最終データ受信でNACK応答の場合を含む)
	        sci_dummy_send_fg = 0;		// ダミー送信フラグのクリア
	 	SCI12.SIMR3.BIT.IICSTIF = 0;     // IICSTIFフラグを“0”にしてから、各条件生成を行う。
	        SCI12.SIMR3.BYTE = 0x54;	    // 停止条件(ストップコンディション)の生成 (STI割り込み発生)
	}
	
}

//
// STI割り込み(送信終了割り込み(TEI) (SCI12)
// 開始条件または、再開始条件または、停止条件の発行完了により割り込み発生

#pragma interrupt (Excep_SCI12_TEI12(vect=241))
void Excep_SCI12_TEI12(void){
	
	if ( sci_iic_sd_pt == 0) {      // 開始条件(スタートコンディション)発行完了
             
	     SCI12.SIMR3.BIT.IICSTIF = 0;  
	     SCI12.SIMR3.BYTE = 0;	// SSDA=シリアルデータ出力, SSCL=シリアルクロック出力
	     
	     SCI12.TDR = sci_iic_sd_data[sci_iic_sd_pt];     // 送信データをライト
	     sci_iic_sd_pt++;				     // 送信データの格納場所インクリメント
	     
	     return;
	}
	
	if ( sci_iic_sd_rcv_fg == 1 ) { 	// マスタ送受信の場合
	
	    if (  sci_iic_sd_pt == 2 ) {	 // 再開始条件(リスタートコンディション)の発行完了
	
	       SCI12.SIMR3.BIT.IICSTIF = 0;  
	       SCI12.SIMR3.BYTE = 0;	// SSDA=シリアルデータ出力, SSCL=シリアルクロック出力
	     
	       SCI12.TDR = sci_iic_sd_data[sci_iic_sd_pt];     // 送信データをライト 　( スレーブアドレス + Rビット)
	       sci_iic_sd_pt++;				     // 送信データの格納場所インクリメント
	    }
	
	    else if ( sci_iic_rcv_pt == 3 ) {	// 停止条件(ストップコンディション)の発行完了
	       SCI12.SIMR3.BIT.IICSTIF = 0; 
	       SCI12.SIMR3.BYTE = 0xf0;	// SSDA,SSCLはハイインピーダンス状態にする
	    
	       sci_iic_com_over_fg = 1;   // 通信完了
	    }
	}
	else {					// マスタ送信、マスタ受信で、STOPコンディション発生
		SCI12.SIMR3.BIT.IICSTIF = 0; 
	        SCI12.SIMR3.BYTE = 0xf0;		// SSDA,SSCLはハイインピーダンス状態にする
	    
	        sci_iic_com_over_fg = 1;   // 通信完了
	     
	}
	
}





//  温湿度センサのステータス読み出し (マスタ受信)(割り込み使用)
// IIC 送信バッファ
//   　sci_iic_sd_data[0] : スレーブアドレス(7bit) + 1(R/W#ビット=Read)
//   受信バッファ
//     sci_iic_rcv_data[0]: ステータス

void sci_rd_sensor_status(void)
{
	sci_iic_sd_data[0] = (( sci_iic_slave_adrs << 1 ) | 0x01 ) ;  // スレーブアドレスから読出し
	
	sci_iic_master_rcv (1);		//　マスタ受信　開始
	
	while( sci_iic_com_over_fg != 1 ) {		// 通信完了待ち(受信完了待ち)
	}
	
	sci_iic_sensor_status = sci_iic_rcv_data[0];  // センサのステータス
	
}




//  温湿度センサからステータスと温湿度データの読み出し (マスタ受信)(割り込み使用)
// IIC 送信バッファ
//   sci_iic_sd_data[0] : スレーブアドレス(7bit) + 1(=Read)
// IIC 受信バッファ
//   sci_iic_rcv_data[0]: ステータス
//             :   [1]: 湿度データ(b19-b12)
//             :   [2]: 湿度データ(b11-b4)
//             :   [3]のb7-b4: 湿度データ(b3-b0)
//             :   [3]のb3-b0: 温度データ(b19-b16)
//             :   [4]: 温度データ(b15-b8)
//             :   [5]: 温度データ(b7-b0)
//             :   [6]: CRC 
void sci_rd_sensor_humi_temp(void)
{
	
	sci_iic_sd_data[0] = (( sci_iic_slave_adrs << 1 ) | 0x01 ) ;  // スレーブアドレスから読出し

	sci_iic_master_rcv (7);		//　マスタ受信　開始
	
	 
}




// 温湿度センサへの測定開始コマンド送信　(マスタ送信)(割り込み使用)
//	 IIC 送信バッファ
//   sci_iic_sd_data[0] : スレーブアドレス(7bit) + 0(=wite)
//                [1] : Trigger measure(0xAC)
//                [2] : Data0(0x33)
//                [3] : Data1(0x00)
//
void sci_wr_sensor_cmd(void)
{
	
	sci_iic_sd_data[0] = ( sci_iic_slave_adrs  << 1 ) ;  // スレーブアドレスへ書き込み
	sci_iic_sd_data[1] = 0xac;
	sci_iic_sd_data[2] = 0x33;
	sci_iic_sd_data[3] = 0x00;
	 
	sci_iic_master_send (4);	// マスタ送信(割り込み使用)
	
	
}



// 
//  温湿度センサから得たデータより、
//  湿度と温度を計算する。
//    CRC異常の場合は、0とする。
//
void sci_cal_humidity_temperature(void)
{
	uint32_t dt;
	uint32_t dt_h;
	uint32_t dt_m;
	uint32_t dt_l;
	
	
	
	sci_crc_x8_x5_x4_1 = Calc_crc_x8_x5_x4_1(&sci_iic_rcv_data[0],6);   // CRC-8(X8+X5+X4+1)の計算
	sci_iic_crc_ng =  Calc_crc_x8_x5_x4_1(&sci_iic_rcv_data[0],7);     // 送信されたCRCを含めて計算
	
	
	if ( sci_iic_crc_ng == 0 ) { // CRCが一致した場合、温湿度の計算
	
		dt_h = sci_iic_rcv_data[1];		// 湿度データ(b19-b12)
		dt_h = dt_h << 12;
	
        	dt_m = sci_iic_rcv_data[2];		// 湿度データ(b11-b4)
		dt_m = dt_m << 4;
	
		dt_l = sci_iic_rcv_data[3];		// b7-b4: 湿度データ(b3-b0)
		dt_l = dt_l >> 4;
	
		dt = dt_h | dt_m | dt_l;
	
		dt =  dt * 1000;		
		dt = dt >> 10;			// 1/1024 = 1/(2^10)
		dt = dt >> 10;
		sci_iic_sensor_humidity = dt;     // 湿度データ (784ならば78.4%)
	
	
		dt_h = sci_iic_rcv_data[3] & 0x0f; // b3-b0: 温度データ(b19-b16)
		dt_h = dt_h << 16;
	
		dt_m = sci_iic_rcv_data[4];		// 温度データ(b15-b8)
		dt_m = dt_m << 8;
	
		dt_l = sci_iic_rcv_data[5];		// 温度データ(b7-b0)
	
		dt = dt_h | dt_m | dt_l;
	
		dt =  dt * 200 *10;		
		dt = dt >> 10;
		dt = dt >> 10;
		dt = dt - 500;
	
		sci_iic_sensor_temperature = dt;		// 温度データ (283ならば28.3℃)
	}
        else {
		sci_iic_sensor_humidity = 0;
		sci_iic_sensor_temperature = 0;
	}
	
}









//
//  放射温度計(サーモパイル)からのデータ読み出し (簡易I2C用)(割り込み使用)
//
//
// 入力: rd_obj= 0: センサの周囲温度(Self temperature)(TA)を読み出す
///            = 1: 測定対象対象物の温度(TO)を読み出す
//
//   IIC 送信バッファ
//   　sci_iic_sd_data[0] : スレーブアドレス(7bit) + Wrビット(0)
//     sci_iic_sd_data[1] : コマンド(読み出しアドレス)  06=周囲温度読み出し, 07=測定対象温度読み出し
//     sci_iic_sd_data[2] : スレーブアドレス(7bit) + Rdビット(1)
//
void sci_rd_thermo_pile( uint32_t rd_obj )
{
	
	
	if ( rd_obj == 0 ) {	// センサの周囲温度(TEMP_THT)を読み出す
		sci_iic_sd_data[1] = 0x70;
	
	}
	else {				// 測定対象対象物の温度(TEMP_THP)を読み出す
		sci_iic_sd_data[1] = 0x71;
		
	}
	
	
	sci_iic_sd_data[0] = ( sci_iic_slave_adrs << 1 );    // 書き込み用 スレーブアドレス
	
	sci_iic_sd_data[2] = ( sci_iic_sd_data[0] | 0x01);   // 読み出し用　スレーブアドレス 
	
	
	sci_iic_sd_rcv_fg = 1;			// マスタ送受信処理
	
	
	sci_iic_sd_start();		// SCI IIC 送信開始
	
		
	
	
		     
	
	
}




// SCI IIC マスタ受信
//   スレーブから、rcv_numバイト受信して、受信バッファ　iic_rcv_data[]へ格納する。
// 入力: rcv_num  受信バイト数
// 

void sci_iic_master_rcv ( uint8_t rcv_num)
{

	sci_iic_sd_num = 1;			// 送信データ数
	sci_iic_rcv_num = rcv_num;		// 受信データ数
	
	sci_iic_sd_rcv_fg = 0;		// マスタ送信またはマスタ受信
	sci_iic_sd_start();		// RIIC 送信開始
		
}


// CRC-8の計算 (AHT25用)
// CRC-8-Maxim: X8+X5+X4+1 (0x31) 初期値=0xff
//
// 下記サンプルプログラムより引用
// STM32 の AHT20 ルーチン (aht20_stm32 demo v1_4)」 (http://www.aosong.com/class-36.html)
// 
//
uint8_t Calc_crc_x8_x5_x4_1(volatile uint8_t *data, uint8_t num)
{
        uint8_t i;
        uint8_t pt;
        uint8_t crc;
	
	crc = 0xff;

	for ( pt = 0; pt < num; pt++ ) {
  
         crc ^= data[pt];
    
	 for ( i = 8 ;i >0 ; --i)  {
    
           if ( crc & 0x80 ) {
               crc = ( crc << 1 ) ^ 0x31;
	   }
           else{
	       crc = ( crc << 1 );
	   }
	 }
       }
 
       return crc;
}


// PECによるデータチェック (サーモパイル用)
// CRC-8-ATM: X8+X2+X1+1 (0x07) 初期値=0x00
//
// 送受信データから、CPUのCRC演算器を使用して求める。
// 例:
//　　iic_sd_data[0] = 0x7a;   (スレーブアドレス=3D + R/W#(Write=0))
//    iic_sd_data[1] = 0x71;   (コマンド Object temperature read)
//    iic_sd_data[2] = 0x7b;   (スレーブアドレス=3D + R/W#(Read=1))
//
//    iic_rcv_data[0] = 0xdd;  (対象物の温度 下位バイト)
//    iic_rcv_data[1] = 0x01;  (対象物の温度 上位バイト)
//    iic_rcv_data[1] = 0xb8;  PEC(Packet error code)
//
//   全データ(0x7a,0x71,0x7b,0xdd,0x01,0xb8)を、CRC.CRCDIRに入れる。
//   CRC.CRCDOR = 0であれば、データに誤り無し。
// 
// 参考: 「RX23E-Aグループ ユーザーズマニュアル　ハードウェア編 (R01UH0801JJ0120 Rev.1.20)」
//　　　　32.2.3 CRC データ出力レジスタ（CRCDOR）
//    
void sci_iic_cal_crc_thermo_pile(void)
{
	uint32_t i;
	
	CRC.CRCCR.BYTE = 0x85;		     // CRCDORレジスタをクリア, MSBファースト通信用にCRCを生成, 8ビットCRC（X8 + X2 + X + 1）

	for ( i = 0 ; i < 3 ; i++ ) {	     // CRC-8の計算(送信データ)
	   CRC.CRCDIR = sci_iic_sd_data[i];
	}
	
	CRC.CRCDIR = sci_iic_rcv_data[0];	    // CRC-8の計算(受信データ)
	CRC.CRCDIR = sci_iic_rcv_data[1];
		     
	sci_smbus_crc_8 = CRC.CRCDOR;	   // CRC計算結果(PEC)
 
	CRC.CRCDIR = sci_iic_rcv_data[2];     // PEC　
	       
	sci_smbus_crc_ng = CRC.CRCDOR;        // 受信したPECまでCRC計算。0ならばデータ正常
}


// 
//  放射温度計(サーモパイル)から得たデータより、
//  Self temperatureとObject temperatureを計算する。
//    CRC異常の場合は、0とする。
//
void sci_iic_cal_Ta_To_temperature(void)
{
	
	if ( sci_smbus_crc_ng == 0 ) {   // CRC 正常の場合
						// 温度の計算
		if( sci_iic_sd_data[1] == 0x70 ) {		// TEMP_THT(Self temperature)の読み出の場合
		    sci_ta_word =  sci_iic_rcv_data[1];
		    sci_ta_word =  ( sci_ta_word << 8 );
		    sci_ta_word =  (sci_ta_word | sci_iic_rcv_data[0]);
		    sci_ta_celsius = ( sci_ta_word * 0.125) - 20.0;  
		    
	         }
	         else if ( sci_iic_sd_data[1] == 0x71 ){	// TEMP_THP(Object temperature)の読み出の場合
	            sci_to_word =  sci_iic_rcv_data[1];
		    sci_to_word =  ( sci_to_word << 8 );
		    sci_to_word =  (sci_to_word | sci_iic_rcv_data[0]);
		    sci_to_celsius = ( sci_to_word * 0.125) - 30.0;  
		 }
	 } 
	 else{					 // CRC異常
		 sci_ta_celsius = 0.0;
		 sci_to_celsius = 0.0;
	 }

}





// SCI IIC マスタ送信
//   スレーブへ　送信バッファ　iic_sd_data[]のデータを sd_numバイト送信する。
// 入力: sd_num  送信バイト数　
// 
//   
void sci_iic_master_send ( uint8_t sd_num)
{
	sci_iic_sd_num = sd_num;	// 送信データ数
	sci_iic_rcv_num = 0;		// 受信データ数
	
	sci_iic_sd_rcv_fg = 0;		// マスタ送信またはマスタ受信
	
	sci_iic_sd_start();		// SCI IIC 送信開始
}



//  SCI IIC 送信開始
void sci_iic_sd_start(void)
{
	sci_iic_sd_pt = 0;			     // 送信データ位置　クリア
	sci_iic_rcv_pt = 0;                          // 受信データ位置
	sci_dummy_send_fg = 0;			// ダミー送信フラグのクリア

	sci_iic_com_over_fg = 0;		    // 通信完了フラグのクリア
	
	SCI12.SIMR2.BIT.IICACKT = 1;   // NACK送信またはACK/NACK受信 (マスタ送信時には、スレーブからの ACK/NACK受信)
		
	SCI12.SCR.BIT.TEIE = 1;		// STI(TEI)割り込み要求の許可
	SCI12.SCR.BIT.TIE = 1;		// 送信データエンプティ割り込み(TXI)要求の許可	
	SCI12.SCR.BIT.RIE = 0;		// 受信データ割り込み(RXI)要求の禁止
	
	SCI12.SIMR3.BIT.IICSTIF = 0;     // IICSTIFフラグを“0”にしてから、各条件生成を行う。
	SCI12.SIMR3.BYTE = 0x51;	// 開始条件(スタートコンディション)の生成 (開始条件　生成終了でSTI割り込み発生)
	
}




// 
// SCI12 初期設定 （簡易IIC通信) 
// 
//ポート: 用途     
//  PB0: SSCL12  
//  PB1: SSDA12  
//
// ・ボーレート
// クロックセレクトビット=0 (PCKLB=32MHz)　
//  SCI12.SMR.BYTE = 0;		// クロック = PCLKB　= 32/1 = 32[MHz] (n=0)
//
//  Nの計算式:	(B=ボーレート bps)			
//    N= (32 x 1000000/(64/2)xB)-1
//     = (32 x 1000000/(32xB)) - 1
//　　
//   B=100Kbps とすると、N= ( 32x 1000000 /(32x100000)) - 1 = 9
//    ( SMBbusは Max 100 KHz)
//    (資料の  28.2.11 ビットレートレジスタ(BRR) より)
//     ( 資料:「 RX23E-Aグループ ユーザーズマニュアル　ハードウェア編」 (R01UH0801JJ0120 Rev.1.20)） 
//
// ・送受信のビットの順
//   MSBファーストで送受信 (b7から送信、受信)
//  SCI12.SCMR.BIT.SDIR = 1 : MSBファーストで送受信
//
void initSCI_12(void)
{
	SCI12.SCR.BYTE = 0;	// 送受信禁止 
	
	MPC.PWPR.BIT.B0WI = 0;   // マルチファンクションピンコントローラ　プロテクト解除
	MPC.PWPR.BIT.PFSWE = 1;  // PmnPFS ライトプロテクト解除
	
	MPC.PB0PFS.BYTE = 0x0C;  // PB0 = SSCL12
	MPC.PB1PFS.BYTE = 0x0C;  // PB1 = SSDA12
	
	MPC.PWPR.BYTE = 0x80;    //  PmnPFS ライトプロテクト 設定
	
	PORTB.PMR.BIT.B0 = 1;   // PB0  周辺機能として使用
	PORTB.PMR.BIT.B1 = 1;   // PB1   :
	
	PORTB.ODR0.BIT.B0 = 1;	// PB0 Nチャネルオープンドレイン出力
	
	PORTB.ODR0.BIT.B2 = 1;	// PB1 Nチャネルオープンドレイン出力
	PORTB.ODR0.BIT.B3 = 0;
	
	SCI12.SIMR3.BIT.IICSDAS = 3 ;  // SSDAn端子はハイインピーダンス状態
	SCI12.SIMR3.BIT.IICSCLS = 3 ;  // SSCLn端子はハイインピーダンス状態
	
	SCI12.SMR.BYTE = 0;		// 簡易I2Cモード, クロック=PCLKB= 32[MHz] (n=0)
	
	SCI12.SCMR.BIT.SDIR = 1;	// MSBファーストで送受信
	SCI12.SCMR.BIT.SINV = 0;	// 送受信データインバート無し
	SCI12.SCMR.BIT.SMIF = 0;	// 非スマートカードインタフェースモード
	
	SCI12.BRR = 9;	 		// 100Kbps  
	
	SCI12.SEMR.BIT.BRME = 0;	// ビットレートモジュレーション機能無効
	SCI12.SEMR.BIT.NFEN = 0;	// SSCLn、SSDAn入力信号のノイズ除去機能無効
	
	SCI12.SNFR.BIT.NFCS = 1;	// 1分周のクロックをノイズフィルタに使用
	
	SCI12.SIMR1.BIT.IICM = 1;	// 簡易I2Cモード
	SCI12.SIMR1.BIT.IICDL = 0x01;	// SSDA出力遅延 0～1サイクル
	
	
	SCI12.SIMR2.BIT.IICINTM = 1;   // 簡易I2C モード時の割り込み要求の要因:受信割り込み、送信割り込みを使用 
	SCI12.SIMR2.BIT.IICACKT = 1;   // NACK送信またはACK/NACK受信 (マスタ送信時には、スレーブからの ACK/NACK受信)
	SCI12.SIMR2.BIT.IICCSC = 1;    // クロック同期を行う
	
	SCI12.SPMR.BYTE = 0;		// SPIモードレジスタのクリア
		
	IPR(SCI12,RXI12) = 5;		// 受信 割り込みレベル = 5（15が最高レベル)
	IR(SCI12,RXI12) = 0;	        // 割り込み要求のクリア
	IEN(SCI12,RXI12) = 1;		// 受信割り込み許可
	
	IPR(SCI12,TXI12) = 5;		// 送信 割り込みレベル = 5 （15が最高レベル)  
        IR(SCI12,TXI12) = 0;		//  割り込み要求のクリア
	IEN(SCI12,TXI12) = 1;		// 送信割り込み許可
	
	IPR(SCI12,TEI12) = 5;		//  STI割り込み(送信完了TEI) 割り込みレベル = 5 （15が最高レベル)
	IR(SCI12,TEI12) = 0;	        //  割り込み要求のクリア
	IEN(SCI12,TEI12) = 1;		//  STI(送信完了)割り込み許可
	

	SCI12.SCR.BYTE = 0x30;		// 送信動作(TE),受信動作(RE)を許可。STI(TEI)割り込み要求, RXI割り込み要求, TXI割り込み要求 は禁止

	
	

}

