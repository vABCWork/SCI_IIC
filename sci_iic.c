#include "iodefine.h"
#include "misratypes.h"
#include "sci_iic.h"



uint8_t sci_iic_slave_adrs;  // �Ȉ�I2C �X���[�u�A�h���X  00: 7bit�A�h���X( ��:100 0000 = 0x40 )

volatile uint8_t sci_iic_rcv_data[16];   // IIC��M�f�[�^
volatile uint8_t sci_iic_sd_data[16];    // ���M�f�[�^
volatile uint8_t sci_iic_sd_pt;	    // ���M�f�[�^�ʒu
volatile uint8_t sci_iic_rcv_pt;         // ��M�f�[�^�ʒu

volatile uint8_t  sci_dummy_send_fg;    // �_�~�[�f�[�^(0xff)�̑��M�ɂ���M�f�[�^�𓾂邽�߂̃t���O

volatile uint8_t  sci_iic_sd_rcv_fg;	    // 0:���M�݂̂܂��͎�M�݂̂̏ꍇ,  1:�}�X�^����M�̏ꍇ(= ���X�^�[�g������ꍇ)
volatile uint8_t  sci_iic_sd_num;	    // ���M�f�[�^��(�X���[�u�A�h���X���܂�)
volatile uint8_t  sci_iic_rcv_num;      // ��M�f�[�^��
volatile uint8_t  sci_iic_com_over_fg;  // 1:STOP�R���f�B�V�����̌��o��


uint8_t sci_smbus_crc_8;	        //  CRC���Z�Ώۃf�[�^: �ŏ��̃X���[�u�A�h���X����A�R�}���h�A�X���[�u�A�h���X(Read�p)�A��M�f�[�^(low���j�A��M�f�[�^(high��)��5�o�C�g��
uint8_t sci_smbus_crc_ng;          // ��L��5byte�ɁA�X���[�u�����瑗�M���ꂽ�APEC�����āACRC�v�Z�����l��0�Ȃ�΁A�ُ�Ȃ��B
			           // ( 32.2.3 CRC �f�[�^�o�̓��W�X�^�iCRCDOR�j�uRX23E-A�O���[�v ���[�U�[�Y�}�j���A���@�n�[�h�E�F�A�ҁv (R01UH0801JJ0120 Rev.1.20) )  
			
		
uint8_t sci_iic_sensor_status;	   // �Z���T�̃X�e�[�^�X
uint32_t sci_iic_sensor_humidity;	   // �Z���T����̎��x�f�[�^
uint32_t sci_iic_sensor_temperature;  // �Z���T����̉��x�f�[�^
					

uint8_t  sci_crc_x8_x5_x4_1;	// �����x�Z���T�p�@CRC-8 (x8 + x5 + X4 + 1)
uint8_t  sci_iic_crc_ng;


uint16_t sci_ta_word;
float  sci_ta_celsius;		  // Self temperature (�Z���T�̎��͉��x Ta)[��]

uint16_t sci_to_word;
float  sci_to_celsius;		// Object temperature (����Ώە��̉��x To)[��]


//
// ��M���荞��(SCI12)
//   1�o�C�g�̎�M�������������_�Ŕ�������
// 
#pragma interrupt (Excep_SCI12_RXI12(vect=239))
void Excep_SCI12_RXI12(void)
{
	  sci_iic_rcv_data[sci_iic_rcv_pt] =  SCI12.RDR;	// ��M�f�[�^�ǂݏo��
	  
	  sci_iic_rcv_pt++;
	  
}


//
// ���M���荞��(TXI12)
//  1�o�C�g�̑��M���������AACK�܂���NACK���o��ɔ���
//
#pragma interrupt (Excep_SCI12_TXI12(vect=240))

void Excep_SCI12_TXI12(void)
{
	if ( SCI12.SISR.BIT.IICACKR == 0 ) {	// ACK�F��
          
	  if ( sci_iic_sd_rcv_fg == 1 ) { 	// �}�X�^����M�̏ꍇ
	   if( sci_dummy_send_fg == 0 ) {    // �_�~�[���M�łȂ��ꍇ
	     if ( sci_iic_sd_pt == 1 ) {    //  ( �X���[�u�A�h���X + W�r�b�g)���M�����̏ꍇ
	
		SCI12.TDR = sci_iic_sd_data[sci_iic_sd_pt];     // ���M�f�[�^�����C�g�@(�R�}���h(�ǂݏo���A�h���X)�̑��M)
	     	sci_iic_sd_pt++;				// ���M�f�[�^�̊i�[�ꏊ�̃C���N�������g
	     }
	     
	     else if ( sci_iic_sd_pt == 2 ) {	 // �R�}���h(�ǂݏo���A�h���X)���M�����̏ꍇ�A�ĊJ�n����(���X�^�[�g�R���f�B�V����)�̔��s
		
	        SCI12.SIMR3.BIT.IICSTIF = 0;     // IICSTIF�t���O���g0�h�ɂ��Ă���A�e�����������s���B
	        SCI12.SIMR3.BYTE = 0x52;	// �ĊJ�n����(���X�^�[�g�R���f�B�V����)�̐��� (STI���荞�ݔ���)
	     }
	     
	     else if ( sci_iic_sd_pt == 3 ) {   // ( �X���[�u�A�h���X + R�r�b�g)���M�����̏ꍇ�A�@
		SCI12.SIMR2.BIT.IICACKT = 0;    // �ȍ~��ACK���M (�}�X�^��M���ɂ́A�X���[�u�ւ� ACK���M)
	        SCI12.SCR.BIT.RIE = 1;		// ��M�f�[�^���荞��(RXI)�v���̋���	
	     
		sci_dummy_send_fg = 1;		// �_�~�[���M�t���O�̃Z�b�g
		SCI12.TDR = 0xff;		// �_�~�[�f�[�^�̑��M
	     }
	   
	   }
	   else {				// �_�~�[���M�̏ꍇ
	      if ( sci_iic_rcv_pt < 3 ) {       // ��M����f�[�^���c���Ă���ꍇ
	          SCI12.TDR = 0xff;		// �ēx�A�_�~�[�f�[�^�̑��M
	      }
	      else {				// �S�Ď�M�ς݂̏ꍇ�A��~����(�X�g�b�v�R���f�B�V�������s)
	        sci_dummy_send_fg = 0;		// �_�~�[���M�t���O�̃N���A
	        
	        SCI12.SIMR3.BIT.IICSTIF = 0;     // IICSTIF�t���O���g0�h�ɂ��Ă���A�e�����������s���B
	        SCI12.SIMR3.BYTE = 0x54;	 // ��~����(�X�g�b�v�R���f�B�V����)�̐��� (STI���荞�ݔ���)
	  
	      } // �S�Ď�M�ς݂�
	   }  // �_�~�[���M
	  }   // �}�X�^����M�̏ꍇ
	  
	  else {						// �}�X�^���M�A�}�X�^��M�̏ꍇ
	  	
		if ( (sci_iic_sd_data[0] & 0x01) == 1 ) {  // �}�X�^��M�̏ꍇ
		
		   if( sci_dummy_send_fg == 0 ) {    // �_�~�[���M�łȂ��ꍇ(�X���[�u�A�h���X��R�r�b�g ���M��)
		   
	        	SCI12.SCR.BIT.RIE = 1;		// ��M�f�[�^���荞��(RXI)�v���̋���	
	     
		        if ( sci_iic_rcv_num == 1 ) {   // ��M�f�[�^��1�o�C�g�̏ꍇ
			    
		           SCI12.SIMR2.BIT.IICACKT = 1;   // NACK���M�̏���
			}
			else {
			   SCI12.SIMR2.BIT.IICACKT = 0;   // ACK���M�̏���
			}
		        sci_dummy_send_fg = 1;		// �_�~�[���M�t���O�̃Z�b�g
			SCI12.TDR = 0xff;		// �_�~�[�f�[�^�̑��M
		   }
		   else {				// �_�~�[���M���̏ꍇ
		   
		        if ( sci_iic_rcv_pt == (sci_iic_rcv_num - 1) ) {   //�@���̎�M���ŏI�f�[�^�̏ꍇ
			   	SCI12.SIMR2.BIT.IICACKT = 1;   // NACK���M�̏���
			 }
			 
		         SCI12.TDR = 0xff;		// �ēx�A�_�~�[�f�[�^�̑��M
	            
		   }
		}     // �}�X�^��M
		
		else{					// �}�X�^���M�̏ꍇ
		  if (  sci_iic_sd_pt < sci_iic_sd_num ) {
		 	SCI12.TDR = sci_iic_sd_data[sci_iic_sd_pt];     // ���M�f�[�^�����C�g�@(�R�}���h(�ǂݏo���A�h���X)�̑��M)
	     		sci_iic_sd_pt++;				// ���M�f�[�^�̊i�[�ꏊ�̃C���N�������g
		  }
		  else {			       // �S�f�[�^�̑��M���� 
	              SCI12.SIMR3.BIT.IICSTIF = 0;     // IICSTIF�t���O���g0�h�ɂ��Ă���A�e�����������s���B
	              SCI12.SIMR3.BYTE = 0x54;	       // ��~����(�X�g�b�v�R���f�B�V����)�̐��� (STI���荞�ݔ���)
		  }
		}  // �}�X�^���M
		
	  }   //  �}�X�^���M�A�}�X�^��M�̏ꍇ
	  
	}     // ACK��M
	
	else {					// NACK�F�� (�ŏI�f�[�^��M��NACK�����̏ꍇ���܂�)
	        sci_dummy_send_fg = 0;		// �_�~�[���M�t���O�̃N���A
	 	SCI12.SIMR3.BIT.IICSTIF = 0;     // IICSTIF�t���O���g0�h�ɂ��Ă���A�e�����������s���B
	        SCI12.SIMR3.BYTE = 0x54;	    // ��~����(�X�g�b�v�R���f�B�V����)�̐��� (STI���荞�ݔ���)
	}
	
}

//
// STI���荞��(���M�I�����荞��(TEI) (SCI12)
// �J�n�����܂��́A�ĊJ�n�����܂��́A��~�����̔��s�����ɂ�芄�荞�ݔ���

#pragma interrupt (Excep_SCI12_TEI12(vect=241))
void Excep_SCI12_TEI12(void){
	
	if ( sci_iic_sd_pt == 0) {      // �J�n����(�X�^�[�g�R���f�B�V����)���s����
             
	     SCI12.SIMR3.BIT.IICSTIF = 0;  
	     SCI12.SIMR3.BYTE = 0;	// SSDA=�V���A���f�[�^�o��, SSCL=�V���A���N���b�N�o��
	     
	     SCI12.TDR = sci_iic_sd_data[sci_iic_sd_pt];     // ���M�f�[�^�����C�g
	     sci_iic_sd_pt++;				     // ���M�f�[�^�̊i�[�ꏊ�C���N�������g
	     
	     return;
	}
	
	if ( sci_iic_sd_rcv_fg == 1 ) { 	// �}�X�^����M�̏ꍇ
	
	    if (  sci_iic_sd_pt == 2 ) {	 // �ĊJ�n����(���X�^�[�g�R���f�B�V����)�̔��s����
	
	       SCI12.SIMR3.BIT.IICSTIF = 0;  
	       SCI12.SIMR3.BYTE = 0;	// SSDA=�V���A���f�[�^�o��, SSCL=�V���A���N���b�N�o��
	     
	       SCI12.TDR = sci_iic_sd_data[sci_iic_sd_pt];     // ���M�f�[�^�����C�g �@( �X���[�u�A�h���X + R�r�b�g)
	       sci_iic_sd_pt++;				     // ���M�f�[�^�̊i�[�ꏊ�C���N�������g
	    }
	
	    else if ( sci_iic_rcv_pt == 3 ) {	// ��~����(�X�g�b�v�R���f�B�V����)�̔��s����
	       SCI12.SIMR3.BIT.IICSTIF = 0; 
	       SCI12.SIMR3.BYTE = 0xf0;	// SSDA,SSCL�̓n�C�C���s�[�_���X��Ԃɂ���
	    
	       sci_iic_com_over_fg = 1;   // �ʐM����
	    }
	}
	else {					// �}�X�^���M�A�}�X�^��M�ŁASTOP�R���f�B�V��������
		SCI12.SIMR3.BIT.IICSTIF = 0; 
	        SCI12.SIMR3.BYTE = 0xf0;		// SSDA,SSCL�̓n�C�C���s�[�_���X��Ԃɂ���
	    
	        sci_iic_com_over_fg = 1;   // �ʐM����
	     
	}
	
}





//  �����x�Z���T�̃X�e�[�^�X�ǂݏo�� (�}�X�^��M)(���荞�ݎg�p)
// IIC ���M�o�b�t�@
//   �@sci_iic_sd_data[0] : �X���[�u�A�h���X(7bit) + 1(R/W#�r�b�g=Read)
//   ��M�o�b�t�@
//     sci_iic_rcv_data[0]: �X�e�[�^�X

void sci_rd_sensor_status(void)
{
	sci_iic_sd_data[0] = (( sci_iic_slave_adrs << 1 ) | 0x01 ) ;  // �X���[�u�A�h���X����Ǐo��
	
	sci_iic_master_rcv (1);		//�@�}�X�^��M�@�J�n
	
	while( sci_iic_com_over_fg != 1 ) {		// �ʐM�����҂�(��M�����҂�)
	}
	
	sci_iic_sensor_status = sci_iic_rcv_data[0];  // �Z���T�̃X�e�[�^�X
	
}




//  �����x�Z���T����X�e�[�^�X�Ɖ����x�f�[�^�̓ǂݏo�� (�}�X�^��M)(���荞�ݎg�p)
// IIC ���M�o�b�t�@
//   sci_iic_sd_data[0] : �X���[�u�A�h���X(7bit) + 1(=Read)
// IIC ��M�o�b�t�@
//   sci_iic_rcv_data[0]: �X�e�[�^�X
//             :   [1]: ���x�f�[�^(b19-b12)
//             :   [2]: ���x�f�[�^(b11-b4)
//             :   [3]��b7-b4: ���x�f�[�^(b3-b0)
//             :   [3]��b3-b0: ���x�f�[�^(b19-b16)
//             :   [4]: ���x�f�[�^(b15-b8)
//             :   [5]: ���x�f�[�^(b7-b0)
//             :   [6]: CRC 
void sci_rd_sensor_humi_temp(void)
{
	
	sci_iic_sd_data[0] = (( sci_iic_slave_adrs << 1 ) | 0x01 ) ;  // �X���[�u�A�h���X����Ǐo��

	sci_iic_master_rcv (7);		//�@�}�X�^��M�@�J�n
	
	 
}




// �����x�Z���T�ւ̑���J�n�R�}���h���M�@(�}�X�^���M)(���荞�ݎg�p)
//	 IIC ���M�o�b�t�@
//   sci_iic_sd_data[0] : �X���[�u�A�h���X(7bit) + 0(=wite)
//                [1] : Trigger measure(0xAC)
//                [2] : Data0(0x33)
//                [3] : Data1(0x00)
//
void sci_wr_sensor_cmd(void)
{
	
	sci_iic_sd_data[0] = ( sci_iic_slave_adrs  << 1 ) ;  // �X���[�u�A�h���X�֏�������
	sci_iic_sd_data[1] = 0xac;
	sci_iic_sd_data[2] = 0x33;
	sci_iic_sd_data[3] = 0x00;
	 
	sci_iic_master_send (4);	// �}�X�^���M(���荞�ݎg�p)
	
	
}



// 
//  �����x�Z���T���瓾���f�[�^���A
//  ���x�Ɖ��x���v�Z����B
//    CRC�ُ�̏ꍇ�́A0�Ƃ���B
//
void sci_cal_humidity_temperature(void)
{
	uint32_t dt;
	uint32_t dt_h;
	uint32_t dt_m;
	uint32_t dt_l;
	
	
	
	sci_crc_x8_x5_x4_1 = Calc_crc_x8_x5_x4_1(&sci_iic_rcv_data[0],6);   // CRC-8(X8+X5+X4+1)�̌v�Z
	sci_iic_crc_ng =  Calc_crc_x8_x5_x4_1(&sci_iic_rcv_data[0],7);     // ���M���ꂽCRC���܂߂Čv�Z
	
	
	if ( sci_iic_crc_ng == 0 ) { // CRC����v�����ꍇ�A�����x�̌v�Z
	
		dt_h = sci_iic_rcv_data[1];		// ���x�f�[�^(b19-b12)
		dt_h = dt_h << 12;
	
        	dt_m = sci_iic_rcv_data[2];		// ���x�f�[�^(b11-b4)
		dt_m = dt_m << 4;
	
		dt_l = sci_iic_rcv_data[3];		// b7-b4: ���x�f�[�^(b3-b0)
		dt_l = dt_l >> 4;
	
		dt = dt_h | dt_m | dt_l;
	
		dt =  dt * 1000;		
		dt = dt >> 10;			// 1/1024 = 1/(2^10)
		dt = dt >> 10;
		sci_iic_sensor_humidity = dt;     // ���x�f�[�^ (784�Ȃ��78.4%)
	
	
		dt_h = sci_iic_rcv_data[3] & 0x0f; // b3-b0: ���x�f�[�^(b19-b16)
		dt_h = dt_h << 16;
	
		dt_m = sci_iic_rcv_data[4];		// ���x�f�[�^(b15-b8)
		dt_m = dt_m << 8;
	
		dt_l = sci_iic_rcv_data[5];		// ���x�f�[�^(b7-b0)
	
		dt = dt_h | dt_m | dt_l;
	
		dt =  dt * 200 *10;		
		dt = dt >> 10;
		dt = dt >> 10;
		dt = dt - 500;
	
		sci_iic_sensor_temperature = dt;		// ���x�f�[�^ (283�Ȃ��28.3��)
	}
        else {
		sci_iic_sensor_humidity = 0;
		sci_iic_sensor_temperature = 0;
	}
	
}









//
//  ���ˉ��x�v(�T�[���p�C��)����̃f�[�^�ǂݏo�� (�Ȉ�I2C�p)(���荞�ݎg�p)
//
//
// ����: rd_obj= 0: �Z���T�̎��͉��x(Self temperature)(TA)��ǂݏo��
///            = 1: ����ΏۑΏە��̉��x(TO)��ǂݏo��
//
//   IIC ���M�o�b�t�@
//   �@sci_iic_sd_data[0] : �X���[�u�A�h���X(7bit) + Wr�r�b�g(0)
//     sci_iic_sd_data[1] : �R�}���h(�ǂݏo���A�h���X)  06=���͉��x�ǂݏo��, 07=����Ώۉ��x�ǂݏo��
//     sci_iic_sd_data[2] : �X���[�u�A�h���X(7bit) + Rd�r�b�g(1)
//
void sci_rd_thermo_pile( uint32_t rd_obj )
{
	
	
	if ( rd_obj == 0 ) {	// �Z���T�̎��͉��x(TEMP_THT)��ǂݏo��
		sci_iic_sd_data[1] = 0x70;
	
	}
	else {				// ����ΏۑΏە��̉��x(TEMP_THP)��ǂݏo��
		sci_iic_sd_data[1] = 0x71;
		
	}
	
	
	sci_iic_sd_data[0] = ( sci_iic_slave_adrs << 1 );    // �������ݗp �X���[�u�A�h���X
	
	sci_iic_sd_data[2] = ( sci_iic_sd_data[0] | 0x01);   // �ǂݏo���p�@�X���[�u�A�h���X 
	
	
	sci_iic_sd_rcv_fg = 1;			// �}�X�^����M����
	
	
	sci_iic_sd_start();		// SCI IIC ���M�J�n
	
		
	
	
		     
	
	
}




// SCI IIC �}�X�^��M
//   �X���[�u����Arcv_num�o�C�g��M���āA��M�o�b�t�@�@iic_rcv_data[]�֊i�[����B
// ����: rcv_num  ��M�o�C�g��
// 

void sci_iic_master_rcv ( uint8_t rcv_num)
{

	sci_iic_sd_num = 1;			// ���M�f�[�^��
	sci_iic_rcv_num = rcv_num;		// ��M�f�[�^��
	
	sci_iic_sd_rcv_fg = 0;		// �}�X�^���M�܂��̓}�X�^��M
	sci_iic_sd_start();		// RIIC ���M�J�n
		
}


// CRC-8�̌v�Z (AHT25�p)
// CRC-8-Maxim: X8+X5+X4+1 (0x31) �����l=0xff
//
// ���L�T���v���v���O���������p
// STM32 �� AHT20 ���[�`�� (aht20_stm32 demo v1_4)�v (http://www.aosong.com/class-36.html)
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


// PEC�ɂ��f�[�^�`�F�b�N (�T�[���p�C���p)
// CRC-8-ATM: X8+X2+X1+1 (0x07) �����l=0x00
//
// ����M�f�[�^����ACPU��CRC���Z����g�p���ċ��߂�B
// ��:
//�@�@iic_sd_data[0] = 0x7a;   (�X���[�u�A�h���X=3D + R/W#(Write=0))
//    iic_sd_data[1] = 0x71;   (�R�}���h Object temperature read)
//    iic_sd_data[2] = 0x7b;   (�X���[�u�A�h���X=3D + R/W#(Read=1))
//
//    iic_rcv_data[0] = 0xdd;  (�Ώە��̉��x ���ʃo�C�g)
//    iic_rcv_data[1] = 0x01;  (�Ώە��̉��x ��ʃo�C�g)
//    iic_rcv_data[1] = 0xb8;  PEC(Packet error code)
//
//   �S�f�[�^(0x7a,0x71,0x7b,0xdd,0x01,0xb8)���ACRC.CRCDIR�ɓ����B
//   CRC.CRCDOR = 0�ł���΁A�f�[�^�Ɍ�薳���B
// 
// �Q�l: �uRX23E-A�O���[�v ���[�U�[�Y�}�j���A���@�n�[�h�E�F�A�� (R01UH0801JJ0120 Rev.1.20)�v
//�@�@�@�@32.2.3 CRC �f�[�^�o�̓��W�X�^�iCRCDOR�j
//    
void sci_iic_cal_crc_thermo_pile(void)
{
	uint32_t i;
	
	CRC.CRCCR.BYTE = 0x85;		     // CRCDOR���W�X�^���N���A, MSB�t�@�[�X�g�ʐM�p��CRC�𐶐�, 8�r�b�gCRC�iX8 + X2 + X + 1�j

	for ( i = 0 ; i < 3 ; i++ ) {	     // CRC-8�̌v�Z(���M�f�[�^)
	   CRC.CRCDIR = sci_iic_sd_data[i];
	}
	
	CRC.CRCDIR = sci_iic_rcv_data[0];	    // CRC-8�̌v�Z(��M�f�[�^)
	CRC.CRCDIR = sci_iic_rcv_data[1];
		     
	sci_smbus_crc_8 = CRC.CRCDOR;	   // CRC�v�Z����(PEC)
 
	CRC.CRCDIR = sci_iic_rcv_data[2];     // PEC�@
	       
	sci_smbus_crc_ng = CRC.CRCDOR;        // ��M����PEC�܂�CRC�v�Z�B0�Ȃ�΃f�[�^����
}


// 
//  ���ˉ��x�v(�T�[���p�C��)���瓾���f�[�^���A
//  Self temperature��Object temperature���v�Z����B
//    CRC�ُ�̏ꍇ�́A0�Ƃ���B
//
void sci_iic_cal_Ta_To_temperature(void)
{
	
	if ( sci_smbus_crc_ng == 0 ) {   // CRC ����̏ꍇ
						// ���x�̌v�Z
		if( sci_iic_sd_data[1] == 0x70 ) {		// TEMP_THT(Self temperature)�̓ǂݏo�̏ꍇ
		    sci_ta_word =  sci_iic_rcv_data[1];
		    sci_ta_word =  ( sci_ta_word << 8 );
		    sci_ta_word =  (sci_ta_word | sci_iic_rcv_data[0]);
		    sci_ta_celsius = ( sci_ta_word * 0.125) - 20.0;  
		    
	         }
	         else if ( sci_iic_sd_data[1] == 0x71 ){	// TEMP_THP(Object temperature)�̓ǂݏo�̏ꍇ
	            sci_to_word =  sci_iic_rcv_data[1];
		    sci_to_word =  ( sci_to_word << 8 );
		    sci_to_word =  (sci_to_word | sci_iic_rcv_data[0]);
		    sci_to_celsius = ( sci_to_word * 0.125) - 30.0;  
		 }
	 } 
	 else{					 // CRC�ُ�
		 sci_ta_celsius = 0.0;
		 sci_to_celsius = 0.0;
	 }

}





// SCI IIC �}�X�^���M
//   �X���[�u�ց@���M�o�b�t�@�@iic_sd_data[]�̃f�[�^�� sd_num�o�C�g���M����B
// ����: sd_num  ���M�o�C�g���@
// 
//   
void sci_iic_master_send ( uint8_t sd_num)
{
	sci_iic_sd_num = sd_num;	// ���M�f�[�^��
	sci_iic_rcv_num = 0;		// ��M�f�[�^��
	
	sci_iic_sd_rcv_fg = 0;		// �}�X�^���M�܂��̓}�X�^��M
	
	sci_iic_sd_start();		// SCI IIC ���M�J�n
}



//  SCI IIC ���M�J�n
void sci_iic_sd_start(void)
{
	sci_iic_sd_pt = 0;			     // ���M�f�[�^�ʒu�@�N���A
	sci_iic_rcv_pt = 0;                          // ��M�f�[�^�ʒu
	sci_dummy_send_fg = 0;			// �_�~�[���M�t���O�̃N���A

	sci_iic_com_over_fg = 0;		    // �ʐM�����t���O�̃N���A
	
	SCI12.SIMR2.BIT.IICACKT = 1;   // NACK���M�܂���ACK/NACK��M (�}�X�^���M���ɂ́A�X���[�u����� ACK/NACK��M)
		
	SCI12.SCR.BIT.TEIE = 1;		// STI(TEI)���荞�ݗv���̋���
	SCI12.SCR.BIT.TIE = 1;		// ���M�f�[�^�G���v�e�B���荞��(TXI)�v���̋���	
	SCI12.SCR.BIT.RIE = 0;		// ��M�f�[�^���荞��(RXI)�v���̋֎~
	
	SCI12.SIMR3.BIT.IICSTIF = 0;     // IICSTIF�t���O���g0�h�ɂ��Ă���A�e�����������s���B
	SCI12.SIMR3.BYTE = 0x51;	// �J�n����(�X�^�[�g�R���f�B�V����)�̐��� (�J�n�����@�����I����STI���荞�ݔ���)
	
}




// 
// SCI12 �����ݒ� �i�Ȉ�IIC�ʐM) 
// 
//�|�[�g: �p�r     
//  PB0: SSCL12  
//  PB1: SSDA12  
//
// �E�{�[���[�g
// �N���b�N�Z���N�g�r�b�g=0 (PCKLB=32MHz)�@
//  SCI12.SMR.BYTE = 0;		// �N���b�N = PCLKB�@= 32/1 = 32[MHz] (n=0)
//
//  N�̌v�Z��:	(B=�{�[���[�g bps)			
//    N= (32 x 1000000/(64/2)xB)-1
//     = (32 x 1000000/(32xB)) - 1
//�@�@
//   B=100Kbps �Ƃ���ƁAN= ( 32x 1000000 /(32x100000)) - 1 = 9
//    ( SMBbus�� Max 100 KHz)
//    (������  28.2.11 �r�b�g���[�g���W�X�^(BRR) ���)
//     ( ����:�u RX23E-A�O���[�v ���[�U�[�Y�}�j���A���@�n�[�h�E�F�A�ҁv (R01UH0801JJ0120 Rev.1.20)�j 
//
// �E����M�̃r�b�g�̏�
//   MSB�t�@�[�X�g�ő���M (b7���瑗�M�A��M)
//  SCI12.SCMR.BIT.SDIR = 1 : MSB�t�@�[�X�g�ő���M
//
void initSCI_12(void)
{
	SCI12.SCR.BYTE = 0;	// ����M�֎~ 
	
	MPC.PWPR.BIT.B0WI = 0;   // �}���`�t�@���N�V�����s���R���g���[���@�v���e�N�g����
	MPC.PWPR.BIT.PFSWE = 1;  // PmnPFS ���C�g�v���e�N�g����
	
	MPC.PB0PFS.BYTE = 0x0C;  // PB0 = SSCL12
	MPC.PB1PFS.BYTE = 0x0C;  // PB1 = SSDA12
	
	MPC.PWPR.BYTE = 0x80;    //  PmnPFS ���C�g�v���e�N�g �ݒ�
	
	PORTB.PMR.BIT.B0 = 1;   // PB0  ���Ӌ@�\�Ƃ��Ďg�p
	PORTB.PMR.BIT.B1 = 1;   // PB1   :
	
	PORTB.ODR0.BIT.B0 = 1;	// PB0 N�`���l���I�[�v���h���C���o��
	
	PORTB.ODR0.BIT.B2 = 1;	// PB1 N�`���l���I�[�v���h���C���o��
	PORTB.ODR0.BIT.B3 = 0;
	
	SCI12.SIMR3.BIT.IICSDAS = 3 ;  // SSDAn�[�q�̓n�C�C���s�[�_���X���
	SCI12.SIMR3.BIT.IICSCLS = 3 ;  // SSCLn�[�q�̓n�C�C���s�[�_���X���
	
	SCI12.SMR.BYTE = 0;		// �Ȉ�I2C���[�h, �N���b�N=PCLKB= 32[MHz] (n=0)
	
	SCI12.SCMR.BIT.SDIR = 1;	// MSB�t�@�[�X�g�ő���M
	SCI12.SCMR.BIT.SINV = 0;	// ����M�f�[�^�C���o�[�g����
	SCI12.SCMR.BIT.SMIF = 0;	// ��X�}�[�g�J�[�h�C���^�t�F�[�X���[�h
	
	SCI12.BRR = 9;	 		// 100Kbps  
	
	SCI12.SEMR.BIT.BRME = 0;	// �r�b�g���[�g���W�����[�V�����@�\����
	SCI12.SEMR.BIT.NFEN = 0;	// SSCLn�ASSDAn���͐M���̃m�C�Y�����@�\����
	
	SCI12.SNFR.BIT.NFCS = 1;	// 1�����̃N���b�N���m�C�Y�t�B���^�Ɏg�p
	
	SCI12.SIMR1.BIT.IICM = 1;	// �Ȉ�I2C���[�h
	SCI12.SIMR1.BIT.IICDL = 0x01;	// SSDA�o�͒x�� 0�`1�T�C�N��
	
	
	SCI12.SIMR2.BIT.IICINTM = 1;   // �Ȉ�I2C ���[�h���̊��荞�ݗv���̗v��:��M���荞�݁A���M���荞�݂��g�p 
	SCI12.SIMR2.BIT.IICACKT = 1;   // NACK���M�܂���ACK/NACK��M (�}�X�^���M���ɂ́A�X���[�u����� ACK/NACK��M)
	SCI12.SIMR2.BIT.IICCSC = 1;    // �N���b�N�������s��
	
	SCI12.SPMR.BYTE = 0;		// SPI���[�h���W�X�^�̃N���A
		
	IPR(SCI12,RXI12) = 5;		// ��M ���荞�݃��x�� = 5�i15���ō����x��)
	IR(SCI12,RXI12) = 0;	        // ���荞�ݗv���̃N���A
	IEN(SCI12,RXI12) = 1;		// ��M���荞�݋���
	
	IPR(SCI12,TXI12) = 5;		// ���M ���荞�݃��x�� = 5 �i15���ō����x��)  
        IR(SCI12,TXI12) = 0;		//  ���荞�ݗv���̃N���A
	IEN(SCI12,TXI12) = 1;		// ���M���荞�݋���
	
	IPR(SCI12,TEI12) = 5;		//  STI���荞��(���M����TEI) ���荞�݃��x�� = 5 �i15���ō����x��)
	IR(SCI12,TEI12) = 0;	        //  ���荞�ݗv���̃N���A
	IEN(SCI12,TEI12) = 1;		//  STI(���M����)���荞�݋���
	

	SCI12.SCR.BYTE = 0x30;		// ���M����(TE),��M����(RE)�����BSTI(TEI)���荞�ݗv��, RXI���荞�ݗv��, TXI���荞�ݗv�� �͋֎~

	
	

}

