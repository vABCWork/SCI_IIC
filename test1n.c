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
	
	clear_module_stop();	//  ���W���[���X�g�b�v�̉���
	
	initSCI_12();		// SCI12(�Ȉ�I2C) �����ݒ�
					
	delay_msec(100);	// �Z���T����҂� (100msec �҂�) 
	
	Timer10msec_Set();      // �^�C�}(10msec)�쐬(CMT0)
     	Timer10msec_Start();    // �^�C�}(10msec)�J�n�@
	

	test_sci_iic_intr_aht25();		//  �����x�Z���T(AHT25)�̃e�X�g  (�Ȉ�I2C ���荞�ݎg�p)
	

//	test_sci_iic_intr_thermo_pile();		// �T�[���p�C���̃e�X�g(�Ȉ�I2C ���荞�ݎg�p)

}


// �����x�Z���T(AHT25)�̃e�X�g  (�Ȉ�I2C ���荞�ݎg�p)
//  
//    0        500msec   1000msec   1500msec      0        500msec   1000msec
//    +----------+----------+----------+----------+----------+----------+----------+
//  task0       task1      task2                task0       task1      task2
// 
//  task0: �}�X�^���M(����J�n�R�}���h)
//  task1: �}�X�^��M(�����x�f�[�^�ǂݏo��) 
//  task2: �����x�̌v�Z
//
void test_sci_iic_intr_aht25(void)
{
	uint32_t flg_task0;
	uint32_t flg_task1;
	uint32_t flg_task2;
	
	flg_task0 = 0;
	flg_task1 = 0;
	flg_task2 = 0;
	
	
	sci_iic_slave_adrs = 0x38;    	//  �X���[�u�A�h���X = 0x3B (�����x�Z���T AHT25)
	
	while(1)			
	{
	  
	  if ( timer_10msec_cnt == 0 ) {   // �ŏ� 		
	     if ( flg_task0 == 0 ) {
		   
	       	   sci_wr_sensor_cmd();	//�����x�Z���T�ւ̑���J�n�R�}���h���M
		   
		   flg_task0 = 1;
	      }
	   }
		  
	   else if ( timer_10msec_cnt == 50 ) {    // 500[msec]�o��
	       if ( flg_task1 == 0 ) {    
	  
		    sci_rd_sensor_humi_temp();	// �����x�f�[�^�̓ǂݏo��  
		    
		    flg_task1 = 1;   
	       }
	   }
	  
	   else if ( timer_10msec_cnt == 100 ) {    // 1000[msec]�o��
	       if ( flg_task2 == 0 ) {    
	  
		    sci_cal_humidity_temperature();	//  ���x�Ɖ��x���v�Z
		    
		    flg_task2 = 1;   
	       }
	   }
	  
	  else if ( timer_10msec_cnt == 150 ) {    // 1500[msec]�o��
	  	
	  	flg_task0 = 0;
		flg_task1 = 0;
		flg_task2 = 0;
	  
	  }	       	  
		
	}
	
}



//�@�T�[���p�C���̃e�X�g(�Ȉ�I2C ���荞�ݎg�p)
//  
//    0        500msec   1000msec   1500msec      0        500msec   1000msec   1500msec
//    +----------+----------+----------+----------+----------+----------+----------+----
//  task0       task1      task2     task3     task0       task1      task2      task3
// 
//  task0: �}�X�^����M(���͉��x�̓ǂݏo��)
//  task1:�@���͉��x�̌v�Z 
//  task2: �}�X�^����M(����Ώە����x�̓ǂݏo��)
//  task3:  ����Ώە����x�̌v�Z
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
	
	sci_iic_slave_adrs = 0x3d;    	//  �X���[�u�A�h���X = 0x3D (7bit; 011 1101)  (�T�[���p�C�� A3D01S)
	
	while(1)			
	{
	  
	  if ( timer_10msec_cnt == 0 ) {   // �ŏ� 		
	     if ( flg_task0 == 0 ) {
		     
		   sci_rd_thermo_pile(0);          // �Z���T�̎��͉��x(Self temperature)(TA)��ǂݏo��
	       	   
		   flg_task0 = 1;
	      }
	   }
		  
	   else if ( timer_10msec_cnt == 50 ) {    // 500[msec]�o��
	       if ( flg_task1 == 0 ) {  
		       
	          sci_iic_cal_crc_thermo_pile();	// PEC�ɂ��f�[�^�`�F�b�N 
	
		  sci_iic_cal_Ta_To_temperature();	// ���͉��x�̌v�Z
		    
		  flg_task1 = 1;   
	       }
	   }
	  
	   else if ( timer_10msec_cnt == 100 ) {    // 1000[msec]�o��
	       if ( flg_task2 == 0 ) {    
	  
		    sci_rd_thermo_pile(1);          // ����Ώە��̉��x(Objet temperature)(TO)��ǂݏo��
		    
		    flg_task2 = 1;   
	       }
	   }
	   
	    else if ( timer_10msec_cnt == 150 ) {    // 1500[msec]�o��
		if ( flg_task3 == 0 ) {   
	  	  
	          sci_iic_cal_crc_thermo_pile();	// PEC�ɂ��f�[�^�`�F�b�N 
	
		  sci_iic_cal_Ta_To_temperature();	// ����Ώە��̉��x�̌v�Z
		 
		  flg_task3 = 1;
	        } 
	    }	       	  
	   
	   else if ( timer_10msec_cnt == 190 ) {    // 1900[msec]�o��
	  	
	  	flg_task0 = 0;
		flg_task1 = 0;
		flg_task2 = 0;
		flg_task3 = 0;
	  
	  }	       	  
		
	}
}




// ���W���[���X�g�b�v�̉���
// �R���y�A�}�b�`�^�C�}(CMT) ���j�b�g0(CMT0, CMT1) 
//   �V���A���R�~���j�P�[�V�����C���^�t�F�[�X12(SCI12)(�Ȉ�I2C�ʐM)
//  CRC ���Z��iCRC�j(RIIC I2C�ʐM�p)
//
void clear_module_stop(void)
{
	SYSTEM.PRCR.WORD = 0xA50F;	// �N���b�N�����A����d�͒ጸ�@�\�֘A���W�X�^�̏������݋���	
	
	MSTP(CMT0) = 0;			// �R���y�A�}�b�`�^�C�}(CMT) ���j�b�g0(CMT0, CMT1) ���W���[���X�g�b�v�̉���
	MSTP(SCI12) = 0;		// SCI12  ���W���[���X�g�b�v�̉���
	MSTP(CRC) = 0;			// CRC ���W���[���X�g�b�v�̉���
	
	SYSTEM.PRCR.WORD = 0xA500;	// �N���b�N�����A����d�͒ጸ�@�\�֘A���W�X�^�������݋֎~
}

