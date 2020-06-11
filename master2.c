#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

#include "ethercat.h"

#define NSEC_PER_SEC 1000000000
#define MSEC_PER_SEC 1000
#define EC_TIMEOUTMON 500
#define DEADLINE 1000000 //deadline in ns
#define RUNTIME 500000 //runtime assicurato in ns
#define EPOS4 1 
#define DELAY 5000
#define PI 3.141592654
#define CAMPIONI 50000
#define PASSO 0.0001
#define AMPIEZZA 180

//Stati EPOS state machine
#define SWITCH_ON_DISABLED 0x40 
#define READY_TO_SWITCH_ON 0x21
#define SWITCHED_ON 0x23
#define OPERATION_ENABLED 0x37 
#define QUICKSTOP_ACTIVE 0x17
#define FAULT_REACTION_ACTIVE 0x1f
#define FAULT 0X8

//Comandi EPOS state machine
#define SHUTDOWN 0X6
#define SWITCH_ON 0x7
#define SWITCH_ON_ENABLE 0xF
#define DISABLE_VOLTAGE 0x0
#define QUICK_STOP 0X2
#define FAULT_RESET 0X80 
#define STOP 0
#define MOVE 1
#define COSTANTE_RIDUZIONE 51
#define RISOLUZIONE 4000
#define ANGOLO_GIRO 360
 
const double deg_1=((double)ANGOLO_GIRO/((double)RISOLUZIONE*(double)COSTANTE_RIDUZIONE)); 
//rappresenta un incremento in gradi 
char IOmap[4096];
pthread_t thread1, thread2,thread3;
int dorun = 0;
int deltat, tmax = 0;
int64 toff, gl_delta;
int expectedWKC;
long int time1;
long int time2;
long int cycle;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
//posizione desiderata in incrementi
int32 target_position_abs[CAMPIONI]; 
int32 position_offset;
int misure[CAMPIONI];
long int prova1;
long int prova2;

/* INIT=sincronizzazione e apertura della mailbox
 * PRE_OP=scambio di SDO via mailbox per settare i PDO e altri valori di default
 * SAFE_OP=apertura collegamento EtherCAT con scambio di PDO
 * OP=scambio di dati consentito sia via mailbox sia via EtherCAT (sincrono e asincrono)*/

/*porta di sopra=eno1
  porta sotto=enp2s0*/

//struttura ceh rappresenta le entry dell'Object Dictionary
typedef struct 
{
        int index;
        int sub_index;
        int size;
        int value;
      
} OBentry;


//struttura che rappresenta gli ingressi dell'EPOS
typedef struct PACKED
{		
	    uint16 controlword;
        int32 target_position;
        int32 position_offset;
      
} out_EPOSt;

out_EPOSt  * out_EPOS;

//struttra che rappresenta le uscite dell'EPOS
typedef struct PACKED
{
        int32 position_actual_value;
        int32 velocity_actual_value;
        int16 torque_actual_value;
        uint16 statusword;
        
} in_EPOSt;

in_EPOSt * in_EPOS;

int deg_to_inc(double deg){
	
	int32 temp=(int32)(deg/deg_1);
	return temp;
	
	}

double inc_to_deg(int inc){
	
	double temp=((double)inc*deg_1);
	return temp;
	
	}

void CSP_PDO_mapping(uint16 slave){
	
	int RxPDOs_number=0; 
	int TxPDOs_number=0;
	int retval;
	
	//mappo i PDO che l'EPOS riceve (RxPDO)
	//numero di RxPDO mappati (settato a 0 per poter cambiare la mappatura)
	OBentry RxPDOs_mapped={0x1600,0x00,sizeof(uint8),0};
	retval=ec_SDOwrite(slave,RxPDOs_mapped.index,RxPDOs_mapped.sub_index,FALSE, RxPDOs_mapped.size,&(RxPDOs_mapped.value),
	EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	
	//primo elemento mappato
	//0x6040=controlword_index, 0x00=controlword_subindex, 0x10=controlword_bitlength
	OBentry RxPDO1={0x1600,0x01,sizeof(uint32),0x60400010};
	retval=ec_SDOwrite(slave,RxPDO1.index,RxPDO1.sub_index,FALSE, RxPDO1.size,&(RxPDO1.value),EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	else RxPDOs_number++;
	
	//secondo elemento mappato
	//0x607A=target_position_index, 0x00=target_position_subindex, 0x20=taget_position_bitlength
	OBentry RxPDO2={0x1600,0x02,sizeof(uint32),0x607A0020}; 
	retval=ec_SDOwrite(slave,RxPDO2.index,RxPDO2.sub_index,FALSE,RxPDO2.size,&(RxPDO2.value),EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	else RxPDOs_number++;
	
	//secondo elemento mappato
	//0x60B0=position_offset_index, 0x00=position_offset_subindex, 0x20=position_offset_bitlength
	OBentry RxPDO3={0x1600,0x03,sizeof(uint32),0x60B00020}; 
	retval=ec_SDOwrite(slave,RxPDO3.index,RxPDO3.sub_index,FALSE,RxPDO3.size,&(RxPDO3.value),EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	else RxPDOs_number++;
	
	//comunico il numero di oggetti mappati
	RxPDOs_mapped.value=RxPDOs_number;
	retval=ec_SDOwrite(slave,RxPDOs_mapped.index,RxPDOs_mapped.sub_index,FALSE, RxPDOs_mapped.size,&(RxPDOs_mapped.value),
	EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	
	//comunico al SyncManager che gestisce i dati master -> slave SM2 le modifiche alla mappatura degli RxPDO
	//setto a 0 il valore per poterlo modificare
	OBentry SM2PDOs_mapped={0x1C12,0x00,sizeof(uint8),0};
	retval=ec_SDOwrite(slave,SM2PDOs_mapped.index,SM2PDOs_mapped.sub_index,FALSE, SM2PDOs_mapped.size,&(SM2PDOs_mapped.value),
	EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	
	//SM2_choose_RxPDO.value conterrà l'indice degli oggetti nell'OB che descrivono la mappatura prescelta
	OBentry SM2_choose_RxPDO={0x1C12,0x01,sizeof(uint16),RxPDO1.index};
	retval=ec_SDOwrite(slave,SM2_choose_RxPDO.index,SM2_choose_RxPDO.sub_index,FALSE, SM2_choose_RxPDO.size,
	&(SM2_choose_RxPDO.value),EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	
	//rendo effettive le modifiche
	SM2PDOs_mapped.value=1;
	retval=ec_SDOwrite(slave,SM2PDOs_mapped.index,SM2PDOs_mapped.sub_index,FALSE, SM2PDOs_mapped.size,&(SM2PDOs_mapped.value),
	EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	
	//mappo i PDO che l'EPOS trasmette
	//numero TxPDO mappati (settato a 0 per poter cambiare la mappatura)
	OBentry TxPDOs_mapped={0x1A00,0x00,sizeof(uint8),0};
	retval=ec_SDOwrite(slave,TxPDOs_mapped.index,TxPDOs_mapped.sub_index,FALSE, TxPDOs_mapped.size,&(TxPDOs_mapped.value),
	EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	
	//primo elemento mappato
	//0x6064=position_actual_value_index, 0x00=position_actual_value_subindex, 0x20=position_actual_value_bitlength
	OBentry TxPDO1={0x1A00,0x01,sizeof(uint32),0x60640020}; 
	retval=ec_SDOwrite(slave,TxPDO1.index,TxPDO1.sub_index,FALSE,TxPDO1.size,&(TxPDO1.value),EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	else TxPDOs_number++;
	
	//secondo elemento mappato
	//0x606C=velocity_actual_value_index, 0x00=velocity_actual_value_subindex, 0x20=velocity_actual_value_bitlength
	OBentry TxPDO2={0x1A00,0x02,sizeof(uint32),0x606C0020}; 
	retval=ec_SDOwrite(slave,TxPDO2.index,TxPDO2.sub_index,FALSE,TxPDO2.size,&(TxPDO2.value),EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	else TxPDOs_number++;
	
	//terzo elemento mappato
	//0x6077=torque_actual_value_index, 0x00=torque_actual_value_subindex, 0x10=torque_actual_value_bitlength
	OBentry TxPDO3={0x1A00,0x03,sizeof(uint32),0x60770010}; 
	retval=ec_SDOwrite(slave,TxPDO3.index,TxPDO3.sub_index,FALSE,TxPDO3.size,&(TxPDO3.value),EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	else TxPDOs_number++;
	
	//quarto elemento mappato
	//0x6041=statusword_index, 0x00=statusword_subindex, 0x10=statusword_bitlength
	OBentry TxPDO4={0x1A00,0x04,sizeof(uint32),0x60410010}; 
	retval=ec_SDOwrite(slave,TxPDO4.index,TxPDO4.sub_index,FALSE,TxPDO4.size,&(TxPDO4.value),EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	else TxPDOs_number++;
	
	//comunico il numero di oggetti 
	TxPDOs_mapped.value=TxPDOs_number;
	retval=ec_SDOwrite(slave,TxPDOs_mapped.index,TxPDOs_mapped.sub_index,FALSE, TxPDOs_mapped.size,&(TxPDOs_mapped.value),
	EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	
	//comunico al SyncManager che gestisce i dati master <- slave SM3 le modifiche precedenti alla mappatura dei TxPDO
	//setto a 0 il valore per poterlo modificare
	OBentry SM3PDOs_mapped={0x1C13,0x00,sizeof(uint8),0};
	retval=ec_SDOwrite(slave,SM3PDOs_mapped.index,SM3PDOs_mapped.sub_index,FALSE, SM3PDOs_mapped.size,&(SM3PDOs_mapped.value),
	EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	
	//SM2_choose_TxPDO.value conterrà l'indice degli oggetti nell'OB che descrivono la mappatura prescelta
	OBentry SM3_choose_TxPDO={0x1C13,0x01,sizeof(uint16),TxPDO1.index};
	retval=ec_SDOwrite(slave,SM3_choose_TxPDO.index,SM3_choose_TxPDO.sub_index,FALSE, SM3_choose_TxPDO.size,
	&(SM3_choose_TxPDO.value),EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	
	//rendo effettive le modifiche
	SM3PDOs_mapped.value=1;
	retval=ec_SDOwrite(slave,SM3PDOs_mapped.index,SM3PDOs_mapped.sub_index,FALSE, SM3PDOs_mapped.size,&(SM3PDOs_mapped.value),
	EC_TIMEOUTSAFE);
	if(retval<0) printf("Scrittura fallita\n");
	
	ec_dcsync0(EPOS4,TRUE,DEADLINE,DELAY);
	
	}

//mancano i parametri dei controllori e l'errore massimo di inseguimento
int CSP_EPOSsetup(uint16 slave){
	
    OBentry mode={0x6060, 0x00, sizeof(int8_t),(int8_t)8}; //modalità di funzionamento 8=CSP
    OBentry period={0x60C2, 0x01, sizeof(uint8),(uint8)1}; //periodo di interpolazione in ms
    OBentry nominal_current={0x3001, 0x01, sizeof(uint32),(uint32)1500}; //corrente nominale in mA
    OBentry max_current={0x3001, 0x02, sizeof(uint32),(uint32)(2*nominal_current.value)}; //corrente massima del motore in mA 
    OBentry thermal_time_constant_winding={0x3001, 0x04, sizeof(uint16),(uint16)33}; //costante di tempo termica di avvolgimento in 0.1s
    OBentry torque_costant={0x3001, 0x05, sizeof(uint32),(uint32)24300}; //costante di coppia in uNm/A
    OBentry following_error_window={0x6065, 0x00, sizeof(uint32),(uint32)10e6}; 
    //scostamento max tra riferimento e posizione attuale
    OBentry actual_position={0x6064, 0x00, sizeof(int32),(int32)0}; 
    
    //parametri da settare
    int8_t mode_prova;
    uint8 period_prova;
    uint32 nominal_current_prova;
    uint32 max_current_prova;
    uint16 thermal_time_constant_winding_prova;
    uint16 torque_costant_prova;
    uint32 following_error_window_prova;
    
    /* non settabili o corretti di default
    int32_t position_offset_prova;
    int16_t torque_offset_prova;
    uint32 main_sensor_resolution_prova;
    uint32 control_prova;
    uint32 sensor_prova;
    uint32 max_gear_input_speed_prova;
    uint32 max_motor_speed_prova;*/
    
	int retval;
	/*int length32=sizeof(uint32);
	int length16=sizeof(int16_t);*/
	
    /* SETUP dei valori di default*/
	retval=ec_SDOwrite(slave, nominal_current.index, nominal_current.sub_index, FALSE, sizeof(uint32), &(nominal_current.value), 
	EC_TIMEOUTSAFE);
	printf("%d\n",retval);
	 
	retval=ec_SDOwrite(slave, max_current.index, max_current.sub_index, FALSE, sizeof(uint32), &(max_current.value), EC_TIMEOUTSAFE);
	printf("%d\n",retval);
	
	retval=ec_SDOwrite(slave, thermal_time_constant_winding.index, thermal_time_constant_winding.sub_index, FALSE, sizeof(uint16), 
	&(thermal_time_constant_winding.value), EC_TIMEOUTSAFE);
	printf("%d\n",retval);
	
	retval=ec_SDOwrite(slave, torque_costant.index, torque_costant.sub_index, FALSE, sizeof(uint32), &(torque_costant.value), 
	EC_TIMEOUTSAFE);
	printf("%d\n",retval);
	
	/* SETUP degli oggetti relativi alla modalità CSP */
	retval=ec_SDOwrite(slave,mode.index, mode.sub_index, FALSE, mode.size, &(mode.value), EC_TIMEOUTSAFE);
    printf("%d\n",retval);
    
    retval=ec_SDOwrite(slave, period.index, period.sub_index, FALSE, sizeof(uint8), &(period.value), EC_TIMEOUTSAFE);
    printf("%d\n",retval);
    
    retval=ec_SDOwrite(slave, following_error_window.index, following_error_window.sub_index, FALSE, 
    sizeof(uint32), &(following_error_window.value), EC_TIMEOUTSAFE);
	printf("%d\n",retval);
	
    //controllo che i parametri siano impostati correttamente
	retval=ec_SDOread(slave, 0x6061, mode.sub_index, FALSE, &(mode.size) , &mode_prova, EC_TIMEOUTSAFE);
	printf("mode=%d,letti=%d,esito=%d\n",mode_prova,mode.size,retval);
	
	retval=ec_SDOread(slave, period.index, period.sub_index, FALSE, &(period.size), &period_prova, EC_TIMEOUTSAFE);
	printf("period=%u,letti=%d,esito=%d\n",period_prova,period.size,retval);
	
	retval=ec_SDOread(slave,nominal_current.index, nominal_current.sub_index, FALSE, &(nominal_current.size), 
	&nominal_current_prova, EC_TIMEOUTSAFE);
	printf("nominal_current=%u,letti=%d, esito=%d\n",nominal_current_prova,nominal_current.size, retval);
	
	retval=ec_SDOread(slave, max_current.index, max_current.sub_index, FALSE, &(max_current.size), 
	&max_current_prova, EC_TIMEOUTSAFE);
	printf("max_current=%u,letti=%d, esito=%d\n",max_current_prova,max_current.size, retval);
	
	retval=ec_SDOread(slave, thermal_time_constant_winding.index, thermal_time_constant_winding.sub_index, 
	FALSE, &(thermal_time_constant_winding.size), &thermal_time_constant_winding_prova, EC_TIMEOUTSAFE);
	printf("thermal_time_constant_winding=%u,letti=%d, esito=%d\n",thermal_time_constant_winding_prova,
	thermal_time_constant_winding.size, retval);
	
	retval=ec_SDOread(slave, torque_costant.index, torque_costant.sub_index, FALSE, &(torque_costant.size), 
	&torque_costant_prova, EC_TIMEOUTSAFE);
	printf("torque_costant=%u,letti=%d, esito=%d\n",torque_costant_prova,torque_costant.size, retval);
	
	retval=ec_SDOread(slave, following_error_window.index, following_error_window.sub_index, 
	FALSE, &(following_error_window.size) , &following_error_window_prova, EC_TIMEOUTSAFE);
	printf("errore=%d,letti=%d,esito=%d\n",following_error_window_prova,following_error_window.size,retval);
	
	retval=ec_SDOread(slave, actual_position.index, actual_position.sub_index, 
	FALSE, &(actual_position.size) , &position_offset, EC_TIMEOUTSAFE);
	printf("offset=%d,letti=%d,esito=%d\n",position_offset,actual_position.size,retval);
	
	/*retval=ec_SDOread(slave, 0x3000, 0x05, FALSE, &length32, &main_sensor_resolution_prova, EC_TIMEOUTSAFE);
	printf("risoluzione=%u,letti=%d, esito=%d\n",main_sensor_resolution_prova,length32,retval);
	
	retval=ec_SDOread(slave, 0x60B2, 00, FALSE, &(length16), &torque_offset_prova, EC_TIMEOUTSAFE);
	printf("torque_offset=%u,letti=%d, esito=%d\n",torque_offset_prova,length16, retval);
	
	retval=ec_SDOread(slave, 0x60B0, 00, FALSE, &(length32), &position_offset_prova, EC_TIMEOUTSAFE);
	printf("position_offset=%u,letti=%d, esito=%d\n",position_offset_prova,length32, retval);
	
	retval=ec_SDOread(slave, 0x3000, 0x02, FALSE, &length32, &control_prova, EC_TIMEOUTSAFE);
	printf("controllo=%8x,letti=%d, esito=%d\n",control_prova,length32,retval);
	
	retval=ec_SDOread(slave, 0x3000, 0x01, FALSE, &length32, &sensor_prova, EC_TIMEOUTSAFE);
	printf("sensor=%x,letti=%d, esito=%d\n",sensor_prova,length32,retval);
	
	retval=ec_SDOread(slave, 0x6080, 0x00, FALSE, &length32, &max_motor_speed_prova, EC_TIMEOUTSAFE);
	printf("max_motor_speed=%u,letti=%d, esito=%d\n",max_motor_speed_prova,length32,retval);
	
	retval=ec_SDOread(slave, 0x3003, 0x03, FALSE, &length32, &max_gear_input_speed_prova, EC_TIMEOUTSAFE);
	printf("max_gear_input_speed=%u,letti=%d, esito=%d\n",max_gear_input_speed_prova,length32,retval);*/
	
	//mappo i PDO
    CSP_PDO_mapping(EPOS4);
	return 1;

}

//macchina a stati per Cia 402, parte di motion control
int Servo_state_machine(int flag){
	
	switch(in_EPOS->statusword & 0xff){
	
	    case SWITCH_ON_DISABLED:
	        if(flag) 
	            out_EPOS->controlword=SHUTDOWN;
	        else 
	            out_EPOS->controlword=DISABLE_VOLTAGE;
	            
	        return 0;
	    break;
	
	    case READY_TO_SWITCH_ON:
	        if(flag)
	            out_EPOS->controlword=SWITCH_ON;
	        else 
	            out_EPOS->controlword=DISABLE_VOLTAGE;
	            
	        return 0;
	    break;
	
	    case SWITCHED_ON:
	        if(flag){
	            out_EPOS->controlword=SWITCH_ON_ENABLE;
					return 1;
				}
				
	        else {
	            out_EPOS->controlword=DISABLE_VOLTAGE;
	            return 0;
			}
	    break;
	    
	    case OPERATION_ENABLED:
	        if(flag){
	            out_EPOS->controlword=SWITCH_ON_ENABLE;
					return 1;
				}
	        else {
	            out_EPOS->controlword=DISABLE_VOLTAGE;
	            return 0;
			}
	    break;
	    
	    case QUICK_STOP:
	        if(flag){
	            out_EPOS->controlword=SWITCH_ON;
	           return 1;
	        }
	        else {
	            out_EPOS->controlword=DISABLE_VOLTAGE;
	            return 0; 
			}
			
	    break;
	
		/*case FAULT_REACTION_ACTIVE:
	        out_EPOS->controlword=FAULT_RESET;
	        return 0; 
	    break;
		
	    case FAULT:
	        out_EPOS->controlword=FAULT_RESET;
	        return 0; 
	    break;*/
	
	    default:
	        printf("%x",in_EPOS->statusword);
	        return 0; 
	    break;
	    
	}
}
	
	  
OSAL_THREAD_FUNC redtest(char *ifname){
   int cnt,i;

   printf("Starting Redundant test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */
      if ( ec_config_init(FALSE) > 0 )
      {
         printf("%d slaves found and configured.\n",ec_slavecount);
         /* wait for all slaves to reach PRE_OP state */
		 ec_slave[0].state = EC_STATE_PRE_OP;
		 ec_writestate (0);
		 ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE*4);
		 
		 ec_slave[EPOS4].PO2SOconfig = CSP_EPOSsetup; 

         /* configure DC options for every DC capable slave found in the list */
         ec_configdc();
         ec_config_map(&IOmap);
        
        out_EPOS = (out_EPOSt*) ec_slave[1].outputs; //output del master
        in_EPOS = (in_EPOSt*) ec_slave[1].inputs;  //input del master
        
        out_EPOS->position_offset=position_offset;
        
        /* read individual slave state and store in ec_slave[] */
         ec_readstate();
         for(cnt = 1; cnt <= ec_slavecount ; cnt++)
         {
            printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                  cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                  ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
          }
          
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         
         ec_slave[0].state = EC_STATE_SAFE_OP;
		 ec_writestate (0);
		 ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE*4);

         printf("Request operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* request OP state for all slaves */
         ec_writestate(0);
         /* activate cyclic process data */
         dorun = 1;
         /* wait for all slaves to reach OP state */
         ec_statecheck(0, EC_STATE_OPERATIONAL,  5 * EC_TIMEOUTSTATE);
        
         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            printf("Operational state reached for all slaves.Actual state=%d\n",ec_slave[0].state);
            inOP = TRUE;
            /* acyclic loop */
            for(i = 1; i <= CAMPIONI; i++)
            {
               printf("PDO n.%d,target_position=%d,cycle1=%ld,cycle2=%d \n",
                  dorun, out_EPOS->target_position,cycle,ec_slave[1].DCcycle);
                  
               printf("statusword %4x,controlword %x, position_actual_value %d",
               in_EPOS->statusword,out_EPOS->controlword,in_EPOS->position_actual_value);
               
               
               fflush(stdout);
               osal_usleep(1000);
            }
            dorun = 0;
            inOP = FALSE;
         }
         else
         {
            printf("Not all slaves reached operational state.\n");
             ec_readstate();
             for(i = 1; i<=ec_slavecount ; i++)
             {
                 if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                 {
                     printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                 }
             }
         }
         
         
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End redundant test, close socket\n");
  
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n",ifname);
   }
}

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime){
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if ( ts->tv_nsec > NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime){
   static int64 integral = 0;
   int64 delta;
   /* set linux sync point 50us later than DC sync, just as example */
   delta = (reftime) % cycletime;
   if(delta> (cycletime / 2)) { delta= delta - cycletime; }
   if(delta>0){ integral++; }
   if(delta<0){ integral--; }
   *offsettime = -(delta / 100) - (integral / 20);
   gl_delta = delta;
}

	
/* RT EtherCAT thread */
OSAL_THREAD_FUNC ecatthread(){
   struct timespec ts, tleft;
   int ht;
   int64 cycletime;
   struct sched_attr attr;
   sched_deadline(&attr,RUNTIME,DEADLINE,DEADLINE,0);
   int i=0;
   int check;

   clock_gettime(CLOCK_MONOTONIC, &ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   cycletime = DEADLINE; /* cycletime in ns */
   toff = 0;
   dorun = 0;
   
   /* eseguo il pinning della pagine attuali e future occupate dal thread per garantire
        prevedibilità nelle prestazioni real-time */
   if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1){
         printf("mlockall failed: %m\n");
         pthread_cancel(pthread_self());
        }
        
   ec_send_processdata();
   while(i<CAMPIONI)
   {
	  prova1=ec_DCtime;
	  time1=ec_DCtime;
      /* calculate next cycle start */
      add_timespec(&ts, cycletime + toff);
      /* wait to cycle start */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
      if (dorun>0)
      {
         wkc = ec_receive_processdata(EC_TIMEOUTRET);
         dorun++;
         
         if (ec_slave[0].hasdc)
         {
            /* calulate toff to get linux time and DC synced */
            ec_sync(ec_DCtime, cycletime, &toff);
         }
         
	     check=Servo_state_machine(MOVE);
	     if(check){
			 misure[i]=in_EPOS->position_actual_value;
			 out_EPOS->target_position=target_position_abs[i];
			 i++;
		 }
         ec_send_processdata();
         time2=ec_DCtime;
         cycle=time2-time1;
         time1=time2;
         
      }
   }
   
   add_timespec(&ts, cycletime + toff);
   clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
   ec_sync(ec_DCtime, cycletime, &toff); 
   Servo_state_machine(STOP); //porto il controller nello stato iniziale
   ec_send_processdata();
   wkc = ec_receive_processdata(EC_TIMEOUTRET);
   
   add_timespec(&ts, cycletime + toff);
   clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
   ec_sync(ec_DCtime, cycletime, &toff);
   ec_slave[0].state = EC_STATE_SAFE_OP;
   ec_writestate(0);
   ec_statecheck(0, EC_STATE_SAFE_OP,  5 * EC_TIMEOUTSTATE);
   ec_dcsync0(EPOS4,FALSE,0,0); //disattivo SYNC0
   prova2=ec_DCtime;	
   ec_close();  //chiudo la connessione
   		
}

OSAL_THREAD_FUNC ecatcheck(){
    int slave;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

#define stack64k (64 * 1024)

int main(int argc, char *argv[]){
  
   printf("SOEM (Simple Open EtherCAT Master)\nRedundancy test\n");
   
   if (argc > 0)
   {
     dorun = 0;
     double tv[CAMPIONI];
     double t=0;
     double f=1;  //frequenza in Hz
     
	   for(int i=0;i<CAMPIONI;i++){
		    target_position_abs[i]=deg_to_inc((double)AMPIEZZA*sin(2*PI*f*t));
		    //target_position_abs[i]=deg_to_inc(AMPIEZZA);
			tv[i]=t;
			t+=PASSO; 
		}
		
		
	/* create RT thread */
      //osal_thread_create(&thread1, stack64k * 2, &ecatthread, NULL);

      /* create thread to handle slave error handling in OP */
      //osal_thread_create(&thread2, stack64k * 4, &ecatcheck, NULL);

      /* start acyclic sudo apt-get install gnuplotx-11part */
      /*osal_thread_create(&thread3, stack64k * 4, &redtest, argv[1]);
      pthread_join(thread1,NULL);*/
      //redtest(argv[1]);
   
	printf("%f\n",((double)prova2-prova1)/((double)NSEC_PER_SEC));
	FILE * fp;
	
	fp=fopen("/run/user/1000/gvfs/smb-share/rasprobotics.lab/students/Alterani"
	,"wt");
	
	/*for(int i=0;i<CAMPIONI;i++)
	       fprintf(fp,"%f %d\n",tv[i],target_position_abs[i]);*/ /*misure[i]*/
	       
	 fclose(fp);
	 /*fp = fopen("comando.txt", "wt"); 

  
    //scrivo sul file il comando da eseguire 
   
  fprintf(fp, "plot \"dati.txt\" with lines\n");

  
   //chiudo il file su cui ho scritto il comando da eseguire 
   
  fclose(fp); 

  
   eseguo il programma GNUplot passandogli il nome del file che 
   contiene il comando da eseguire
   
  system("gnuplot -persist comando.txt"); */     
}
  else
   {
      printf("Usage: red_test ifname2 cycletime\nifname = eth0 for example\ncycletime in us\n");
   }
 printf("End program\n");
   return (0);
}
