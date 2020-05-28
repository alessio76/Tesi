//COMMIT

#include <stdio.h>
#include <inttypes.h>
#include <linux/kernel.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <linux/types.h>
/*#include <sched.h>
#include <linux/sched.h>*/
#include <sys/types.h>
#include <sys/mman.h>
#include <pthread.h>
#include <math.h>
#include "ethercat.h"

#define EC_TIMEOUTMON 500
#define NSEC_PER_SEC 1000000000
#define DEADLINE 1000000 //deadline in ns
#define RUNTIME 500000 //runtime assicurato in ns
#define STACK_SIZE_3M 3*1024*1024
#define STACK_SIZE_ERROR 128000
#define EPOS4 1 
#define DELAY 0

/* INIT=sincronizzazione e apertura della mailbox
 * PRE_OP=scambio di SDO via mailbox per settare i PDO e altri valori di default
 * SAFE_OP=apertura collegamento EtherCAT con scambio di PDO
 * OP=scambio di dati consentito sia via mailbox sia via EtherCAT (sincrono e asincrono)*/

/*porta di sopra=eno1
  porta sotto=enp2s0*/

char IOmap[4096];
int expectedWKC;
boolean needlf;
volatile int wkc;  
boolean inOP;
uint8 currentgroup = 0;
int dorun = 0;
pthread_t  thread_error,thread_main,thread_RT;
int64 toff,gl_delta;
int64 time1,time2=0;


//struttra che rappresenta gli ingressi dell'EPOS
typedef struct
{
        int32 target_position;
      
} in_EPOSt;
in_EPOSt  * in_EPOS;

//struttra che rappresenta le uscite dell'EPOS
typedef struct
{
        int32 position_actual_value;
        int32 velocity_actual_value;
        int16 torque_actual_value;
        int16 statusword;
      
} out_EPOSt;
out_EPOSt * out_EPOS;

//struttura ceh rappresenta le entry dell'Object Dictionary
typedef struct 
{
        int index;
        int sub_index;
        int size;
        int value;
      
} OBentry;

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
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
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
   static int64 integral = 0;
   int64 delta;
   /* set linux sync point 50us later than DC sync, just as example */
   delta = (reftime - 50000) % cycletime;
   if(delta> (cycletime / 2)) { delta= delta - cycletime; }
   if(delta>0){ integral++; }
   if(delta<0){ integral--; }
   *offsettime = -(delta / 100) - (integral / 20);
   gl_delta = delta;
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
	//0x607A=taget_position_index, 0x00=target_position_subindex, 0x20=taget_position_bitlength
	OBentry RxPDO1={0x1600,0x01,sizeof(uint32),0x607A0020}; 
	retval=ec_SDOwrite(slave,RxPDO1.index,RxPDO1.sub_index,FALSE,RxPDO1.size,&(RxPDO1.value),EC_TIMEOUTSAFE);
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
	
	}

//mancano i parametri dei controllori e l'errore massimo di inseguimento
int CSP_EPOSsetup(uint16 slave)
{
	
    OBentry mode={0x6060, 0x00, sizeof(int8_t),(int8_t)8}; //modalità di funzionamento 8=CSP
    OBentry period={0x60C2, 0x01, sizeof(uint8),(uint8)1}; //periodo di interpolazione in ms
    OBentry nominal_current={0x3001, 0x01, sizeof(uint32),(uint32)1500}; //corrente nominale in mA
    OBentry max_current={0x3001, 0x02, sizeof(uint32),(uint32)(2*nominal_current.value)}; //corrente massima del motore in mA 
    OBentry thermal_time_constant_winding={0x3001, 0x04, sizeof(uint16),(uint16)33}; //costante di tempo termica di avvolgimento in 0.1s
    OBentry torque_costant={0x3001, 0x05, sizeof(uint32),(uint32)24300}; //costante di coppia in uNm/A
   
    //parametri da settare
    int8_t mode_prova;
    uint8 period_prova;
    uint32 nominal_current_prova;
    uint32 max_current_prova;
    uint16 thermal_time_constant_winding_prova;
    uint16 torque_costant_prova;
    
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



OSAL_THREAD_FUNC ethercat_RT(){
	
   struct sched_attr attr;
   sched_deadline(&attr,RUNTIME,DEADLINE,DEADLINE,0);
   struct timespec ts, tleft;
   int ht;
   int64 cycletime;

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
  
   while(1)
   {
	   
      /* calculate next cycle start */
      add_timespec(&ts, cycletime + toff);
      /* wait to cycle start */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
      if(dorun>0)
      {
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
      dorun++;
		if (ec_slave[0].hasdc)
             {
            /* calulate toff to get linux time and DC synced */
            ec_sync(ec_DCtime, cycletime, &toff);
             }
       ec_send_processdata();
    
       }
   }	
	
}

OSAL_THREAD_FUNC ethercat_main(char *ifname)
{
    int  oloop, iloop, state, chk,i, j;
    needlf = FALSE;
    inOP = FALSE;
    struct timespec ts, tleft;
    ts.tv_nsec=100000000;
    ts.tv_sec=0;
   
   printf("Starting ethercat_main\n");
   //inizializzazione e effettua il binding della socket ifname
   if (ec_init(ifname)){
      printf("ec_init on %s succeeded.\n",ifname);

		//trova e configura gli slave
       if (ec_config_init(FALSE) > 0 ){
         printf("%d slaves found and configured.\n",ec_slavecount);
         
         //sincronizzo gli slave
          if(!ec_configdc()) printf("Sincronizzazione fallita\n"); 
          
		//impongo PRE_OP poichè è l'unico stato in cui posso mappare i PDO
         ec_slave[0].state = EC_STATE_PRE_OP;
		 ec_writestate (0);
		 state=ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE*4);
		 
		 printf("Stato attuale=%d\n",state);
		 
		  ec_dcsync0(EPOS4, TRUE, DEADLINE,1000);
		 
		/* EPOS setup */
          ec_slave[EPOS4].PO2SOconfig = CSP_EPOSsetup; 
         
        //mappo i PDO degli slave nel mio buffer locale
         if(ec_config_map(&IOmap) < 0) printf("Mappaggio fallito\n");
         
         //controllo quanti byte ho da inviare e leggere
         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
         if (oloop > 8) oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
         if (iloop > 8) iloop = 8;
         
         //aspetto che tutti gli slave si portino in SAFE_OP
         ec_slave[0].state = EC_STATE_SAFE_OP ;
		 ec_writestate (0);
         state=ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
         printf("Stato attuale=%d\n",state);
         
         chk=200;
         ts.tv_nsec=1000000;
		 ts.tv_sec=0;
		 
         do
			{
			ec_send_processdata();
			ec_receive_processdata(EC_TIMEOUTRET);
			usleep(1000);
			//nanosleep(&ts,&tleft); // CYCLE_TIME
			}
		while (chk--);
         ts.tv_nsec=100000000;
		 ts.tv_sec=0;
		 usleep(1000);
		 */

         /* connect struct pointers to slave I/O pointers */
         in_EPOS = (in_EPOSt*) ec_slave[EPOS4].inputs;
         out_EPOS = (out_EPOSt*) ec_slave[EPOS4].outputs;

         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to init SM in slaves */
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         //richiede stato OPERATIONAL per tutti
         ec_writestate(0);
         chk = 200;
         
         //aspetta che tutti raggiungano lo stato OPERATIONAL
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         }
         while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         dorun=1;
         
         if (ec_slave[0].state == EC_STATE_OPERATIONAL ){
			  inOP = TRUE;
          
              /* loop */
            for(i = 1; i <= 5000; i++)
            {
			   
               printf("Processdata cycle %5d , Wck %3d, DCtime %12ld dt=%ld  O:",
                  dorun, wkc , ec_DCtime, gl_delta);
               for(j = 0 ; j < oloop; j++)
               {
                  printf(" %2.2x", *(ec_slave[0].outputs + j));
               }
               printf(" I:");
               for(j = 0 ; j < iloop; j++)
               {
                  printf(" %2.2x", *(ec_slave[0].inputs + j));
               }
               printf("\r");
               fflush(stdout);
               osal_usleep(20000);
            }
                dorun=0;
                inOP = FALSE;      
            }
            else{
				//ec_dcsync0(1, FALSE, 8000, 0); // SYNC0 off
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                //controllo quale slave non è in OP State e ne stampo le info
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            
            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            //richiede stato INIT per tutti
            ec_writestate(0);
        }
        
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
       
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
        
    }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */
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

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nMaster EtherCAT\n");
   
   if (argc > 1)
   {
	   dorun=0;
      /* start cyclic part */
      osal_thread_create(&thread_main, STACK_SIZE_3M ,&ethercat_main,argv[1]);
      
      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread_error, STACK_SIZE_ERROR , &ecatcheck, (void*) &ctime);
      
      //faccio partire il thread RT dopo che sono certo di essere nello stato operational
	  osal_thread_create(&thread_RT, STACK_SIZE_3M, &ethercat_RT, NULL);
      
   }
   
   else
   {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
   }
	
   pthread_join(thread_main,NULL);
   printf("End program\n");
   return (0);
}
