// Входные параметры --> index_PCI (0) + номер канала (0,1,2)
//---------------------------------------------------------------------------
// Con: 0 - внутренний, 1 - внешний с перемычками
//***************************************************************************
#include <conio.h>
#include <stdio.h>
#include "tx_drv_mmk.h"
#include <sys/psinfo.h>

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/name.h>
#include <sys/kernel.h>
#include <time.h>
#include <sys/osinfo.h>
#include <process.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <math.h>
#include <fcntl.h>
#include <sys/dev.h>

#include "my_pcs.h"

char rele7[6]={0x31, 0x32, 0x73, 0xff,0x00, 0x00}; //0x74 - БУ1, 0x73 - БУ2
int chan=1;
STOP_PCS_s stop_pcs_s; //Сообщение-запрос при останове канала ПЦС
STOP_PCS_r stop_pcs_r; //Сообщение-ответ при останове канала ПЦС
pid_t proxy_DRV=0, proxy_r=0, proxy_485,pid_wrk;
UN char data_BU[28],error,error_k7,for_UM10[7],from_UM[4],errorUM,errorBSV4;
int msg=25, rmsg, i, j, rez_BU_wr=0, rez_BU_rd=0, cntl_code=0, ii=1, er_r90;
unsigned short form_receive[2], form_send[2], form_send_24[7], add_value;
int St_time, Fin_time; //прием ПЦС
unsigned int dat;
int prohod=0,prohod1=0;
int addr_BUP[3]={1,0,2};

unsigned short set_pr1[11]={0,0,0,0,0,0,0,0,0,0,0};//слово состояние прибора 1
dev_tx_t *dev_mmk;
unsigned int index=0, channel=1;
unsigned int n_ou=2;
int rezult, i , i1, UM_WORK=0, //наличие УМ
	bsv4, //включен и исправен БСВЧ
	bsv4_request;// запрос уровня БСВЧ
unsigned short mass[]={0x00,0x00,0x00,0x08,0x10,0x00,0x00,0x00};
unsigned short mass_r[]={0x00,0x00,0x00,0x08,0x10,0x00,0x00,0x00};
unsigned short mass_w[]={0x00,0x00,0x00,0x08,0x10,0x00,0x00,0x00};


UN char * ar_str[2]={"Запрос идентификатора устройства ",
					 "Запрос версии устройства "};

char cvs10_s[20]={	0xc1, 0xc0,   0xc2, 0xc0,   0xc3, 0xc0, 
					0xc4, 0xc0,   0xc5, 0xc0, 	0xc6, 0xc0, 
					0xc7, 0xc0};
unsigned cvs10_r[10]=    {0xc4, 0xc0, 0xc8, 0xc0, 0xcc, 0xc0, 0xd0, 0xc0};
unsigned cvs10_r_bkp[10]={0xc4, 0xc0, 0xc8, 0xc0, 0xcc, 0xc0, 0xd0, 0xc0};

unsigned char BUP_r[3][2],BUP_r_prev[3][2]; //data from BUP
unsigned char BUP_w[3][2]; //data for BUP

clear_CNL(int N_CNL);
unsigned char k7_keeper(unsigned char n_rele);
start_pcs_bu_and485();
start_pcs_rs();
stop_pcs();
short k6_keeper(unsigned char n_rele);
int um_bsv4(int on);//on/off bu bsvch
int RES90(int on); //rele res90
UN short BU_LVL(); //
int BUP_data(int N_BUP); //get BUP data
int get_UM_data(); // SEND 7 PACKS and RECIEVE 4 PACKS
//---------------------------------M A I N------------------------------------------
main(int argc, char *argv[]) {

//--- Разбор параметров
while( (i=getopt(argc, argv, "I:C:O:") )!=-1)	{
	switch(i){
		case 'I' :	sscanf(optarg,"%d",&index); break;   //Index PCI (0)
		case 'C' :	sscanf(optarg,"%d",&channel); break; //N канала
		case 'O' :	sscanf(optarg,"%d",&n_ou); break;	 //N ОУ
	}//switch
}//while
if(index>0) {printf("!!! Defect index_pci=%d (0)\n", index); exit(-1);}
if(channel>2) {printf("!!! Defect channal=%d (0,1,2)\n", channel); exit(-1);}

//=======================
start_pcs_bu_and485();
//======================= 
// Инициализация модуля
dev_mmk=OpenTx(index);
printf("index_pci=%d channel=%d dev_tx_t=%d\n", dev_mmk->pci_index, channel, dev_mmk);
printf("proxyR=%d proxyERR=%d proxy_mode=%d\n",
 dev_mmk->proxyR[channel], dev_mmk->proxyERR[channel], dev_mmk->proxyMODE[channel]);
// Настройка в режим ОУ
rezult=regim_ou(dev_mmk, channel, n_ou, 0);
printf("rezult ou = %d\n", rezult);
printf("proxyR=%d proxyERR=%d proxy_MODE=%d\n",
 dev_mmk->proxyR[channel], dev_mmk->proxyERR[channel], dev_mmk->proxyMODE[channel]);

 while(1){
// Принять 8 слов из КК
	while((rezult=ou_read(dev_mmk, channel, 1)) != 0);
	//printf("rezult ou_read = %d\n", rezult);
//	for(i=0; i<12; i++) printf(" %x", dev_mmk->tx_B[i]);
//	printf("\n in  - ");
//	for(i=0; i<3; i++) printf(" %04x", dev_mmk->tx_B[i+4]);

	//normirovanie uglov
	for(i=0; i<3; i++) dev_mmk->tx_B[i+4]=dev_mmk->tx_B[i+4]&0x0fff;

	//("\n");
	//dev_mmk->tx_B[10]=dev_mmk->tx_B[10]&0x0380;
	//printf("k7=%d    old=%d    ",dev_mmk->tx_B[10]&0x0380,set_pr1[10]&0x0380);
  //prohod++;if (prohod==3) dev_mmk->tx_B[10]=0x0600;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Обработка
			//NE RABOTAET TVK
			mass[4]=mass[4]|0x0400;
			if(dev_mmk->tx_B[7]&0x8000) mass[4]=mass[4]|0xf000;else mass[4]=mass[4]&0x0fff;


if (dev_mmk->tx_B[4]==0) dev_mmk->tx_B[4]=0x7c7;
//-------------------------BUP---------------------------------
//for(i=0;i<3;i++) mass[i]=dev_mmk->tx_B[i+4]; //возвращаем коды угла назад

for(i=0;i<3;i++)  //!!!!!
{
	if (i==1) {BUP_w[1][0]=(dev_mmk->tx_B[1+4]>>7)&0x1f; BUP_w[1][1]=(dev_mmk->tx_B[1+4]>>2)&0x1f;}
	  	 else {BUP_w[i][0]=(dev_mmk->tx_B[i+4]>>6)&0x3f; BUP_w[i][1]=(dev_mmk->tx_B[i+4])&0x3f;}
	//mass[i]=mass[i]&0x7fff;
	
	if (BUP_data(i)==0)
	{	
		if (i==1)	{mass[i]=BUP_r[i][0]&0x1f;mass[i]=mass[i]<<7;mass[i]+=(BUP_r[i][1]&0x1f)<<2;}
		else 	    {mass[i]=BUP_r[i][0]&0x3f;mass[i]=mass[i]<<6;mass[i]+=BUP_r[i][1]&0x3f;}

		//уменьшение ошибки
/*		if ((i==1)&&(mass_r[i]!=mass[i]))
		{
			mass_r[1]=mass[1];
//			printf("mass=%d  mass_p=%d\n",mass[i],mass_w[i]);
		//	if (( abs(dev_mmk->tx_B[i+4]-(mass[i]&0x0fff) )>2500)
		//	   ||(abs(dev_mmk->tx_B[i+4]-(mass[i]&0x0fff) )<700) ) 
			{
				//printf("owibka1=%d\n",dev_mmk->tx_B[i+4]-(mass[i]&0x0fff));			
			if  (dev_mmk->tx_B[i+4]>(mass[i]&0x7fff)) 
				mass[i]=dev_mmk->tx_B[i+4]+rand()/2000;
			else
				mass[i]=dev_mmk->tx_B[i+4]-rand()/2000;			
			}

			if (dev_mmk->tx_B[i+4]==0) 	mass[i]=(dev_mmk->tx_B[i+4]-20)&0xfff;			
			mass_w[1]=mass[1];

//			printf("mass_f_PR=%d  mass_ot_BUP=%d STEND=%d \n",mass[i],mass_r[i],dev_mmk->tx_B[5]);
//			printf("owibka1=%d\n",dev_mmk->tx_B[1+4]-(mass[1]&0x0fff));						
		}
		else if(i!=0) mass[1]=mass_w[1];

		if ((i==2)&&(mass_r[i]!=mass[i]))
		{
			mass_r[2]=mass[2];
			if (( abs(dev_mmk->tx_B[i+4]-(mass[i]&0x0fff) )>3)
			   &&(abs(dev_mmk->tx_B[i+4]-(mass[i]&0x0fff) )<150) ) 
			{
			if  (dev_mmk->tx_B[i+4]>(mass[i]&0x7fff)) 
				mass[i]=dev_mmk->tx_B[i+4]+rand()/14000;
			else
				mass[i]=dev_mmk->tx_B[i+4]-rand()/14000;			
			}
			if (dev_mmk->tx_B[6]==0) mass[2]=0;
	        mass_w[2]=mass[2];
//			printf("owibka2=%d\n",dev_mmk->tx_B[i+4]-(mass[i]&0x0fff));			
		}
		else if(i!=0) mass[2]=mass_w[2];

//			if ((abs(dev_mmk->tx_B[4]-(mass[0]&0x0fff) )>3)
//			   &&(abs(dev_mmk->tx_B[4]-(mass[0]&0x0fff) )<40) ) 
			if  (abs(dev_mmk->tx_B[4]-(mass[0]&0x0fff))<40) 
			{
			if  (dev_mmk->tx_B[4]>(mass[0]&0x7fff)) 
				mass[0]=dev_mmk->tx_B[4];//+rand()/20000;
			else
				mass[0]=dev_mmk->tx_B[4];//-rand()/20000;			

			printf("%04x %04x\n",dev_mmk->tx_B[4],mass[0]);

			}
	*/
	}
	else {mass[i]=mass[i]|0x8000;clear_CNL(8);}
	
}
//-----------------------K7------------------------------------
// Обработка команд для реле K7
if ((set_pr1[10]&0x0380) != (dev_mmk->tx_B[10]&0x0380))
{
    switch((dev_mmk->tx_B[10]&0x0380)>>7)	
	{
		case 0 : i=0;break;
		case 1 : i=4;break;
		case 2 : i=2;break;
	    case 3 : i=6;break;
		case 4 : i=1;break;
		case 5 : i=5;break;
		case 6 : i=3;break; 
	}
	error_k7=k7_keeper(i); //printf("rele = %d  error = %x\n", i, error);
    mass[5]=mass[5]&0xfe07;

    //  ошибочная работа БУ. Может выдавать ошибку от К6
	//for(i=0;i<6;i++) if(error_k7&(1<<i)) mass[5]+=0x0100>>i;

   // printf("error_k7=%x  \n",error_k7);
}
//----------------------K6------------------------------------
if ( (set_pr1[10]&0xFC00) != (dev_mmk->tx_B[10]&0xFC00))
{
	dat=dev_mmk->tx_B[10]>>10;
	dat=((dat&0x20)>>5) + ((dat&0x10)>>2) + (dat&8) + ((dat&4)<<2) + ((dat&2)<<4) + ((dat&1)<<1);
 	error=k6_keeper(dat);
//Обработка ошибки реле К6
	  mass[5]=mass[5]&0x03ff;
/* убрано из-за не правильно работающего БУ
      if (error&1) mass[5]=mass[5]+0x8000; 
      if (error&2) mass[5]=mass[5]+0x0400;//4000 
      if (error&0x04) mass[5]=mass[5]+0x4000;//0800; 
      if (error&0x08) mass[5]=mass[5]+0x2000; 
      if (error&0x10) mass[5]=mass[5]+0x1000;//1000 
      if (error&0x20) mass[5]=mass[5]+0x0800;//0400
*/
   // printf("rele K1-K6 = %x  error = %x \n", dat, error);
}
//---------------------RES90--------------------------------
if ( (set_pr1[10]&0x0040) != (dev_mmk->tx_B[10]&0x0040))
{
 	er_r90=RES90(dev_mmk->tx_B[10]&0x0040);
    mass[5]=(mass[5]&0xffbf)|(er_r90<<9);
}
//------------------------UM BSVCH--------------------------
if ( (set_pr1[10]&0x0020) != (dev_mmk->tx_B[10]&0x0020))
{
 	error=um_bsv4(dev_mmk->tx_B[10]&0x0020);
	
	switch(error)
	{
		case 0 : errorBSV4=4;break;
		case 1 : errorBSV4=6;break;
		case 2 : errorBSV4=0;break;
		case 3 : errorBSV4=2;break;		
	}
	//printf(" |error_bsv4=%d| ",error);
    mass[5]=(mass[5]&0xfff8)|errorBSV4;
}
//--------------------------PRD LEVEL-------------------------
if ((bsv4&6)==6) //если БЧСЧ включен и исправен (6)
{
	bsv4_request++;
	if (bsv4_request==3) //разрядим запросы
	{
		mass[6]=BU_LVL();//printf("\nBU_LVL=%d\n",mass[6]);
		bsv4_request=0;
	}
}
bsv4=mass[5];
//------------------------------UM1_0-------------------------
if (   ((set_pr1[7]&0xFFC0) != (dev_mmk->tx_B[7]&0xFFC0)) 
     | ((set_pr1[8]&0xFFFE) != (dev_mmk->tx_B[8]&0xFFFE)) 
     |  (set_pr1[9] != dev_mmk->tx_B[9]))
{    //проверка изменения командных слов

	for(i=0;i<7;i++) for_UM10[i]=0;//обнуление слов    
	 //заполнение данных для отправки в УМ	
	for(i=0;i<5;i++) for_UM10[0]+=((dev_mmk->tx_B[7]>>(14-i))&1)<<(5-i);//1-5
	for_UM10[0]+=(dev_mmk->tx_B[7]>>15)&1;     //6

	for_UM10[1]+=((dev_mmk->tx_B[8]>>5)&1)<<5; //7
	for(i=0;i<4;i++) for_UM10[1]+=((dev_mmk->tx_B[8]>>(12+i))&1)<<(4-i);//8-11
	for_UM10[1]+=(dev_mmk->tx_B[8]>>4)&1; //12

	for(i=0;i<6;i++) for_UM10[2]+=((dev_mmk->tx_B[8]>>(6+i))&1)<<(5-i);

	for_UM10[3]+=((dev_mmk->tx_B[9]>>5)&1)<<5;//19
	for(i=0;i<4;i++) for_UM10[3]+=((dev_mmk->tx_B[9]>>(12+i))&1)<<(4-i);//20-23
	for_UM10[3]+=(dev_mmk->tx_B[9]>>4)&1;//24

	for(i=0;i<6;i++) for_UM10[4]+=((dev_mmk->tx_B[9]>>(6+i))&1)<<(5-i);

	for_UM10[5]+=((dev_mmk->tx_B[9]>>3)&1)<<5;//31
	for_UM10[5]+=((dev_mmk->tx_B[9]>>2)&1)<<4;//32

	for_UM10[5]+=dev_mmk->tx_B[8]&0x0e; //33-35
	for_UM10[5]+=(dev_mmk->tx_B[9]>>1)&1;     //36

	for_UM10[6]+=(dev_mmk->tx_B[9]&1)<<4;//38
	for(i=0;i<4;i++) for_UM10[6]+=((dev_mmk->tx_B[7]>>(6+i))&1)<<(3-i);//39-42

	//printf("\n DATA for UM ");
	//for(i=0;i<7;i++) printf(" %d.%x ",i+1,for_UM10[i]);printf("\n");
	//if(get_UM_data()==-1) get_UM_data();

	//USILITEL NE RABOTAET
	//if (for_UM10[5]&0x20) {for_UM10[1]=for_UM10[1]|0x20;printf("1\n");}
	//if (for_UM10[3]&0x20) {for_UM10[1]=for_UM10[1]|0x20;printf("2\n");}
	//---------------------------------
	//!!!!!mass[6]++;
}
	//if (UM_WORK<5) //
	{
		prohod++;
		if (prohod==2) 
		{
			i=get_UM_data();
			prohod=0;
			if (i==-1) {clear_CNL(8);UM_WORK++;} else UM_WORK=0; //копим кол-во ошибок
		}
	}
	//else printf("\nUM not find. Stop requesting!\n");
	//printf("%d ",prohod);
//------------------------------------------------------------
for(i=0; i<=11; i++) set_pr1[i]=dev_mmk->tx_B[i]; //запомним состояние упр слова
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
for(i=0; i<3; i++) mass[i]=mass[i]&0xfff;

  // Передать 8 слов КК
//    printf(" mass[5]=%x \n",mass[5]);
	rezult=ou_write(dev_mmk, channel, 1, 8, mass);
//	printf("\n out - ");
 //for(i=0; i<3; i++) printf(" %04x",mass[i]);printf("\n");
   
	//printf("rezult ou_write = %d\n", rezult);
	//delay(5);
} // while 1

CloseTx(dev_mmk);
//printf("Процесс ts_mmk OK!!!\n");
exit(0);
} //*************************************************************************
clear_CNL(int N_CNL)
{
	int rez1;
	//очистка буфера
    msg_ic.type=RD_CPCS;
	msg_ic.wr_s.cnl=N_CNL;		
	rez1=Send(pid_drv, &msg_ic, &msg_oc, sizeof(RD_CPCS_s),sizeof(RD_CPCS_r));

  	if(rez1!=-1&&!msg_oc.rd_r.status)     
	{//норма приема-передачи по каналу
			printf("\n!!cnt%d=%d!!\n",N_CNL,msg_oc.rd_r.cnt);
	}
}
//-----------------------------------------------
//***** ЗАПУСК КАНАЛА ДЛЯ СВЯЗИ С БУ2 ***********
start_pcs_bu_and485() {
int	rez1=0;
// Проверка запуска драйвера модуля ПЦС ЦВС-3-1
	pid_drv=get_pid_process("PCS",CVS_NODE);
	if (!pid_drv) {
		printf("!!! Драйвер не запущен\n");
		exit(-1);
	} 
// Инициализация канала ПЦС для связи с БУ2
	init_pcs_s.type=1;
	init_pcs_s.dev=chan;

	i=Send(pid_drv,&init_pcs_s,&init_pcs_r,sizeof(init_pcs_s),sizeof(init_pcs_r));
	if (i==-1) printf("Нет общения с драйвером\n");
	if (init_pcs_r.type==1) 
		if (init_pcs_r.status!=0) {
			printf("!!! Инициализация канала не прошла\n");
			exit(-1);
		}
	printf("Инициализация канала прошла\n");

// Старт канала ПЦС для связи с БУ2
	start_pcs_s.type=2;
	start_pcs_s.cnl=chan;
	start_pcs_s.ID_P=getpid();

	printf("my_id= %d\n",start_pcs_s.ID_P);
	Send(pid_drv,&start_pcs_s,&start_pcs_r,sizeof(start_pcs_s),sizeof(start_pcs_r));
	if (start_pcs_r.type==2) {
		if (start_pcs_r.status==-1) {
			printf("!!! Старт канала не выполнен\n");
			exit(-1);
		} 
		else { proxy_DRV=start_pcs_r.Proxy; printf("Старт канала выполнен, proxy=%d\n", proxy_DRV);}
	} 

	//очистка буфера
    msg_ic.type=RD_CPCS;
	msg_ic.wr_s.cnl=1;		
	rez1=Send(pid_drv, &msg_ic, &msg_oc, sizeof(RD_CPCS_s),sizeof(RD_CPCS_r));

  	if(rez1!=-1&&!msg_oc.rd_r.status)     
	{//норма приема-передачи по каналу
			printf("\n!!cnt422=%d!!\n",msg_oc.rd_r.cnt);
	}
	      
//-----------------------485---------------------------
	//проводим тестирование модуля ПЦС
	//инициализируем канал модуля
    msg_ic.type=INIT_PCS;
	msg_ic.ini_s.cnl=8;		//8 как RS-485 для ЦВС-1.0УМ
	msg_ic.ini_s.speed=38400;
	msg_ic.ini_s.b_info=8;
	msg_ic.ini_s.b_StrStp=1;
	msg_ic.ini_s.b_prt='O';
	msg_ic.ini_s.lvl_inp=40;   //уровень заполнения буфера приема //до прерывания	  
	msg_ic.ini_s.dev=485;	     //работа по каналу RS-485

	Send(pid_drv, &msg_ic, &msg_oc, sizeof(INIT_PCS_s),sizeof(INIT_PCS_r));

	//стартуем канал 8 ПЦС
    msg_ic.type=START_PCS;
	msg_ic.str_s.cnl=8;	
	msg_ic.str_s.ID_P=getpid();
	Send(pid_drv, &msg_ic, &msg_oc, sizeof(START_PCS_s),sizeof(START_PCS_r));
	//получаем pid Proxy для канала
	proxy_485=msg_oc.str_r.Proxy;
	printf("\ncnl=7 Proxy_CNL=%x",  proxy_485);	//???????????
	//очистка буфера
    msg_ic.type=RD_CPCS;
	msg_ic.wr_s.cnl=7+1;		
	rez1=Send(pid_drv, &msg_ic, &msg_oc, sizeof(RD_CPCS_s),sizeof(RD_CPCS_r));

  	if(rez1!=-1&&!msg_oc.rd_r.status)     
	{//норма приема-передачи по каналу
			printf("\n!!cnt485=%d!!\n",msg_oc.rd_r.cnt);
	}
}

//-----------------------------------------------
//***** ОСТАНОВ КАНАЛА ДЛЯ СВЯЗИ С БУ2 **********
stop_pcs() {
	stop_pcs_s.type=3;
	stop_pcs_s.cnl=chan;
	Send(pid_drv,&stop_pcs_s,&stop_pcs_r,sizeof(stop_pcs_s),sizeof(stop_pcs_r));
	if (stop_pcs_r.type==3) {
		if (stop_pcs_r.status==-1) {
			printf("!!! Останов канала не выполнен\n");
			exit(-1);
		} 
		else printf("Останов канала выполнен\n");
	}       
}
//-----------------------------------------------
//***** ЗАПУСК КАНАЛА ДЛЯ СВЯЗИ С RS ************
start_pcs_rs() {
// Инициализация канала ПЦС для связи с каналом chan (ЦВС-1.0УМ)
	init_pcs_s.type=1;
	init_pcs_s.cnl=chan;			// номер канала ПЦС
	init_pcs_s.speed=115200;		// скорость передачи данных
	init_pcs_s.b_info=8;			// кол-во бит в информационном байте
	init_pcs_s.b_StrStp=1;			// кол-во стоп-бит
	init_pcs_s.b_prt=0;				// наличие и тип паритета (нечетность)
	init_pcs_s.dev=0; 				// тип у-ва (0 - обычный RS канал) 
	init_pcs_s.lvl_inp=45;			// уровень заполнения FIFO до прерывания

	i=Send(pid_drv,&init_pcs_s,&init_pcs_r,sizeof(init_pcs_s),sizeof(init_pcs_r));
	if (i==-1) printf("Нет общения с драйвером\n");
	if (init_pcs_r.type==1) 
		if (init_pcs_r.status!=0) {
			printf("!!! Инициализация канала %d не прошла\n", chan);
			exit(-1);
		}
	printf("Инициализация канала %d прошла\n", chan);
// Старт канала chan
	start_pcs_s.type=2;
	start_pcs_s.cnl=chan;
	start_pcs_s.ID_P=getpid();

	printf("my_id= %d\n",start_pcs_s.ID_P);
	Send(pid_drv,&start_pcs_s,&start_pcs_r,sizeof(start_pcs_s),sizeof(start_pcs_r));
	if (start_pcs_r.type==2) {
		if (start_pcs_r.status==-1) {printf("!!! Старт канала %d не выполнен\n", chan);	exit(-1); } 
		else { proxy_DRV=start_pcs_r.Proxy; printf("Старт канала %d выполнен, proxy=%d\n", chan, proxy_DRV); }
	}       
}
//-----------------------------------------------
// Управление реле K7
unsigned char k7_keeper (unsigned char n_rele)
{
int ii,j=0;
if (n_rele<=6)
	{ 
		wr_cpcs_s.type=5;
		wr_cpcs_s.cnl=chan;
		wr_cpcs_s.cnt=6;
		for(ii=0;ii<4;ii++) wr_cpcs_s.uom.dt[ii]=rele7[ii];
		wr_cpcs_s.uom.dt[4]=n_rele;
	  	wr_cpcs_s.uom.dt[5]=0;    //КС
		for (ii=0;ii<5;ii++) wr_cpcs_s.uom.dt[5]^=wr_cpcs_s.uom.dt[ii];  
		Send(pid_drv,&wr_cpcs_s,&wr_cpcs_r,sizeof(wr_cpcs_s),sizeof(wr_cpcs_r));
		//	Прием ответного пакета
		while (1) {//Ожидание приема данных
					proxy_r=Creceive(0,0,6);
					if (proxy_r==proxy_DRV) break;
					j++; if (j==10000) break;
				  } //while 1
		rd_cpcs_s.type=4;
		rd_cpcs_s.cnl=chan;
		Send(pid_drv,&rd_cpcs_s,&rd_cpcs_r,sizeof(rd_cpcs_s),sizeof(rd_cpcs_r));
		if(n_rele!=0) printf("Реле K7, релейная группа %d   data=", n_rele);
	  	else printf("Реле K7, выключение   data=");
		for(ii=0;ii<rd_cpcs_r.cnt;ii++) printf(" %02x",rd_cpcs_r.uim.dt[ii]); printf("\n");
	} 
 return (rd_cpcs_r.uim.dt[2]);
}

// Управление реле K1-K6
short k6_keeper (unsigned char n_rele)
{
	int ii;
    if (n_rele<64)
    {
		data_BU[0]=0xff; data_BU[1]=n_rele; 
		work_bu_write(BU_K1K6, MSG_WBU_P, BU1, data_BU);
		work_bu_read(BU1, proxy_DRV, data_BU);
		printf("Реле %x   data=", n_rele);
		for(ii=0; ii<2; ii++) printf(" %x", data_BU[ii]); printf("\n");
	}   
	else return(-1);
return (data_BU[1]); 
}

UN short BU_LVL ()
{
	UN short i;
	data_BU[0]=0xff; data_BU[1]=0xff; 
	work_bu_write(BU_UR, MSG_WBU_P, BU1, data_BU);
	for(i=0;i<10;i++) data_BU[i]=0;
	work_bu_read(BU1, proxy_DRV, data_BU);
	//printf("\nLEVEL ");	for(i=0; i<5; i++) printf(" %d.=%d,  ", 4-i,data_BU[4-i]); printf("\n");
	i=data_BU[1]+data_BU[0]*256;
	return (i); 
}

int um_bsv4(int on)
{
int i,cntl_code=0,r;
if (on)// Включение УМ БСВЧ
	{
	data_BU[0]=0x1;  //Адрес БСВЧ
	data_BU[1]=0xC1; //Код команды
	data_BU[2]=0x5;  //Длина
	data_BU[3]=0xE1;    
	data_BU[4]=0;    //КС
	for (i=0;i<4;i++) data_BU[4]^=data_BU[i];  
	cntl_code=data_BU[2];
	work_bu_write(cntl_code,MSG_WBU_R,BU1,data_BU); 
	delay(50);
	rez_BU_rd=work_bu_read(BU1, proxy_DRV, data_BU);
	if (rez_BU_rd==-2) printf("Канал не стартовал\n");
	if (rez_BU_rd==-1) printf("Данных нет\n");
	if (rez_BU_rd==1) printf("Неверный тип сообщения\n");
	if (rez_BU_rd==2) printf("Истекло время ожидания прокси_on\n");
	if (rez_BU_rd==3) printf("Получен пакет с ошибкой\n");
	if (!rez_BU_rd)  printf("Запрос на чтение выполнен\n");
	printf("\nFrom BU(on)  ");
	}
	else
	{
	// Выключение УМ БСВЧ
	data_BU[0]=0x1;  //Адрес БСВЧ
	data_BU[1]=0xC1; //Код команды
	data_BU[2]=0x5;  //Длина
	data_BU[3]=0xE0;    
	data_BU[4]=0;    //КС
	for (i=0;i<4;i++) data_BU[4]^=data_BU[i];  
	cntl_code=data_BU[2];
	work_bu_write(cntl_code,MSG_WBU_R,BU1,data_BU); 
	delay(50);
	rez_BU_rd=work_bu_read(BU1, proxy_DRV, data_BU);
	if (rez_BU_rd==-2) printf("Канал не стартовал\n");
	if (rez_BU_rd==-1) printf("Данных нет\n");
	if (rez_BU_rd==1) printf("Неверный тип сообщения\n");
	if (rez_BU_rd==2) printf("Истекло время ожидания прокси_off\n");
	if (rez_BU_rd==3) printf("Получен пакет с ошибкой\n");
	if (!rez_BU_rd)  printf("Запрос на чтение выполнен\n");
	printf("\nFrom BU(off) ");
	mass[6]=0;
	}

	for(r=0;r<6;r++) printf("%x ",data_BU[r]);
	printf("\n"); 
  return (data_BU[3]);
}

// Управление реле РЭС90
int RES90 (int on)
{
	if (on) //		Замыкание реле
	{
		data_BU[0]=0xff; data_BU[1]=0x1; 
		work_bu_write(BU_RES90, MSG_WBU_P, BU1, data_BU);
		work_bu_read(BU1, proxy_DRV, data_BU);
		printf("Включение  реле РЭС90");//for(i=0; i<2; i++) printf(" %x", data_BU[i]); printf("\n");	
	}
	else //		Размыкание реле
	{
		data_BU[0]=0xff; data_BU[1]=0x0; 
		work_bu_write(BU_RES90, MSG_WBU_P, BU1, data_BU);
		work_bu_read(BU1, proxy_DRV, data_BU);
		printf("Выключение реле РЭС90");//for(i=0; i<2; i++) printf(" %x", data_BU[i]); printf("\n");
	}
return (data_BU[1]&1);
}

int BUP_data(int N_BUP)
{
int ii,kkk,ii1,j,rez1,i;
UN char fl_str=0,adr_BUP;
	
	//передаем запрос в канал
   	   pid_wrk=-1;

	   switch(N_BUP)
	   {
		case 0 : adr_BUP=2;break;//2
		case 1 : adr_BUP=1;break;//4
		case 2 : adr_BUP=4;break;//1
	   }

	   BUP_w[N_BUP][1]=BUP_w[N_BUP][1]|(adr_BUP<<5);
	   for(j=0; j<2 ;j++)  msg_ic.wr_s.uom.dt[j]=BUP_w[N_BUP][j];
	   //msg_ic.wr_s.uom.dt[0]=0xFF;
	   //msg_ic.wr_s.uom.dt[1]=0xFF;
		

	
	   //заполняем данные для БУПов
	   //for(j=0; j<2 ;j++)   printf(" w%d=%x ",j,msg_ic.wr_s.uom.dt[j]);	    
	   //printf("\nПередаем в канал 7  0x%x 0x%x", ar_msg[ii1][0],ar_msg[ii1][1]);

	   msg_ic.type=WR_CPCS;
	   msg_ic.wr_s.cnl=8;		
       msg_ic.wr_s.cnt=2;
	   Send(pid_drv, &msg_ic, &msg_oc, sizeof(WR_CPCS_s),sizeof(WR_CPCS_r));

	   //printf("\ndata for BUP%d = %x %x",N_BUP,msg_ic.wr_s.uom.dt[0],msg_ic.wr_s.uom.dt[1]);

	   kkk=0;
	   while(kkk < 3)
	 	{  
		 delay(1);
	     //ждем прихода данных и принимаем
         if((pid_wrk=Creceive(0, &msg_ic, sizeof(msg_ic)))!=-1)
	       {kkk++;
		      if(pid_wrk==proxy_485)
	          {msg_ic.type=RD_CPCS;
	           msg_ic.wr_s.cnl=7+1;		

	           rez1=Send(pid_drv, &msg_ic, &msg_oc, sizeof(RD_CPCS_s),sizeof(RD_CPCS_r));

  			   //printf("\nПринят Proxy=%x канал 7: \n", proxy_485);

	           if(rez1!=-1&&!msg_oc.rd_r.status)     
	             {//норма приема-передачи по каналу
				  if (msg_oc.rd_r.cnt!=2) return -1;			//ERROR
				  //for (i=0;i<2;i++) if (msg_oc.rd_r.uim.dt[i]>0xc0) {printf("\nError_BUP_data\n");return -1;} //ERROR

				  if ((msg_oc.rd_r.uim.dt[1]>>6)!=addr_BUP[N_BUP])  
				  {printf("\nError_BUP%d_data\n",N_BUP);
				   for (i=0;i<2;i++) printf("0x%x  ",msg_oc.rd_r.uim.dt[i]);
				   return -1;
				  } //ERROR
				  for (i=0;i<2;i++) BUP_r[N_BUP][i]=msg_oc.rd_r.uim.dt[i];
				/*  //if (BUP_r[N_BUP][1]!=BUP_r_prev[N_BUP][1])
				  {
					printf("\ndata for BUP%d = %x %x",N_BUP,BUP_w[N_BUP][0],BUP_w[N_BUP][1]);

				  	printf("\nПринят ответ от БУП%d:  ",N_BUP);
				  	for(i=0;i<msg_oc.rd_r.cnt;i++) printf("0x%x ", msg_oc.rd_r.uim.dt[i]);				  	

				  	for (i=0;i<2;i++) BUP_r_prev[N_BUP][i]=BUP_r[N_BUP][i];

				  }	*/				
		          break;	//!!!!!
			     }
	           else printf("Данные не пришли\n");//нет данных
	          }  
	       }
	     else
	       {//данные в связанный канал не пришли
		    kkk++;
			if (kkk>2) {printf("\nБУП%d не ответил",N_BUP);return -1;}//ERROR
		   }
		}
	return 0;
}


int get_UM_data()
{
int ii,kkk,ii1,j,rez1,i1,i;
UN char fl_str=0, word_num;

//	   msg_ic.type=7; // RESET CHANEL
//	   msg_ic.wr_s.cnl=8;		
      // msg_ic.wr_s.cnt=2;

//	   Send(pid_drv, &msg_ic, &msg_oc, sizeof(WR_CPCS_s),sizeof(WR_CPCS_r)); //send data to 7 chanel

	for(ii1=0; ii1<7; ii1++)
	  {//передаем запрос в канал
   	   pid_wrk=-1;
	   //for(j=0; j<2 ;j++)  msg_ic.wr_s.uom.dt[j]=cvs10_s[j+ii1*2];
	   msg_ic.wr_s.uom.dt[0]=cvs10_s[ii1*2];
       msg_ic.wr_s.uom.dt[1]=cvs10_s[ii1*2+1]|for_UM10[ii1];

	   msg_ic.type=WR_CPCS;
	   msg_ic.wr_s.cnl=8;		
       msg_ic.wr_s.cnt=2;
	   Send(pid_drv, &msg_ic, &msg_oc, sizeof(WR_CPCS_s),sizeof(WR_CPCS_r)); //send data to 7 chanel

	   kkk=0;
	   while(kkk < 3)
	 	{
		 delay(1); //20!!!
	     //ждем прихода данных и принимаем
         if((pid_wrk=Creceive(0, &msg_ic, sizeof(msg_ic)))!=-1)
	       {
			kkk++;
		    if(pid_wrk==proxy_485)
	          {msg_ic.type=RD_CPCS;
	           msg_ic.wr_s.cnl=8;		
	           rez1=Send(pid_drv, &msg_ic, &msg_oc, sizeof(RD_CPCS_s),sizeof(RD_CPCS_r));
  			   //printf("\nПринят Proxy=%x канал 7: \n", proxy_485);

	           if(rez1!=-1&&!msg_oc.rd_r.status)     
	             {//норма приема-передачи по каналу
					//printf("kkk=%d\n",kkk);
					//printf("\n UM 485");   for (i=0;i<2;i++) printf(" %x",msg_oc.rd_r.uim.dt[i]);
					if (msg_oc.rd_r.cnt!=2) return -1;			//ERROR
					for (i=0;i<2;i++) if (msg_oc.rd_r.uim.dt[i]<0xc0) {printf("\nError_UM_data %x\n",msg_oc.rd_r.uim.dt[i]);return -1;} //ERROR
					if (!(msg_oc.rd_r.uim.dt[0]&2)) {printf("\nerror prev mess\n");return -1;}		//ERROR			

					word_num = (msg_oc.rd_r.uim.dt[0] & 0x1c) >> 2; //определение номера байта
					if (word_num>5 | word_num==0) {printf("\ner_UM_N_word\n");return -1;}    //ERROR
					cvs10_r[(word_num-1)*2] =  msg_oc.rd_r.uim.dt[0];
					cvs10_r[(word_num-1)*2+1] =  msg_oc.rd_r.uim.dt[1];
				  //fflush(stdout);
		          break;	//!!!!! закончили ждать
			     }
	           else printf("Данные не пришли\n");//нет данных
	          }  
	       }
	     else
	       {//данные в связанный канал не пришли
		    //printf("Тайм-аут 3 млс прошел. Приема данных нет\n");
		    kkk++;
			if (kkk>2) {printf("\nError_UM_time\n");return -1;}
		   }
		} //end proxy waiting 
		
		for (i=0; i<8; i+=2)
		if ((cvs10_r[i] != cvs10_r_bkp[i]) || (cvs10_r[i+1] != cvs10_r_bkp[i+1])) //проверка на налиие изменений
			{//если есть изменения	
			//printf("%d: было = %x %x -> стало = %x %x ", 
			//	i/2, cvs10_r_bkp[i], cvs10_r_bkp[i+1], cvs10_r[i], cvs10_r[i+1]);
			cvs10_r_bkp[i] = cvs10_r[i]; //сохраняем в backup
			cvs10_r_bkp[i+1] = cvs10_r[i+1];

			//вносим изменения в ответный массив данных
			mass[3]=0;mass[4]=mass[4]&0x0f;

			for (i=0;i<3;i++) mass[4]+=((cvs10_r_bkp[1]>>(3+i))&1)<<(12-i);
			mass[4]+=((cvs10_r_bkp[1]>>2)&1)<<15;
			mass[4]+=((cvs10_r_bkp[1]>>1)&1)<<13;
			mass[4]+=(cvs10_r_bkp[1]&1)<<14;
	
			mass[3]+=((cvs10_r_bkp[3]>>5)&1)<<12;
			mass[3]+=((cvs10_r_bkp[3]>>4)&1)<<14;
			mass[3]+=((cvs10_r_bkp[3]>>3)&1)<<13;
			mass[3]+=((cvs10_r_bkp[3]>>2)&1)<<15;
			mass[4]+=((cvs10_r_bkp[3]>>1)&1)<<8;
			mass[4]+=(cvs10_r_bkp[3]&1)<<9;
	
			for (i1=0;i1<6;i1++) mass[3]+=((cvs10_r_bkp[5]>>i1)&1)<<(11-i1);
			for (i1=0;i1<4;i1++) mass[3]+=((cvs10_r_bkp[7]>>i1)&1)<<(5-i1);

			//переворачиваем исправность ИВЭП
			mass[3]=mass[3]^0x03f8;
			//NE RABOTAET TVK
//			mass[4]=mass[4]|0x0400;
//			if(dev_mmk->tx_B[7]&0x8000) mass[4]=mass[4]|0xf000;else mass[4]=mass[4]&0x0fff;
			
            //printf("    mass[3]=%x  mass[4]=%x  \n",mass[3],mass[4]);
			}
	   //delay(1); //delay after send bite	  
	  }
	return 0;
}