// Beware: Lines marked compulsory1-20 must be there for this simulation software to
// function properly !!  
// The user can change the other lines according to his/her simulation and must then 
// save the file under its own name. 

/*********************************************************************************/
/**********         		Gyro Telescope Seeker               **********/
/*********************************************************************************/

#pragma hdrstop							// compulsory1
#include "SIMPROTO_2017.H"                                          	// compulsory2
#include "SIMINCDF_2017.H"                                          	// compulsory3
#include "SIMULDCL_2017.H"                                      	// compulsory4
#include "Graphics.h"                                           	// compulsory5

/*********************************************************************************/
/**********       	PARAMETERS, CONSTANTS & VARIABLES           	**********/
/*********************************************************************************/

//***	Setting model parameters	
// Gimbal Parameters
const double Ixxg    = 0.0267;   //kgm^2
const double Iyyg    = 0.0159;   //kgm^2
const double Izzg    = 0.0123;   //kgm^2
const double Iyzg    = 0.0;
const double Izyg    = 0.0;

// Platform Parameters
const double Ixxp    = 0.0048;   //kgm^2
const double Iyyp    = 0.0164;   //kgm^2
const double Izzp    = 0.0166;   //kgm^2
const double Ixzp    = 0.0009;   //kgm^2
const double Izxp    = 0.0009;   //kgm^2

// Gimbal Parameters
/*const double Ixxg    = 0.02573;   //kgm^2
const double Iyyg    = 0.01553;   //kgm^2
const double Izzg    = 0.0174;    //kgm^2
const double Iyzg    = 0.00042;
const double Izyg    = 0.00042;

// Platform Parameters
const double Ixxp    = 0.003278;   //kgm^2
const double Iyyp    = 0.008858;   //kgm^2
const double Izzp    = 0.009268;   //kgm^2
const double Ixzp    = -0.000003;   //kgm^2
const double Izxp    = -0.000003;   //kgm^2*/

// Motor Parameters
const double Vm      = 24.0;           //V

// Faulhaber 3257 024 CR (73 mNm)
const double Lm      = 270e-6;               //H
const double Rm      = 1.63;                 //Ohm
const double Km      = 37.7e-3;              //Nm/A

const double Stic_p  = 1.0*0.018;                    //Nm
// Original
//const double Fric_p  = 0.3*Stic_p;               //Nm
//const double K_fr_p  = 7.0*(30.0/(10.9*1000.0*pi));  //Nm/(rad/s)
// Adjusted from model verification
const double Fric_p  = 0.7*Stic_p;               //Nm
const double K_fr_p  = 1.5*(30.0/(10.9*1000.0*pi));  //Nm/(rad/s)

const double Stic_y  = 0.041;               //Nm
// Original
//const double Fric_y  = 0.6*Stic_y;               //Nm
//const double K_fr_y  = 2.0*(30.0/(10.9*1000.0*pi));               //Nm/(rad/s)
// Adjusted from model verification
const double Fric_y  = 0.7*Stic_y;               //Nm
const double K_fr_y  = 2.0*(30.0/(10.9*1000.0*pi));               //Nm/(rad/s)

// Control Parameters
// Stab
const double Kpr	 = 1800;
const double Kyr     = 2550;
const double p_pr    = 1.0/(2.0*pi*3.0);
const double p_yr    = 1.0/(2.0*pi*3.0);//2.3);
const double T1      = 0.0;
const double T2      = 1.0/(2.0*pi*150.0);
const double T3      = 0.0;
const double T4      = 1.0/(2.0*pi*150.0);

// Track
const double Kpt     = 6.0;
const double Kyt     = 6.0;
const double Ta      = 0.0;
const double Tb      = 1.0/(2.0*pi*10.0);
const double Tc      = 0.0;
const double Td      = 1.0/(2.0*pi*10.0);
const double tau     = 0.0/(2.0*pi*2.0);
const double Tsamp 	 = 0.002;

// Gyro Parameters
const double wg      = 2*pi*1000.0;
const double wf      = 2*pi*188.0;
const double zg      = 0.7;
const double N_rms 	 = 0.0015;

const double pi_4[]  = {pi/4, pi/4, pi/4};
double zero[]  = {0, 0, 0};
double Xtarg[3],Ytarg[3],Ztarg[3];	//	m	Target position, relative to Seeker
double Wxb[3], Wyb[3], Wzb[3], Txb[3];	//	rad/s	Base motion rates
double Wxp[3], Wyp[3], Wzp[3];
double Ydiff[3], Pdiff[3], Pdiff_p[3], Pte[3], Yte[3];		//	rad		
		//	Yaw and Pitch difference between target and telescope line of sight
double Psi[3], Psi_M[3], The[3], The_M[3];	//	rad	Seeker internal angles (yaw and pitch)

double Psi_i[3], Phi_i[3], The_i[3];
double Psi_i_dot[3], Phi_i_dot[3], The_i_dot[3];
double B1[3], B2[3], B3[3];

/************.0*********************************************************************/
/**********                     	MAIN PROGRAM                        **********/
/*********************************************************************************/

#pragma argsused  						// compulsory6
		// pragma suppresses "calling argument not used" warnings	
BOOL WINAPI DllEntryPoint(						// compulsory7
HINSTANCE hinst, DWORD reason, LPVOID reserved)	       		// compulsory8
{								// compulsory9
return TRUE;							// compulsory10
}								// compulsory11
__declspec(dllexport)						// compulsory12

void SIMUL_C(void)                                        		// compulsory13
{                                                                   	// compulsory14

/**********       		INITIALIZATION 			**********/

  SETSIGNAL(0.0,Phi_i);		//	rad	Phi_i	Initial Phi angle of base
  SETSIGNAL(0.0,Psi_i);		//	rad	Psi_i	Initial Psi angle of base
  SETSIGNAL(0.0,The_i);		//	rad	The_i	Initial The angle of base	
  SETSIGNAL(pi/4,The);
  SETSIGNAL(-pi/4,Psi);
  

/*****************************************************************************/
/**********                     PROGRAM LOOP                        **********/
/*****************************************************************************/

  for (Tsimulc=0.1*TDELT; Tsimulc<TEND; Tsimulc=Tsimulc+TDELT)  	// compulsory15
  {                                                               	// compulsory16
    JMsimulc=-1; JSsimulc=-1; JDsimulc=-1; JDLHsimulc=-1;         	// compulsory17
	

    //ZSTEP(0.0,0.0,V5);											//Use with Comparator. Comment out with Rel Geom
    //ZSTEP(0.0,0.0,V35);											//Use with Comparator. Comment out with Rel Geom
	//BODECALC(Wyb,1.1,Wyb,Wyp);

    /*A2D(V2500, Tsamp, -2*pi, 2*pi, 65535L, V2);					//Use with Comparator. Comment out with Rel Geom
	A2D(V2530, Tsamp, -2*pi, 2*pi, 65535L, V32);					//Use with Comparator. Comment out with Rel Geom
	
	A2D(V2200, Tsamp, -2*pi, 2*pi, 65535L, V1);						//Use with Comparator. Comment out with Rel Geom
	A2D(V2230, Tsamp, -2*pi, 2*pi, 65535L, V31);					//Use with Comparator. Comment out with Rel Geom*/

//***	Target position
	CREATESIGNAL(1000.0,Xtarg);
	CREATESIGNAL(-1000,Ytarg);
	CREATESIGNAL(-1414.2,Ztarg);
	
//***	Base motion rates
	
	/*FMLINSWP(0.505,pi,1.5,1.5,1.5,7,Wxb);
	FMLINSWP(0.0,pi,1.5,3.5,1.5,9.5,Wyb);
	FMLINSWP(0.0,pi,1.0,6,5,12,Wzb);*/

    FMLINSWP(0.25,pi,1.0,1.5,5.0,7,Wxb);
	FMLINSWP(0.25,pi,1.0,3.5,5.0,9.5,Wyb);
	FMLINSWP(0.25,pi,1.0,6.0,5.0,12,Wzb);

	// Simulated base motion signal
	/*SINE(0.0,0.00518,pi/2,0.0014,V700);
	SINE(0.0,0.00643,pi/2,0.0383,V701);
	SUM(V700,V701,V702);	
	SINE(0.0,0.00602,pi/2,0.0790,V703);
	SUM(V702,V703,V704);
	SINE(0.0,0.00610,pi/2,0.1631,V705);
	SUM(V704,V705,V706);
	SINE(0.0,0.00133,pi/2,0.6151,V707);
	SUM(V706,V707,V708);
	SINE(0.0,0.00495,pi/2,1.2644,V709);
	SUM(V708,V709,V710);
	SINE(0.0,0.00069,pi/2,49.9481,V711);
	SUM(V710,V711,V712);
	SINE(0.0,0.00041,pi/2,75.3132,V713);
	SUM(V712,V713,V714);
	SINE(0.0,0.00049,pi/2,96.5739,V715);
	SUM(V714,V715,V716);
	GAUSSNOISE(0.0,0.0,0.0058,V717);
	SUM(V716,V717,Wxb);
	
	SINE(0.0,0.01085,pi/2,0.0057,V720);
	SINE(0.0,0.01130,pi/2,0.0547,V721);
	SUM(V720,V721,V722);	
	SINE(0.0,0.00822,pi/2,0.1548,V723);
	SUM(V722,V723,V724);
	SINE(0.0,0.00239,pi/2,1.2760,V725);
	SUM(V724,V725,V726);
	SINE(0.0,0.00019,pi/2,14.5354,V727);
	SUM(V726,V727,V728);
	SINE(0.0,0.00018,pi/2,21.7833,V729);
	SUM(V728,V729,V730);
	SINE(0.0,0.00043,pi/2,41.0057,V731);
	SUM(V730,V731,V732);
	SINE(0.0,0.00036,pi/2,53.0976,V733);
	SUM(V732,V733,V734);
	SINE(0.0,0.00023,pi/2,96.9980,V735);
	SUM(V734,V735,V736);
	GAUSSNOISE(0.0,0.0,0.0018,V737);
	SUM(V736,V737,Wyb);

	SINE(0.0,0.01657,pi/2,0.0058,V740);
	SINE(0.0,0.03119,pi/2,0.0202,V741);
	SUM(V740,V741,V742);	
	SINE(0.0,0.02924,pi/2,0.0401,V743);
	SUM(V742,V743,V744);
	SINE(0.0,0.02594,pi/2,0.0781,V745);
	SUM(V744,V745,V746);
	SINE(0.0,0.01028,pi/2,0.1507,V747);
	SUM(V746,V747,V748);
	SINE(0.0,0.00315,pi/2,1.1630,V749);
	SUM(V748,V749,V750);
	SINE(0.0,0.00098,pi/2,5.4172,V751);
	SUM(V750,V751,V752);
	SINE(0.0,0.00067,pi/2,36.1728,V753);
	SUM(V752,V753,V754);
	SINE(0.0,0.00020,pi/2,87.2822,V755);
	SUM(V754,V755,V756);
	GAUSSNOISE(0.0,0.0,0.0025,V757);
	SUM(V756,V757,Wzb);*/

//***	Relative Geometry
	MATASS( 0.0,	sin(Phi_i[0])/cos(The_i[0]),	cos(Phi_i[0])/cos(The_i[0]),	
		0.0,	cos(Phi_i[0]),	-sin(Phi_i[0]),							
		1.0,	sin(Phi_i[0])*tan(The_i[0]),	cos(Phi_i[0])*tan(The_i[0]), M10);
	MVMULT(M10,Wxb,Wyb,Wzb,Psi_i_dot,The_i_dot,Phi_i_dot);
	INTEGR(Psi_i_dot,1.0,Psi_i);
	INTEGR(The_i_dot,1.0,The_i);
	INTEGR(Phi_i_dot,1.0,Phi_i);
	
	MATASS(	cos(The[0]),	0.0,	-sin(The[0]),
		0.0,		1.0,	0.0,		
		sin(The[0]),	0.0,	cos(The[0]), M1);
		
	MATASS(	cos(Psi[0]),	sin(Psi[0]),	0.0,
		-sin(Psi[0]),	cos(Psi[0]),	0.0,	
		0.0,		0.0,		1.0, M2);

	MATASS(	1.0,	0.0,		0.0,
		0.0,	cos(Phi_i[0]),	sin(Phi_i[0]),	
		0.0,	-sin(Phi_i[0]),	cos(Phi_i[0]), M3);
		
	MATASS(	cos(The_i[0]),	0.0,	-sin(The_i[0]),
		0.0,		1.0,	0.0,		
		sin(The_i[0]),	0.0,	cos(The_i[0]), M4);
		
	MATASS(	cos(Psi_i[0]),	sin(Psi_i[0]),	0.0,
		-sin(Psi_i[0]),	cos(Psi_i[0]),	0.0,	
		0.0,		0.0,		1.0, M5);
	
	MMMULT(M1,M2,M6);
	MMMULT(M6,M3,M7);
	MMMULT(M7,M4,M8);
	MMMULT(M8,M5,M9);

	MVMULT(M9,Xtarg,Ytarg,Ztarg,B1,B2,B3);
	
	CREATESIGNAL(-atan2(B3[0],B1[0]),Pte);
	CREATESIGNAL(atan2(B2[0],B1[0]),Yte);
	
	//DELAY(Pte,0.095,V2);
	//DELAY(Yte,0.095,V32);

	//A2D(V2, Tsamp, -pi/10, pi/10, 65535L, V3);					//Use with Rel Geom. Comment out with Comparator
	//A2D(V32, Tsamp, -pi/10, pi/10, 65535L, V33);					//Use with Rel Geom. Comment out with Comparator
	
	A2D(V27, Tsamp, -250*pi/180.0, 250*pi/180.0, 65535L, V6);		//Wyp measured
	A2D(V121, Tsamp, -250*pi/180.0, 250*pi/180.0, 65535L, V36);		//Wzp measured
	

//***	Pitch Track Controller
	//ZDIFF(V1,V2,Pte);												//Use with Comparator. Comment out with Rel Geom
	ZDELAY(Pte,Tsamp,0.095,V3);										//Use with Comparator. Comment out with Rel Geom
	ZGAIN(V3, Kpt, V4);														
	ZLEADLAG(V4,Tsamp+2.0*Ta,Tsamp-2.0*Ta,Tsamp+2.0*Tb,Tsamp-2.0*Tb,V5);		

//***	Yaw Track Controller
	//DIFF(V31,V32,Yte);											//Use with Comparator. Comment out with Rel Geom
	ZDELAY(Yte,Tsamp,0.095,V33);									//Use with Comparator. Comment out with Rel Geom
	ZGAIN(V33, Kyt, V34);														
	ZLEADLAG(V34,Tsamp+2.0*Tc,Tsamp-2.0*Tc,Tsamp+2.0*Td,Tsamp-2.0*Td,V35);

//***	Pitch Stab Controller and H-Bridge Model
	ZDIFF(V5,V6,V7);
	ZLEADLAG(V7,Tsamp+2.0*p_pr,Tsamp-2.0*p_pr,2.0,-2.0,V8);
	ZLIMIT(+1.1*Vm/Kpr,-1.1*Vm/Kpr,V8);
	ZGAIN(V8,Kpr*2376.0/Vm,V9);
	ZLEADLAG(V9,Tsamp +2.0*T1,Tsamp-2.0*T1,Tsamp+2.0*T2,Tsamp-2.0*T2,V10);
	ZLIMIT(+2376,-2376,V10);
	ZGAIN(V10,Vm/2400,V11);
	D2A(V11,V12);

//***	Yaw Stab Controller and H-Bridge Model
	ZDIFF(V35,V36,V37);
	ZLEADLAG(V37,Tsamp+2.0*p_yr,Tsamp-2.0*p_yr,2.0,-2.0,V38);
	ZLIMIT(+1.1*Vm/Kyr,-1.1*Vm/Kyr,V38);
	ZGAIN(V38,Kyr*2376.0/Vm,V39);
	ZLEADLAG(V39,Tsamp+2.0*T3,Tsamp-2.0*T3,Tsamp+2.0*T4,Tsamp-2.0*T4,V40);
	ZLIMIT(+2376,-2376,V40);
	ZGAIN(V40,Vm/2400,V41);
	D2A(V41,V42);
			
//***	Dynamics

  // 	Base motion effects
   
  	DERIV(Wxb,1.0,V100);
	DERIV(Wyb,1.0,V101);
	COSGAIN(V100,Psi,V102);
	SINGAIN(V101,Psi,V103);
	COSGAIN(Wyb,Psi,V105);
	SINGAIN(Wxb,Psi,V106);
	COSGAIN(Wxb,Psi,V107);
	SINGAIN(Wyb,Psi,V108);
	
	SUM(V102,V103,V104);
	DIFF(V105,V106,V22);
	SUM(V107,V108,V109);
	
	GAIN(V22,V62[0],V110);
	
	SUM(V110,V104,V111);

	SINGAIN(V61,The,V112);
	COSGAIN(V61,The,V113);
	SINGAIN(V109,The,V114);
	COSGAIN(V109,The,V115);
	DIFF(V115,V112,V117);//W_xp
	SUM(V113,V114,V116);//W_zp
	
  //	Disturbance torques
  
	COSGAIN(V111,The,V125);
	SINGAIN(V125,The,V126);
	GAIN(V126,Izzp-Ixxp,V47);
	
	GAIN(V22,V109[0],V127);
	GAIN(V127,Iyyg-Ixxg,V48);
	
	GAIN(V117,V21[0],V128);
	COSGAIN(V128,The,V129);
	GAIN(V129,Iyyp-Ixxp,V50);
	
	GAIN(V116,V21[0],V130);
	SINGAIN(V130,The,V131);
	GAIN(V131,Iyyp-Izzp,V52);
	
	GAIN(V117,V23[0],V132);
	COSGAIN(V132,The,V133);
	GAIN(V133,Izzp,V54);
	
	GAIN(V116,V23[0],V134);
	SINGAIN(V134,The,V135);
	GAIN(V135,Ixxp,V56);
	
	GAIN(V117,V116[0],V136);
	GAIN(V136,Ixxp-Izzp,V17);
	
	double Ieq = Izzg+(Izzp)*cos(The[0])*cos(The[0])+Ixxp*sin(The[0])*sin(The[0]); 

//***	Pitch Motor

	DIFF(V12,V13,V14);
	LEADLAG(V14,0.0,1.0,Lm,Rm,V15);
	GAIN(V15,Km,V16);
	DIFF(V16,V17,V18);
	STICLIM(V18,V22,Stic_p,Fric_p,K_fr_p,Iyyp,200.0*pi/180.0,-20.0*pi/180.0,V19,V20,V21,V23,The);
	GAIN(V23,Km,V13);

//***	Yaw Motor

	DIFF(V42,V43,V44);
	LEADLAG(V44,0.0,1.0,Lm,Rm,V45);
	GAIN(V45,Km,V46);

	SUM(V47,V48,V49);
	SUM(V49,V50,V51);
	SUM(V51,V52,V53);
	SUM(V53,V54,V55);
	SUM(V55,V56,V57);

	DIFF(V46,V57,V58);

	STICLIM(V58,Wzb,Stic_y,Fric_y,K_fr_y,Ieq,2*pi,-2*pi,V59,V60,V61,V62,Psi);
	GAIN(V62,Km,V43);
	
//***	Pitch Feedback

	
	BIQUAD(V21,0.0,0.0,wf*wf,1.0,2.0*wf*zg,wf*wf,V24);
	GAUSSNOISE(0.0,0.0,N_rms,V25);
	SUM(V24,V25,V26);
	DELAY(V26,0.0019,V27);
	INTEGR(V27,1.0,V2500);

//***	Yaw Feedback
	
	
	BIQUAD(V116,0.0,0.0,wf*wf,1.0,2.0*wf*zg,wf*wf,V118);
	GAUSSNOISE(0.0,0.0,N_rms,V119);
	SUM(V118,V119,V120);
	DELAY(V120,0.0019,V121);
	INTEGR(V121,1.0,V2530);

	SINGAIN(V117,The,V2400);
	COSGAIN(V116,The,V2401);
	DIFF(V2401,V2400,V124);											// Analogue version of w_zg (V36)

	//RMS(Wzb,0.0,V2998);
	//RMS(V116,0.0,V2999);

	Wyp[0] = V21[0];
	Wxp[0] = V117[0];
	Wzp[0] = V116[0];

/**********                  	GRAPHING SECTION                  	**********/
  
  	

 	
	YTDRAW(Wxb, 	V0,  	V0,
			Wyb, 	V0,  	V0,
			Wzb, 	V0,  	V0,	
			Wyp, 	V0, 	V0,
			Wzp, 	Wxp,  	V124,
			V12,	V0, 	V0, 
			V42,	V0, 	V0, 
			Pte, 	V0,  	V0,
			Yte, 	V0,		V0); 

	/*YTDRAW(Pte, 	V0, 	V0,
			Yte, 	V0,		V0,
			Ztarg, 	V0,		V0,	
			Yte, 	V15,		V0,
			Pte, 	V45,		V0,
			V0, 	V0,		V0,
			V0, 	V0,		V0,
			V0, 	V0,		V0,
			V0, 	V0,		V0);
   YTDRAW(V5, 	V6,  	V7,
			V35, 	V36,  	V37,
			V0, 	V0,  	V0,	
			V0, 	V0,  	V0,
			V0, 	V0,  	V0,mode
			V0, 	V0,  	V0,
			V0, 	V0,  	V0,
			V0, 	V0,  	V0,
			V0, 	V0,		V0);

	YTDRAW(V1, 	V2,  	Pte,
			V31, 	V32,  	Yte,
			V0, 	V0,  	V0,	
			V0, 	V0,  	V0,
			V0, 	V0,  	V0,
			V0, 	V0,  	V0,
			V0, 	V0,  	V0,
			V0, 	V0,  	V0,
			V0, 	V0,		V0);
			
   
			
	YTDRAW(	V1000, 	V1001, 	Pdiff,
			V3, 	V4,  	V5,	
			Psi_i_dot0, 	Psi_i_dot1, 	Ydiff,
			V33, 	V34,  	V35,
			V15, 	V0,  	V0,
			V55, 	V0,  	V0,
			V12, 	V0,  	V0,
			V42, 	V0,		V0,
			The, 	Psi,  	V0);


			
	YTDRAW(	Xtarg, 	Ytarg, 	Ztarg,
			Pdiff, 	Ydiff, 	V0,
			V2605, 	V0,  	V0,	
			V2606, 	V0,  	V0,
			Wyb, 	V0,  	V0,
			V18, 	V0,  	V0,
			The, 	V0,  	V0,
			Psi, 	V0,  	V0,
			V55, 	V0,		V0);

	YTDRAW(	Xtarg, 	Ytarg, 	Ztarg,
			Pdiff_z, 	Ydiff_z, 	V0,
			V12, 	V0,  	V0,	
			V42, 	V0,  	V0,
			V13, 	V0,  	V0,
			V43, 	V0,  	V0,
			The, 	V0,  	V0,
			Psi, 	V0,  	V0,
			V55, 	V0,		V0);

	YTDRAW(	The, 	The_M,  	V0,
			Psi, 	Psi_M,  	V0,
			Wxb, 	V0,  	V0,	
			V21, 	V0,  	V0,
			V116, 	V0,  	V0,
			The, 	V0,  	V0,
			Psi, 	V0,  	V0,
			V16, 	V0,  	V0,
			V46, 	V0,		V0);
	YTDRAW(	V35, 	V36,  	V37,
			V5, 	V6,  	V7,
			Psi, 	The,  	V0,	
			V21, 	V0,  	V0,
			V116, 	V0,  	V0,
			The, 	V0,  	V0,
			Psi, 	V0,  	V0,
			V16, 	V0,  	V0,
			V46, 	V0,		V0);*/


    XYDRAW(	V0, 	V0,
			V0, 	V0,
			V0, 	V0,
			V0, 	V0,
			V0, 	V0,
			V0, 	V0,
			V0, 	V0,
			V0, 	V0,
			V0, 	V0,
			V0, 	V0,
			V0, 	V0);
			
/*********************************************************************************/

  }	// End of PROGRAM LOOP                                 	// compulsory18
}	// End of void SIMUL_C(void)  				// compulsory19

#pragma package(smart_init)					// compulsory20

/*********************************************************************************/
/**********                       END OF PROGRAM                   	**********/
/*********************************************************************************/
