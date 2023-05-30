#include "stepFall.h"
#include "calSleepTime.h"
#include "alpsfall.h"
#define cJOI (210) 
#define aDZS (180)
#define cAMA (50)
#define aQCU (5)
#define cMPC (0.30f)
#define aABO (1.2f)
#define bWNW (10)
#define aMEQ (15)
#define cIQY
#ifdef cIQY
#define aDCI (128)
#define bZOQ (0)
#define aPFK (43)
#define cLRS (0)
#define aAVG (0)
#define bXHO (21)
#define aMYI (64)
#define cJKQ (64)
#define aDWA (21)
#define cAII (0)
#endif 
typedef struct {TUInt8 aPZC;TInt32 cMLK;TInt32 aAGM;}bWSU;
#ifdef cIQY
typedef struct {TUInt8 aMJO;TUInt8 aDVS;TInt32 cIVW[aQCU];TInt32 aDHG[aQCU];}bZTO;
#endif 
typedef struct {TUInt8 aPKI;TInt32 cLWQ[aQCU];}aBAE;typedef struct {TUInt8 bXMM;TInt16 aNDG;TInt16 cJPO;TInt32 aEAY;TInt32 cANG
;TInt32 aQEA;TInt32 cMQI;TInt32 aAAC;TInt32 bWMK;TInt32 aMDE;TInt32 cIPM;}aDAW;typedef struct {TUInt8 bZNE;TInt16 aPDY;TInt16 
cLQG;TInt16 aATU;TInt16 bXGC;TInt16 aMWW;TInt16 cJJE;TInt32 steps;TInt32 aDUO;}cAGW;typedef struct {TUInt8 aPXQ;TUInt16 cMJY;
TUInt16 aAFA;TUInt32 bWRI;TUInt32 aMIC;}cIUK;typedef struct {TUInt8 wear_flag;TUInt32 aDFU;TUInt32 bZSC;TUInt32 aPIW;}cLVE;
typedef struct {TUInt8 aAYS;TUInt32 bXLA;TUInt32 aNBU;}cJOC;typedef struct {
#ifdef aDZM 
TInt16 acc_x;TInt16 acc_y;TInt16 acc_z;TInt32 cALU;TInt32 aQCO;
#endif
bWSU cMOW;
#ifdef cIQY
bZTO aABI;
#endif 
aBAE bWNQ;aDAW aMEK;cAGW cIQS;cIUK aDCC;cLVE bZOK;cJOC aPFE;}cLRM,*aAVA;static cLRM bXHI ={0 };static TInt32 cIPS =512;static 
TUInt32 aMYC =0;static void cJKK(void);static TUInt8 aDVU(TInt32 cAIC);static void aPYW(void);static TInt32 cMLE(TInt32 aAGG);
static void bWSO(void);static void aMJI(TInt32 cIVQ);static TInt32 aDHA(void);static TInt32 bZTI(void);static TInt32 aPKC(void)
;static void cLWK(void);static void aAZY(void);static void bXMG(void);static void aNDA(void);static void cJPI(void);static void
aEAS(TInt32 aEAQ);static void cANA(void);static void aQDU(TUInt32 cMQC);static void aAAK(void);static void bWMS(void);void 
set_acc_sens(TInt32 sens){cIPS =sens;cJKK();aPYW();bWSO();cLWK();init_judege_walking(0);set_dect_step_threshold(4,0.8f,cMPC);
set_wearthreshold(bWNW,aABO);}static void cJKK(void){bXHI.cMOW.aPZC =0;bXHI.cMOW.aAGM =(2 *cIPS);bXHI.cMOW.cMLK =0;}static 
TUInt8 aDVU(TInt32 cAIC){if (bXHI.cMOW.aPZC ==0){if (bXHI.cMOW.cMLK <cAIC){bXHI.cMOW.cMLK =cAIC;}if (bXHI.cMOW.aAGM >cAIC){bXHI
.cMOW.aAGM =cAIC;}if (bXHI.cMOW.cMLK -bXHI.cMOW.aAGM >(TInt32)(0.5*cIPS)){bXHI.cMOW.aPZC =1;}}return bXHI.cMOW.aPZC;}static 
void aPYW(void){
#ifdef cIQY
bXHI.aABI.aMJO =0;bXHI.aABI.aDVS =0;
#endif
}static TInt32 cMLE(TInt32 aAGG){TInt32 aMDM;TInt32 cIPU;
#ifdef cIQY
if (bXHI.aABI.aDVS ==0){cIPU =aAGG;}else {if (bXHI.aABI.aDVS ==1){aMDM =bXHO *aAGG;aMDM +=aMYI *bXHI.aABI.aDHG[0];aMDM -=bZOQ *
bXHI.aABI.cIVW[0];}else if (bXHI.aABI.aDVS ==2){aMDM =bXHO *aAGG;aMDM +=aMYI *bXHI.aABI.aDHG[1];aMDM -=bZOQ *bXHI.aABI.cIVW[1];
aMDM +=cJKQ *bXHI.aABI.aDHG[0];aMDM -=aPFK *bXHI.aABI.cIVW[0];}else if (bXHI.aABI.aDVS ==3){aMDM =bXHO *aAGG;aMDM +=aMYI *bXHI.
aABI.aDHG[2];aMDM -=bZOQ *bXHI.aABI.cIVW[2];aMDM +=cJKQ *bXHI.aABI.aDHG[1];aMDM -=aPFK *bXHI.aABI.cIVW[1];aMDM +=aDWA *bXHI.
aABI.aDHG[0];aMDM -=cLRS *bXHI.aABI.cIVW[0];}else {TInt8 aMJO =bXHI.aABI.aMJO;aMDM =bXHO *aAGG;aMJO =(--aMJO <0)?(aQCU -1):(
aMJO);aMDM +=aMYI *bXHI.aABI.aDHG[aMJO];aMDM -=bZOQ *bXHI.aABI.cIVW[aMJO];aMJO =(--aMJO <0)?(aQCU -1):(aMJO);aMDM +=cJKQ *bXHI.
aABI.aDHG[aMJO];aMDM -=aPFK *bXHI.aABI.cIVW[aMJO];aMJO =(--aMJO <0)?(aQCU -1):(aMJO);aMDM +=aDWA *bXHI.aABI.aDHG[aMJO];aMDM -=
cLRS *bXHI.aABI.cIVW[aMJO];;aMJO =(--aMJO <0)?(aQCU -1):(aMJO);aMDM +=cAII *bXHI.aABI.aDHG[aMJO];aMDM -=aAVG *bXHI.aABI.cIVW[
aMJO];;}if (aMDM >=0){cIPU =((TUInt32)aMDM +64)>>7;}else {cIPU =-((-(aMDM -64))>>7);}}if (bXHI.aABI.aDVS <aQCU){bXHI.aABI.aDVS
++;}bXHI.aABI.aDHG[bXHI.aABI.aMJO]=aAGG;bXHI.aABI.cIVW[bXHI.aABI.aMJO]=cIPU;bXHI.aABI.aMJO++;if (bXHI.aABI.aMJO >=aQCU){bXHI.
aABI.aMJO =0;}
#else
cIPU =aAGG;
#endif 
return cIPU;}static void bWSO(void){TUInt8 cLVC;for (cLVC =0;cLVC <aQCU;cLVC++){bXHI.bWNQ.cLWQ[cLVC]=0;}bXHI.bWNQ.aPKI =0;}
static void aMJI(TInt32 cIVQ){TUInt8 cLVC;if (bXHI.bWNQ.aPKI <aQCU){bXHI.bWNQ.cLWQ[bXHI.bWNQ.aPKI]=cIVQ;bXHI.bWNQ.aPKI++;}else 
{for (cLVC =0;cLVC <aQCU -1;cLVC++){bXHI.bWNQ.cLWQ[cLVC]=bXHI.bWNQ.cLWQ[cLVC +1];}bXHI.bWNQ.cLWQ[aQCU -1]=cIVQ;}}static TInt32 
aDHA(void){TInt32 aDBE =0;aDBE =2 *(bXHI.bWNQ.cLWQ[4]-bXHI.bWNQ.cLWQ[0])+(bXHI.bWNQ.cLWQ[3]-bXHI.bWNQ.cLWQ[1]);return aDBE;}
static TInt32 bZTI(void){TInt32 cLVC =0;TInt32 bZNM =bXHI.bWNQ.cLWQ[0];for (cLVC =1;cLVC <aQCU;cLVC++){if (bZNM >bXHI.bWNQ.cLWQ
[cLVC]){bZNM =bXHI.bWNQ.cLWQ[cLVC];}}return bZNM;}static TInt32 aPKC(void){TInt32 cLVC =0;TInt32 aPEG =bXHI.bWNQ.cLWQ[0];for (
cLVC =1;cLVC <aQCU;cLVC++){if (aPEG <bXHI.bWNQ.cLWQ[cLVC]){aPEG =bXHI.bWNQ.cLWQ[cLVC];}}return aPEG;}static void cLWK(void){
bXHI.aMEK.bXMM =0;bXHI.aMEK.aEAY =0;bXHI.aMEK.cMQI =0;bXHI.aMEK.aAAC =0;bXHI.aMEK.aQEA =(2 *cIPS);bXHI.aMEK.cANG =0;bXHI.aMEK.
cJPO =0;bXHI.aMEK.aNDG =0;}void set_dect_step_threshold(float maxHz,float minHz,float amplitude){if (maxHz >6)maxHz =6;if (
minHz <0.6)minHz =0.6f;if (maxHz <=minHz){maxHz =4;minHz =0.8f;}if (amplitude <0.15f){amplitude =0.15f;}bXHI.aMEK.bWMK =(TInt32
)(cAMA /maxHz +0.5f);bXHI.aMEK.aMDE =(TInt32)(cAMA /minHz +0.5f);bXHI.aMEK.cIPM =(TInt32)(amplitude*cIPS);}static void aAZY(
void){TInt32 aDBE =0;aDBE =aDHA();bXHI.aMEK.cJPO++;bXHI.aMEK.bXMM =0;bXHI.aMEK.cMQI =0;if ((bXHI.aMEK.aEAY <0)&&(aDBE >0)){if (
bXHI.aMEK.cJPO >=bXHI.aMEK.bWMK){bXHI.aMEK.aQEA =bZTI();if ((bXHI.aMEK.cANG >0)&&(bXHI.aMEK.aQEA >0)){bXHI.aMEK.cMQI =bXHI.aMEK
.cANG -bXHI.aMEK.aQEA;}if (bXHI.aMEK.cMQI >=bXHI.aMEK.cIPM){bXHI.aMEK.bXMM =1;bXHI.aMEK.cJPO =0;bXHI.aMEK.aNDG =1;bXHI.cIQS.
cJJE++;}}}else if ((bXHI.aMEK.aEAY >0)&&(aDBE <0)){bXHI.aMEK.cANG =aPKC();}bXHI.aMEK.aEAY =aDBE;if (bXHI.cIQS.bZNE ==0){if (
bXHI.aMEK.aNDG){bXHI.aMEK.aNDG++;if (bXHI.aMEK.aNDG >bXHI.aMEK.aMDE){cJKK();aPYW();bWSO();cLWK();init_judege_walking(0);aNDA();
}}}}void set_judge_walking_threshold(float startTime,float endTime,TInt32 judgesteps){if (startTime <6){startTime =6;}if (
endTime <2.5){endTime =2.5;}if (judgesteps <6){judgesteps =6;}if (judgesteps >3 *startTime){judgesteps =(TInt32)(3 *startTime);
}bXHI.cIQS.aATU =(TInt16)(cAMA*startTime);bXHI.cIQS.bXGC =(TInt16)(cAMA*endTime);bXHI.cIQS.aMWW =judgesteps;}void 
init_judege_walking(TUInt8 cLQO){bXHI.cIQS.bZNE =0;bXHI.cIQS.aPDY =0;bXHI.cIQS.cLQG =0;bXHI.cIQS.cJJE =0;if (cLQO){bXHI.cIQS.
steps =0;bXHI.cIQS.aDUO =0;}}static void bXMG(void){if (bXHI.cIQS.bZNE ==0){if ((bXHI.aMEK.bXMM)&&(bXHI.cIQS.aPDY ==0)){bXHI.
cIQS.aPDY =1;}else if (bXHI.cIQS.aPDY){bXHI.cIQS.aPDY++;}if (bXHI.cIQS.aPDY >=bXHI.cIQS.aATU){if (bXHI.cIQS.cJJE >=bXHI.cIQS.
aMWW){bXHI.cIQS.bZNE =1;}else {cJKK();aPYW();bWSO();cLWK();init_judege_walking(0);aNDA();}}}else {if (bXHI.aMEK.bXMM){bXHI.cIQS
.cLQG =1;}else if (bXHI.cIQS.cLQG){bXHI.cIQS.cLQG++;}if (bXHI.cIQS.cLQG >bXHI.cIQS.bXGC){cJKK();aPYW();bWSO();cLWK();
init_judege_walking(0);aNDA();}}if (bXHI.cIQS.bZNE){bXHI.cIQS.steps +=bXHI.cIQS.cJJE;bXHI.cIQS.cJJE =0;}}static void aNDA(void)
{bXHI.aDCC.aPXQ =0;bXHI.aDCC.bWRI =0;bXHI.aDCC.cMJY =0;bXHI.aDCC.aAFA =0;bXHI.cIQS.aDUO =bXHI.cIQS.steps;}void 
set_walkstatusthreshold(TInt32 mseconds){if (mseconds <5){mseconds =5;}bXHI.aDCC.aMIC =(mseconds *cAMA);}static void cJPI(void)
{TUInt32 aAUC =0;bXHI.aMEK.aAAC +=bXHI.aMEK.cMQI;if (bXHI.aDCC.bWRI){bXHI.aDCC.bWRI++;}else if (bXHI.aMEK.bXMM ==1){bXHI.aDCC.
bWRI =1;}if (bXHI.cIQS.bZNE){if (bXHI.aDCC.bWRI >=bXHI.aDCC.aMIC){aAUC =bXHI.cIQS.steps -bXHI.cIQS.aDUO;if (aAUC ==0){bXHI.aDCC
.cMJY =0;bXHI.aDCC.aAFA =0;}else {bXHI.aDCC.cMJY =100 *aAUC /(bXHI.aDCC.bWRI /cAMA);bXHI.aDCC.aAFA =100 *bXHI.aMEK.aAAC /aAUC /
cIPS;}bXHI.aDCC.bWRI =1;bXHI.cIQS.aDUO =bXHI.cIQS.steps;bXHI.aMEK.aAAC =0;if ((bXHI.aDCC.cMJY ==0)&&(bXHI.aDCC.aAFA ==0)){bXHI.
aDCC.aPXQ =0;}else if ((bXHI.aDCC.cMJY <cJOI)&&(bXHI.aDCC.aAFA <aDZS)){bXHI.aDCC.aPXQ =1;}else if ((bXHI.aDCC.cMJY >=cJOI)&&(
bXHI.aDCC.aAFA <aDZS)){bXHI.aDCC.aPXQ =2;}else if ((bXHI.aDCC.cMJY <cJOI)&&(bXHI.aDCC.aAFA >=aDZS)){bXHI.aDCC.aPXQ =3;}else if 
((bXHI.aDCC.cMJY >=cJOI)&&(bXHI.aDCC.aAFA >=cJOI)){bXHI.aDCC.aPXQ =4;}}}}static void aEAS(TInt32 aEAQ){TInt32 bXGK =0;if (aDVU(
aEAQ)){bXGK =cMLE(aEAQ);aMJI(bXGK);if (bXHI.bWNQ.aPKI ==aQCU){aAZY();bXMG();cJPI();}}
#ifdef aDZM
bXHI.aQCO =bXGK;
#endif 
}static void cANA(void){bXHI.bZOK.wear_flag =0;bXHI.bZOK.bZSC =0;}void set_wearthreshold(TInt32 minutetime,float move_g){if (
minutetime <10)minutetime =10;if (move_g <1.1)move_g =(float)1.1;bXHI.bZOK.aDFU =(minutetime *60);bXHI.bZOK.aPIW =(TUInt32)(
move_g*cIPS +0.5);}static void aQDU(TUInt32 cMQC){if (cMQC >=bXHI.bZOK.aPIW){bXHI.bZOK.wear_flag =0;bXHI.bZOK.bZSC =0;}else {
bXHI.bZOK.bZSC +=(aMYC*cAMA +1);if ((bXHI.bZOK.bZSC /cAMA)>=bXHI.bZOK.aDFU){bXHI.bZOK.bZSC =0;bXHI.bZOK.wear_flag =1;}}}void 
aMXE(void){bXHI.bZOK.wear_flag =0;bXHI.bZOK.bZSC =0;}static void aAAK(void){bXHI.aPFE.aNBU =0;bXHI.aPFE.aAYS =0;}void 
set_sedentarythreshold(TUInt16 minutetime){if (minutetime <10)minutetime =10;bXHI.aPFE.bXLA =(minutetime *60);}static void bWMS
(void){if (bXHI.bZOK.wear_flag ==0){if (bXHI.cIQS.bZNE){bXHI.aPFE.aAYS =0;bXHI.aPFE.aNBU =0;}else {bXHI.aPFE.aNBU +=(aMYC*cAMA 
+1);if ((bXHI.aPFE.aNBU/cAMA)>=bXHI.aPFE.bXLA){bXHI.aPFE.aNBU =0;bXHI.aPFE.aAYS =1;}}}else {bXHI.aPFE.aAYS =0;bXHI.aPFE.aNBU =0
;}}void clear_sedentary_flag(void){bXHI.aPFE.aAYS =0;bXHI.aPFE.aNBU =0;}
#ifdef cJJM
#define aDUW 2
#define cAHE 16
static float aPXY =0;static TInt32 cMKG(TInt32 aAFI){float bWRQ =0;if (aPXY !=0){bWRQ =1.0*(aPXY*cAHE +aDUW*aAFI -aDUW *aPXY)/
cAHE;}else {bWRQ =aAFI;}aPXY =bWRQ;return (TInt32)(bWRQ +0.5);}
#endif
void ALPSLIB_Init(void){cJKK();aPYW();bWSO();cLWK();init_judege_walking(1);cANA();aAAK();aNDA();set_dect_step_threshold(4,0.8f,
cMPC);set_judge_walking_threshold(10,1.5f,10);set_wearthreshold(bWNW,aABO);set_sedentarythreshold(aMEQ);set_walkstatusthreshold
(10);cMQA();}HumanActivityInfo ALPSLIB_PutData_Time(TInt16 acc_x,TInt16 acc_y,TInt16 acc_z,TUInt32 aMDK,TUInt16 w_year,TUInt8 
w_month,TUInt8 w_date,TUInt8 hour,TUInt8 min,TUInt8 sec){HumanActivityInfo aMIK ={0 };TInt32 aEAQ =0;TUInt8 fall_flag =0;TInt32
pressure =0;aMYC =cLVK(w_year,w_month,w_date,hour,min,sec);
#ifdef cJJM
if (aMYC){aPXY =0;}pressure =cMKG(aMDK);
#else
pressure =aMDK;
#endif 
if(aMYC){cJKK();aPYW();bWSO();cLWK();init_judege_walking(0);}aEAQ =(TInt32)sqrt(acc_x*acc_x +acc_y*acc_y +acc_z*acc_z);aEAS(
aEAQ);aQDU(aEAQ);bWMS();fall_flag =bWMQ(acc_x,acc_y,acc_z,aEAQ,pressure,cIPS);aMIK.steps =bXHI.cIQS.steps;aMIK.walk_run_status 
=bXHI.aDCC.aPXQ;aMIK.wear_flag =bXHI.bZOK.wear_flag;aMIK.sedentary_flag =bXHI.aPFE.aAYS;aMIK.fall_flag =fall_flag;aMIK.
averFrequercy =1.0*bXHI.aDCC.cMJY/100;aMIK.averAmp =1.0*bXHI.aDCC.aAFA/100;
#ifdef aDZM
bXHI.acc_x =acc_x;bXHI.acc_y =acc_y;bXHI.acc_z =acc_z;bXHI.cALU =aEAQ;
#endif 
return aMIK;}DEBUGINFOR cIUS ={0 };PDEBUGINFO get_alg_information(void){cIUS.sens =cIPS;cIUS.d1 =bXHI.aMEK.bWMK;cIUS.d2 =bXHI.
aMEK.aMDE;cIUS.d3 =bXHI.aMEK.cIPM;cIUS.d4 =bXHI.cIQS.aATU;cIUS.d5 =bXHI.cIQS.bXGC;cIUS.d6 =bXHI.cIQS.aMWW;cIUS.d7 =bXHI.bZOK.
aDFU;cIUS.d8 =bXHI.bZOK.aPIW;cIUS.d9 =bXHI.aPFE.bXLA;cIUS.d10 =bXHI.aDCC.aMIC;return &cIUS;}
