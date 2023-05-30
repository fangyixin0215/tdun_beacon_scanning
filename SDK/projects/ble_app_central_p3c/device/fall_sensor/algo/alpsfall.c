#include "alpsfall.h"
#include "commonALG.h"
//#include "log.h"
#define aAAA (300) 
#define bWMI (85)
#define aMDC (512)
#define cIPK (60) 
#define aDAU (10) 
static TInt16 bZNC =128;static TInt8 aPDW =6;static TInt16 cLQE =1200;static TInt8 aATS =5;static int bXGA =0;static int aMWU =
0;typedef struct {TInt16 cJJC;TInt16 aDUM;TInt16 cAGU;TInt16 aPXO;TInt32 pressure;}cMJW;static TUInt8 aAEY =0;static TInt32 
bWRG =0;static TInt32 aMIA[cIPK]={0 };cMJW cIUI[aAAA];
#define aDFS(bZSA) ((bZSA)>0?(bZSA):-(bZSA))
void aPIU(TInt16 cJJC,TInt16 aDUM,TInt16 cAGU,TInt32 aPXO,TInt32 pressure){int cLVC =0;if (bXGA <aAAA){cIUI[bXGA].cJJC =cJJC;
cIUI[bXGA].aDUM =aDUM;cIUI[bXGA].cAGU =cAGU;cIUI[bXGA].aPXO =aPXO;cIUI[bXGA].pressure =pressure;bXGA++;}else {for (cLVC =0;cLVC
<aAAA -1;cLVC++){cIUI[cLVC].aPXO =cIUI[cLVC+1].aPXO;cIUI[cLVC].pressure =cIUI[cLVC +1].pressure;cIUI[cLVC].cJJC =cIUI[cLVC +1].
cJJC;cIUI[cLVC].aDUM =cIUI[cLVC +1].aDUM;cIUI[cLVC].cAGU =cIUI[cLVC +1].cAGU;}cIUI[aAAA -1].aPXO =aPXO;cIUI[aAAA -1].pressure =
pressure;cIUI[aAAA -1].cJJC =cJJC;cIUI[aAAA -1].aDUM =aDUM;cIUI[aAAA -1].cAGU =cAGU;}}int aAYQ(int bXKY,int aNBS,int cJOA){int 
pressure =0,cLVC =bXKY,aDZK =0;for (cLVC =bXKY;(aDZK <aNBS)&&(cLVC<aAAA);cLVC +=cJOA){pressure +=cIUI[cLVC].pressure;aDZK++;}
return (int)(0.5+1.0*pressure/aDZK);}int cALS(int bXKY,int aNBS,int cJOA){int aQCM =1100000,cMOU =0,cLVC =bXKY,aABG =aNBS,bWNO 
=cJOA;cLVC =40;while (cLVC >=15){cMOU =aAYQ(cLVC,aABG,bWNO);cLVC -=bWNO;if (aQCM >=cMOU)aQCM =cMOU;}return aQCM;}int aMEI(int 
bXKY,int aNBS,int cJOA,int cIQQ){int aDCA =0,cMOU =0,cLVC =bXKY,aABG =aNBS,bWNO =cJOA;cLVC =bXKY +aABG *bWNO -bWNO;while (cLVC 
<=cIQQ){cMOU =aAYQ(cLVC,aABG,bWNO);cLVC +=bWNO;if(aDCA <cMOU)aDCA =cMOU;}return aDCA;}int bZOI(void){int cLVC =0,aDZK =0;for (
cLVC =0;cLVC <15;cLVC++){if((aDFS(cIUI[cLVC].aDUM)>aDFS(cIUI[cLVC].cJJC))&&(aDFS(cIUI[cLVC].aDUM)>aDFS(cIUI[cLVC].cAGU))){aDZK
++;}}return aDZK;}static void aPFC(float cLRK,TInt8 aAUY,float bXHG,TInt8 aMYA){bZNC =(TInt16)(cLRK*aMDC +0.5);aPDW =aAUY;cLQE 
=(TInt16)(bXHG*aMDC +0.5);aATS =aMYA;}static TInt8 cJKI(void){TInt8 aDVS =aPDW;TInt8 aDZK =bWMI-1;while (aDVS){aDVS--;if (cIUI[
aDZK--].aPXO >bZNC){return 0;}if(aDZK<40)return 0;}return 1;}
#if 1
static TInt8 cAIA(void){TInt16 cLVC;for(cLVC =0;cLVC <bWMI-15;cLVC++){if(cIUI[cLVC].aDUM <25)return 0;}return 1;}
#else
static TInt8 cAIA(void){TInt8 aDVS =3;TInt16 cLVC =bWMI-1,aDZK=0;while(aDVS){if(cIUI[cLVC--].aDUM >100)return 0;aDVS--;}return 
1;}
#endif
#if 0
判断跌倒前40个里的最小加速度是否小于0.5g static TInt8 aPYU(void){TInt8 aDVS =cMLC;TInt8 cLVC =bWMI-1;while(aDVS--){if(cIUI[cLVC
].aPXO <aAGE)return 1;cLVC--;}return 0;}
#endif
typedef struct {TUInt16 aDVS;TUInt16 bWSM;TUInt16 aMJG;}cIVO;cIVO aDGY;void bZTG(void){aDGY.aDVS =0;aDGY.bWSM =0;aDGY.aMJG =
65535;}TInt8 aPKA(TInt16 cJJC,TInt16 aDUM,TInt16 cAGU,TUInt16 aPXO){if (aPXO >aDGY.bWSM){aDGY.bWSM =aPXO;}if (aPXO <aDGY.aMJG){
aDGY.aMJG =aPXO;}if (aDGY.bWSM <=aDGY.aMJG +80){if((aDFS(cJJC)>aDFS(aDUM)+75)||((aDFS(cAGU)>aDFS(aDUM)+75)))aDGY.aDVS++;}else {
bZTG();}if (aDGY.aDVS >=30){return 1;}return 0;}static TInt32 cLWI(TInt32 aAZW){TUInt16 cLVC;TInt32 bXME =0;TInt32 aNCY =0;for 
(cLVC =aAZW;cLVC <bWRG;cLVC++){bXME +=aMIA[cLVC];}aNCY =(TInt32)(bXME /(bWRG -aAZW)+0.5);bXME =0;for (cLVC =aAZW;cLVC <bWRG;
cLVC++){bXME +=(aMIA[cLVC]-aNCY)*(aMIA[cLVC]-aNCY);}aNCY =(TInt32)(bXME /(bWRG -aAZW)+0.5);return aNCY;}static TInt32 cJPG(
TInt32 aEAQ){TInt32 cAMY =0;if (aAEY ==1){bWRG =0;aAEY =2;}if (aAEY ==2){if (bWRG <cIPK){aMIA[bWRG]=aEAQ;bWRG++;}if (bWRG ==
cIPK){cAMY =cLWI(aDAU);if ((50*50*cAMY <=(3 *aMDC)*(3 *aMDC))){aAEY =3;}else {aAEY =0;bWRG =0;}}}return (TInt32)(aAEY/3);}void 
aQDS(void){aAEY =0;bWRG =0;}void cMQA(void){bXGA =0;aMWU =0;aPFC(0.25f,7,2.34f,5);bZTG();}static int aAAI =aAAA;TUInt8 bWMQ(
TInt16 acc_x,TInt16 acc_y,TInt16 acc_z,TInt32 aEAQ,TInt32 aMDK,TInt32 cIPS){TInt32 aDBC =0;int bZNK =0,aPEE =0,cLQM =0,aQCM =0;
int aDZK =0;aPIU(acc_x,acc_y,acc_z,aEAQ,aMDK);if (bXGA ==aAAA){if (cJKI()==1){bXGA =0;aMWU =1;return 0;}if (aMWU ==1){bXGA =0;
aMWU =0;return 0;}if (cIUI[bWMI].aPXO >=cLQE){if(cAIA()){bZNK =aMEI(bWMI +30,5,5,bWMI+70);aPEE =aMEI(bWMI +155,5,5,bWMI+195);
cLQM =aMEI(bWMI-5,3,5,bWMI+10);aQCM =cALS(bWMI,5,5);if (((bZNK -aQCM >=aATS)&&(aPEE -aQCM >=aATS))||((cLQM <aQCM)&&(aPEE -aQCM 
>=aATS))){aDZK =bZOI();if (aDZK >=8){aAAI =1;}else {aDZK +=0;}}}}if (aAAI <50)aAAI++;if ((aAAI !=aAAA)&&(aAAI >=50)){bZTG();
aAAI =aAAA;}if (aPKA(acc_x,acc_y,acc_z,aEAQ)&&(aAAI <50)){aAAI =aAAA;bZTG();aMWU =1;bXGA =0;aAEY =1;}}aDBC =cJPG(aEAQ);if(aDBC)
{aMWU =1;bXGA =0;aQDS();}return aDBC;}
