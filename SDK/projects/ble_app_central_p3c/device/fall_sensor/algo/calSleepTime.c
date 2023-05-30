#include "calSleepTime.h"
typedef struct {TUInt8 sec;TUInt8 min;TUInt8 hour;TUInt8 w_date;TUInt8 w_month;TUInt16 w_year;}bXGI;static const TUInt8 aMXC[12
]={31,28,31,30,31,30,31,31,30,31,30,31 };static bXGI cJJK ={0 };static TUInt8 aDUU(TUInt16 cAHC);static TUInt32 aPXW(TUInt16 
cMKE,TUInt8 aAFG,TUInt8 bWRO,TUInt8 aMII);static TUInt32 cIUQ(TUInt8 aDGA,TUInt8 bZSI);static TUInt8 aDUU(TUInt16 cAHC){if (
cAHC %4 ==0){if (cAHC %100 ==0){if (cAHC %400 ==0)return 1;else return 0;}else return 1;}else return 0;}static TUInt32 aPXW(
TUInt16 cMKE,TUInt8 aAFG,TUInt8 bWRO,TUInt8 aMII){TUInt32 aPJC =0;TUInt8 cLVC=1;for(cLVC=1;cLVC<aAFG;cLVC++){aPJC +=aMXC[cLVC-1
]*24;}if(aDUU(cMKE)){if(aAFG >2){aPJC +=24;}}aPJC +=(bWRO-1)*24;aPJC +=aMII;return aPJC;}static TUInt32 cIUQ(TUInt8 aDGA,TUInt8
bZSI){TUInt32 sec =0;sec +=(aDGA*60);sec +=bZSI;return sec;}TUInt32 cLVK(TUInt16 cMKE,TUInt8 aAFG,TUInt8 bWRO,TUInt8 aMII,
TUInt8 aDGA,TUInt8 bZSI){TInt32 cLVK =0;TInt32 aAYY[2]={0};TInt32 bXLG[2]={0};if(cJJK.w_month ==0){cJJK.w_year =cMKE;cJJK.
w_month =aAFG;cJJK.w_date =bWRO;cJJK.hour =aMII;cJJK.min =aDGA;cJJK.sec =bZSI;cLVK =0;}else{if(cMKE >cJJK.w_year){aAYY[0]+=(365
*24);if(aDUU(cJJK.w_year)){aAYY[0]+=24;}}aAYY[0]+=aPXW(cMKE,aAFG,bWRO,aMII);aAYY[1]+=aPXW(cJJK.w_year,cJJK.w_month,cJJK.w_date,
cJJK.hour);bXLG[0]=cIUQ(aDGA,bZSI);bXLG[1]=cIUQ(cJJK.min,cJJK.sec);cLVK =aAYY[0]-aAYY[1];cLVK *=3600;cLVK +=(bXLG[0]-bXLG[1]);
if(cLVK <=1){cLVK =0;}}cJJK.w_year =cMKE;cJJK.w_month =aAFG;cJJK.w_date =bWRO;cJJK.hour =aMII;cJJK.min =aDGA;cJJK.sec =bZSI;
return cLVK;}
