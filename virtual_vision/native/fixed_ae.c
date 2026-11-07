#include "ax_isp_3a_api.h"
#include <stdint.h>
// Q10 gain: 1024 = 1x. Set every gain stage, independent of prior AE.
int fixed_apply(uint32_t us,uint32_t a,uint32_t d,uint32_t isp,uint32_t hcg){
 AX_ISP_IQ_AE_PARAM_T p={0};
 int rc=AX_ISP_IQ_GetAeParam(0,&p); if(rc)return rc;
 p.nEnable=0; p.tExpManual.nHcgLcg=hcg; p.tExpManual.nShutter=us;
 p.tExpManual.nAGain=a; p.tExpManual.nDgain=d; p.tExpManual.nIspGain=isp;
 p.tExpManual.nSysTotalGain=(uint64_t)a*d*isp/1048576;
 return AX_ISP_IQ_SetAeParam(0,&p);
}
int fixed_read(uint32_t *out){
 AX_ISP_IQ_AE_PARAM_T p={0}; AX_ISP_IQ_AE_STATUS_T s={0};
 int rc=AX_ISP_IQ_GetAeParam(0,&p); if(rc)return rc;
 rc=AX_ISP_IQ_GetAeStatus(0,&s);if(rc)return rc;
 out[0]=p.nEnable;out[1]=p.tExpManual.nShutter;out[2]=p.tExpManual.nAGain;
 out[3]=p.tExpManual.nDgain;out[4]=p.tExpManual.nIspGain;
 out[5]=s.tExpStatus.nShutter;out[6]=s.tExpStatus.nAGain;
 out[7]=s.tExpStatus.nDgain;out[8]=s.tExpStatus.nIspGain;out[9]=s.tExpStatus.nSysTotalGain;
 out[10]=p.tExpManual.nHcgLcg;out[11]=s.tExpStatus.nHcgLcg;
 return 0;
}
