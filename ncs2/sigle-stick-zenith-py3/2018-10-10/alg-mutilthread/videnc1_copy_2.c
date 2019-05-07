/* 
 * Copyright (c) 2010, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/*
 *  ======== videnc1_copy.c ========
 *  Video encoder "copy" algorithm.
 *
 *  This file contains an implementation of the IVIDENC1 interface
 *  defined by XDM.
 */
#include <xdc/std.h>
//#include <string.h>

#include <ti/xdais/dm/ividenc1.h>

#ifdef __TI_COMPILER_VERSION__
/* XDAIS Rule 13 - this #pragma should only apply to TI codegen */
#pragma CODE_SECTION(VIDENC1COPY_TI_control, ".text:algControl")
#pragma CODE_SECTION(VIDENC1COPY_TI_process, ".text:algProcess")
#pragma CODE_SECTION(VIDENC1COPY_TI_initObj, ".text:algInit")
#pragma CODE_SECTION(VIDENC1COPY_TI_free,    ".text:algFree")
#pragma CODE_SECTION(VIDENC1COPY_TI_alloc,   ".text:algAlloc")
#endif

//#include <std.h>


//#include "stdio.h"
#include "../inc/videnc1_copy_ti.h"
#include "videnc1_copy_ti_priv.h"
#include "arith.h"		



#ifdef GT_ENALBE
#include <ti/sdo/ce/trace/gt.h>		//added by david 20130327
#endif

/* TODO, need to autogenerate this */
#define VERSIONSTRING "1.00.00.00"

/* buffer definitions */
#define MININBUFS       1
#define MINOUTBUFS      1
#define MININBUFSIZE    1
#define MINOUTBUFSIZE   1
 
#define DETECTRECT_WIDTH_MAX1 768

//by david 20130323
#define MTAB_NRECS 9    ////共分了9块内存

#define IALGFXNS  \
    &VIDENC1COPY_TI_IALG,       /* module ID */                         \
    NULL,                       /* activate */                          \
    VIDENC1COPY_TI_alloc,       /* alloc */                             \
    VIDENC1COPY_TI_control,     /* control (NULL => no control ops) */  \
    NULL,                       /* deactivate */                        \
    VIDENC1COPY_TI_free,        /* free */                              \
    VIDENC1COPY_TI_initObj,     /* init */                              \
    NULL,                       /* moved */                             \
    VIDENC1COPY_TI_numAlloc     /* numAlloc (NULL => IALG_MAXMEMRECS) */

/*
 *  ======== VIDENC1COPY_TI_IVIDENCCOPY ========
 *  This structure defines TI's implementation of the IVIDENC1 interface
 *  for the VIDENC1COPY_TI module.
 */
IVIDENC1_Fxns VIDENC1COPY_TI_IVIDENC1COPY = {    /* module_vendor_interface */
    {IALGFXNS},
    VIDENC1COPY_TI_process,
    VIDENC1COPY_TI_control,
};

/*
 *  ======== VIDENC1COPY_TI_IALG ========
 *  This structure defines TI's implementation of the IALG interface
 *  for the VIDENC1COPY_TI module.
 */
Int VIDENC1COPY_TI_numAlloc(void)
{
	return	MTAB_NRECS;
}
#ifdef __TI_COMPILER_VERSION__
/* satisfy XDAIS symbol requirement without any overhead */
#if defined(__TI_ELFABI__) || defined(__TI_EABI_SUPPORT__)

/* Symbol doesn't have any leading underscores */
asm("VIDENC1COPY_TI_IALG .set VIDENC1COPY_TI_IVIDENC1COPY");

#else

/* Symbol has a single leading underscore */
asm("_VIDENC1COPY_TI_IALG .set _VIDENC1COPY_TI_IVIDENC1COPY");

#endif
#else

/*
 *  We duplicate the structure here to allow this code to be compiled and
 *  run using non-TI toolchains at the expense of unnecessary data space
 *  consumed by the definition below.
 */
IALG_Fxns VIDENC1COPY_TI_IALG = {      /* module_vendor_interface */
    IALGFXNS
};

#endif

//by david 20130323: in lieu of using IVIDENC1_Params, 
//	we use codec-specific extended struc IVIDENC1CPY_Params.

// IVIDENC1_Params VIDENC1COPY_TI_PARAMS = {
   // sizeof(IVIDENC1_Params),
   // 0,                              /* encodingPreset - TODO */
   // 0,                              /* rateControlPreset - TODO */
   // 1024,                           /* maxHeight */
   // 1024,                           /* maxWidth */
   // 30000,                          /* maxFrameRate */
   // 10000000,                       /* maxBitRate */
   // XDM_BYTE,                       /* dataEndianness */
   // 1,                              /* maxInterFrameInterval */
   // XDM_CHROMAFORMAT_DEFAULT,       /* inputChromaFormat */
   // IVIDEO_CONTENTTYPE_DEFAULT,     /* inputContentType */
   // XDM_CHROMAFORMAT_DEFAULT        /* reconChromaFormat */
// };

IVIDENC1CPY_Params VIDENC1COPY_TI_PARAMS = {
    sizeof(IVIDENC1_Params),
    0,                              /* encodingPreset - TODO */
    0,                              /* rateControlPreset - TODO */
    1024,                           /* maxHeight */
    1024,                           /* maxWidth */
    30000,                          /* maxFrameRate */
    10000000,                       /* maxBitRate */
    XDM_BYTE,                       /* dataEndianness */
    1,                              /* maxInterFrameInterval */
    XDM_CHROMAFORMAT_DEFAULT,       /* inputChromaFormat */
    IVIDEO_CONTENTTYPE_DEFAULT,     /* inputContentType */
    XDM_CHROMAFORMAT_DEFAULT        /* reconChromaFormat */
};

#ifdef GT_ENALBE
/* tracing information */
#define GTNAME "codecs.videnc_detector"
static GT_Mask curTrace = {NULL,NULL};
#endif

#ifdef USE_ACPY3

/* Implementation of IDMA3 interface functions & IDMA3_Fxns vtable */
#include <ti/xdais/idma3.h>
#include <ti/sdo/fc/acpy3/acpy3.h>

#define NUM_LOGICAL_CH 1

/*
 *  ======== VIDENC1COPY_TI_dmaChangeChannels ========
 *  Update instance object with new logical channel.
 */
Void VIDENC1COPY_TI_dmaChangeChannels(IALG_Handle handle,
    IDMA3_ChannelRec dmaTab[])
{
    VIDENC1COPY_TI_Obj *videncObj = (Void *)handle;

    videncObj->dmaHandle1D1D8B = dmaTab[0].handle;
}


/*
 *  ======== VIDENC1COPY_TI_dmaGetChannelCnt ========
 *  Return max number of logical channels requested.
 */
Uns VIDENC1COPY_TI_dmaGetChannelCnt(Void)
{
    return (NUM_LOGICAL_CH);
}


/*
 *  ======== VIDENC1COPY_TI_dmaGetChannels ========
 *  Declare DMA resource requirement/holdings.
 */
Uns VIDENC1COPY_TI_dmaGetChannels(IALG_Handle handle, IDMA3_ChannelRec dmaTab[])
{
    VIDENC1COPY_TI_Obj *videncObj = (Void *)handle;
    int i;

    /* Initial values on logical channels */
    dmaTab[0].handle = videncObj->dmaHandle1D1D8B;

    dmaTab[0].numTransfers = 1;
    dmaTab[0].numWaits = 1;

    /*
     * Request logical DMA channels for use with ACPY3
     * AND with environment size obtained from ACPY3 implementation
     * AND with low priority.
     */
    for (i = 0; i < NUM_LOGICAL_CH; i++) {
        dmaTab[i].priority = IDMA3_PRIORITY_LOW;
        dmaTab[i].persistent = FALSE;
        dmaTab[i].protocol = &ACPY3_PROTOCOL;
    }

    return (NUM_LOGICAL_CH);
}


/*
 *  ======== VIDENC1COPY_TI_dmaInit ========
 *  Initialize instance object with granted logical channel.
 */
Int VIDENC1COPY_TI_dmaInit(IALG_Handle handle, IDMA3_ChannelRec dmaTab[])
{
    VIDENC1COPY_TI_Obj *videncObj = (Void *)handle;

    videncObj->dmaHandle1D1D8B = dmaTab[0].handle;

    return (IALG_EOK);
}


/*
 *  ======== VIDENC1COPY_TI_IDMA3 ========
 *  This structure defines TI's implementation of the IDMA3 interface
 *  for the VIDENC1COPY_TI module.
 */
IDMA3_Fxns VIDENC1COPY_TI_IDMA3 = {    /* module_vendor_interface */
    &VIDENC1COPY_TI_IALG,              /* IALG functions */
    VIDENC1COPY_TI_dmaChangeChannels,  /* ChangeChannels */
    VIDENC1COPY_TI_dmaGetChannelCnt,   /* GetChannelCnt */
    VIDENC1COPY_TI_dmaGetChannels,     /* GetChannels */
    VIDENC1COPY_TI_dmaInit             /* initialize logical channels */
};

#endif


/*
 *  ======== VIDENC1COPY_TI_alloc ========
 *  Return a table of memory descriptors that describe the memory needed
 *  to construct our object.
 */
/* ARGSUSED - this line tells the TI compiler not to warn about unused args. */
/*****************************************************************
** 函数名: VIDENC1COPY_TI_alloc
** 输入:
** 输出:
** 返回值:
** 功能描述:
**		内存分配
** 作者 david
** 日期 201305
*****************************************************************/
Int VIDENC1COPY_TI_alloc(const IALG_Params *algParams,
    IALG_Fxns **pf, IALG_MemRec memTab[])
{ 
	IVIDENC1CPY_Params 	*pParamsUsed;
	IVIDENC1CPY_Params 	defaultParams;
	XDAS_Int32 	uRegin_width;
	XDAS_Int32 	uRegin_height;
  //	XDAS_Int32	i;
	XDAS_Int32 	size;
	XDAS_Int32				paramType = -1;
//	XDAS_Int32				uLaneAmount;

		
#ifdef GT_ENALBE

	if (curTrace.modName == NULL) {   /* initialize GT (tracing) */
        GT_create(&curTrace, GTNAME);
    }

   	GT_3trace(curTrace, GT_ENTER, "DSP->xDM------------------: VIDENC1COPY_TI_alloc(0x%x, 0x%x, 0x%x)\n",
        algParams, pf, memTab);
		
#endif
	
	if( (algParams == NULL) || 
    		(algParams->size == sizeof(*algParams)) ||
    		(algParams->size == sizeof(IVIDENC1_Params)) ) {	//默认设置
			
		//构建默认参数
		defaultParams.videnc1Params.size = sizeof(defaultParams);
		defaultParams.cmd = 0xF0F0;
		defaultParams.laneAmount = LANE_AMOUNT_DEFAULT;


		pParamsUsed = &defaultParams;

		paramType = 0;
	}
	else if(algParams->size == sizeof(IVIDENC1CPY_Params))
	{
		pParamsUsed = (IVIDENC1CPY_Params *)algParams;

		paramType = 1;
	}
	else 
	{
		paramType = -1;
    }

	if(paramType<0)
	{
#ifdef GT_ENALBE
   	GT_1trace(curTrace, GT_ENTER, "DSP->xDM: VIDENC1COPY_TI_alloc: invalid params, return: %d\n", \
        IVIDENC1_EUNSUPPORTED);
#endif
		return (IVIDENC1_EUNSUPPORTED);
	}
	else if(paramType==0)
	{
#ifdef GT_ENALBE
   	GT_0trace(curTrace, GT_ENTER, "DSP->xDM: VIDENC1COPY_TI_alloc, default params\n");  
#endif
	}
	else if(paramType==1)
	{
#ifdef GT_ENALBE
   	GT_0trace(curTrace, GT_ENTER, "DSP->xDM: VIDENC1COPY_TI_alloc, user defined params\n");
#endif
	}

	//解析命令字
	if( 0xF0F0 == pParamsUsed->cmd ) //检测器配置信息首部
	{			
		//uLaneAmount = pParamsUsed->laneAmount; //by david,全部设为最大值
		
		/* Request memory for my object */
		memTab[0].size = sizeof(VIDENC1COPY_TI_Obj);
		memTab[0].alignment = 0;
		memTab[0].space = IALG_EXTERNAL;
		memTab[0].attrs = IALG_PERSIST;
		
		/* Request memory */
		memTab[1].size = sizeof(ALGCFGS); 
		memTab[1].alignment = 0;		
		//memTab[1].space = IALG_DARAM0;	//mod by david 20130513
		memTab[1].space = IALG_EXTERNAL;	
		memTab[1].attrs = IALG_PERSIST;
		
		/* Request memory*/
		memTab[2].size = sizeof(ALGPARAMS); 
		memTab[2].alignment = 0;		
		//memTab[2].space = IALG_DARAM0;	//mod by david 20130513
		memTab[2].space = IALG_EXTERNAL;
		memTab[2].attrs = IALG_SCRATCH;		//
		
		/* Request memory*/
		size = 0;
		uRegin_width = DETECTRECT_WIDTH_MAX1;
		uRegin_height = DETECTRECT_HEIGHT_MAX;
		//size = uRegin_width*uRegin_height*5; 		//每个speed检测器有五个大小相同的buf
	//	size = uRegin_width*uRegin_height*7; 		//每个speed检测器有6个大小相同的buf, mod by david 20130826 for ivd2.0
		size = uRegin_width*uRegin_height*13;                                           //20140706  增加4个  queue
		memTab[3].size = size;				//可变大小，由下载配置项决定
		memTab[3].alignment = 0;		
		//memTab[3].space = IALG_DARAM0;  	//暂时使用外存
		memTab[3].space = IALG_EXTERNAL;	//暂时使用外存
		memTab[3].attrs = IALG_SCRATCH;
		
		
		/* Request memory */
		size = DETECTRECT_WIDTH_MAX1 * DETECTRECT_HEIGHT_MAX * sizeof(Uint32);	//按最大区域设置
		
		////by david,全部default为最大
		// size = sizeof(CAMERA_STRUCT) + \
				// sizeof(LANE_STRUCT_DEF)*MAX_LANE + \
				// uLaneAmount * sizeof(SPEED_DETECT_STRUCT) + \
				// size;
		size = sizeof(CAMERA_STRUCT) + \
				sizeof(LANE_STRUCT_DEF)*MAX_LANE + \
				MAX_LANE * sizeof(SPEED_DETECT_STRUCT) + \
				size;
		memTab[4].size = size;			
		memTab[4].alignment = 0;		
		memTab[4].space = IALG_EXTERNAL;
		memTab[4].attrs = IALG_PERSIST;
		
		/* Request memory */
		size = 0;
		uRegin_width = DETECTRECT_WIDTH_MAX1;////768 
		uRegin_height = DETECTRECT_HEIGHT_MAX;
		//size = uRegin_width*uRegin_height*5; 	
	//	size = uRegin_width*uRegin_height*6; 	//mod by david 20130826 for ivd2.0
		size = uRegin_width*uRegin_height*10;   ////20140706 add 4 memory quene  按照最大的768×576 分配
		memTab[5].size = size;					
		memTab[5].alignment = 0;		
		memTab[5].space = IALG_EXTERNAL;
		memTab[5].attrs = IALG_PERSIST;
		
		/* Request memory */	//added by david 20130903 for ivd2.0
		size = 0; 	
		size = 400 *300* sizeof(CPoint); 	
		
		memTab[6].size = size;					
		memTab[6].alignment = 0;		
		memTab[6].space = IALG_EXTERNAL;
		memTab[6].attrs = IALG_PERSIST;

		/* Request memory */	//2014
		size = 0; 	
		size = 1 * sizeof(CForeground_Array); 	
		
		memTab[7].size = size;					
		memTab[7].alignment = 0;		
		memTab[7].space = IALG_EXTERNAL;
		memTab[7].attrs = IALG_PERSIST;
////20140708  
		size = 0; 	
		size = uRegin_width *uRegin_height* sizeof(unsigned char); 	////20140706  
		
		memTab[8].size = size;					
		memTab[8].alignment = 0;		
		memTab[8].space = IALG_EXTERNAL;
		memTab[8].attrs = IALG_PERSIST;





		

	}
	
#ifdef GT_ENALBE
   	GT_0trace(curTrace, GT_ENTER, "DSP->xDM: memery alloc return\n");
#endif

    return (MTAB_NRECS);

}


/*
 *  ======== VIDENC1COPY_TI_free ========
 *  Return a table of memory pointers that should be freed.  Note
 *  that this should include *all* memory requested in the
 *  alloc operation above.
 */
/* ARGSUSED - this line tells the TI compiler not to warn about unused args. */
/*****************************************************************
** 函数名: VIDENC1COPY_TI_free
** 输入:
** 输出:
** 返回值:
** 功能描述:
** 		内存释放
** 作者 david
** 日期 201305
*****************************************************************/
Int VIDENC1COPY_TI_free(IALG_Handle handle, IALG_MemRec memTab[])
{
    /*
     * Because our internal object size doesn't depend on create
     * params, we can just leverage the algAlloc() call with default
     * (NULL) create params.
     */
	 
	Int n;
    VIDENC1COPY_TI_Obj *videncObj = (Void *)handle;

    n = VIDENC1COPY_TI_alloc(NULL, NULL, memTab);

    memTab[0].base = handle;

    memTab[1].base = videncObj->pCfgs;
	memTab[2].base = videncObj->pParams;
	memTab[3].base = videncObj->pParams->puPointNewImage;
	memTab[4].base = videncObj->pCfgs->CameraLocalPara.CameraCfgEntry;
	memTab[5].base = videncObj->pCfgs->CameraLocalPara.ImageStorageEntry;
	memTab[6].base = videncObj->pParams->m_pPointArray;
	memTab[7].base = videncObj->pParams->ForegroundArray;
////20140708 
	memTab[8].base = videncObj->pParams->MidImage;


	
#ifdef GT_ENALBE
   		//GT_1trace(curTrace, GT_ENTER, "DSP->xDM: videnc1copy free return: %d\n", n);
#endif

    return (n);
}


/*
 *  ======== VIDENC1COPY_TI_initObj ========
 *  Initialize the memory allocated on our behalf (including our object).
 */
/* ARGSUSED - this line tells the TI compiler not to warn about unused args. */
/*****************************************************************
** 函数名: VIDENC1COPY_TI_initObj
** 输入:
** 输出:
** 返回值:
** 功能描述:
**	  	算法初始化
** 注:
**    	算法的初始化移到VIDENC1COPY_TI_control
** 作者 david
** 日期 201305
*****************************************************************/
Int VIDENC1COPY_TI_initObj(IALG_Handle handle, const IALG_MemRec memTab[],
        IALG_Handle parent, const IALG_Params *algParams)
{
    //const IVIDENC1_Params *params = (IVIDENC1_Params *)algParams;
	//IVIDENC1CPY_Params *params;
	VIDENC1COPY_TI_Obj *videncObj = (Void *)handle;
	Uint16 	size;
	Uint32	i;
	Bool 	bParamsValid = TRUE;
	IVIDENC1CPY_Params 	*pParamsUsed;
	IVIDENC1CPY_Params 	defaultParams;
 
	videncObj->pCfgs = memTab[1].base;
	videncObj->pParams = memTab[2].base;
	videncObj->pParams->puPointNewImage = memTab[3].base;
	videncObj->pCfgs->CameraLocalPara.CameraCfgEntry = memTab[4].base;	
	videncObj->pCfgs->CameraLocalPara.ImageStorageEntry = memTab[5].base;
	videncObj->pParams->m_pPointArray = memTab[6].base;
	videncObj->pParams->ArraySize = memTab[6].size;

	videncObj->pParams->ForegroundArray = memTab[7].base;
    videncObj->pParams->MidImage = 	memTab[8].base;
	

    /*
     * In lieu of XDM defined default params, use our codec-specific ones.
     * Note that these default values _should_ be documented in your algorithm
     * documentation so users know what to expect.
     */
    if( (algParams == NULL) || 
    		(algParams->size == sizeof(*algParams)) ||
    		(algParams->size == sizeof(IVIDENC1_Params)) ) {
        //params = &VIDENC1COPY_TI_PARAMS;
		bParamsValid = FALSE;

    }
	else if (algParams->size == sizeof(IVIDENC1CPY_Params)) 
	{
		bParamsValid = TRUE;
	}
	else {

#ifdef GT_ENALBE
		GT_1trace(curTrace, GT_ENTER, "DSP->xDM: VIDENC1COPY_TI_init return: %d\n",IVIDENC1_EUNSUPPORTED);
#endif
        //outArgs->extendedError = XDM_UNSUPPORTEDPARAM;
        return (IVIDENC1_EUNSUPPORTED);
    }

	//目前在control中初始化
	//ArithInit(0, (IVIDENC1CPY_Params *)algParams, videncObj->pCfgs, videncObj->pParams);

#ifdef GT_ENALBE
   	GT_1trace(curTrace, GT_ENTER, "DSP->xDM: VIDENC1COPY_TI_init return: %d\n",IALG_EOK);
#endif

	
    return (IALG_EOK);
}


/*
 *  ======== VIDENC1COPY_TI_process ========
 */
/* ARGSUSED - this line tells the TI compiler not to warn about unused args. */
/*****************************************************************
** 函数名: VIDENC1COPY_TI_process
** 输入:
** 输出:
** 返回值:
** 功能描述:
**	  	执行算法处理
**
** 作者 david
** 日期 201305
*****************************************************************/
XDAS_Int32 VIDENC1COPY_TI_process(IVIDENC1_Handle h, IVIDEO1_BufDescIn *inBufs,
    XDM_BufDesc *outBufs, IVIDENC1_InArgs *inArgs, IVIDENC1_OutArgs *outArgs)
{
	XDAS_Int32 i/*,k*/;
    XDAS_Int32 numInBytes;
	VIDENC1COPY_TI_Obj *videncObj = (VIDENC1COPY_TI_Obj *)h;

	//IVIDENC1CPY_OutArgs *cpyOutArgs;
	XDAS_Int16 errorFlag = FALSE;

	XDAS_Int8 *tempPt;


	XDAS_Int16	 uLaneAmount,LaneID;
	XDAS_Int16	 ChNum = 0;	//固定为零

#ifdef GT_ENALBE
   	GT_0trace(curTrace, GT_ENTER, "DSP->xDM: enter VIDENC1COPY_TI_process\n");
#endif
								

#ifdef USE_ACPY3
    const Uint32 maxTransferChunkSize       = 0xffff;
    Uint32       thisTransferChunkSize      = 0x0;
    Uint32       remainingTransferChunkSize;
    Uint32       thisTransferSrcAddr, thisTransferDstAddr;

    ACPY3_Params params;
    //VIDENC1COPY_TI_Obj *videncObj = (VIDENC1COPY_TI_Obj *)h;
#endif
    /*
     * Validate arguments - this codec supports "base" XDM and an extended
     * inArgs and outArgs.
     */


    if ( (inArgs->size != sizeof(*inArgs) && inArgs->size != sizeof(IVIDENC1CPY_InArgs)) ||
         (outArgs->size != sizeof(*outArgs))
        )
    {

        outArgs->extendedError = XDM_UNSUPPORTEDPARAM;

#ifdef GT_ENALBE
		GT_1trace(curTrace, GT_ENTER, "DSP->xDM: process return %d\n", IVIDENC1_EUNSUPPORTED);
#endif
        return (IVIDENC1_EUNSUPPORTED);		//-3
    }

    /* validate that there's at least a single inBuf and outBuf */
//    if ((inBufs->numBufs < 1) || (outBufs->numBufs < 1)) {
//        outArgs->extendedError = XDM_UNSUPPORTEDPARAM;

//        return (IVIDENC1_EFAIL);		//-1
//    }

    /* validate inputID */
//    if (inArgs->inputID == 0) {
//        outArgs->extendedError = XDM_UNSUPPORTEDPARAM;

//        return (IVIDENC1_EFAIL);		//-1
//    }

	//cpyOutArgs = (IVIDENC1CPY_OutArgs *)outArgs;
	/*
     * Validate arguments - this codec supports "base" XDM and an extended
     * outArgs.
     */

    /* there's an available in and out buffer, how many samples? */

    numInBytes = sizeof(RESULTMSG) < outBufs->bufSizes[0] ?
        sizeof(RESULTMSG) : outBufs->bufSizes[0];


//	numInBytes = inBufs->bufDesc[0].bufSize < outBufs->bufSizes[0] ?
//        inBufs->bufDesc[0].bufSize : outBufs->bufSizes[0];

    /* and honor the extended maxBytes if it was provided */
	/* //by david 20130502
    if (inArgs->size == sizeof(IVIDENC1CPY_InArgs)) {
        if (numInBytes > ((IVIDENC1CPY_InArgs *)inArgs)->maxBytes) {
            numInBytes = ((IVIDENC1CPY_InArgs *)inArgs)->maxBytes;
        }
    }
	*/
#ifdef USE_ACPY3
    /*
     * Activate Channel scratch DMA channels.
     */
    ACPY3_activate(videncObj->dmaHandle1D1D8B);

    thisTransferSrcAddr        = (Uint32)inBufs->bufDesc[0].buf;
    tempPt        = (videncObj->hLocalBuffer0->puPointNewImage);
	thisTransferDstAddr = (Uint32)tempPt;
	//thisTransferDstAddr        = (Uint32)outBufs->bufs[0];
    remainingTransferChunkSize = numInBytes;	

    while (remainingTransferChunkSize > 0) {

        if (remainingTransferChunkSize > maxTransferChunkSize) {
           thisTransferChunkSize = maxTransferChunkSize;
        }
        else {
           thisTransferChunkSize = remainingTransferChunkSize;
        }

        /* Configure the logical channel */
        params.transferType = ACPY3_1D1D;
        params.srcAddr      = (void *)thisTransferSrcAddr;
        params.dstAddr      = (void *)thisTransferDstAddr;
        params.elementSize  = remainingTransferChunkSize;
        params.numElements  = 1;
        params.waitId       = 0;
        params.numFrames    = 1;

		//added by david 20130402
		params.srcElementIndex = 0;
    	params.dstElementIndex = 0;
    	params.srcFrameIndex = 0;
    	params.dstFrameIndex = 0;

        remainingTransferChunkSize -= thisTransferChunkSize;
        thisTransferSrcAddr += thisTransferChunkSize;
        thisTransferDstAddr += thisTransferChunkSize;

        /* Configure logical dma channel */
        ACPY3_configure(videncObj->dmaHandle1D1D8B, &params, 0);

        /* Use DMA to copy data */
        ACPY3_start(videncObj->dmaHandle1D1D8B);

        /* wait for transfer to finish  */
        ACPY3_wait(videncObj->dmaHandle1D1D8B);
    }


    /*
     * Deactivate Channel scratch DMA channels.
     */
    ACPY3_deactivate(videncObj->dmaHandle1D1D8B);

	for (i = 0; i < numInBytes; i++)
    {
        if (videncObj->hLocalBuffer0->puPointNewImage[i] != inBufs->bufDesc[0].buf[i]) {
		//if (outBufs->bufs[0][i] != inBufs->bufDesc[0].buf[i]) {
			
            errorFlag = TRUE;
        }
    }
	if(errorFlag == TRUE)
	{

	}
	

    /*
     * Because we used DMA, the algo processor didn't accessed either of the
     * 2 data buffers.
     */
    XDM_CLEARACCESSMODE_READ(inBufs->bufDesc[0].accessMask);
    XDM_CLEARACCESSMODE_WRITE(inBufs->bufDesc[0].accessMask);

    XDM_CLEARACCESSMODE_READ(outArgs->encodedBuf.accessMask);
    XDM_CLEARACCESSMODE_WRITE(outArgs->encodedBuf.accessMask);


#else
    /* everything looks good, do the 'encode', set outArgs and return */
	
	//memcpy(videncObj->hLocalBuffer0->puPointNewImage, inBufs->bufDesc[0].buf, numInBytes);


    /* memcpy read from inBuf and wrote to encodedBuf (in our case, outBuf) */
    XDM_CLEARACCESSMODE_WRITE(inBufs->bufDesc[0].accessMask);
    XDM_SETACCESSMODE_READ(inBufs->bufDesc[0].accessMask);

    XDM_CLEARACCESSMODE_READ(outArgs->encodedBuf.accessMask);
    XDM_SETACCESSMODE_WRITE(outArgs->encodedBuf.accessMask);
#endif


	//ArithProc(ChNum, videncObj->hLocalBuffer0->puPointNewImage, outBufs->bufs[0], videncObj->hLocalBuffer0, NULL);
	ArithProc(ChNum, (Uint8 *)inBufs, (Uint8*)outBufs->bufs[0], numInBytes, videncObj->pCfgs, videncObj->pParams);

    /* outArgs->bytesGenerated reports the total number of bytes encoded */
    outArgs->bytesGenerated = numInBytes;

    /* report that the encoded data is in outArgs->encodedBuf */
    outArgs->encodedBuf.buf = outBufs->bufs[0];
    outArgs->encodedBuf.bufSize = outBufs->bufSizes[0];

    /* Fill out the rest of the outArgs struct */
    outArgs->extendedError = 0;
    outArgs->encodedFrameType = 0;     /* TODO */
    outArgs->inputFrameSkip = IVIDEO_FRAME_ENCODED;
    outArgs->outputID = inArgs->inputID;

    /* TODO - add support for reconBufs */
	outArgs->reconBufs.numBufs = 0;
#ifdef GT_ENALBE
/*	GT_1trace(curTrace, GT_ENTER, "DSP->xDM:*********************bday =  %d\n",  videncObj->pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bDay);
//	GT_1trace(curTrace, GT_ENTER, "DSP->xDM:*********pParams->NormalDetectCfg.cSpeedDesc.fitflag =  %d\n",  videncObj->pParams->NormalDetectCfg.cSpeedDesc.fitflag);
//	GT_1trace(curTrace, GT_ENTER, "DSP->xDM:********************.bAuto =  %d\n",  videncObj->pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bAuto);
//	GT_1trace(curTrace, GT_ENTER, "DSP->xDM:*********pParams->NormalDetectCfg.cSpeedDesc.testvalue =  %d\n",  videncObj->pParams->NormalDetectCfg.cSpeedDesc.testvalue);
//	GT_1trace(curTrace, GT_ENTER, "DSP->xDM:*********pParams->NormalDetectCfg.cSpeedDesc.testvalue1 =  %d\n",  videncObj->pParams->NormalDetectCfg.cSpeedDesc.testvalue1);

//GT_1trace(curTrace, GT_ENTER, "DSP->xDM:*********pParams->NormalDetectCfg.cSpeedDesc.testvalue2 =  %d\n",  videncObj->pParams->NormalDetectCfg.cSpeedDesc.testvalue2);
*/

	GT_1trace(curTrace, GT_ENTER, "DSP->xDM:*********pCfgs->gThisFrameTime =  %d\n",  videncObj->pCfgs->gThisFrameTime);
	GT_1trace(curTrace, GT_ENTER, "DSP->xDM:*********pCfgs->guTest =  %d\n",  videncObj->pCfgs->guTest);
	GT_1trace(curTrace, GT_ENTER, "DSP->xDM:*********pCfgs->guTest1 =  %d\n",  videncObj->pCfgs->guTest1);
	GT_1trace(curTrace, GT_ENTER, "DSP->xDM:*********pCfgs->guTest2 =  %d\n",  videncObj->pCfgs->guTest2);
	GT_1trace(curTrace, GT_ENTER, "DSP->xDM:*********pCfgs->guTest3  =  %d\n",  videncObj->pCfgs->guTest3);
    
	GT_6trace(curTrace, GT_ENTER, "DSP->xDM:*********pCfgs->guTestA  =  %d--%d--%d--%d--%d--%d\n",  videncObj->pCfgs->guTestA[0],videncObj->pCfgs->guTestA[1],videncObj->pCfgs->guTestA[2],videncObj->pCfgs->guTestA[3],videncObj->pCfgs->guTestA[4],videncObj->pCfgs->guTestA[5]);
	//for(k=0;k<24;k++)
	//GT_1trace(curTrace, GT_ENTER, "DSP->xDM:*********pCfgs->guTestB  =  %d\n",  videncObj->pCfgs->guTestB[k]);
	
	GT_6trace(curTrace, GT_ENTER, "DSP->xDM:*********pCfgs->guTestC  =  %d--%d--%d--%d--%d--%d\n",  videncObj->pCfgs->guTestC[0],videncObj->pCfgs->guTestC[1],videncObj->pCfgs->guTestC[2],videncObj->pCfgs->guTestC[3],videncObj->pCfgs->guTestC[4],videncObj->pCfgs->guTestC[5]);
	GT_6trace(curTrace, GT_ENTER, "DSP->xDM:*********pCfgs->guTestD  =  %d--%d--%d--%d--%d--%d\n",  videncObj->pCfgs->guTestD[0],videncObj->pCfgs->guTestD[1],videncObj->pCfgs->guTestD[2],videncObj->pCfgs->guTestD[3],videncObj->pCfgs->guTestD[4],videncObj->pCfgs->guTestD[5]);



	GT_1trace(curTrace, GT_ENTER, "DSP->xDM:*********pCfgs->thresholdValue =  %d\n",  videncObj->pCfgs->thresholdValue);

#endif


	/////////////////////////////////////////////////////////////////////
	/*TODO - add our own code to return the detect result*/
	/////////////////////////////////////////////////////////////////////

#ifdef GT_ENALBE
   	GT_1trace(curTrace, GT_ENTER, "DSP->xDM: process return %d\n", IVIDENC1_EOK);
#endif

    return (IVIDENC1_EOK);
}


/*
 *  ======== VIDENC1COPY_TI_control ========
 */
/* ARGSUSED - this line tells the TI compiler not to warn about unused args. */
/*****************************************************************
** 函数名: VIDENC1COPY_TI_control
** 输入:
** 输出:
** 返回值:
** 功能描述:
**	  	修改算法动态参数
** 注:
**    	算法在此函数中执行初始化
** 作者 david
** 日期 201305
*****************************************************************/
XDAS_Int32 VIDENC1COPY_TI_control(IVIDENC1_Handle handle, IVIDENC1_Cmd id,
    IVIDENC1_DynamicParams *params, IVIDENC1_Status *status)
{
    XDAS_Int32 	retVal;
	XDAS_Int32 	cfgSize;
	CFGMSG		*pCfgMsg;
	SPEEDCFGSEG	*pSpeedCfgSeg;
	CFGINFOHEADER	*pCfgHeader;
	VIDENC1COPY_TI_Obj *videncObj = (VIDENC1COPY_TI_Obj *)handle;
	XDAS_UInt16	x;
    XDAS_UInt16 y;
 	XDAS_UInt32 i;
	
	Uint16		laneID;
	Uint16		uDayNightJudgeMinContiuFrame;
	Uint16		uComprehensiveSens;
	Uint16		uDetectSens1;
	Uint16		uDetectSens2;
	Uint16		uStatisticsSens1;
	Uint16		uStatisticsSens2;
	Uint16		uSobelThreshold;
			
	float		uSpeedTransFactor;
	float		uSpeedCorrectionFactor;
	float		uLengthTransFactor;
	float		uLengthCorrectionFactor;
	
	Uint16		uTransFactor;
	
	Uint32 		uSpeedCounterChangedThreshold;
	Uint32 		uSpeedCounterChangedThreshold1;
	Uint32 		uSpeedCounterChangedThreshold2;
	Uint32		uGraySubThreshold;
	
	Bool		bInitRetValue = FALSE;
	

    /* validate arguments - this codec only supports "base" XDM. */
    if ((params->size != sizeof(*params)) ||
            (status->size != sizeof(*status))) {
        return (IVIDENC1_EUNSUPPORTED);
    }

    /* initialize for the general case where we don't access the data buffer */
    XDM_CLEARACCESSMODE_READ(status->data.accessMask);
    XDM_CLEARACCESSMODE_WRITE(status->data.accessMask);

    switch (id) {
        case XDM_GETSTATUS:
        case XDM_GETBUFINFO:
            status->extendedError = 0;

            status->bufInfo.minNumInBufs = MININBUFS;
            status->bufInfo.minNumOutBufs = MINOUTBUFS;
            status->bufInfo.minInBufSize[0] = MININBUFSIZE;
            status->bufInfo.minOutBufSize[0] = MINOUTBUFSIZE;

            retVal = IVIDENC1_EOK;
            break;

        case XDM_SETPARAMS:
            retVal = IVIDENC1_EOK;
            break;

        case XDM_SETDEFAULT:
            retVal = IVIDENC1_EOK;
            break;

        case XDM_RESET:
        case XDM_FLUSH:
            retVal = IVIDENC1_EOK;
            break;

        case XDM_GETVERSION:
            if ((status->data.buf != NULL) &&
                (status->data.bufSize >= strlen(VERSIONSTRING))) {
                strncpy((char *)status->data.buf, VERSIONSTRING,
                    strlen(VERSIONSTRING));

                /* null terminate the string */
                status->data.buf[strlen(VERSIONSTRING)] = '\0';

                /* strncpy wrote to the data buffer */
                XDM_SETACCESSMODE_WRITE(status->data.accessMask);

                retVal = IVIDENC1_EOK;
            }
            else {
                retVal = IVIDENC1_EFAIL;
            }

            break;
			
		/*USER EXTENDED CMD*/
		case IVIDENC1_TI_CMD0:	//初始化参数(检测器配置信息)
			//简单的参数检验,仅用于目前的车辆存在检测器配置(speedDetector) 
			pCfgMsg = (CFGMSG*)status->data.buf;
			pCfgHeader = (CFGINFOHEADER*)( status->data.buf + sizeof(MSGHEADER) );
			pSpeedCfgSeg = (SPEEDCFGSEG*)( status->data.buf + sizeof(CFGMSG) );
			x = pSpeedCfgSeg->uSegData[0].SpeedEachLane[7].ptFourCorner[0].x;
			y = pSpeedCfgSeg->uSegData[0].SpeedEachLane[7].ptFourCorner[0].y;
			
			
			laneID = 0;
			uDayNightJudgeMinContiuFrame = 0;
			uComprehensiveSens = 0;
			uDetectSens1 = 0;
			uDetectSens2 = 0;
			uStatisticsSens1 = 0;
			uStatisticsSens2 = 0;
			uSobelThreshold = 0;
			
			uSpeedTransFactor = 0;
			uSpeedCorrectionFactor = 0;
			uLengthTransFactor = 0;
			uLengthCorrectionFactor = 0;
	
			laneID = pSpeedCfgSeg->uSegData[0].uLaneTotalNum - 1;
			uDayNightJudgeMinContiuFrame = pSpeedCfgSeg->uSegData[0].uDayNightJudgeMinContiuFrame;	//环境转换灵敏度
			uComprehensiveSens = pSpeedCfgSeg->uSegData[0].uComprehensiveSens;		//综合灵敏度
			uDetectSens1 = pSpeedCfgSeg->uSegData[0].uDetectSens1;			//检测灵敏度1		
			uDetectSens2 = pSpeedCfgSeg->uSegData[0].uDetectSens2;			//检测灵敏度2
			uStatisticsSens1 = pSpeedCfgSeg->uSegData[0].uStatisticsSens1;		//统计灵敏度1
			uStatisticsSens2 = pSpeedCfgSeg->uSegData[0].uStatisticsSens2;		//统计灵敏度2
			uSobelThreshold = pSpeedCfgSeg->uSegData[0].uSobelThreshold;		//阀值灵敏度
			
			// uSpeedTransFactor = pSpeedCfgSeg->uSegData[0].SpeedEachLane[laneID].uSpeedTransFactor;
			// uSpeedCorrectionFactor = pSpeedCfgSeg->uSegData[0].SpeedEachLane[laneID].uSpeedCorrectionFactor;
			// uLengthTransFactor = pSpeedCfgSeg->uSegData[0].SpeedEachLane[laneID].uLengthTransFactor;
			// uLengthCorrectionFactor = pSpeedCfgSeg->uSegData[0].SpeedEachLane[laneID].uLengthCorrectionFactor;
			
			uTransFactor = pSpeedCfgSeg->uSegData[0].SpeedEachLane[laneID].uTransFactor;
			
			uSpeedCounterChangedThreshold = pSpeedCfgSeg->uSegData[0].SpeedEachLane[laneID].uSpeedCounterChangedThreshold;
			uSpeedCounterChangedThreshold1 = pSpeedCfgSeg->uSegData[0].SpeedEachLane[laneID].uSpeedCounterChangedThreshold1;
			uSpeedCounterChangedThreshold2 = pSpeedCfgSeg->uSegData[0].SpeedEachLane[laneID].uSpeedCounterChangedThreshold2;
			uGraySubThreshold = pSpeedCfgSeg->uSegData[0].SpeedEachLane[laneID].uGraySubThreshold;

#ifdef GT_ENALBE
	GT_2trace(curTrace, GT_ENTER, "DSP->xDM: control, uFlag:0x%x, uCmd:0x%x\n",pCfgMsg->uMsgHeader.uFlag,pCfgMsg->uMsgHeader.uCmd);
	GT_2trace(curTrace, GT_ENTER, "DSP->xDM: control, uDetectPosition:%d, %d\n",pCfgHeader->uDetectPosition ,pCfgHeader->uDetectFuncs[0]);
	GT_2trace(curTrace, GT_ENTER, "DSP->xDM: control, uType:%d, uNum:%d\n",pSpeedCfgSeg->uType,pSpeedCfgSeg->uNum);
	GT_1trace(curTrace, GT_ENTER, "DSP->xDM: control, uLaneTotalNum:%d\n",pSpeedCfgSeg->uSegData[0].uLaneTotalNum);
	
	GT_4trace(curTrace, GT_ENTER, "DSP->xDM: control, DayNightJudge:%d, %d, %d, %d\n",  \
						uDayNightJudgeMinContiuFrame, uComprehensiveSens, uDetectSens1, uDetectSens2);
	GT_3trace(curTrace, GT_ENTER, "DSP->xDM: control, uStatisticsSens1:%d, %d, %d\n",  \
						uStatisticsSens1, uStatisticsSens2, uSobelThreshold);
	GT_4trace(curTrace, GT_ENTER, "DSP->xDM: control, uSpeedTransFactor:%f, %f, %f, %f\n",  \
						uSpeedTransFactor, uSpeedCorrectionFactor, uLengthTransFactor, uLengthCorrectionFactor);
						
						
	GT_4trace(curTrace, GT_ENTER, "DSP->xDM: control, DayNightJudge:%d, %d, %d, %d\n",  \
						uSpeedCounterChangedThreshold, uSpeedCounterChangedThreshold1, uSpeedCounterChangedThreshold2, uGraySubThreshold);
	
#endif

			cfgSize = sizeof(CFGMSG)+sizeof(SPEEDCFGSEG);

			//GT_2trace(curTrace, GT_ENTER, "DSP->xDM:before cfgSize return %d,%d\n", cfgSize,status->data.bufSize);
			if( cfgSize == status->data.bufSize)
			{
				//GT_2trace(curTrace, GT_ENTER, "DSP->xDM:cfgSize return %d,%d\n", cfgSize,status->data.bufSize);
				pCfgMsg = (CFGMSG*)status->data.buf;
				if( (0xAA == pCfgMsg->uMsgHeader.uFlag)&&(0xF0F0 == pCfgMsg->uMsgHeader.uCmd) )		//协议
				{
					pCfgHeader = (CFGINFOHEADER*)( status->data.buf + sizeof(MSGHEADER) );
					pSpeedCfgSeg = (SPEEDCFGSEG*)( status->data.buf + sizeof(CFGMSG) );
				
					/*配置信息初始化*/
	                
					bInitRetValue = ArithInit(0, pCfgHeader, pSpeedCfgSeg, videncObj->pCfgs, videncObj->pParams);
					//ArithInit_all(0, pCfgHeader, pSpeedCfgSeg, videncObj->pCfgs, videncObj->pParams);
//	GT_1trace(curTrace, GT_ENTER, "DSP->xDM:ArithInit control return pCfgs->guTestB[5] %d\n",pCfgs->guTestB[5]++); 
	
						
					/*修改消息头*/
					pCfgMsg->uMsgHeader.uFlag = 0x55;	//dsp处理消息完毕
					
					if( FALSE == bInitRetValue)
					{
						retVal = -100;		//init erro
					}
					else
					{
						retVal = IVIDENC1_EOK;		//0, ok
					}
					
				}
				else
				{
				//	retVal = IVIDENC1_EFAIL;	//协议错误 -1
				    retVal = 0xEE;
				}
			}
			else
			{
			//	retVal = IVIDENC1_EFAIL;	//参数无效  -1
				retVal = 0xDD;
			}
			
			break;

        default:
            /* unsupported cmd */
            retVal = IVIDENC1_EUNSUPPORTED;		//-3
            break;
    }

#ifdef GT_ENALBE
   	GT_1trace(curTrace, GT_ENTER, "DSP->xDM: control return %d\n", retVal);
#endif

    return (retVal);
}
/*
 *  @(#) ti.xdais.dm.examples.videnc1_copy; 2, 0, 0,12; 5-10-2010 12:31:47; /db/wtree/library/trees/dais/dais.git/src/
 */




























