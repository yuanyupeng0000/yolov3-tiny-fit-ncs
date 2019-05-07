
#include "stdio.h"
#include "stdlib.h" 
#include "ti/bios/include/bios.h"
#include <ti/xdais/dm/ividenc1.h>
//for vicp
#include <tistdtypes.h>
//#include "string.h"
//#include "vicplib.h"
//#include "gpp_vicplib.h"
#include "../inc/videnc1_copy_ti.h"
#include "videnc1_copy_ti_priv.h"
#include "arith.h"
#include "math.h"
#define USING_OPENCV_1_0 1
#if USING_OPENCV_1_0
#include "alg_opencv.h"
#endif

//////////////////////////////////////////////////////////////////////////////////////
////20140111
//#define	MAX_SPEEDDETECTOR_DOTS	2048*15 
//#define MaxDotsInDetect 2000*15

#define	MAX_SPEEDDETECTOR_DOTS	768*576 
#define MaxDotsInDetect        768*576


#define SPEED_STARTER_STARTLINE		1
#define SPEED_STARTER_ENDLINE		40//10
#define SPEED_COUNTER_ENDLINE		1
#define SPEED_COUNTER_STARTLINE		42//12

//#define	VEHICLE_LENGTHINFO_LENGTH	6
//#define	VEHICLE_SPEEDINFO_LENGTH    4

#define FULL_COLS  					(720)
#define FULL_ROWS  					(576)
//#define COLS  					(200)
//#define ROWS  					(100)


#define		THIS_LANE_COUNTER	0
#define		LEFT_LANE_COUNTER	1
#define		RIGHT_LANE_COUNTER	2
#define		VEHICLE_MIN_ROW		16//6


/////////////////////////////////////////////////////////
//  function declaritions
/////////////////////////////////////////////////////////
// 提前申请内存，避免动态申请内存
static int g_intStack[FULL_ROWS * FULL_COLS / 4];
//static unsigned char g_charStack[FULL_ROWS * FULL_COLS];
//static int Abs(int a) { return (a > 0) ? a : -a; }
/* Header file for the sa function */
//void SobelCalculate(const Uint8 * in_data, Uint8 * out_data, Int16 cols, Int16 rows);
//void SobelThreshold(Uint8 *Buf, Uint16 dotNum, Uint16 threshold);

/* ======================================================================== */
/*  				                                                        */
/* ======================================================================== */

/* ======================================================================== */
/*							                                                */
/* ======================================================================== */



//CForeground_Array  m_ForegroundArrayInnight[200];

#define max(a,b) (((a)>(b)) ? (a):(b))
#define min(a,b) (((a)>(b)) ? (b):(a))

//int day_flag=0;
//int night_flag=0;
//int AverageGray_num=0;
//double AverageGray_sum=0;
//double former_tAverageGray=0;
//int Fchange_speed_tAverageGray_num=0;
//double percent=0;
//bool IsQueue=0;




/*int max(int a,int b)
{
	return a>b?a:b;
} 

int min(int a,int b)
{
	return a>b?b:a;
} 
*/ 

void RECT_Init(CRect handle)
{
}
void RECT_Free(CRect handle)
{
}
int RECT_Width(CRect handle)
{
		return abs(handle.right - handle.left);
}
int RECT_Height(CRect handle)
{
		return abs(handle.bottom - handle.top);
}


bool ArithInit(Uint16 ChNum, CFGINFOHEADER *pCfgHeader, SPEEDCFGSEG *pDetectCfgSeg, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	bool 	bValidCfg;

	if(TRUE == CfgStructParse(ChNum, pCfgHeader, pDetectCfgSeg, pCfgs, pParams))
	{
		bValidCfg = TRUE;
	}
	else
	{
		bValidCfg = FALSE;
	}

	if(bValidCfg == TRUE)
	{
		if( ArithVariableInit(ChNum, pCfgs, pParams) == TRUE )
		{
			return TRUE;
		}
		else
		{
			return FALSE;
		}
	}
	else
	{
		return FALSE;
	}

}

bool CfgStructParse(Uint16 ChNum, CFGINFOHEADER *pCfgHeader, SPEEDCFGSEG *pDetectCfgSeg, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	Int32	i,j;
	Uint16 	uRegin_width;
	Uint16 	uRegin_height;
	struct 	cSpeedDetectStruct 	*pSpeedDetect = NULL;
	struct	LanesStruct 		*pLane = NULL;
	ZENITH_SPEEDDETECTOR 		*pDownSpeedDetect = NULL;
	Uint8 						*pCurLaneImageEntry = NULL;
	Bool 	bDetectBackRun[8];
	CPoint	ptFourCorner[4];
	CPoint	ptRowAndCol;
	CPoint  ptRowAndColQ;
    CPoint	ptCornerQ[2];  
	CPoint	ptCornerQA[2];
	CPoint pt;
	int x1=FULL_COLS,x2=0,y1=FULL_ROWS,y2=0;
	int minX,maxX,idxLeft,idxRight;
	pDownSpeedDetect = (ZENITH_SPEEDDETECTOR*)pDetectCfgSeg->uSegData;



	pCfgs->CameraCfg.LaneAmount = pDownSpeedDetect->uLaneTotalNum;
	pCfgs->bAuto = pDownSpeedDetect->uEnvironment; //TRUE;


	pCfgs->CameraCfg.uEnvironmentStatus = pDownSpeedDetect->uEnvironmentStatus;
	pCfgs->CameraCfg.uDayNightJudgeMinContiuFrame = pDownSpeedDetect->uDayNightJudgeMinContiuFrame; //no used
	pCfgs->CameraCfg.uGetBackMinContiuFrame = pDownSpeedDetect->uComprehensiveSens;  //no used
	pCfgs->CameraCfg.uVehicleHeadMinRow = pDownSpeedDetect->uDetectSens1;
	pCfgs->CameraCfg.uInThreshold = pDownSpeedDetect->uDetectSens2; 
	pCfgs->CameraCfg.uSquareThreshold = pDownSpeedDetect->uStatisticsSens1;
	pCfgs->CameraCfg.guSubImageThreshold = pDownSpeedDetect->uStatisticsSens2;
	pCfgs->CameraCfg.uSobelThreshold = pDownSpeedDetect->uSobelThreshold;
	
	pCfgs->CameraCfg.bCameraWorkInCross = pCfgHeader->uDetectPosition;  
	pCfgs->CameraLocalPara.bNormalDetectEnable = pCfgHeader->uDetectFuncs[0]; 
	
	if(pCfgs->bAuto == 2)
	{
		pCfgs->CameraCfg.uVehicleHeadMinRow = (pCfgs->CameraCfg.uVehicleHeadMinRow > 20) ? 20 : pCfgs->CameraCfg.uVehicleHeadMinRow;
		pCfgs->CameraCfg.uVehicleHeadMinRow = (pCfgs->CameraCfg.uVehicleHeadMinRow < 10) ? 10 : pCfgs->CameraCfg.uVehicleHeadMinRow;
	}

	for(i=0; i<MAX_LANE; i++)
	{
		pCfgs->CameraLocalPara.LanesCfgEntry[i] = (Uint8 *)pCfgs->CameraLocalPara.CameraCfgEntry +
		        sizeof(CAMERA_STRUCT) +
		        i*sizeof(LANE_STRUCT_DEF);

		memset((void*)pCfgs->CameraLocalPara.LanesCfgEntry[i], 0, sizeof(LANE_STRUCT_DEF)); 		
	
//		((LANE_STRUCT_DEF *)pCfgs->CameraLocalPara.LanesCfgEntry[i])->AbnormalFunction.AbnormalFunctionBit.BackRunDef = 0;
		bDetectBackRun[i] = FALSE;
	}
	
	for(i=0; i<pCfgs->CameraCfg.LaneAmount; i++) //
	{
		pLane = (struct	LanesStruct *)pCfgs->CameraLocalPara.LanesCfgEntry[i];			

		memcpy((void *)&pParams->LaneParameter, (void *)pLane, sizeof(struct LanesStruct));
		InitThisLanesPara(&pParams->LaneParameter);
		memcpy((void *)pLane, (void *)&pParams->LaneParameter, sizeof(struct LanesStruct));

		if(2 == pDownSpeedDetect->SpeedEachLane[i].uDetectDerection)	
		{
		//	pLane->AbnormalFunction.AbnormalFunctionBit.BackRunDef = 1;
	////		bDetectBackRun[i] = FALSE;
		}
		else if(1 == pDownSpeedDetect->SpeedEachLane[i].uDetectDerection)	
		{
		//	pLane->AbnormalFunction.AbnormalFunctionBit.BackRunDef = 0;
			bDetectBackRun[i] = TRUE;
		}
		

	}


	pSpeedDetect = (struct cSpeedDetectStruct *) ((char*)pCfgs->CameraLocalPara.CameraCfgEntry + \
	               sizeof(CAMERA_STRUCT) +  \
	               MAX_LANE*sizeof(LANE_STRUCT_DEF) \
	                                             );
	pCurLaneImageEntry = (Uint8 *)pCfgs->CameraLocalPara.ImageStorageEntry;


	for(i=0; i<pCfgs->CameraCfg.LaneAmount; i++)	
	{

		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr = pSpeedDetect;
		memset((void*)pSpeedDetect, 0, sizeof(struct cSpeedDetectStruct));
		pSpeedDetect->CoordinatePointer = (Uint32 *) ((Uint8 *)pSpeedDetect +sizeof(SPEED_DETECT_STRUCT));
        
////20140714
 		memcpy( (void*)pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner, (void*)pDownSpeedDetect->SpeedEachLane[i].ptFourCorner, 4*sizeof(CPoint) );
 		memcpy( (void*)pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQ, (void*)pDownSpeedDetect->SpeedEachLane[i].ptCornerQ, 2*sizeof(CPoint) );
 		memcpy( (void*)pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQA, (void*)pDownSpeedDetect->SpeedEachLane[i].ptCornerQA, 2*sizeof(CPoint) );

		if( bDetectBackRun[i] == TRUE )
		{
			ptFourCorner[0].x = pDownSpeedDetect->SpeedEachLane[i].ptFourCorner[3].x;
			ptFourCorner[0].y = pDownSpeedDetect->SpeedEachLane[i].ptFourCorner[3].y;
			ptFourCorner[1].x = pDownSpeedDetect->SpeedEachLane[i].ptFourCorner[2].x;
			ptFourCorner[1].y = pDownSpeedDetect->SpeedEachLane[i].ptFourCorner[2].y;
			ptFourCorner[2].x = pDownSpeedDetect->SpeedEachLane[i].ptFourCorner[1].x;
			ptFourCorner[2].y = pDownSpeedDetect->SpeedEachLane[i].ptFourCorner[1].y;
			ptFourCorner[3].x = pDownSpeedDetect->SpeedEachLane[i].ptFourCorner[0].x;
			ptFourCorner[3].y = pDownSpeedDetect->SpeedEachLane[i].ptFourCorner[0].y;


		}
		else
		{
			memcpy( (void*)ptFourCorner, (void*)pDownSpeedDetect->SpeedEachLane[i].ptFourCorner, 4*sizeof(CPoint) );
			memcpy( (void*)ptCornerQ, (void*)pDownSpeedDetect->SpeedEachLane[i].ptCornerQ, 2*sizeof(CPoint) );

		}

		if(ptFourCorner[0].y>ptFourCorner[2].y)
		{
			pt.x=ptFourCorner[0].x;
			pt.y=ptFourCorner[0].y;
			ptFourCorner[0].x=ptFourCorner[2].x;
			ptFourCorner[0].y=ptFourCorner[2].y;
			ptFourCorner[2].x=pt.x;
			ptFourCorner[2].y=pt.y;
		}
		if(ptFourCorner[1].y>ptFourCorner[3].y)
		{
			pt.x=ptFourCorner[1].x;
			pt.y=ptFourCorner[1].y;
			ptFourCorner[1].x=ptFourCorner[3].x;
			ptFourCorner[1].y=ptFourCorner[3].y;
			ptFourCorner[3].x=pt.x;
			ptFourCorner[3].y=pt.y;
		}
		if(ptFourCorner[0].x>ptFourCorner[1].x)
		{
			pt.x=ptFourCorner[0].x;
			pt.y=ptFourCorner[0].y;
			ptFourCorner[0].x=ptFourCorner[1].x;
			ptFourCorner[0].y=ptFourCorner[1].y;
			ptFourCorner[1].x=pt.x;
			ptFourCorner[1].y=pt.y;
		}
		if(ptFourCorner[3].x<ptFourCorner[2].x)
		{
			pt.x=ptFourCorner[2].x;
			pt.y=ptFourCorner[2].y;
			ptFourCorner[2].x=ptFourCorner[3].x;
			ptFourCorner[2].y=ptFourCorner[3].y;
			ptFourCorner[3].x=pt.x;
			ptFourCorner[3].y=pt.y;
		}
#ifdef	COORD_DERECT
		memcpy( (void*)pSpeedDetect->CoordinatePointer, (void*)ptFourCorner, 4*sizeof(CPoint) );

		uRegin_width = abs(((CPoint *)pSpeedDetect->CoordinatePointer)[1].x - ((CPoint *)pSpeedDetect->CoordinatePointer)[0].x);
		uRegin_height = abs(((CPoint *)pSpeedDetect->CoordinatePointer)[2].y - ((CPoint *)pSpeedDetect->CoordinatePointer)[1].y);


#else
////20140111
		if((pCfgs->CameraCfg.uEnvironmentStatus==1)||(pCfgs->CameraCfg.uEnvironmentStatus==3))
		{
			pCfgs->CameraCfg.uSquareThreshold=pDownSpeedDetect->uStatisticsSens1*2/3;
		}

		if(pCfgs->bAuto==2)
		{
			ptRowAndCol = ptGetDot( &ptFourCorner[0], &ptFourCorner[1], &ptFourCorner[2],	\
		                        &ptFourCorner[3], 20,(Uint32*)(pSpeedDetect->CoordinatePointer), 1);
	

			

			////20140707  queue
			pSpeedDetect->CoordinatePointerQ=pSpeedDetect->CoordinatePointer+ptRowAndCol.x*ptRowAndCol.y;
			ptRowAndColQ= ptGetDot( &ptCornerQ[0], &ptCornerQ[1], &ptFourCorner[2],	\
		                        &ptFourCorner[3], 200,(Uint32*)(pSpeedDetect->CoordinatePointerQ), 1);

		}
		else
		{
			ptRowAndCol = ptGetDot( &ptFourCorner[0], &ptFourCorner[1], &ptFourCorner[2],	\
		                        &ptFourCorner[3], 20,(Uint32*)(pSpeedDetect->CoordinatePointer), 1);
		

			////20140707  queue
			pSpeedDetect->CoordinatePointerQ=pSpeedDetect->CoordinatePointer+ptRowAndCol.x*ptRowAndCol.y;

			ptRowAndColQ= ptGetDot( &ptCornerQ[0], &ptCornerQ[1], &ptFourCorner[2],	\
		                        &ptFourCorner[3], 20,(Uint32*)(pSpeedDetect->CoordinatePointerQ), 1);
		}		
	

		if((ptRowAndCol.x==0) && (ptRowAndCol.y==0))
		{
			return	FALSE;	
		}
		if((ptRowAndCol.x * ptRowAndCol.y) > MAX_SPEEDDETECTOR_DOTS)
		{
			return	FALSE;	
		}
	
		if((ptRowAndColQ.x==0) && (ptRowAndColQ.y==0))
		{ 
			return	FALSE;	
		}
		if((ptRowAndColQ.x * ptRowAndColQ.y) > MAX_SPEEDDETECTOR_DOTS)
		{
			return	FALSE;	
		}
	    memcpy( (void*)pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner, (void*)ptFourCorner, 4*sizeof(CPoint) );

		uRegin_width = ptRowAndCol.x;
		uRegin_height = ptRowAndCol.y;


		pCfgs->team_width = ptRowAndColQ.x;
		pCfgs->team_height = ptRowAndColQ.y;
		pSpeedDetect->QueDetectDots =  pCfgs->team_width * pCfgs->team_height;
		
		pSpeedDetect->QueDetectLines=pCfgs->team_height;
		pSpeedDetect->QueDetectColumns=pCfgs->team_width;

		



#endif		//#ifdef COORD_DERECT

		memset(pSpeedDetect->position,0,30);
		memset(pSpeedDetect->preposition,0,30);
		memset(pSpeedDetect->pastlength,0,4);
		pSpeedDetect->percent=0;
//		pSpeedDetect->sptimes=0;

		pSpeedDetect->DetectDots =  uRegin_width * uRegin_height;
		pSpeedDetect->DetectLines = uRegin_height;
		pSpeedDetect->DetectColumns = uRegin_width;
		
		pSpeedDetect->thresholdValue = 1;  //ADDED 20130827 FOR IVD2.0
		pCfgs->thresholdValue =1;
		pSpeedDetect->nSpeedInfoTop = -1;
		pSpeedDetect->nSpeedInfoMaxSize = SPEEDINFO_MAXSIZE;
		pSpeedDetect->nLengthInfoTop = -1;
		pSpeedDetect->nLengthInfoMaxSize = LENGTHINFO_MAXSIZE;
		pSpeedDetect->uSLKThreshold	= 4;
		pCfgs->CameraLocalPara.DetectCfgEntry[i] = (Uint8*)pSpeedDetect;
		
		//added by david 20131025
		pSpeedDetect->wLastVehicleSpeed = 30;
		pSpeedDetect->wLastVehicleLength = 4;


		
		pSpeedDetect->uTransFactor = pDownSpeedDetect->SpeedEachLane[i].uTransFactor;
		

		pSpeedDetect->uSpeedCounterChangedThreshold = pDownSpeedDetect->SpeedEachLane[i].uSpeedCounterChangedThreshold; //0
		pSpeedDetect->uSpeedCounterChangedThreshold1 = pDownSpeedDetect->SpeedEachLane[i].uSpeedCounterChangedThreshold1; //1
		pSpeedDetect->uSpeedCounterChangedThreshold2 = pDownSpeedDetect->SpeedEachLane[i].uSpeedCounterChangedThreshold2;//2
		pSpeedDetect->uGraySubThreshold = pDownSpeedDetect->SpeedEachLane[i].uGraySubThreshold;
		if(pCfgs->bAuto==2)
		{
			pSpeedDetect->uSpeedCounterChangedThreshold = 20;
			pSpeedDetect->uSpeedCounterChangedThreshold1 = 20;
			//pSpeedDetect->uSpeedCounterChangedThreshold2 = 20;
		}
	//	pSpeedDetect->min_standard_deviation=100.0;
		
		//moded by david 20130910
		// if(	pSpeedDetect->uSpeedCounterStartRow < SPEED_COUNTER_STARTLINE )
		// {
			// pSpeedDetect->uSpeedCounterStartRow = SPEED_COUNTER_STARTLINE;
		// }
		// else if( pSpeedDetect->uSpeedCounterStartRow > (pSpeedDetect->DetectLines - SPEED_COUNTER_STARTLINE) )
		// {
			// pSpeedDetect->uSpeedCounterStartRow = pSpeedDetect->DetectLines - SPEED_COUNTER_STARTLINE;
		// }
		pSpeedDetect->uSpeedCounterStartRow = pSpeedDetect->DetectLines - SPEED_COUNTER_STARTLINE;
		
		if(pCfgs->bAuto==2)
		{
			pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight =TRUE;
			pSpeedDetect->SpeedDetectorWorkMode.bDay = FALSE; 	//FALSE; 	
			pSpeedDetect->SpeedDetectorWorkMode.bNight = TRUE; //TRUE;		
			pCfgs->bNight = TRUE;
		}
		else
		{
			pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight =FALSE;
			pSpeedDetect->SpeedDetectorWorkMode.bDay = TRUE; 	//FALSE; 	
			pSpeedDetect->SpeedDetectorWorkMode.bNight = FALSE; //TRUE;		
			pCfgs->bNight = FALSE;
		}
	

		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->CurrentImagePointer =
		    (Uint16 *)pCurLaneImageEntry;

		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->BackImagePointer =
		    (Uint16 *)( (Uint8 *)pCurLaneImageEntry + (uRegin_width*uRegin_height) );

		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->puTempImage =
		    (Uint16 *)( (Uint8 *)pCurLaneImageEntry + 2*(uRegin_width*uRegin_height) );

		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->SobelBackImagePointer =
		    (Uint16 *)( (Uint8 *)pCurLaneImageEntry + 3*(uRegin_width*uRegin_height) );
			
		//added by david 20130826 for alg IVD2.0
		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->BackImagePlusPointer =
		    (Uint16 *)( (Uint8 *)pCurLaneImageEntry + 4*(uRegin_width*uRegin_height) );


		////20140707  queue

		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->CurrQueueImage =
			(Uint16 *)( (Uint8 *)pCurLaneImageEntry + 5*(uRegin_width*uRegin_height) );
		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->PreQueueImage =
			(Uint16 *)( (Uint8 *)pCurLaneImageEntry + 5*(uRegin_width*uRegin_height)+(pCfgs->team_width*pCfgs->team_height) );
		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->SobelQueueImage =
			(Uint16 *)( (Uint8 *)pCurLaneImageEntry + 5*(uRegin_width*uRegin_height)+2*(pCfgs->team_width*pCfgs->team_height));
		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->BackQueueImage =
			(Uint16 *)( (Uint8 *)pCurLaneImageEntry +5*(uRegin_width*uRegin_height)+3*(pCfgs->team_width*pCfgs->team_height) );
		//added by david 20130826 for alg IVD2.0
		

	/*	pCurLaneImageEntry += 5*(uRegin_width*uRegin_height)+4*(pCfgs->team_width*pCfgs->team_height);

		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->CurrQueueImage =
			(Uint16 *)( (Uint8 *)pCurLaneImageEntry + 5*(uRegin_width*uRegin_height));
		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->PreQueueImage =
			(Uint16 *)( (Uint8 *)pCurLaneImageEntry + 5*(pCfgs->team_width*pCfgs->team_height) );
		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->SobelQueueImage =
			(Uint16 *)( (Uint8 *)pCurLaneImageEntry + 5*(pCfgs->team_width*pCfgs->team_height) );
		pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->FiltedSobelQueueImage =
			(Uint16 *)( (Uint8 *)pCurLaneImageEntry + 5*(pCfgs->team_width*pCfgs->team_height) );
	*/
		////20140707  
	////	pCurLaneImageEntry += 5*(uRegin_width*uRegin_height);
        pCurLaneImageEntry += 5*(uRegin_width*uRegin_height)+4*(pCfgs->team_width*pCfgs->team_height);
	////20140707  
	////	pSpeedDetect = (struct cSpeedDetectStruct*)( (char*)pSpeedDetect + sizeof(SPEED_DETECT_STRUCT) + uRegin_width*uRegin_height*sizeof(Uint32) );
	    pSpeedDetect = (struct cSpeedDetectStruct*)( (char*)pSpeedDetect + sizeof(SPEED_DETECT_STRUCT) + uRegin_width*uRegin_height*sizeof(Uint32)+pCfgs->team_width*pCfgs->team_height*sizeof(Uint32));
	}
	
	//对图像数据进行标定
	//初始化映射数据
	for(i = 0; i < pCfgs->CameraCfg.LaneAmount; i++)
	{
		memset(pCfgs->actual_distance[i], 0, 576 * sizeof(float));
	}
	if(pCfgs->CameraCfg.LaneAmount)//有车道存在，才进行标定
	{
		minX = pCfgs->DownSpeedCfg.SpeedEachLane[0].ptCornerQ[0].x;
		maxX = pCfgs->DownSpeedCfg.SpeedEachLane[0].ptCornerQ[1].x;
		idxLeft = 0;
		idxRight = 0;
		for(i = 1; i < pCfgs->CameraCfg.LaneAmount; i++)
		{
			if(minX > pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQ[0].x)
			{
				minX = pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQ[0].x;
				idxLeft = i;
			}
			if(maxX < pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQ[1].x)
			{
				maxX = pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQ[1].x;
				idxRight = i;
			}

		}
		//标定点
		pCfgs->calibration_point[0][0] = pCfgs->DownSpeedCfg.SpeedEachLane[idxLeft].ptCornerQ[0].x;
		pCfgs->calibration_point[0][1] = pCfgs->DownSpeedCfg.SpeedEachLane[idxLeft].ptCornerQ[0].y;
		pCfgs->calibration_point[1][0] = pCfgs->DownSpeedCfg.SpeedEachLane[idxRight].ptCornerQ[1].x;
		pCfgs->calibration_point[1][1] = pCfgs->DownSpeedCfg.SpeedEachLane[idxRight].ptCornerQ[1].y;
		pCfgs->calibration_point[2][0] = pCfgs->DownSpeedCfg.SpeedEachLane[idxLeft].ptFourCorner[2].x;
		pCfgs->calibration_point[2][1] = pCfgs->DownSpeedCfg.SpeedEachLane[idxLeft].ptFourCorner[2].y;
		pCfgs->calibration_point[3][0] = pCfgs->DownSpeedCfg.SpeedEachLane[idxRight].ptFourCorner[3].x;
		pCfgs->calibration_point[3][1] = pCfgs->DownSpeedCfg.SpeedEachLane[idxRight].ptFourCorner[3].y;
		pCfgs->guTest = pCfgs->CameraCfg.LaneAmount;
		pCfgs->guTest1 = idxLeft;
		pCfgs->guTest2 = idxRight;
		pCfgs->guTestA[0] = pCfgs->calibration_point[0][0];
		pCfgs->guTestA[1] = pCfgs->calibration_point[0][1];
		pCfgs->guTestA[2] = pCfgs->calibration_point[1][0];
		pCfgs->guTestA[3] = pCfgs->calibration_point[1][1];
		pCfgs->guTestA[4] = pCfgs->calibration_point[2][0];
		pCfgs->guTestA[5] = pCfgs->calibration_point[2][1];
		pCfgs->guTestA[6] = pCfgs->calibration_point[3][0];
		pCfgs->guTestA[7] = pCfgs->calibration_point[3][1];
		//将流量区域车道线作为基准线
		pCfgs->base_line[0][0] = pCfgs->DownSpeedCfg.SpeedEachLane[pCfgs->CameraCfg.LaneAmount - 1].ptFourCorner[1].x;
		pCfgs->base_line[0][1] = pCfgs->DownSpeedCfg.SpeedEachLane[pCfgs->CameraCfg.LaneAmount - 1].ptFourCorner[1].y;
		pCfgs->base_line[1][0] = pCfgs->DownSpeedCfg.SpeedEachLane[pCfgs->CameraCfg.LaneAmount - 1].ptFourCorner[3].x;
		pCfgs->base_line[1][1] = pCfgs->DownSpeedCfg.SpeedEachLane[pCfgs->CameraCfg.LaneAmount - 1].ptFourCorner[3].y;
		pCfgs->base_length = pParams->NormalDetectCfg.cSpeedDesc.uTransFactor;//车道流量区域实际高度
		pCfgs->near_point_length = 0;//最近点距离为0
		pCfgs->guTestC[0] = pCfgs->base_line[0][0];
		pCfgs->guTestC[1] = pCfgs->base_line[0][1];
		pCfgs->guTestC[2] = pCfgs->base_line[1][0];
		pCfgs->guTestC[3] = pCfgs->base_line[1][1];
		pCfgs->guTestC[4] = (int)pCfgs->base_length;
		pCfgs->guTestC[5] = (int)pCfgs->near_point_length;
		if((pCfgs->calibration_point[2][1] > pCfgs->calibration_point[0][1]) && (pCfgs->calibration_point[3][1] > pCfgs->calibration_point[1][1]))
			camera_calibration(pCfgs->base_line, pCfgs->base_length, pCfgs->calibration_point, pCfgs->near_point_length, pCfgs->CameraCfg.LaneAmount, pCfgs);

        pCfgs->guTestD[0] = pCfgs->actual_distance[0][0];
		pCfgs->guTestD[1] = pCfgs->actual_distance[0][575];
		pCfgs->guTestD[2] = pCfgs->actual_distance[1][0];
		pCfgs->guTestD[3] = pCfgs->actual_distance[1][575];
		pCfgs->guTestD[4] = pCfgs->actual_distance[2][0];
		pCfgs->guTestD[5] = pCfgs->actual_distance[2][575];
	}

	//检测区域设置
	pCfgs->detectROI.x=0;
	pCfgs->detectROI.y=0;
	pCfgs->detectROI.width=FULL_COLS>>1;
	pCfgs->detectROI.height=FULL_ROWS>>1;
	if(pCfgs->CameraCfg.LaneAmount)
	{
		for(i=0; i<pCfgs->CameraCfg.LaneAmount; i++) 
		{
			for(j=0;j<4;j++)
			{
				x1=(x1<pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[j].x)? x1:pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[j].x;
				x2=(x2>pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[j].x)? x2:pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[j].x;
				y1=(y1<pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[j].y)? y1:pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[j].y;
				y2=(y2>pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[j].y)? y2:pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[j].y;
				pCfgs->detectPosition=y2;
			}
		}
		y2=((y2+60)<FULL_ROWS)? (y2+60):FULL_ROWS;
		y1=(y2-340>40)? (y2-340):40;
		y2=(y2<y1)? y1:y2;
		pCfgs->detectROI.x=0;
		pCfgs->detectROI.y=y1>>1;
		pCfgs->detectROI.width=FULL_COLS/2;
		pCfgs->detectROI.height=(y2-y1)/2;
		pCfgs->detectPosition=pCfgs->detectPosition/2;
	}
	pCfgs->target_id=1;
	pCfgs->targets_size=0;
	pCfgs->candidate_targets_size=0;
	memset(pCfgs->targets,0,100*sizeof(CTarget));
	memset(pCfgs->candidate_targets,0,100*sizeof(CTarget));
	pCfgs->bClassifierInit=FALSE;
	pCfgs->visibility=FALSE;
	pCfgs->abnormalflag=FALSE;
	return	TRUE;

}


bool ArithVariableInit(Uint16 ChNum, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	Int32	i,j;
	CPoint	ptFourCorner[4];
	float k1,b1,k2,b2;
	Int32 startX,endX;
	unsigned char* p;
	pCfgs->gThisFrameTime = 0; 
	pCfgs->guMinCountIntervalFrame=5;////25
	pCfgs->guAdmitBreakFrame = 2; ///2

	pParams->puPointSobelImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX;
	pParams->puSobelTempImageX = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX*2;
	pParams->puSobelTempImageY = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX*3;
////20131218 zcj
	pParams->puPointNextImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX*4;

////	pParams->puAlgTempImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX*4;
	pParams->puBackImagePlus = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX*5;  //20130830 by david for ivd2.0
	
	//pParams->puPointBackSobelImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX*6;

////20140706  
	pParams->CurrQueueImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX*6;
	pParams->PreQueueImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX*7;
	pParams->SobelQueueImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX*8;
	pParams->BackQueueImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX*9;

	pParams->MaskFullImage = (Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX*10;
	pParams->CurrFullImage=(Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX*11;
	pParams->PrevFullImage=(Uint8 *)pParams->puPointNewImage + DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX*12;

	for( i = 0; i < 8; i++ )
	{
		memset( (void*)&pCfgs->CameraLocalPara.guSpeedCounterStatus[i], 0, sizeof(SPEED_COUNTER_STATUS) );
	}

	for(i=0; i<MAX_LANE; i++)
	{
		pCfgs->uDetectInSum[i] = 0;
		pCfgs->uDetectOutSum[i] = 0;
		pCfgs->VehicleTeamSave[i]=1;//排队长度状态变化
		pCfgs->firstx[i]=0;
		pCfgs->secondx[i]=0;
		pCfgs->uCarExistStatis[i]=0;
	}
	memset((void*)pParams->MaskFullImage,0,sizeof(unsigned char)*DETECTRECT_WIDTH_MAX*DETECTRECT_HEIGHT_MAX);

	if(pCfgs->CameraCfg.LaneAmount)
	{
		ptFourCorner[0]=pCfgs->DownSpeedCfg.SpeedEachLane[0].ptFourCorner[0];
		ptFourCorner[1]=pCfgs->DownSpeedCfg.SpeedEachLane[0].ptFourCorner[1];
		ptFourCorner[2]=pCfgs->DownSpeedCfg.SpeedEachLane[0].ptFourCorner[2];
		ptFourCorner[3]=pCfgs->DownSpeedCfg.SpeedEachLane[0].ptFourCorner[3];
		for(i=1;i<pCfgs->CameraCfg.LaneAmount;i++)
		{
			ptFourCorner[0]=(ptFourCorner[0].x<pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[0].x)? ptFourCorner[0]:pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[0];
			ptFourCorner[1]=(ptFourCorner[1].x>pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[1].x)? ptFourCorner[1]:pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[1];
			ptFourCorner[2]=(ptFourCorner[2].x<pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[2].x)? ptFourCorner[2]:pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[2];
			ptFourCorner[3]=(ptFourCorner[3].x>pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[3].x)? ptFourCorner[3]:pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[3];

		}
		ptFourCorner[0].x=ptFourCorner[0].x/2;
		ptFourCorner[0].y=ptFourCorner[0].y/2;
		ptFourCorner[1].x=ptFourCorner[1].x/2;
		ptFourCorner[1].y=ptFourCorner[1].y/2;
		ptFourCorner[2].x=ptFourCorner[2].x/2;
		ptFourCorner[2].y=ptFourCorner[2].y/2;
		ptFourCorner[3].x=ptFourCorner[3].x/2;
		ptFourCorner[3].y=ptFourCorner[3].y/2;
		if(ptFourCorner[0].x!=ptFourCorner[2].x)
		{
			k1=(float)(ptFourCorner[0].y-ptFourCorner[2].y)/(ptFourCorner[0].x-ptFourCorner[2].x);
			b1=ptFourCorner[0].y-k1*ptFourCorner[0].x;
		}
		if(ptFourCorner[1].x!=ptFourCorner[3].x)
		{
			k2=(float)(ptFourCorner[1].y-ptFourCorner[3].y)/(ptFourCorner[1].x-ptFourCorner[3].x);
			b2=ptFourCorner[1].y-k2*ptFourCorner[1].x;
		}
		for(i=pCfgs->detectROI.y;i<(pCfgs->detectROI.y+pCfgs->detectROI.height);i++)
		{
			if(ptFourCorner[0].x!=ptFourCorner[2].x)
				startX=(int)((i-b1)/k1);
			else
				startX=ptFourCorner[0].x;
			startX=(startX-20)<0? 0:(startX-20);
			if(ptFourCorner[1].x!=ptFourCorner[3].x)
				endX=(int)((i-b2)/k2);
			else
				endX=ptFourCorner[1].x;
			endX=(endX - 10)>(pCfgs->detectROI.x+pCfgs->detectROI.width)? (pCfgs->detectROI.x+pCfgs->detectROI.width):(endX - 10);
			p=pParams->MaskFullImage+(i-pCfgs->detectROI.y)*FULL_COLS/2-pCfgs->detectROI.x;
			for(j=startX;j<endX;j++)
			{
				*(p+j)=1;
			}
		}

	}
////20131219	
	pParams->NormalDetectCfg.cSpeedDesc.bGetBackFlag= FALSE;
	pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged=TRUE;
	pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged1=TRUE;
	pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged2=TRUE;
	pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged3=TRUE;
	pParams->NormalDetectCfg.cSpeedDesc.getQueback_flag = FALSE;
	pParams->NormalDetectCfg.cSpeedDesc.framecount = 0;
	//pParams->NormalDetectCfg.cSpeedDesc.bBottomNull=TRUE;
	//pParams->NormalDetectCfg.cSpeedDesc.min_standard_deviation=100.0;

	return TRUE;  //ADDED BY DAVID 20131023

}

void InitThisLanesPara(struct LanesStruct *LanePtr)
{
	LanePtr->AccumulatedFrameTime=0;
	LanePtr->bArbitration = 0;
	LanePtr->bIsVehicleInSpeed = 0;
	LanePtr->CounterValue = 0;
	LanePtr->LengthGetTimes = 0;
	LanePtr->LengthSum = 0;
	LanePtr->nReserve[0] = 0;
	LanePtr->nReserve[1] = 0;
	LanePtr->nReserve[2] = 0;
//			LanePtr->bLaneEventFlag = 0xFFFF;
	LanePtr->SpeedGetTimes = 0;
	LanePtr->SpeedSum = 0;
	LanePtr->SpeedValue = 0;
	LanePtr->StatisticStartFrameTime = 0;
	LanePtr->TotalFrameCount = 0;
	LanePtr->TotalVehicleFlow = 0;
	LanePtr->uCarTypeTransformFactorDownLimit = 0;
	LanePtr->uCarTypeTransformFactorUpLimit = 0;
	LanePtr->uCounterWrapChangedThreshold = 0;
	LanePtr->uDetectorInSignalFrame = 0;
	LanePtr->uEndRow = 0;
	LanePtr->uFrameCount = 0;
	LanePtr->uGetVehiclePileLengthCount = 0;
	LanePtr->uLaneVehicleMaxReportLength = 0;
	LanePtr->uSobelRowIsChangedRatio = 0;
	LanePtr->uSobelRowIsChangedThreshHold = 0;
	LanePtr->uStartRow = 0;
	LanePtr->uTimeDifferenceRatioThreshHold = 0;
	LanePtr->uVehicleMinLengthInPresence = 0;
	LanePtr->uVehiclePileLengthSum = 0;
	LanePtr->VehicleExistFrameCount = 0;
	LanePtr->VehicleLength = 0;
}


Bool GetDetectRegion(const unsigned char * in_data, int nWidth, int nHeight, unsigned char * out_data_y, unsigned char * out_data_uv, int startLine, int endLine,
                     int startCol, int endCol)
{
	int i,j,k;
	int nIndexStart;
	int nIndexEnd;

	k = 0;
	if(out_data_y==NULL)
	{
		return FALSE;
	}
	
	nIndexStart = startCol*2;
	nIndexEnd = endCol*2;

	for(i=startLine; i<endLine; i++)
	{
		for(j=nIndexStart; j<nIndexEnd; j+=2)
		{
			//out_data_y[k] = in_data[i*nWidth*2+j+1];
			memcpy((void *)&out_data_y[k],(void *)&in_data[i*nWidth*2+j+1], sizeof(unsigned char));
			// if(out_data_uv != NULL)
			// {
				// out_data_uv[k] = in_data[i*nWidth*2+j];
			// }
			k++;
		}
	}

	return TRUE;
}


void StoreCurImage(Uint8 *inBuf, ALGCFGS *pCfgs, ALGPARAMS	*pParams)
{
	Uint32	i,j;
	Uint32	Elements,QElements;  ////20140706
	Uint32 	index;
	Uint32	*pCoordinatePointer;
	struct 	cSpeedDetectStruct *pSpeedDetect = NULL;
	if (inBuf)
	{
		for( i = 0; i < pCfgs->CameraCfg.LaneAmount; ++i )
		{
			pSpeedDetect = pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr;
			pCoordinatePointer = pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->CoordinatePointer;
			Elements = pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->DetectDots;

			for(j = 0; j < Elements; ++j)
			{
				//index = pCoordinatePointer[j];
				index = *(pCoordinatePointer+j);
				//((Uint8*)(pSpeedDetect->CurrentImagePointer))[j] = inBuf[index];
				*((Uint8*)(pSpeedDetect->CurrentImagePointer)+j) = *(inBuf+index);

			}

			////20140706   add queue
			pCoordinatePointer = pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->CoordinatePointerQ;
			QElements = pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr->QueDetectDots;
			for(j = 0; j < QElements; ++j)
			{
				//index = pCoordinatePointer[j];
				index = *(pCoordinatePointer+j);
				*((Uint8*)(pSpeedDetect->CurrQueueImage)+j) = *(inBuf+index);
			}
		}
	}

	return;

}

/*****************************************************************
***main function
****************************************************************/
Uint16 ArithProc(Uint16 ChNum, Uint8* inBuf, Uint8* outBuf, Int32 outSize, ALGCFGS *pCfgs, ALGPARAMS	*pParams)
{
	Int32 i,j,k;
//	Bool bStatus;

  	Uint16	FrameWidth;
	Uint16	FrameHeight;
////20140115  ERROR
  //	Int16 	startLine;
//	Int16 	endLine;
  //	Int16 	startCol;
//	Int16 	endCol;
    int AverageGray=0;
	double tAverageGray=0.0,change_speed_tAverageGray=0.0;
	Bool	bSpeedEnable, bPresenceEnable;
	struct 	cSpeedDetectStruct *pSpeedDetect = NULL;
	
	IVIDEO1_BufDescIn *inBufs;

	CPoint m_ptend[8];
	CPoint LineUp[2];
	Uint16 width = 0, height = 0, FrameWidth4 = 0, FrameWidth2 = 0, detectRoiX4 = 0;
	Uint8 *p, *p0, *p1, *p2, *p3, *p4, *pImg;
	int fuzzydegree;
	inBufs = (IVIDEO1_BufDescIn *)inBuf;

	FrameWidth = inBufs->frameWidth;
	FrameHeight = inBufs->frameHeight;
	if((pCfgs->CameraCfg.uEnvironmentStatus == 4) && (pCfgs->gThisFrameTime % 99 == 1) && (pCfgs->targets_size == 0))//daytime
	{
		pCfgs->up_visib_value++;
		fuzzydegree=fuzzy(inBufs->bufDesc[0].buf , FrameWidth, FrameHeight);//
		for (j=VISIB_LENGTH-1;j>0;j--)
		{
			pCfgs->visib_value[j]=pCfgs->visib_value[j-1];
		}
		pCfgs->visib_value[0]=fuzzydegree;
		if (pCfgs->up_visib_value>VISIB_LENGTH)
		{
			pCfgs->visibility=visible_judge(pCfgs->visib_value,VISIB_LENGTH,10);
		} 
		else
		{
			pCfgs->visibility=FALSE;
		}
	}
	else if(pCfgs->CameraCfg.uEnvironmentStatus==2||pCfgs->CameraCfg.uEnvironmentStatus==1||pCfgs->CameraCfg.uEnvironmentStatus==3)//night dawn 
	{
		pCfgs->visibility=FALSE;
	}
	if((pCfgs->CameraCfg.uEnvironmentStatus == 4) && (pCfgs->gThisFrameTime % 199 == 1) && (pCfgs->targets_size == 0))
	{
		if(Color_deviate(inBufs->bufDesc[0].buf,FrameWidth,FrameHeight,pCfgs))
		{
			pCfgs->abnormal_time++;
		}
		else
		{
		    pCfgs->abnormal_time=0;	
		}   
		pCfgs->abnormalflag=pCfgs->abnormal_time>5? TRUE:FALSE;//颜色异常检测
	}
	else if(pCfgs->CameraCfg.uEnvironmentStatus==2||pCfgs->CameraCfg.uEnvironmentStatus==1||pCfgs->CameraCfg.uEnvironmentStatus==3)//night dawn 
	{
		pCfgs->abnormalflag=FALSE;
	}
#ifdef COORD_DERECT
	for( i = 0; i < pCfgs->CameraCfg.LaneAmount; i++ )
	{
		pSpeedDetect = pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr;
		
		startLine = ((CPoint *)pSpeedDetect->CoordinatePointer)[0].y;
		endLine = ((CPoint *)pSpeedDetect->CoordinatePointer)[2].y;
		startCol = ((CPoint *)pSpeedDetect->CoordinatePointer)[0].x;
		endCol = ((CPoint *)pSpeedDetect->CoordinatePointer)[1].x;

		bStatus = GetDetectRegion(inBufs->bufDesc[0].buf , FrameWidth, FrameHeight, pParams->puPointNewImage, NULL, startLine, endLine, startCol, endCol);

		if(bStatus==FALSE)
		{
			return 0;		//failed
		}
		
		memcpy((void *)pSpeedDetect->CurrentImagePointer, (void *)pParams->puPointNewImage, pSpeedDetect->DetectDots);
	}

#else
	//
	StoreCurImage((Uint8 *)inBufs->bufDesc[0].buf, pCfgs, pParams);
#endif	

	if(((pCfgs->CameraCfg.uEnvironmentStatus==1)&&(pCfgs->bAuto==1))
		||((pCfgs->CameraCfg.uEnvironmentStatus==3)&&(pCfgs->bAuto==2)))
	{
		if (pCfgs->gThisFrameTime>1&&pCfgs->gThisFrameTime%50==0)
		{
			AverageGray=AveGray(inBufs->bufDesc[0].buf,FrameWidth,FrameHeight,pCfgs);
			pCfgs->AverageGray_num++;
			if (pCfgs->AverageGray_num>99)//
			{	
			
				tAverageGray=pCfgs->AverageGray_sum/pCfgs->AverageGray_num;
				if(pCfgs->former_tAverageGray>tAverageGray)
					change_speed_tAverageGray=(pCfgs->former_tAverageGray-tAverageGray)/pCfgs->former_tAverageGray;
				else
					change_speed_tAverageGray=(tAverageGray-pCfgs->former_tAverageGray)/tAverageGray;
				
				if(change_speed_tAverageGray>0.05&&pCfgs->former_tAverageGray!=0)
				{
					pCfgs->Fchange_speed_tAverageGray_num+=1;
				}
				else
				{
					pCfgs->Fchange_speed_tAverageGray_num=0;			
				}
				if (pCfgs->Fchange_speed_tAverageGray_num>=pCfgs->CameraCfg.uDayNightJudgeMinContiuFrame&&(pCfgs->CameraCfg.uEnvironmentStatus==1)&&(pCfgs->bAuto==1))
				{
					pCfgs->bAuto=2;
					pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight =TRUE ;
					pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bDay =FALSE ;
				}
				else if(pCfgs->Fchange_speed_tAverageGray_num>=pCfgs->CameraCfg.uDayNightJudgeMinContiuFrame&&(pCfgs->CameraCfg.uEnvironmentStatus==3)&&(pCfgs->bAuto==2))
				{
					pCfgs->bAuto=1;
					pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight =FALSE ;
					pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bDay =TRUE ;
				}
				pCfgs->AverageGray_num=0;
				pCfgs->AverageGray_sum=0;
				pCfgs->former_tAverageGray=tAverageGray;
			
			} 
			else
			{			
				pCfgs->AverageGray_sum=pCfgs->AverageGray_sum+AverageGray;
			}
				
		}
	}
	else if (pCfgs->CameraCfg.uEnvironmentStatus==2)
	{
		pCfgs->bAuto=2;
		pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight =TRUE ;
		pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bDay =FALSE ;
	
	}
	else if(pCfgs->CameraCfg.uEnvironmentStatus==4)
	{
		pCfgs->bAuto=1;
		pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight =FALSE;
		pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bDay =TRUE;
	
	}


	// pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight = FALSE;
	 
	pCfgs->bNight = pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight;	
	pCfgs->vehicleROI_num=0;
	pCfgs->candidateROI_num=0;
	//if(!pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight)
	{
		width=pCfgs->detectROI.width;
		height=pCfgs->detectROI.height;
		FrameWidth2 = FrameWidth << 1;
		FrameWidth4 = FrameWidth << 2;
		detectRoiX4 = pCfgs->detectROI.x << 2;
		pImg = (Uint8*)inBufs->bufDesc[0].buf;
        p0 = pParams->CurrFullImage; // sub image's first line
        p1 = pParams->CurrFullImage + (height - 1)* width; // sub image's last line
		p2 = pImg + pCfgs->detectROI.y * FrameWidth4 + detectRoiX4;
        p3 = pImg + (height - 1 + pCfgs->detectROI.y) * FrameWidth4 + detectRoiX4;
		for(k=0;k<width;k++)
		{
            j = k << 2 + 1;
            p0[k] = p2[j];
            p1[k] = p3[j];
		}
		/*for(j = 0; j < height; j++)//
		{
			*(pParams->CurrFullImage + j * width) = *(pImg + (j + pCfgs->detectROI.y) * FrameWidth4 + (pCfgs->detectROI.x) * 4 + 1);
			*(pParams->CurrFullImage + j * width + width - 1) = *(pImg + (j + pCfgs->detectROI.y) * FrameWidth4 + (width - 1 + pCfgs->detectROI.x) * 4 + 1);
		}*/
		p = pImg + (1 + pCfgs->detectROI.y) * FrameWidth4 + detectRoiX4 + 1;
		p1 = p0 + width;
		for(j=1;j<height-1;j++, p += FrameWidth4, p1 += width)
		{
			for(k=1;k<width-1;k++)
			{
                p2 = p + ( k << 2);
                p1[k] = (p2[0] + p2[-2] + p2[2] + p2[-FrameWidth2] + p2[FrameWidth2]) / 5;
			}
		}
	    get_target(pParams->CurrFullImage,pParams->PrevFullImage,pParams->MaskFullImage,pParams->puSobelTempImageX,width,height,pCfgs,pParams,0);
		memcpy((void *)pParams->PrevFullImage, (void *)pParams->CurrFullImage, width*height);
	}
	pCfgs->gThisFrameTime++;

	memset((void *)&pCfgs->ResultMsg, 0, sizeof(pCfgs->ResultMsg));
	memcpy((void *)&pCfgs->ResultMsg, (void *)outBuf, outSize);

	bSpeedEnable = (Bool)(pCfgs->CameraLocalPara.bNormalDetectEnable & 0x00000001);
	bPresenceEnable = (Bool)(pCfgs->CameraLocalPara.bNormalDetectEnable & 0x00000002);

	pCfgs->ResultMsg.uResultInfo.LaneSum = pCfgs->CameraCfg.LaneAmount;
	for( i = 0; i < pCfgs->CameraCfg.LaneAmount; i++ )
	{

		//exchange lane
		memcpy((void *)&pParams->LaneParameter, (void *)pCfgs->CameraLocalPara.LanesCfgEntry[i], sizeof(struct	LanesStruct));

		if(bSpeedEnable == TRUE)
		{
			pSpeedDetect = pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr;
     ////20140709       
            pCfgs->team_height=pSpeedDetect->QueDetectLines;
			pCfgs->team_width=pSpeedDetect->QueDetectColumns;
//			pCfgs->percent=pSpeedDetect->percent;

			//exchange 
			memcpy((void *)&pParams->NormalDetectCfg, (void *)pSpeedDetect, sizeof(struct	cSpeedDetectStruct));

			//exchange newimage 
			memcpy((void *)pParams->puPointNewImage, (void *)pSpeedDetect->CurrentImagePointer, pSpeedDetect->DetectDots);

			//exchange puPointSobelImage 
			memcpy((void *)pParams->puPointSobelImage, (void *)pSpeedDetect->puTempImage, pSpeedDetect->DetectDots);

			memcpy((void *)pParams->puSobelTempImageX, (void *)pSpeedDetect->BackImagePointer, pSpeedDetect->DetectDots);
			
			memcpy((void *)pParams->puBackImagePlus, (void *)pSpeedDetect->BackImagePlusPointer, pSpeedDetect->DetectDots);
			
					//txl,0630
			memcpy((void *)pParams->CurrQueueImage, (void *)pSpeedDetect->CurrQueueImage, pSpeedDetect->QueDetectDots);
			
			memcpy((void *)pParams->PreQueueImage, (void *)pSpeedDetect->PreQueueImage, pSpeedDetect->QueDetectDots);
			
			memcpy((void *)pParams->SobelQueueImage, (void *)pSpeedDetect->CurrQueueImage, pSpeedDetect->QueDetectDots);

			//memcpy((void *)pParams->FiltedSobelQueueImage, (void *)pSpeedDetect->CurrQueueImage, pSpeedDetect->QueDetectDots);
			memcpy((void *)pParams->BackQueueImage, (void *)pSpeedDetect->BackQueueImage, pSpeedDetect->QueDetectDots);
  
 

		////20131217  zcj
			SpeedCaculate( i, ChNum, pParams->puPointNewImage, outBuf, outSize,pCfgs, pParams);
			
 

			//if(pCfgs->gThisFrameTime%25==0)//当计算排队长度时，计算排队长度两点的坐标
			{
				m_ptend[0]= pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQ[0]; 
				m_ptend[1]= pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQ[1];
				m_ptend[2]= pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[3];  
				m_ptend[3]= pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[2];

				m_ptend[6]= pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQA[0];  
				m_ptend[7]= pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQA[1];  

				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.LineUp[0].x=0;
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.LineUp[0].y=0;
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.LineUp[1].x=0;
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.LineUp[1].y=0;

				LineUp[0] = pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.LineUp[0];
				LineUp[1] = pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.LineUp[1];

				pCfgs->percent = pParams->NormalDetectCfg.cSpeedDesc.percent;

				LineUp[0].x=(m_ptend[2].x+m_ptend[3].x)/2;

				if (m_ptend[2].x-m_ptend[3].x)
					LineUp[0].y=(m_ptend[2].y+m_ptend[3].y)/2;//(LineUp[0].x-m_ptend[3].x)*(m_ptend[2].y-m_ptend[3].y)/(m_ptend[2].x-m_ptend[3].x)+m_ptend[3].y;
				//if((m_ptend[0].x-m_ptend[3].x))
				//{	//LineUp[1].x=m_ptend[3].x+(LineUp[1].y-LineUp[0].y)*(m_ptend[0].x-m_ptend[3].x)/(m_ptend[0].y-m_ptend[3].y);
				LineUp[1].x=LineUp[0].x+((m_ptend[0].x+m_ptend[1].x)/2-LineUp[0].x)*pCfgs->percent;
				if((m_ptend[0].x+m_ptend[1].x)/2-LineUp[0].x)
					LineUp[1].y=LineUp[0].y-(double)(LineUp[0].y-(m_ptend[0].y+m_ptend[1].y)/2)/(double)(LineUp[0].x-(m_ptend[0].x+m_ptend[1].x)/2)*(LineUp[0].x-LineUp[1].x);
				else
					LineUp[1].y=LineUp[0].y-pCfgs->percent*pCfgs->team_height;

			}
			if (LineUp[1].y<(m_ptend[6].y+m_ptend[7].y)/2)
			{
				pCfgs->sptimes[i]++;
				pCfgs->noqtimes[i]=0;

			}
			else
			{
				pCfgs->noqtimes[i]++;
				pCfgs->sptimes[i]=0;

			}

			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.LineUp[0] = LineUp[0];
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.LineUp[1] = LineUp[1];
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.uVehicleQueueLength = (int)(pCfgs->actual_distance[i][LineUp[1].y] + 0.5);
			pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.uVehicleSpeed = pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.uVehicleQueueLength;
			if (pCfgs->sptimes[i]>50)
			{
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.AlarmLineflag=2;

				if(pCfgs->VehicleTeamSave[i]==1)//排队报警
				{
					pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.bInfoValid=1;
					pCfgs->VehicleTeamSave[i] =2;
				}
			}
			else if (pCfgs->noqtimes[i]>30)
			{
				pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.AlarmLineflag=1;
				if(pCfgs->VehicleTeamSave[i]==2)//结束排队报警
				{
					pCfgs->ResultMsg.uResultInfo.uEachLaneData[i].SpeedDetectInfo.bInfoValid=1;
					pCfgs->VehicleTeamSave[i] =1;
				}
			}
			//pCfgs->guTestA[i] = pCfgs->sptimes[i];

			memcpy((void *)pSpeedDetect->puTempImage, (void *)pParams->puPointNewImage, pParams->NormalDetectCfg.cSpeedDesc.DetectDots);
			memcpy((void *)pCfgs->CameraLocalPara.dSpeedIdIndex[i].ExtStorageAddr, (void *)&pParams->NormalDetectCfg, sizeof(struct	cSpeedDetectStruct));
			memcpy((void *)pSpeedDetect->BackImagePlusPointer, (void *)pParams->puBackImagePlus, pSpeedDetect->DetectDots);


			////20140708  queue
			memcpy((void *)pSpeedDetect->PreQueueImage, (void *)pParams->CurrQueueImage, pParams->NormalDetectCfg.cSpeedDesc.QueDetectDots);
			memcpy((void *)pSpeedDetect->BackQueueImage, (void *)pParams->BackQueueImage, pParams->NormalDetectCfg.cSpeedDesc.QueDetectDots);
			/*if (pCfgs->gThisFrameTime%25==0)
			{
				memcpy((void *)pSpeedDetect->FiltedSobelQueueImage, (void *)pParams->SobelQueueImage, pParams->NormalDetectCfg.cSpeedDesc.QueDetectDots);
			}*/

		}
		//
		if(bPresenceEnable == TRUE)
		{
			//reserved
			
		}//bPresenceEnable END

		//
		memcpy((void *)pCfgs->CameraLocalPara.LanesCfgEntry[i], (void *)&pParams->LaneParameter, sizeof(struct	LanesStruct));

	}//for ends
////20131217   zcj
	/*pParams->NormalDetectCfg.cSpeedDesc.track_flag++;//?????????????????
	if(pParams->NormalDetectCfg.cSpeedDesc.track_flag>=20) 
	    pParams->NormalDetectCfg.cSpeedDesc.track_flag=0;*/

	//
	pCfgs->ResultMsg.uMsgHeader.uFlag = 0x55;
	pCfgs->ResultMsg.uMsgHeader.uCmd = 0xE0E0;

	pCfgs->ResultMsg.uResultInfo.uEnvironmentStatus = 	pCfgs->bAuto; //added by david 20131014
	pCfgs->ResultMsg.uResultInfo.bVisibility = (pCfgs->visibility) ?  1 : 0;
	pCfgs->ResultMsg.uResultInfo.bAabnormalFlag = (pCfgs->abnormalflag) ? 1 : 0;
	memcpy((void *)outBuf, (void *)&pCfgs->ResultMsg, outSize);

	return 1;///change the results(1: SUCCEED)

}

int sort_aver(int array[10],int num)
{
	Uint16 i=0,j=0;
	int temp=0;
	int array_temp[10]={0};
	memcpy(array_temp,array,10*sizeof(int));
	for(i=0;i<=num/2;i++)
	{
		for(j=i+1;j<num;j++)
		{
			if(array_temp[i]>array_temp[j])
			{
				temp=array_temp[i];
				array_temp[i]=array_temp[j];
				array_temp[j]=temp;
			}
		}
	}
	return array_temp[num/2];
}
void get_averback(ALGPARAMS *pParams,ALGCFGS *pCfgs,unsigned char* currImage,bool bCarExist)
{
	Uint16 aver_idx=0;
	int aver_back=0;
	Uint16 i=0;
	if(pCfgs->gThisFrameTime==0)
	{
		memset( pParams->NormalDetectCfg.cSpeedDesc.stat_aver_back,0,10*sizeof(int) );


	}
	if(pParams->NormalDetectCfg.cSpeedDesc.bGet1backflag&&(pParams->NormalDetectCfg.cSpeedDesc.uTrafficFlow>10||pParams->NormalDetectCfg.cSpeedDesc.begin_update_standard_deviation))
	{
		if(pParams->NormalDetectCfg.cSpeedDesc.outflag%3000==0&&pParams->NormalDetectCfg.cSpeedDesc.m_uContiuCount>5)
		{
			memset( pParams->NormalDetectCfg.cSpeedDesc.stat_aver_back,0,10*sizeof(int) );

		}
		if (pCfgs->gThisFrameTime%100==0&&(pParams->NormalDetectCfg.cSpeedDesc.outflag==0||(pParams->NormalDetectCfg.cSpeedDesc.outflag>100&&bCarExist==FALSE))&&pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged==FALSE&&pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged1==FALSE&&pParams->NormalDetectCfg.cSpeedDesc.updateParams==FALSE)
		{
			pParams->NormalDetectCfg.cSpeedDesc.updateParams=TRUE;
			aver_back=AveGray(currImage,pCfgs->m_iWidth,pCfgs->m_iHeight,pCfgs);
			aver_idx=pParams->NormalDetectCfg.cSpeedDesc.stat_aver_back[9];
			if(aver_idx<9)
			{
				pParams->NormalDetectCfg.cSpeedDesc.stat_aver_back[aver_idx]=aver_back;
				pParams->NormalDetectCfg.cSpeedDesc.stat_aver_back[9] += 1;
				if(pParams->NormalDetectCfg.cSpeedDesc.stat_aver_back[9]>4)
				{
					pParams->NormalDetectCfg.cSpeedDesc.aver_back=sort_aver(pParams->NormalDetectCfg.cSpeedDesc.stat_aver_back,aver_idx+1);
				}
			}
			else
			{
				for (i=0;i<9;++i)
				{
					pParams->NormalDetectCfg.cSpeedDesc.stat_aver_back[i]=pParams->NormalDetectCfg.cSpeedDesc.stat_aver_back[i+1];
				}
				pParams->NormalDetectCfg.cSpeedDesc.stat_aver_back[8]=aver_back;
			}
		}
		if(pCfgs->gThisFrameTime%1000==0)
		{
			aver_idx=pParams->NormalDetectCfg.cSpeedDesc.stat_aver_back[9];
			aver_back=sort_aver(pParams->NormalDetectCfg.cSpeedDesc.stat_aver_back,aver_idx);
			if(aver_idx>=9)
			{
				if(pParams->NormalDetectCfg.cSpeedDesc.aver_back>1)
				{
					if(abs(pParams->NormalDetectCfg.cSpeedDesc.aver_back-aver_back)>30)
					{
						memset( pParams->NormalDetectCfg.cSpeedDesc.stat_aver_back,0,10*sizeof(int) );

					}
					else
					{
						pParams->NormalDetectCfg.cSpeedDesc.aver_back = (pParams->NormalDetectCfg.cSpeedDesc.aver_back * 922 + aver_back * 102) >> 10;
					}
				}
				else
					pParams->NormalDetectCfg.cSpeedDesc.aver_back=aver_back;
			}
			else if(pParams->NormalDetectCfg.cSpeedDesc.aver_back>1&&aver_idx)
			{
				pParams->NormalDetectCfg.cSpeedDesc.aver_back=aver_back;
			}
		}
	}
}
void SpeedCaculate(Uint16 LaneID, Uint16 nChannelNum, Uint8* inBuf, Uint8* outBuf, Int32 outSize,ALGCFGS *pCfgs, ALGPARAMS	*pParams)
{
//	Uint16 uLines;
//	Uint16 bCountCalcluate = TRUE;
	Uint16 bSpeedCalculate = TRUE;
//	Uint16 bLengthCalculate = TRUE;
//	VEHICLE_SPEEDINFO TempVehicleSpeedInfo;
//	VEHICLE_LENGTHINFO TempVehicleLengthInfo;
	SPEED_COUNTER_STATUS SpeedStatus;
	struct 	cSpeedDetectStruct *pSpeedDetect = NULL;
//	Int16  nVehicleHeadRow;
//	float s, t, fCurrentSpeed, fCurrentLength;
	
//	Uint16		uSpeed;
//	Uint16		uLength;
//	Int16		tempSize;
	
	//int txlposition1 = 0;
	//int txlposition2 = 0;
	int standard_deviation=0;
	//bool bWuChe = TRUE;
	int Elements;
////20140707 	
	double teamlength=0;

	double teamtest;
	Uint8* inBufQ;
	bool bBack=FALSE;
	int aver_curr=0;
	Uint16 i=0,j=0;
	Uint16 flag=0;
	Uint16 vehicle_bottom=0;
	bool bCarIn=FALSE,bCarExist=FALSE,bCarExit=FALSE,inLaneCar = FALSE;
	int x1=0,x2=0,x3=0;
	Uint16 detectId=1001;
	CPoint ptFourCorner[4];
	int startX =0, startY = 0, endX = 0, endY = 0;
	int nonZeroNum = 0;
	bool bQueueBack = FALSE;
	ptFourCorner[0]=pCfgs->DownSpeedCfg.SpeedEachLane[LaneID].ptFourCorner[0];
	ptFourCorner[1]=pCfgs->DownSpeedCfg.SpeedEachLane[LaneID].ptFourCorner[1];
	ptFourCorner[2]=pCfgs->DownSpeedCfg.SpeedEachLane[LaneID].ptFourCorner[2];
	ptFourCorner[3]=pCfgs->DownSpeedCfg.SpeedEachLane[LaneID].ptFourCorner[3];
	inBufQ = pParams->CurrQueueImage;
//	uLines = pParams->NormalDetectCfg.cSpeedDesc.DetectLines;
//	uColumns = NormalDetectPtr->cSpeedDesc.DetectColumns;
	Elements = pParams->NormalDetectCfg.cSpeedDesc.DetectDots;

	pCfgs->m_iHeight = pParams->NormalDetectCfg.cSpeedDesc.DetectLines;
	pCfgs->m_iWidth = pParams->NormalDetectCfg.cSpeedDesc.DetectColumns;

	pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight = pCfgs->bNight;

	ShallowLightRemovePreProcess( inBuf, pParams->puPointSobelImage, pParams->puSobelTempImageX, \
		pParams->puSobelTempImageY, &(pParams->NormalDetectCfg.cSpeedDesc),nChannelNum, pCfgs, pParams );
	pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged = bIsSpeedCounterChanged( pParams->puPointSobelImage, \
		pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterStartRow, \
		&pParams->NormalDetectCfg.cSpeedDesc.SpeedCounterStatus, \
		0, pCfgs, pParams);//pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterStartRow->pCfgs->m_iHeight/2
	pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged1 = bIsSpeedCounterChanged1( pParams->puPointSobelImage, \
		SPEED_STARTER_ENDLINE, \
		&pParams->NormalDetectCfg.cSpeedDesc.SpeedCounterStatus,\
		0,LaneID, pCfgs, pParams );
	if (pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight) // 夜间不进行背景差
	{
		pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged2 = pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged;
	}
	else
	{	
		//SobelCalculate(inBuf,pParams->puPointNextImage,pCfgs->m_iWidth,pCfgs->m_iHeight);
		standard_deviation=StandardDeviation (inBuf,pCfgs->m_iWidth,pCfgs->m_iHeight,0,pCfgs->m_iHeight);
		if ((standard_deviation!=0)&&(standard_deviation-pParams->NormalDetectCfg.cSpeedDesc.min_standard_deviation)<10/*&&((pParams->NormalDetectCfg.cSpeedDesc.cur_part_std[5]-pParams->NormalDetectCfg.cSpeedDesc.min_part_std[5])<10)*/
			&&standard_deviation<50&&(pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged == FALSE)&&(pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged1 == FALSE))//(bIsSobleBlockUnChanged( pParams->puPointSobelImage, SPEED_STARTER_STARTLINE, SPEED_STARTER_ENDLINE ,0, pCfgs, pParams)
		{
			if(pCfgs->CameraCfg.uEnvironmentStatus==4)
			{
				if (pParams->NormalDetectCfg.cSpeedDesc.outflag&&pParams->NormalDetectCfg.cSpeedDesc.aver_back)
				{

					aver_curr=AveGray(inBuf,pCfgs->m_iWidth,pCfgs->m_iHeight,pCfgs); 
					bBack=(pParams->NormalDetectCfg.cSpeedDesc.outflag>10&&abs(aver_curr-pParams->NormalDetectCfg.cSpeedDesc.aver_back)<20);			
				}
				else
				{
					bBack=TRUE;
				}
			}		
			else
			{
				bBack=TRUE;
			}
			if(bBack)
			{
				memcpy((void *)pParams->puBackImagePlus,(void *)inBuf , Elements);//pSpeedDetect->puTempImage
				pParams->NormalDetectCfg.cSpeedDesc.bGet1backflag=TRUE;
				pParams->NormalDetectCfg.cSpeedDesc.bGetBackFlag=1;
			}
		}
		if(pParams->NormalDetectCfg.cSpeedDesc.bGetBackFlag==TRUE)
		{
			pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged2 = bIsSpeedCounterChanged3( pParams->puPointNextImage,pParams->puPointSobelImage, \
				inBuf, \
				pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterStartRow, \
				pParams->puBackImagePlus, pCfgs, pParams );
		}
		else	
		{			
			pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged2 = bIsSpeedCounterChanged2(pParams->puPointNextImage , pParams->puPointSobelImage, \
				pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterStartRow, \
				&pParams->NormalDetectCfg.cSpeedDesc.SpeedCounterStatus, \
				pCfgs, pParams );
		}
	}
	//分析目标
	//计算车辆离开线圈的位置和帧数，用于计算速度
	if(pCfgs->targets_size)
	{
		for (i=0;i<pCfgs->targets_size;++i)
		{
			if (pCfgs->targets[i].vehicle_id==pParams->NormalDetectCfg.cSpeedDesc.DetectId)
			{
				pParams->NormalDetectCfg.cSpeedDesc.position2 = pCfgs->targets[i].vehicle.y + pCfgs->targets[i].vehicle.height / 2;//车辆在线圈的位置
				pParams->NormalDetectCfg.cSpeedDesc.secondline = pCfgs->gThisFrameTime;
				break;
			}
		}
	}
	if(pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight)
	{
		if(pCfgs->candidate_targets_size)
		{
			for (i=0;i<pCfgs->candidate_targets_size;++i)
			{
				vehicle_bottom=(pCfgs->candidate_targets[i].vehicle.y+pCfgs->candidate_targets[i].vehicle.height)*2;
				if(vehicle_bottom*2>(ptFourCorner[0].y+ptFourCorner[1].y))
				{
					x1=(vehicle_bottom-ptFourCorner[0].y)*(ptFourCorner[2].x-ptFourCorner[0].x)/(ptFourCorner[2].y-ptFourCorner[0].y)+ptFourCorner[0].x;
					x2=(vehicle_bottom-ptFourCorner[1].y)*(ptFourCorner[3].x-ptFourCorner[1].x)/(ptFourCorner[3].y-ptFourCorner[1].y)+ptFourCorner[1].x;
					x3=min(x2,(pCfgs->candidate_targets[i].vehicle.x+pCfgs->candidate_targets[i].vehicle.width)*2)-max(x1,2*pCfgs->candidate_targets[i].vehicle.x);
					if(x3 > pCfgs->candidate_targets[i].vehicle.width)
					{	
						inLaneCar = TRUE;
					}

				}
			}
		}
	}
	flag=0;
	//pCfgs->guTest=0;
	if(pCfgs->targets_size)
	{
		for (i=0;i<pCfgs->targets_size;++i)
		{
			vehicle_bottom=(pCfgs->targets[i].vehicle.y+pCfgs->targets[i].vehicle.height)*2;
			if(vehicle_bottom*2>(ptFourCorner[0].y+ptFourCorner[1].y))
			{
				x1=(vehicle_bottom-ptFourCorner[0].y)*(ptFourCorner[2].x-ptFourCorner[0].x)/(ptFourCorner[2].y-ptFourCorner[0].y)+ptFourCorner[0].x;
				x2=(vehicle_bottom-ptFourCorner[1].y)*(ptFourCorner[3].x-ptFourCorner[1].x)/(ptFourCorner[3].y-ptFourCorner[1].y)+ptFourCorner[1].x;
				x3=min(x2,(pCfgs->targets[i].vehicle.x+pCfgs->targets[i].vehicle.width)*2)-max(x1,2*pCfgs->targets[i].vehicle.x);
				if(x3 > pCfgs->targets[i].vehicle.width)
				{	
					inLaneCar = TRUE;
					if(pParams->NormalDetectCfg.cSpeedDesc.DetectId!=pCfgs->targets[i].vehicle_id)
					{
						detectId=1001;
						for (j=0;j<pCfgs->targets_size;++j)
						{
							if(pCfgs->targets[j].vehicle_id==pParams->NormalDetectCfg.cSpeedDesc.DetectId)
							{
								detectId=j;
								break;
							}
						}
						if(detectId<1001)//存在
						{
							if(pCfgs->targets[i].flow_count==FALSE&&(pCfgs->gThisFrameTime-pParams->NormalDetectCfg.cSpeedDesc.jgflag>30)&&pParams->NormalDetectCfg.cSpeedDesc.waitDetect==FALSE)
							{
								//pCfgs->guTest=1;
								pParams->NormalDetectCfg.cSpeedDesc.DetectId=pCfgs->targets[i].vehicle_id;
								pCfgs->targets[i].flow_count=TRUE;
								pParams->NormalDetectCfg.cSpeedDesc.waitDetect=TRUE;
                                DeleteTarget(&pCfgs->targets_size, &detectId, pCfgs->targets);
								bCarExit=TRUE;
								//bCarExist=TRUE;
								if(pCfgs->uCarExistStatis[LaneID]==1)
									pCfgs->uCarExistStatis[LaneID]=2;
								break;

							}
						}
						else
						{
							if(pCfgs->targets[i].flow_count==FALSE)
							{
								if(pParams->NormalDetectCfg.cSpeedDesc.calarflag&&pCfgs->uCarExistStatis[LaneID]==1&&(pCfgs->gThisFrameTime-pParams->NormalDetectCfg.cSpeedDesc.jgflag)>=6)
								{
									if(pCfgs->targets[i].num_detected>=2&&vehicle_bottom*2<(ptFourCorner[2].y+ptFourCorner[3].y))
									{
										//pCfgs->guTest=2;
										pParams->NormalDetectCfg.cSpeedDesc.DetectId=pCfgs->targets[i].vehicle_id;
										pCfgs->targets[i].flow_count=TRUE;
										pParams->NormalDetectCfg.cSpeedDesc.waitDetect=TRUE;
										//bCarExist=TRUE;
										pCfgs->uCarExistStatis[LaneID]=2;
										break;
									}
								}
								if(pCfgs->uCarExistStatis[LaneID]==0)
								{
									bCarIn=TRUE;
									pCfgs->targets[i].flow_count=TRUE;
									pParams->NormalDetectCfg.cSpeedDesc.DetectId=pCfgs->targets[i].vehicle_id;
								}
							}
						}
					}
					//if(pCfgs->targets[i].flow_count)
					//	bCarExist=TRUE;	
				}
			}
			if(pCfgs->targets[i].vehicle_id==pParams->NormalDetectCfg.cSpeedDesc.DetectId)
			{
				flag=1;
				bCarExist=TRUE;
				if(pCfgs->targets[i].exist_num>30)
				{
					if (pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight)
					{
						startX = pCfgs->targets[i].vehicle.x - pCfgs->detectROI.x;
						startY = pCfgs->targets[i].vehicle.y - pCfgs->detectROI.y;
						endX = startX + pCfgs->targets[i].vehicle.width;
						endY = startY + pCfgs->targets[i].vehicle.height;
						if ((pCfgs->targets[i].num_lost_detected > 5) /*&& (vehicle_bottom * 2 > (m_ptend[0].y + m_ptend[2].y))*/)
						{
							// 夜间统计亮像素个数，因车灯在目标区域的下侧，这里从0.6倍高度开始统计非零像素个数
							nonZeroNum = CountNonZero(pParams->CurrFullImage,
								startX, startY + (600 * pCfgs->targets[i].vehicle.height) >> 10,
								endX, endY, pCfgs->detectROI.width, 180);
							if (nonZeroNum < 50)
							{
								flag=0;
								bCarExist=FALSE;
								DeleteTarget(&pCfgs->targets_size, &i, pCfgs->targets);
								bCarExit=TRUE;
								break;
							}
						}
					}
					else
					{
						if(bBack/*&&(vehicle_bottom*2>(m_ptend[0].y+m_ptend[2].y))*/&&(pParams->NormalDetectCfg.cSpeedDesc.begin_update_standard_deviation||pCfgs->uDetectOutSum[LaneID]>10)/*&&pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged==FALSE&&pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged1==FALSE&&pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged2==FALSE*/)//??????????
						{
							//pCfgs->guTest=3;
							flag=0;
							bCarExist=FALSE;
							DeleteTarget(&pCfgs->targets_size, &i, pCfgs->targets);
							bCarExit=TRUE;
							break;
						}
					}
				}
			}
		}
	}
	if(flag==0)
	{
		bCarExit=TRUE;
		if(pParams->NormalDetectCfg.cSpeedDesc.waitDetect == FALSE)
		{
			pParams->NormalDetectCfg.cSpeedDesc.yellowflag++;
		}
	}
	if(pParams->NormalDetectCfg.cSpeedDesc.waitDetect&&pCfgs->uCarExistStatis[LaneID]==0)
	{
		pParams->NormalDetectCfg.cSpeedDesc.waitDetect=FALSE;
		bCarIn=TRUE;
	}
	if (!pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight)
	{
		updateMinStd(standard_deviation,pParams,bCarExist);
		get_averback(pParams,pCfgs,inBuf,bCarExist);
	}
	pParams->LaneParameter.TotalFrameCount++;
	
	//QueueLength
	if(pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight ==TRUE )
	{
		 
		  //NightLightDetect(inBuf,pParams->puPointSobelImage,pCfgs,pParams);
		  //ProjectSubAndSobelEdge(pParams->puPointSobelImage,pCfgs,pParams);

////20140707  
		if(pCfgs->gThisFrameTime % 10 == 0 && inLaneCar == FALSE)
		{
			memcpy((void *)pParams->BackQueueImage, (void *)pParams->CurrQueueImage, pCfgs->team_width * pCfgs->team_height);
			pParams->NormalDetectCfg.cSpeedDesc.getQueback_flag = TRUE;
		}
		if (pCfgs->gThisFrameTime%25==0 && pParams->NormalDetectCfg.cSpeedDesc.getQueback_flag)
		{
			//teamlength=30*TeamLength(pParams->puPointNextImage,inBuf,pCfgs,pParams,1,LineUp);
			memcpy((void *)pParams->puSobelTempImageY, (void *)pParams->BackQueueImage, pCfgs->team_width * pCfgs->team_height);
			iSubStractImage1(pParams->CurrQueueImage, pParams->puSobelTempImageY, 15, 0, pCfgs->team_height, pCfgs->team_width);
			teamlength=100*TeamLength1(pParams->puSobelTempImageY,pCfgs,pParams,pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo.LineUp,inLaneCar,LaneID);

		}
	
	
	}
	else
	{

		if(pCfgs->gThisFrameTime % 2 == 0)
		{
			memcpy((void *)pParams->puSobelTempImageY, (void *)pParams->PreQueueImage, pCfgs->team_width * pCfgs->team_height);
			iSubStractImage(pParams->CurrQueueImage, pParams->puSobelTempImageY, 3, 0, pCfgs->team_height, pCfgs->team_width);
			pParams->NormalDetectCfg.cSpeedDesc.cur_part_std[0] = StandardDeviation(pParams->puSobelTempImageY, pCfgs->team_width, pCfgs->team_height, 0, pCfgs->team_height / 4);
			pParams->NormalDetectCfg.cSpeedDesc.cur_part_std[1] = StandardDeviation(pParams->puSobelTempImageY, pCfgs->team_width, pCfgs->team_height, pCfgs->team_height / 4, pCfgs->team_height / 2);
			pParams->NormalDetectCfg.cSpeedDesc.cur_part_std[2] = StandardDeviation(pParams->puSobelTempImageY, pCfgs->team_width,pCfgs->team_height,pCfgs->team_height / 2, pCfgs->team_height * 3 / 4);
			pParams->NormalDetectCfg.cSpeedDesc.cur_part_std[3] = StandardDeviation(pParams->puSobelTempImageY, pCfgs->team_width, pCfgs->team_height, pCfgs->team_height * 3 / 4, pCfgs->team_height);
			//20151226
			if (pParams->NormalDetectCfg.cSpeedDesc.cur_part_std[0] > 10)
			{
				pParams->NormalDetectCfg.cSpeedDesc.cur_maxstd_pos = 0;
			}
			else if (pParams->NormalDetectCfg.cSpeedDesc.cur_part_std[1] > 10)
			{
				pParams->NormalDetectCfg.cSpeedDesc.cur_maxstd_pos = 1;
			}
			else if (pParams->NormalDetectCfg.cSpeedDesc.cur_part_std[2] > 10)
			{
				pParams->NormalDetectCfg.cSpeedDesc.cur_maxstd_pos = 2;
			}
			else if (pParams->NormalDetectCfg.cSpeedDesc.cur_part_std[3] > 10)
			{
				pParams->NormalDetectCfg.cSpeedDesc.cur_maxstd_pos = 3;
			}
			else 
			{
				pParams->NormalDetectCfg.cSpeedDesc.cur_maxstd_pos = 4;
			}
			if (pParams->NormalDetectCfg.cSpeedDesc.cur_part_std[0] < 5 &&
				pParams->NormalDetectCfg.cSpeedDesc.cur_part_std[1] < 5 &&
				pParams->NormalDetectCfg.cSpeedDesc.cur_part_std[2] < 5 &&
				pParams->NormalDetectCfg.cSpeedDesc.cur_part_std[3] < 5)
			{
				pParams->NormalDetectCfg.cSpeedDesc.nullflag++;
			} 
			else
			{
				pParams->NormalDetectCfg.cSpeedDesc.nullflag = 0;
			}

			if ((pCfgs->visibility == FALSE) && pParams->NormalDetectCfg.cSpeedDesc.nullflag > 5 && pParams->NormalDetectCfg.cSpeedDesc.begin_update_standard_deviation == TRUE
				&& (standard_deviation - pParams->NormalDetectCfg.cSpeedDesc.min_standard_deviation) < 10
				&& ((pParams->NormalDetectCfg.cSpeedDesc.cur_maxstd_pos == 4 && pParams->NormalDetectCfg.cSpeedDesc.last_maxstd_pos == 3) || (pParams->NormalDetectCfg.cSpeedDesc.getQueback_flag == FALSE && pParams->NormalDetectCfg.cSpeedDesc.framecount > 1500)))
			{
				memcpy((void *)pParams->BackQueueImage, (void *)pParams->CurrQueueImage, pCfgs->team_width * pCfgs->team_height);
				pParams->NormalDetectCfg.cSpeedDesc.getQueback_flag = TRUE;
				pParams->NormalDetectCfg.cSpeedDesc.percent = 0;
				pParams->NormalDetectCfg.cSpeedDesc.framecount = 0;
				bQueueBack = TRUE;

			}
			else
			{
				pParams->NormalDetectCfg.cSpeedDesc.framecount++;
			}
			if (pParams->NormalDetectCfg.cSpeedDesc.last_maxstd_pos<4)
			{
				pParams->NormalDetectCfg.cSpeedDesc.last_maxstd_pos = (pParams->NormalDetectCfg.cSpeedDesc.cur_maxstd_pos < 4) ? pParams->NormalDetectCfg.cSpeedDesc.cur_maxstd_pos : pParams->NormalDetectCfg.cSpeedDesc.last_maxstd_pos;
			}
		}
		if(bQueueBack == FALSE)
		{
			if ((pCfgs->gThisFrameTime % 25 == 0) && (pCfgs->DownSpeedCfg.SpeedEachLane[0].ptCornerQ[0].x != 0))
			{		

				memcpy((void *)pParams->puSobelTempImageY, (void *)pParams->BackQueueImage, pCfgs->team_height * pCfgs->team_height);//背景差
				//iSubStractImage(pParams->CurrQueueImage, pParams->PreQueueImage, 15, 0, pCfgs->team_height, pCfgs->team_height);//帧差
				//iSubStractImage(pParams->CurrQueueImage, pParams->puSobelTempImageY, 3, 0, pCfgs->team_height, pCfgs->team_height);
				iSubStractImage2(pParams->CurrQueueImage, pParams->PreQueueImage, pParams->puSobelTempImageY, 15, 3, 0, pCfgs->team_height, pCfgs->team_height);//帧差和背景差
				SobelCalculate(pParams->puSobelTempImageY, pParams->SobelQueueImage, 100, pCfgs->team_width, pCfgs->team_height);//前景上计算边缘图像
				//SobelCalculate(pParams->CurrQueueImage,pParams->SobelQueueImage,100,pCfgs->team_width,pCfgs->team_height);//前景上计算边缘图像
				SobelConectivity(pParams->SobelQueueImage, pParams->puSobelTempImageX, pCfgs, pParams);	//边缘图像进行连通域分析
				teamtest = QueueLengthDetect(pParams->CurrQueueImage, pParams->PreQueueImage, pParams->SobelQueueImage, pParams->puSobelTempImageX, pCfgs, pParams,LaneID);

			}
		}
	}
	//pCfgs->guTestD[LaneID] = pParams->NormalDetectCfg.cSpeedDesc.getQueback_flag;
	//
	// parse the result, check the obj, calc the speed and length
	//
	if( pCfgs->CameraCfg.bCameraWorkInCross == 1 )
	{
		

	}
	else if(pCfgs->CameraCfg.bCameraWorkInCross == 2) 
	{
		while( bSpeedCalculate == TRUE )
		{
			bSpeedCalculate = FALSE;
	
			//if( pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight == FALSE )
			//{

			
				if( pParams->NormalDetectCfg.cSpeedDesc.uCountIntervalFrame < pCfgs->guMinCountIntervalFrame )
				{
					pParams->NormalDetectCfg.cSpeedDesc.uCountIntervalFrame++;
					break;
				}
				else if( pParams->NormalDetectCfg.cSpeedDesc.SpeedCounterStatus.bCanBeginCount == FALSE )
				{
					if(bCarIn)
					{
						if(pCfgs->gThisFrameTime-pParams->NormalDetectCfg.cSpeedDesc.jgflag<6)
							break;
						{
							pParams->NormalDetectCfg.cSpeedDesc.jgflag=pCfgs->gThisFrameTime;
						}
						//计算车辆进入线圈的位置和帧数，用于计算速度
						for (i=0;i<pCfgs->targets_size;++i)
						{
							if (pCfgs->targets[i].vehicle_id==pParams->NormalDetectCfg.cSpeedDesc.DetectId)
							{
								pParams->NormalDetectCfg.cSpeedDesc.position1 = pCfgs->targets[i].vehicle.y + pCfgs->targets[i].vehicle.height / 2;//进入线圈的初始位置
								break;
							}
						}
						pParams->NormalDetectCfg.cSpeedDesc.updateParams=FALSE;
						pCfgs->uCarExistStatis[LaneID]=1;
						pParams->NormalDetectCfg.cSpeedDesc.SpeedCounterStatus.bCanBeginCount = TRUE;
						pParams->NormalDetectCfg.cSpeedDesc.calarflag = 1;
						//pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo.calarflag=1;
						//pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo.car_in++;
						pCfgs->uDetectInSum[LaneID]++;
						pParams->NormalDetectCfg.cSpeedDesc.speedflag = 1;
						pParams->NormalDetectCfg.cSpeedDesc.firstline = pCfgs->gThisFrameTime;
						pParams->NormalDetectCfg.cSpeedDesc.lengthflag = 1;
						//pParams->NormalDetectCfg.cSpeedDesc.secondline = pCfgs->gThisFrameTime;
						pParams->NormalDetectCfg.cSpeedDesc.yellowflag = 0;
						if(pParams->NormalDetectCfg.cSpeedDesc.SpeedCounterStatus.bIsCounting==FALSE)
						{
							pParams->NormalDetectCfg.cSpeedDesc.SpeedCounterStatus.bIsCounting=TRUE;
						}
						pCfgs->ResultMsg.uMsgHeader.uFlag = 0x55;
						pCfgs->ResultMsg.uMsgHeader.uCmd = 0xE0E0;
						pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].uLaneID = LaneID;
						pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo.bInfoValid = 1; 	
						pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo.bVehicleSta = 1;
						pParams->NormalDetectCfg.cSpeedDesc.m_uContiuCount++;
					}
				}
				if(pParams->NormalDetectCfg.cSpeedDesc.SpeedCounterStatus.bCanBeginCount)
				{
					if(bCarExit)
					{
						pParams->NormalDetectCfg.cSpeedDesc.SpeedCounterStatus.bCanBeginCount=FALSE;
					}
				}
				/*detectId=2000;
				for(j=0;j<pCfgs->targets_size;j++)
				{
					if(pParams->NormalDetectCfg.cSpeedDesc.DetectId==pCfgs->targets[j].vehicle_id)
					{
						detectId=j;
						break;
					}
				}*/
				if((pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged2||pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged||pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged1||
					pParams->NormalDetectCfg.cSpeedDesc.outflag<4||bCarExit==FALSE||bCarExist/*||
					(pCfgs->targets_size>detectId&&pCfgs->targets[detectId].flow_count&&((pCfgs->targets[detectId].vehicle.y+pCfgs->targets[detectId].vehicle.height)*4>(m_ptend[8].y+m_ptend[9].y-20)&&(pCfgs->gThisFrameTime-pParams->NormalDetectCfg.cSpeedDesc.jgflag)<15))*/)&&pParams->NormalDetectCfg.cSpeedDesc.SpeedCounterStatus.bIsCounting)
				{
					pParams->NormalDetectCfg.cSpeedDesc.outflag++;
				}
				else 
				{
					pParams->NormalDetectCfg.cSpeedDesc.m_uContiuCount=0;
					//pParams->NormalDetectCfg.cSpeedDesc.yellowflag++;
					pParams->NormalDetectCfg.cSpeedDesc.calarflag = 0;
					//pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo.calarflag=0;
					pParams->NormalDetectCfg.cSpeedDesc.outflag=0;
					pParams->NormalDetectCfg.cSpeedDesc.SpeedCounterStatus.bIsCounting=FALSE;
					if(pCfgs->uCarExistStatis[LaneID]==1)
					{
						pCfgs->uCarExistStatis[LaneID]=2;
					}
				}

				if(pCfgs->uCarExistStatis[LaneID]==2)
				{
					pParams->NormalDetectCfg.cSpeedDesc.SpeedCounterStatus.bCanBeginCount=FALSE;
					//pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo.car_out++;
					pCfgs->uDetectOutSum[LaneID]++;
					pParams->NormalDetectCfg.cSpeedDesc.uTrafficFlow++;
					pCfgs->ResultMsg.uMsgHeader.uFlag = 0x55;
					pCfgs->ResultMsg.uMsgHeader.uCmd = 0xE0E0;
					pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].uLaneID = LaneID;
					//pParams->NormalDetectCfg.cSpeedDesc.wVehicleSpeed = (float)(pParams->NormalDetectCfg.cSpeedDesc.uTransFactor) / ((pCfgs->gThisFrameTime - pParams->NormalDetectCfg.cSpeedDesc.firstline) * 0.067) * 3.6;
					pParams->NormalDetectCfg.cSpeedDesc.wVehicleSpeed = (float)(pParams->NormalDetectCfg.cSpeedDesc.uTransFactor * (pParams->NormalDetectCfg.cSpeedDesc.position2 - pParams->NormalDetectCfg.cSpeedDesc.position1) / pCfgs->m_iHeight) / ((pParams->NormalDetectCfg.cSpeedDesc.secondline - pParams->NormalDetectCfg.cSpeedDesc.firstline) * 0.067) * 3.6;//15帧
					pParams->NormalDetectCfg.cSpeedDesc.wVehicleSpeed = (pParams->NormalDetectCfg.cSpeedDesc.wVehicleSpeed  < 0)? -pParams->NormalDetectCfg.cSpeedDesc.wVehicleSpeed : pParams->NormalDetectCfg.cSpeedDesc.wVehicleSpeed;
					if (pParams->NormalDetectCfg.cSpeedDesc.wVehicleSpeed > 60)
					{
						pParams->NormalDetectCfg.cSpeedDesc.wVehicleSpeed =50+rand()%20;// 100 - (pParams->NormalDetectCfg.cSpeedDesc.wVehicleSpeed - 100) * (pParams->NormalDetectCfg.cSpeedDesc.wVehicleSpeed - 100) / 100;
					}
					else if(pParams->NormalDetectCfg.cSpeedDesc.wVehicleSpeed<15)
						pParams->NormalDetectCfg.cSpeedDesc.wVehicleSpeed =10+rand()%20;
					pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo.uVehicleSpeed \
						=((unsigned int) pParams->NormalDetectCfg.cSpeedDesc.wVehicleSpeed);
					//pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo.uVehicleSpeed=pCfgs->guTest;

					pParams->NormalDetectCfg.cSpeedDesc.wVehicleLength = pParams->NormalDetectCfg. \
						cSpeedDesc.wVehicleSpeed * (pCfgs->gThisFrameTime - pParams->NormalDetectCfg.cSpeedDesc.firstline) * 0.067 / 3.6 - (float)(pParams->NormalDetectCfg.cSpeedDesc.uTransFactor);
					if( (pParams->NormalDetectCfg.cSpeedDesc.wVehicleLength > 4) \
						&&(pParams->NormalDetectCfg.cSpeedDesc.wVehicleLength < 7) )
					{
						pParams->NormalDetectCfg.cSpeedDesc.wVehicleLength = 4 + \
							(7 - pParams->NormalDetectCfg.cSpeedDesc.wVehicleLength) * \
							(pParams->NormalDetectCfg.cSpeedDesc.wVehicleLength - 4) / 4;
					}
					else if (pParams->NormalDetectCfg.cSpeedDesc.wVehicleLength > 7)
					{
						pParams->NormalDetectCfg.cSpeedDesc.wVehicleLength = rand() % 3 + 4;
					}
					else if(pParams->NormalDetectCfg.cSpeedDesc.wVehicleLength<3)
					{
						pParams->NormalDetectCfg.cSpeedDesc.wVehicleLength=rand() % 2+3;
					}
					pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo.uVehicleLength  \
						= ((unsigned int )pParams->NormalDetectCfg.cSpeedDesc.wVehicleLength);
					pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo.bInfoValid = 1; 
					pCfgs->ResultMsg.uResultInfo.uEachLaneData[LaneID].SpeedDetectInfo.bVehicleSta = 0;	
					pCfgs->uCarExistStatis[LaneID]=0;
				}

		} //while( bSpeedCalculate == TRUE ) ends

		
	}//if,cross
}

////20131218 add
bool bIsSpeedCounterChanged3(unsigned char* puSobelImage1, unsigned char* puSobelImage, unsigned char* GrayImage, unsigned short uSpeedCounterStartRow, unsigned char* BackImage, ALGCFGS *pCfgs, ALGPARAMS *pParams )
{
	int nRow=0,nCol=0;
	int value=0/*,svalue=0*/;
	int rcount=0,count=0;
	
	//int srcount=0,scount=0;
	//int nchangedRow=0;
	//unsigned int uThreshold = 20;
	//unsigned int uThreshold = pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold2; 
	int GrayImageValue=0,BackImageValue=0;
	
	int startline;
	//if(uSpeedCounterStartRow)
	//{
	//	startline=0/*pCfgs->m_iHeight/2*/;
	//}
	//else
	//{
		startline=0;
	//}
	for (nRow=startline;nRow<pCfgs->m_iHeight;nRow++)
	{
        Uint32 offset = nRow * pCfgs->m_iWidth;
        unsigned char* grayPtr = GrayImage + offset;
        unsigned char* backPtr = BackImage + offset;
		count = 0;
		for (nCol=0;nCol<pCfgs->m_iWidth;nCol++)
		{
            GrayImageValue = grayPtr[nCol];
            BackImageValue = backPtr[nCol];
		/*	value=abs(GetImagePointValue(nCol, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, GrayImage)
				-GetImagePointValue(nCol, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, BackImage));
		*/
			value=GrayImageValue - BackImageValue;

			if (value<20&&value>-20)//20
				count++;
			/*svalue=GetImagePointValue(nCol, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage1);
			if (svalue)
				scount++;*/
		}
		if (count>(/*pCfgs->m_iWidth>>1*/pCfgs->m_iWidth*3/4))
		{
			/*for (nCol=0;nCol<pCfgs->m_iWidth;nCol++)
			{
				SetImagePointValue(nCol,nRow,pCfgs->m_iHeight,pCfgs->m_iWidth,0,GrayImage);
			}*/
			//count=0;
			//rcount++;
		}
		else
		{
			//count=0;
			//rcount=0;
            rcount++;
		}
		
		

		//if (scount>(pCfgs->m_iWidth>>1))
		//{
			/*for (nCol=0;nCol<pCfgs->m_iWidth;nCol++)
			{
				SetImagePointValue(nCol,nRow,pCfgs->m_iHeight,pCfgs->m_iWidth,0,GrayImage);
			}*/
		//	scount=0;
		//	srcount++;
		//}
		//else
		//{
		//	scount=0;
		//	srcount=0;
		//}
	
	}
	/*for (nRow=startline; nRow<pCfgs->m_iHeight; nRow++)
	{
		if (GetImagePointValue( pCfgs->m_iWidth - 1, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage ) >uThreshold)
		{
			nchangedRow++;
		}
	}*/
////20140107
		if (rcount<pCfgs->m_iHeight/4)
		{
	
			return FALSE;//leave
		}
		else
		{
	
			return TRUE;//exist
		}
	
}

////20131218 delete
/*
bool bIsSpeedCounterChanged3( unsigned char* GrayImage, unsigned short uSpeedCounterStartRow, unsigned char* BackImage, ALGCFGS *pCfgs, ALGPARAMS *pParams )
{
	int nRow=0,nCol=0;
	int value=0;
	int rcount=0,count=0;
	
	for (nRow = uSpeedCounterStartRow + 2;nRow < pCfgs->m_iHeight -1; nRow++)
	{
		for (nCol = 0;nCol < pCfgs->m_iWidth;nCol++)
		{
			if ( (nRow == uSpeedCounterStartRow + 2) || (nRow == pCfgs->m_iHeight -1) )
			{
				SetImagePointValue(nCol,nRow,pCfgs->m_iHeight,pCfgs->m_iWidth,0,BackImage);
			}
			else if ( (nCol == 0) || (nCol == pCfgs->m_iWidth - 1) )
			{
				SetImagePointValue(nCol,nRow,pCfgs->m_iHeight,pCfgs->m_iWidth,0,BackImage);
			}
			else
			{
			
				value=(abs(GetImagePointValue(nCol-1, nRow-1, pCfgs->m_iHeight, pCfgs->m_iWidth, GrayImage)+2*
					GetImagePointValue(nCol, nRow-1, pCfgs->m_iHeight, pCfgs->m_iWidth, GrayImage)+
					GetImagePointValue(nCol+1, nRow-1, pCfgs->m_iHeight, pCfgs->m_iWidth, GrayImage)-
					GetImagePointValue(nCol-1, nRow+1, pCfgs->m_iHeight, pCfgs->m_iWidth, GrayImage)
					-2*GetImagePointValue(nCol, nRow+1, pCfgs->m_iHeight, pCfgs->m_iWidth, GrayImage)-
					GetImagePointValue(nCol+1,nRow+1, pCfgs->m_iHeight, pCfgs->m_iWidth, GrayImage)) );//y
				if (value>60)
					SetImagePointValue(nCol,nRow,pCfgs->m_iHeight,pCfgs->m_iWidth,1,BackImage);
				else
					SetImagePointValue(nCol,nRow,pCfgs->m_iHeight,pCfgs->m_iWidth,0,BackImage);
			}
		}
	}
	
	for (nCol = 0;nCol < pCfgs->m_iWidth;nCol++)
	{
		for (nRow = uSpeedCounterStartRow + 2;nRow < pCfgs->m_iHeight -1; nRow++)
		{
			if (GetImagePointValue( nCol , nRow , pCfgs->m_iHeight, pCfgs->m_iWidth, BackImage) > 0)
			{
				rcount++;
			}
		}
		if (rcount > 10)//50
			count++;
		if (count >= 100) {//50
			return TRUE;
		}
	}
	
	return FALSE;
}
*/




bool bIsSpeedCounterChanged1( unsigned char* puSobelImage, Uint16 EndLine, SPEED_COUNTER_STATUS* pSpeedCounterStatus ,int flag, Uint16 lane_id, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	int nLeftBlockColumn = pCfgs->m_iWidth / 3, nRightBlockColumn = pCfgs->m_iWidth * 2 / 3;
	int nPosSum = 0, nRowCount = 0, nRow;
	int nUnChangedRow = 0;
	unsigned int uThreshold ;//1
	unsigned int i;
	int k;
	unsigned char* ptr1 = 0;
    
	if (flag == 0)
	{
		//uThreshold = 10; //by david 20130905 from 5 to 2
		uThreshold = pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold1; //by david 20130905
		k = 2;
	
	    ptr1 = puSobelImage + pCfgs->m_iWidth + pCfgs->m_iWidth - 1;
		for( nRow = 1; nRow < EndLine; nRow++, ptr1 += pCfgs->m_iWidth)
		{
			if(*ptr1 < uThreshold )
			{
				//if (!nUnChangedRow)
				//{
					//Rowposition1 = nRow;  
				//}
				nUnChangedRow++;
			}
			if( (nUnChangedRow * k) > pCfgs->CameraCfg.uVehicleHeadMinRow )
			{
				pSpeedCounterStatus->bRightDestroyed = FALSE;
				pSpeedCounterStatus->bLeftDestroyed = FALSE;
	////20131218  zcj
				//for(i=0; i<20; i++)
				//{
				//	pParams->NormalDetectCfg.cSpeedDesc.rectify_lane_direction[i][lane_id][0]=FALSE;
				//	pParams->NormalDetectCfg.cSpeedDesc.rectify_lane_direction[i][lane_id][1]=0;
				//}
				return FALSE;
			}
		}
	}
	
	else////night 
	{

		uThreshold=pCfgs->m_iWidth / 5;
	 // 	uThreshold = pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold1; 
	
		k = 1;
		ptr1 = puSobelImage + pCfgs->m_iWidth - 1;
		for( nRow = 0; nRow < EndLine; nRow++, ptr1 += pCfgs->m_iWidth)
		{
			if(*ptr1 < uThreshold )
			{
				nUnChangedRow++;
			}
			if( (nUnChangedRow*k) >EndLine-3 )//why multipled by 2? david
			{
				pSpeedCounterStatus->bRightDestroyed = FALSE;
				pSpeedCounterStatus->bLeftDestroyed = FALSE;
				for(i=0;i<20;i++)
				{  
					pParams->NormalDetectCfg.cSpeedDesc.rectify_lane_direction[i][lane_id][0]= FALSE;
					pParams->NormalDetectCfg.cSpeedDesc.rectify_lane_direction[i][lane_id][1]=0;
					//rectify_lane_direction[i][lane_id][0]=false;
					//rectify_lane_direction[i][lane_id][1]=0;
				}
				return FALSE;
			}
		}
	}
	if( pSpeedCounterStatus->bIsCounting == FALSE ){
		ptr1 = puSobelImage + pCfgs->m_iWidth + pCfgs->m_iWidth - 1;
		for( nRow = 1; nRow < SPEED_STARTER_ENDLINE; nRow++, ptr1 += pCfgs->m_iWidth)
		{
		    if(*ptr1)
			{
			nPosSum += GetImagePointValue(0, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage );
			nRowCount++;
			}
		}
		if( (nPosSum  / nRowCount) > nRightBlockColumn )
		{
			//if(pParams->NormalDetectCfg.cSpeedDesc.track_flag==0)  
			//	pParams->NormalDetectCfg.cSpeedDesc.track_flag=20;
			if((( pCfgs->m_iWidth-(nPosSum/ nRowCount))>(pCfgs->m_iWidth/4))/*&&(pParams->NormalDetectCfg.cSpeedDesc.rectify_lane_direction[pParams->NormalDetectCfg.cSpeedDesc.track_flag-1][lane_id][0]==FALSE)*/)
			{   
			//	TRACE("\n2b[%d][%d][21]=%d\n",frame_track-1,lane_id,rectify_lane_direction[frame_track-1][lane_id][1]);
				pSpeedCounterStatus->bRightDestroyed = TRUE;
				pSpeedCounterStatus->bLeftDestroyed = FALSE;

			}
			else 
			{
				pSpeedCounterStatus->bRightDestroyed = FALSE;
				pSpeedCounterStatus->bLeftDestroyed = FALSE;

				//pParams->NormalDetectCfg.cSpeedDesc.rectify_lane_direction[pParams->NormalDetectCfg.cSpeedDesc.track_flag][lane_id][0]=TRUE;
				//pParams->NormalDetectCfg.cSpeedDesc.rectify_lane_direction[pParams->NormalDetectCfg.cSpeedDesc.track_flag][lane_id][1]+=1;
			////	TRACE("\nrectify_lane_direction[%d][%d][21]=%d\n",frame_track,lane_id,rectify_lane_direction[frame_track][lane_id][1]);
				return FALSE;
			}
		} 
		else if( (nPosSum  / nRowCount) < nLeftBlockColumn )
		{    
			//if(pParams->NormalDetectCfg.cSpeedDesc.track_flag==0) pParams->NormalDetectCfg.cSpeedDesc.track_flag=20;
//if(((nPosSum*1.0 / nRowCount)>(pCfgs->m_iWidth/4))&&(pParams->NormalDetectCfg.cSpeedDesc.rectify_lane_direction[pParams->NormalDetectCfg.cSpeedDesc.track_flag-1][lane_id][0]==FALSE))
			if(((nPosSum / nRowCount)>(pCfgs->m_iWidth/4))/*&&(pParams->NormalDetectCfg.cSpeedDesc.rectify_lane_direction[pParams->NormalDetectCfg.cSpeedDesc.track_flag-1][lane_id][0]==FALSE)*/)
			{
			
			//	TRACE("\n2b1[%d][%d][21]=%d\n",(frame_track-1),lane_id,rectify_lane_direction[frame_track-1][lane_id][1]);
				pSpeedCounterStatus->bLeftDestroyed = TRUE;
				pSpeedCounterStatus->bRightDestroyed = FALSE;


			}			
			else{
				pSpeedCounterStatus->bLeftDestroyed = FALSE;
				pSpeedCounterStatus->bRightDestroyed = FALSE;

				//pParams->NormalDetectCfg.cSpeedDesc.rectify_lane_direction[pParams->NormalDetectCfg.cSpeedDesc.track_flag][lane_id][0]=TRUE;
				//pParams->NormalDetectCfg.cSpeedDesc.rectify_lane_direction[pParams->NormalDetectCfg.cSpeedDesc.track_flag][lane_id][1]+=1;
				return FALSE;
			}
		}
		else{
			pSpeedCounterStatus->bRightDestroyed = TRUE;
			pSpeedCounterStatus->bLeftDestroyed = TRUE;
		}
	}
	return TRUE;
	
}

/*
bool bIsSpeedCounterChanged1( unsigned char* puSobelImage, Uint16 EndLine, SPEED_COUNTER_STATUS* pSpeedCounterStatus ,int flag, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	int nLeftBlockColumn = pCfgs->m_iWidth / 3, nRightBlockColumn = pCfgs->m_iWidth * 2 / 3;
	int nPosSum = 0, nRowCount = 0, nRow;
	int nUnChangedRow = 0;
	unsigned int uThreshold ;//1
	float k;
	if (flag == 0)
	{
		//uThreshold = 10; //by david 20130905 from 5 to 2
		uThreshold = pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold1; //by david 20130905
		k = 2;
	}
	else{
		uThreshold=pCfgs->m_iWidth / 5;
		k = 0.8;
	}
	for( nRow = 0; nRow < EndLine; nRow++ )
	{
		if( GetImagePointValue( pCfgs->m_iWidth - 1, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage ) < uThreshold )
		{
			if (!nUnChangedRow)
			{
				//Rowposition1 = nRow; 
			}
			nUnChangedRow++;
		}
		if( (nUnChangedRow * k) > pCfgs->CameraCfg.uVehicleHeadMinRow )
		{
			pSpeedCounterStatus->bRightDestroyed = FALSE;
			pSpeedCounterStatus->bLeftDestroyed = FALSE;
			return FALSE;
		}
	}
	
	if( pSpeedCounterStatus->bIsCounting == FALSE ){
		for( nRow = 0; nRow < SPEED_STARTER_ENDLINE; nRow++ )
		{
			nPosSum += GetImagePointValue(0, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage );
			nRowCount++;
		}
		if( (nPosSum * 1.0 / nRowCount) > nRightBlockColumn )
			pSpeedCounterStatus->bRightDestroyed = TRUE;
		else if( (nPosSum * 1.0 / nRowCount) < nLeftBlockColumn )
			pSpeedCounterStatus->bLeftDestroyed = TRUE;
		else{
			pSpeedCounterStatus->bRightDestroyed = TRUE;
			pSpeedCounterStatus->bLeftDestroyed = TRUE;
		}
	}
	return TRUE;
	
}
*/
////20131218  add
bool bIsSpeedCounterChanged2(unsigned char* puSobelImage1, unsigned char* puSobelImage, Uint16 uSpeedCounterStartRow, SPEED_COUNTER_STATUS* pSpeedCounterStatus, ALGCFGS *pCfgs, ALGPARAMS *pParams )
{
	int nRow=0/*,nCol=0*/;
	//int value=0;
	//int rcount=0,*ccount=NULL;
	//int hig=0,bel=0;
	int nUnChangedRow = 0;
//	unsigned int uThreshold = 10; //by david 20130905 from 1 to 10
	unsigned int uThreshold = pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold2; //by david 20130905 
//	int middleThreshold = 90;
	int startline = (uSpeedCounterStartRow)? pCfgs->m_iHeight/2 : 0;


	
	for( nRow = startline; nRow < pCfgs->m_iHeight; nRow++ )
	{
		if( GetImagePointValue( pCfgs->m_iWidth - 1, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage ) >uThreshold )//
			nUnChangedRow++;
		/*for (nCol=0; nCol<pCfgs->m_iWidth; nCol++)
		{
			if (GetImagePointValue( nCol, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage1 ) >uThreshold)
				count++;
		
		}
		if(count>pCfgs->m_iWidth/6)
			rcount++;
		count=0;*/
////20140107 
		if(pParams->NormalDetectCfg.cSpeedDesc.bGet1backflag)
		{
			if( (nUnChangedRow) > pCfgs->m_iHeight/5/*||rcount>pCfgs->m_iHeight/4*/)
			{
			
				return TRUE;//exist
			}
		}
		else
		{
		////20140108
		//	if( (nUnChangedRow) > pCfgs->m_iHeight/5||rcount>pCfgs->m_iHeight/5)//
			if( (nUnChangedRow) > pCfgs->m_iHeight/5/*||rcount>5*pCfgs->m_iHeight/12*/)//
			{
			
				return TRUE;//exist
			}
		}
	}

	
	return FALSE;
}

////20131218 delete
/*
bool bIsSpeedCounterChanged2( unsigned char* puSobelImage1, unsigned char* puSobelImage, Uint16 uSpeedCounterStartRow, SPEED_COUNTER_STATUS* pSpeedCounterStatus, ALGCFGS *pCfgs, ALGPARAMS *pParams )
{
	int nRow=0,nCol=0;
	//int value=0;
	//int rcount=0,*ccount=NULL;
	//int hig=0,bel=0;
	int nUnChangedRow = 0;
//	unsigned int uThreshold = 10; //by david 20130905 from 1 to 10
	unsigned int uThreshold = pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold2; //by david 20130905 
	int middleThreshold = 90;

	for( nRow = SPEED_STARTER_ENDLINE; nRow < uSpeedCounterStartRow + 2; nRow++ )
	{
		if( GetImagePointValue( pCfgs->m_iWidth - 1, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage ) >uThreshold )//txl<gaiwei>
			nUnChangedRow++;
		if( (nUnChangedRow) > 30)
		{
			
			return TRUE;
		}
	}
	
	return FALSE;
}
*/
//***********************************************
// newly added funcs end
//***********************************************

void ShallowLightRemovePreProcess(Uint8* puNewImage,Uint8* puTempImage,Uint8*puBackImage,Uint8* puTempStoreY,struct cSpeedDetectStruct* pSpeedDetectStruct,Uint16 uChanlNum,ALGCFGS *pCfgs, ALGPARAMS	*pParams)
{
//	SOBELINPUT SobelInput;
	//float dTempGrey;
//	bool bUpdateBack = FALSE;

	if( pCfgs->CameraCfg.bCameraWorkInCross == 1 )
	{
		

	}
	else if(pCfgs->CameraCfg.bCameraWorkInCross == 2)
	{
		//to add code not working in cross
		iSubStractImage( puNewImage, puTempImage, pCfgs->CameraCfg.guSubImageThreshold, 0, pCfgs->m_iHeight, pCfgs->m_iWidth);////
		
	//	pSpeedDetectStruct->SpeedDetectorWorkMode.bAuto = FALSE;
	//	pSpeedDetectStruct->SpeedDetectorWorkMode.bDay = FALSE;
				
		SLKCalculate( puTempImage, pCfgs->CameraCfg.uSobelThreshold, pCfgs, pParams );
		
		//////////////////get night or day info
		//if( bIsBlockAllEmpty( puTempImage, 1, SPEED_STARTER_ENDLINE, 1, 10, pCfgs, pParams ) == TRUE )
		//{
			//		fSetDetectStateFlag( NormalDetectPtr->cSpeedDesc.DetectId, FALSE );
		//	if( pSpeedDetectStruct->m_uContiuCount == 0 )
		//		pSpeedDetectStruct->m_uPassTimeCount++;
		//	pSpeedDetectStruct->m_uAvergreyMax = 0;
		//	pSpeedDetectStruct->m_uContiuCount = 1;
		//}
		//else if( pSpeedDetectStruct->m_uContiuCount != 0 )
		//{
			//		fSetDetectStateFlag( NormalDetectPtr->cSpeedDesc.DetectId, TRUE );
		//	dTempGrey = dAverageGreyInBlock( puNewImage, 1, SPEED_STARTER_ENDLINE, pCfgs, pParams );
		//	if( dTempGrey > (double)pSpeedDetectStruct->m_uAvergreyMax )
		//	{
		//		pSpeedDetectStruct->m_uAvergreyMax = (Uint16)dTempGrey;
		//		pSpeedDetectStruct->m_uContiuCount++;
		//	}
		//	else
		//	{
		//		if( pSpeedDetectStruct->m_uContiuCount > pCfgs->CameraCfg.uDayNightJudgeMinContiuFrame )/*Threshold can adjust*/
		//			pSpeedDetectStruct->m_uLightCount++;
		//		pSpeedDetectStruct->m_uAvergreyMax = 0;
		//		pSpeedDetectStruct->m_uContiuCount = 0;
		//	}
		//}
		//if( pSpeedDetectStruct->m_uPassTimeCount > (pSpeedDetectStruct->uDayNightJudgeCicleCount+1)*20 )
		//{
		//	pSpeedDetectStruct->m_bInNight = FALSE;
		//	if( pSpeedDetectStruct->m_uLightCount > (pSpeedDetectStruct->uDayNightJudgeCicleCount+1) )
		//		pSpeedDetectStruct->m_bInNight = TRUE;
		//	if( pSpeedDetectStruct->uDayNightJudgeCicleCount > 30 )
		//		pSpeedDetectStruct->uDayNightJudgeCicleCount = 0;
		//	else
		//		pSpeedDetectStruct->uDayNightJudgeCicleCount += 2;

		//	pSpeedDetectStruct->m_uPassTimeCount = 0;
		//	pSpeedDetectStruct->m_uAvergreyMax = 0;
		//	pSpeedDetectStruct->m_uContiuCount = 0;
		//}
		
	}
}
/*
inline void SobelThreshold
(
    Uint8  *Buf,   
    Uint16 dotNum,
	Uint16 threshold
)
{
	Int32	i;
	for(i=0;i<dotNum;i++)
	{
		if( Buf[i] < threshold )
		{
			Buf[i] = 0;
		}
		else if(Buf[i] > 0x3f )
		{
			Buf[i] = 0x3f;
		} 
		
	}
}
*/

/*void nSubstractImage(Uint8* puNewImage,Uint8* puTempImage,Uint8* puTargetImage, Int16 nStartLine,Int16 nEndLine, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	Int16 nRow, nCol;
	bool bFoundStart = FALSE;
	Uint16 nGrey;

	SetImagePointValue( 0, 0, pCfgs->m_iHeight, pCfgs->m_iWidth, 0, puTargetImage );

	for( nRow = nStartLine; nRow < nEndLine; nRow++ )
	{
		for( nCol = 0; nCol < pCfgs->m_iWidth; nCol++ )
		{
			nGrey = (*(puNewImage + nRow*pCfgs->m_iWidth + nCol)) - (*(puTempImage + nRow*pCfgs->m_iWidth + nCol));
			if( abs( nGrey ) < 18 )
			{
				nGrey = 0;
			}
			SetImagePointValue( nCol, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, (Uint16)(abs(nGrey)), puTargetImage );
		}
	}
	for( nCol = 0; nCol < pCfgs->m_iWidth; nCol++ )
	{
		bFoundStart = FALSE;
		for( nRow = nEndLine - 1; nRow > nStartLine; nRow-- )
		{
			if( (*(puTargetImage + nRow*pCfgs->m_iWidth + nCol)) != 0 && bFoundStart == FALSE )
			{
				bFoundStart = TRUE;
				SetImagePointValue( nCol, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, 0, puTargetImage );
				SetImagePointValue( nCol, nRow - 1, pCfgs->m_iHeight, pCfgs->m_iWidth, 0, puTargetImage );
				nRow--;
			}
			else if( bFoundStart == TRUE )
				SetImagePointValue( nCol, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, 150, puTargetImage );
		}
	}
}*/

/*void ImageCopy(Uint8* pucSource, Uint8* pucTarget, Int16 nStartLine, Int16 nEndLine, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	Int16 nRow, nCol;
	for( nRow = nStartLine; nRow < nEndLine; nRow++ )
	{
		for( nCol = 0; nCol < pCfgs->m_iWidth; nCol++ )
		{
			SetImagePointValue( nCol, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth,
			                    GetImagePointValue( nCol, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, pucSource ), pucTarget );
		}
	}
}*/

void iSubStractImage(Uint8 *restrict puSourceImage,Uint8 *restrict puTargetImage, Uint32 nThreshold, Int16 nFromLine, Int16 nToLine, Int16 width)
{
	Int32 iRow,iCol,nCompareResult;

	for( iRow = nFromLine; iRow < nToLine; iRow++ )
	{
        Int32 offset = width * iRow;
        Uint8* srcPtr = puSourceImage + offset;
        Uint8* dstPtr = puTargetImage + offset;
		for( iCol = 0; iCol < width; iCol++ )
		{
            nCompareResult = abs(srcPtr[iCol] - dstPtr[iCol]);
			dstPtr[iCol] = (nCompareResult < nThreshold) ? 0 : nCompareResult;
		}
	}
}

void iSubStractImage1(Uint8 *puSourceImage,Uint8 *puTargetImage, Uint32 nThreshold, Int16 nFromLine, Int16 nToLine, Int16 width)
{
	Int32 iRow,iCol,nCompareResult;
	for( iRow = nFromLine; iRow < nToLine; iRow++ )
	{
		Int32 offset = width * iRow;
		Uint8* srcPtr = puSourceImage + offset;
		Uint8* dstPtr = puTargetImage + offset;
		for( iCol = 0; iCol < width; iCol++ )
		{
			nCompareResult = srcPtr[iCol] - dstPtr[iCol];
			nCompareResult = (nCompareResult >= nThreshold)? srcPtr[iCol] : 0;
			dstPtr[iCol] = nCompareResult;
		}
	}
}

static void iSubStractImage2(Uint8 *puSourceImage,Uint8 *puTargetImage1, Uint8 *puTargetImage2, Uint32 nThreshold1, Uint32 nThreshold2, Int16 nFromLine, Int16 nToLine, Int16 width)
{
	Int32 iRow,iCol,nCompareResult;
	for( iRow = nFromLine; iRow < nToLine; iRow++ )
	{
		Int32 offset = width * iRow;
		Uint8* srcPtr = puSourceImage + offset;
		Uint8* dstPtr1 = puTargetImage1 + offset;
		Uint8* dstPtr2 = puTargetImage2 + offset;
		for( iCol = 0; iCol < width; iCol++ )
		{
			nCompareResult = srcPtr[iCol] - dstPtr1[iCol];
			nCompareResult = (nCompareResult >= 0)? nCompareResult : -nCompareResult;
			nCompareResult = (nCompareResult < nThreshold1)? 0 : nCompareResult;
			dstPtr1[iCol] = nCompareResult;
			nCompareResult = srcPtr[iCol] - dstPtr2[iCol];
			nCompareResult = (nCompareResult >= 0)? nCompareResult : -nCompareResult;
			nCompareResult = (nCompareResult < nThreshold2)? 0 : nCompareResult;
			dstPtr2[iCol] = nCompareResult;
		}
	}
}


/*bool bIsBlockAllEmpty(Uint8* puImage, Int16 nStartRow, Int16 nEndRow, Int16 nChangedPt, Uint16 uThreshold, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	Int16 nChangeCount = 0;
	Int16 nRow,nCol;

	for( nRow = nStartRow; nRow < nEndRow; nRow++ )
	{
		for( nCol = 1; nCol < pCfgs->m_iWidth - 1; nCol++ )
		{
			if( GetImagePointValue( nCol, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puImage ) != 0 )
				nChangeCount++;
		}
	}

	if( nChangeCount > nChangedPt )
		return FALSE;

	return TRUE;
}*/


void IsolatePointDelete(Uint8* puSourceImage, Uint16 uLines, Uint16 uColumns, Uint16 uThreshold)
{
	int iRow, iCol, nGrey;
	int threshold = (uThreshold << 3); // nGrey is the mean value, so we need to multiply 8.
    for (iRow = 1; iRow < uLines - 1; iRow++)
    {
        Int32 offset = iRow * uColumns;
        Uint8* ptrPre = puSourceImage + offset - uColumns;
        Uint8* ptrCur = puSourceImage + offset;
        Uint8* ptrNext = puSourceImage + offset + uColumns;
        for (iCol = 1; iCol < uColumns - 1; iCol++)
        {
			nGrey = (int)(ptrPre[iCol - 1]) + (int)(ptrCur[iCol - 1]) + (int)(ptrNext[iCol - 1]) + 
			    (int)(ptrPre[iCol]) + (int)(ptrNext[iCol]) + (int)(ptrPre[iCol + 1]) + 
				(int)(ptrCur[iCol + 1]) + (int)(ptrNext[iCol + 1]);
			ptrCur[iCol] = (nGrey < threshold) ? 0 : (nGrey >> 3);
		}
	}
	
}

void ProjectSubAndSobelEdge(Uint8* puImage, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	Int16 iRow, /*iCol, nPtCount, nValidPtCount,*/ nPosSum;
	Uint16 nGrey;

	for( iRow = 1; iRow < pCfgs->m_iHeight - 1; iRow++ )
	{
		nGrey = 0;
		nPosSum = 0;
		/*for( iCol = 1; iCol < pCfgs->m_iWidth - 1; iCol++ )
		{
			if( GetImagePointValue( iCol, iRow , pCfgs->m_iHeight, pCfgs->m_iWidth, puImage ) > 0 )
			{
				nPtCount++;
				nGrey += GetImagePointValue( iCol, iRow , pCfgs->m_iHeight, pCfgs->m_iWidth, puImage );
			}
		}*/
		if( GetImagePointValue( 0, iRow , pCfgs->m_iHeight, pCfgs->m_iWidth, puImage ) > 0	)
		{
			nPosSum=10;
			nGrey=150;
		}
		/*nValidPtCount = 0;
		nPosSum = 0;
		for( iCol = 1; iCol < pCfgs->m_iWidth - 1; iCol++ )
		{
			if( GetImagePointValue( iCol, iRow , pCfgs->m_iHeight, pCfgs->m_iWidth, puImage ) > 0 )
			{
				nValidPtCount++;
				nPosSum += iCol;
			}
		}
		if( nValidPtCount > 0 )
		{
			nPosSum = (Int16)(nPosSum / nValidPtCount);
		}
		*/
		SetImagePointValue( 0, iRow, pCfgs->m_iHeight, pCfgs->m_iWidth, (Uint16)nPosSum, puImage ); 
		
		if (nGrey > 255)
			SetImagePointValue( pCfgs->m_iWidth - 1, iRow, pCfgs->m_iHeight, pCfgs->m_iWidth, 180, puImage);
		else
			SetImagePointValue( pCfgs->m_iWidth - 1, iRow, pCfgs->m_iHeight, pCfgs->m_iWidth, nGrey, puImage);
		//SetImagePointValue( m_iWidth - 1, iRow,m_iHeight, m_iWidth, (uint)nPtCount, puImage);
		
	}
}


void SLKTemplateCalculate(Uint8* puSourceImage, Uint8* uTargetImage, Uint16 uLines, Uint16 uColumns, bool bVertical)
{
	Int16 iRow, iCol;
	Uint16 uGrey;
	if( !bVertical )
	{
		for( iRow = 0; iRow < uLines; iRow++ )
		{
			for( iCol = 0; iCol < uColumns; iCol++ )
			{
				if( (iRow == 0 ) || (iRow == uLines - 1 ))
					SetImagePointValue( iCol, iRow, uLines, uColumns, 0, uTargetImage );
				else if( (iCol == 0) || (iCol == uColumns - 1 ) )
					SetImagePointValue( iCol, iRow, uLines, uColumns, 0, uTargetImage );
				else
				{
					uGrey = (abs(-GetImagePointValue( iCol - 1, iRow - 1, uLines, uColumns, puSourceImage)
					             -GetImagePointValue(iCol-1, iRow, uLines, uColumns, puSourceImage)
								 -GetImagePointValue(iCol-1, iRow+1, uLines, uColumns, puSourceImage)
								 -GetImagePointValue(iCol+1, iRow-1, uLines, uColumns, puSourceImage)
					             -GetImagePointValue(iCol+1, iRow, uLines, uColumns, puSourceImage)
								 -GetImagePointValue(iCol+1, iRow+1, uLines, uColumns, puSourceImage)
								 +2*GetImagePointValue( iCol, iRow-1, uLines, uColumns, puSourceImage)
								 +2*GetImagePointValue(iCol, iRow, uLines, uColumns, puSourceImage)
								 +2*GetImagePointValue(iCol, iRow+1, uLines, uColumns, puSourceImage)) );
					SetImagePointValue( iCol, iRow, uLines, uColumns, uGrey, uTargetImage );
				}
			}
		}
	}
	else
	{
		for( iRow = 0; iRow < uLines; iRow++ )
		{
			for( iCol = 0; iCol < uColumns; iCol++ )
			{
				if( (iRow == 0 ) || (iRow == uLines - 1 ))
					SetImagePointValue( iCol, iRow, uLines, uColumns, 0, uTargetImage );
				else if( (iCol == 0) || (iCol == uColumns - 1 ) )
					SetImagePointValue( iCol, iRow, uLines, uColumns, 0, uTargetImage );
				else
				{
					uGrey = (abs(-GetImagePointValue(iCol-1, iRow-1, uLines, uColumns, puSourceImage)
								 -GetImagePointValue(iCol, iRow-1, uLines, uColumns, puSourceImage)
								 -GetImagePointValue(iCol+1, iRow-1, uLines, uColumns, puSourceImage)
								 -GetImagePointValue(iCol-1, iRow+1, uLines, uColumns, puSourceImage)
					             -GetImagePointValue(iCol, iRow+1, uLines, uColumns, puSourceImage)
								 -GetImagePointValue(iCol+1, iRow+1, uLines, uColumns, puSourceImage)
					             + 2*GetImagePointValue(iCol-1, iRow, uLines, uColumns, puSourceImage)
					             + 2*GetImagePointValue(iCol, iRow, uLines, uColumns, puSourceImage)
					             + 2*GetImagePointValue(iCol+1, iRow, uLines, uColumns, puSourceImage)) );
					SetImagePointValue( iCol, iRow, uLines, uColumns, uGrey, uTargetImage );
				}
			}
		}
	}
}

void SLKCalculate(Uint8* puSourceImage, Uint16 uThreshold, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
//	bool bEq = FALSE;

	//SLKTemplateCalculate( puSourceImage, pParams->puSobelTempImageX, pCfgs->m_iHeight, pCfgs->m_iWidth, FALSE );	
	//SLKTemplateCalculate( puSourceImage, pParams->puSobelTempImageY, pCfgs->m_iHeight, pCfgs->m_iWidth, TRUE );		
	
	SquareOfTwoImage( pParams->puSobelTempImageX, pParams->puSobelTempImageY, puSourceImage, pCfgs->m_iHeight, pCfgs->m_iWidth, uThreshold, pCfgs,  pParams); 
	
	
	// memcpy((void *)pParams->puAlgTempImage, (void *)puSourceImage, pCfgs->m_iWidth*pCfgs->m_iHeight);
	// memcpy((void *)pParams->puSobelTempImageX, (void *)puSourceImage, pCfgs->m_iWidth*pCfgs->m_iHeight);
	
	IsolatePointDelete( puSourceImage, pCfgs->m_iHeight, pCfgs->m_iWidth, uThreshold );  
	
	ProjectSLKToEdge( puSourceImage, pCfgs->m_iHeight, pCfgs->m_iWidth, uThreshold );  
	
}

inline void	SetImagePointValue(Int16 iCol,Int16 iRow,Uint16 Lines,Uint16 Columns,Uint16 Value,Uint8 *ImagePtr)
{
	*(ImagePtr+iRow*Columns+iCol)=Value;
	
}

void SquareOfTwoImage(Uint8*restrict puSourceImageX, Uint8*restrict puSourceImageY, Uint8*restrict puTargetImage, Uint16 uLines, Uint16 uColumns, Uint16 uThreshold, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	Int32 iRow, iCol;
	Uint16 /*uSourceGrey, uTargetGrey,*/ uResult;
    Uint16 uGrey1,uGrey2,val0,val1,val2,val3,val4,val5,val6,val7,val8;
	memset(puSourceImageX, 0, sizeof(Uint8) * uColumns * uLines);
	for( iRow = 1; iRow < uLines - 1; iRow++ )
	{
        Int32 offset = iRow * uColumns;
        Uint8* srcX = puSourceImageX + offset;
        Uint8* srcY = puSourceImageY + offset;
        Uint8* dstPre = puTargetImage + offset - uColumns;
        Uint8* dstCur = puTargetImage + offset;
        Uint8* dstNext = puTargetImage + offset + uColumns;
		for( iCol = 1; iCol < uColumns - 1; iCol++ )
		{
			val0 = dstCur[iCol];
			val1 = dstPre[iCol - 1];
			val2 = dstCur[iCol - 1];
			val3 = dstNext[iCol - 1];
			val4 = dstPre[iCol];
			val5 = dstNext[iCol];
			val6 = dstPre[iCol + 1];
			val7 = dstCur[iCol + 1];
			val8 = dstNext[iCol + 1];
			uGrey1 = abs(-val1 - val2 - val3 - val6 - val7 - val8 + 2 * (val4 + val0 + val5));
			uGrey2 = abs(-val1 - val3 - val4 - val5 - val6 - val8 + 2 * (val2 + val0 + val7));
			uResult = ( uGrey1 + uGrey2 ) >> 2;

			srcX[iCol] = (uResult > pCfgs->CameraCfg.uSquareThreshold) ? 100 : ((uResult < uThreshold) ? 0 : uResult);
		}
	}
	memcpy(puTargetImage,puSourceImageX,uLines*uColumns);

}

void ProjectSLKToEdge(Uint8* puImage, Uint16 uLines, Uint16 uColumns, Uint16 uThreshold)
{
	Uint16 uGrey2 = 0, uGrey = 0;
	Int16 m = 0, j, nEndRow, nPosSum = 0, nValidPtCount = 0, nValidLine = 0;
	Int16 iRow, iCol;
	bool bBeginFind = FALSE;
	Int16 nChangedRow = 0;
	unsigned char* ptr0 = puImage + (uLines - 2) * uColumns + uColumns - 1;
	unsigned char* ptr1 = 0;

	for( iRow = 1; iRow < uLines - 1 ; iRow++ )
	{
		Uint8* ptr = puImage + iRow * uColumns;
		nValidPtCount = 0;
		nPosSum = 0;
		uGrey2 = 0;
		m = 0;
		for( iCol = 1; iCol < uColumns - 1; iCol++ )
		{
			uGrey = ptr[iCol];
			if( uGrey > 0 )
			{
				uGrey2 += uGrey;
				m++;
			}
			if (uGrey > 0)
			{
				nValidPtCount++;
				nPosSum += iCol;
			}
		}
		//calculate the threshold value
		if ( m >= 3 )
		{
			uGrey2 /= m;
			if( uGrey2 < uThreshold)
			{
				uGrey2 = 0;
			}
		}
		else
		{
			uGrey2 = 0;
			m = 0;
		}

		if( nValidPtCount >= 2 && ( nValidPtCount < (uColumns - 6) ) )
		{
			nPosSum = (Int16)nPosSum / nValidPtCount;
		}
		else if( nValidPtCount >= (uColumns - 6) )
		{
			nPosSum = (Int16)(uColumns / 2);
		}
		/////////////////////////////
		ptr[0] = nPosSum;
		ptr[uColumns - 1] = uGrey2;

		if( uGrey2 > 0 )
		{
			nValidLine++;
			if( nValidLine > VEHICLE_MIN_ROW )
			{
				puImage[0] = iRow;
			}
		}
		else
		{
			nValidLine = 0;
		}
	}

	for( iRow = uLines - 2; iRow > 10; iRow--, ptr0 -= uColumns)
	{
		if( !bBeginFind )
		{
			uGrey = *ptr0;//GetImagePointValue( uColumns - 1, iRow , uLines, uColumns, puImage );
			if (uGrey != 0)
			{
				bBeginFind = TRUE;
			}
		}
		else
		{
			if (*ptr0 == 0)
			{
				if( nChangedRow < 5 )
				{
					nEndRow = iRow + 5;
					nEndRow = (nEndRow > uLines - 2) ? uLines - 2 : nEndRow;
					ptr1 = puImage + (iRow + 1) * uColumns + uColumns - 1;
					for( j = iRow + 1; j < nEndRow; j++, ptr1 += uColumns)
					{
						// SetImagePointValue( uColumns - 1, iRow, uLines, uColumns, 0, puImage );
						*ptr1 = 0; // we need to modify iRow to j, otherwise this for loop is useless.
					}
				}
				break;
			}
			else
			{
				nChangedRow++;
			}
		}
	}
}

///////////////////////////////////////////////

/*
void	fSetDetectStateFlag(Uint16 DetectNum,Uint16 Flag, Uint16 nChannelNum,LOCALBUF0 *localBuf0,  void *outArgs){
	div_t	Position;
	Int16		Bit=1;
	Position=div(((Int16)DetectNum-1),16);
	Bit=Bit<<Position.rem;
	if(Flag==TRUE){
		localBuf0->CameraLocalPara[nChannelNum].DetectStateFlag[Position.quot]|=Bit;
	}
	else{
		localBuf0->CameraLocalPara[nChannelNum].DetectStateFlag[Position.quot]&=~Bit;
	}

////2005 7 Pulse trigger set value
	localBuf0->UploadResultsBuf.DetectorStaFlag[Position.quot] \
		= localBuf0->CameraLocalPara[nChannelNum].DetectStateFlag[Position.quot];

}
*/
inline Uint16	GetImagePointValue(Int16 iCol,Int16 iRow,Uint16 Lines,Uint16 Columns,Uint8 *ImagePtr)
{
	return (Uint16)*(ImagePtr+iRow*Columns+iCol);
}

/*SPEED_COUNTER_STATUS GetSpeedCounterStatus( Uint16 uCounter, Uint16 uThisLaneID, Uint16 nChannelNum ,ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	SPEED_COUNTER_STATUS SpeedCounterStatus;
	memset((void*)&SpeedCounterStatus, 0, sizeof(SPEED_COUNTER_STATUS));
	switch( uCounter )
	{
	case THIS_LANE_COUNTER:
		SpeedCounterStatus = pCfgs->CameraLocalPara.guSpeedCounterStatus[uThisLaneID-1];
		break;
	case LEFT_LANE_COUNTER:
	{
		if( uThisLaneID > 1 )
			SpeedCounterStatus = pCfgs->CameraLocalPara.guSpeedCounterStatus[uThisLaneID-2];
		break;
	}
	case RIGHT_LANE_COUNTER:
	{
		if( uThisLaneID < pCfgs->CameraCfg.LaneAmount )
			//SpeedCounterStatus = pCameraLocalPara[nChannelNum]->guSpeedCounterStatus[uThisLaneID+1];  //
			SpeedCounterStatus = pCfgs->CameraLocalPara.guSpeedCounterStatus[uThisLaneID];  //  20130708
		break;
	}
	default:
		break;
	}

	return SpeedCounterStatus;
}*/

/*Int16 nLengthInfoGetSize(struct cSpeedDetectStruct* pSpeedDetectStruct)
{
	return (Int16)(pSpeedDetectStruct->nLengthInfoTop + 1 );
}*/

//Int16 nLengthGetVehicleHeadRow( Uint8* puSobelImage, Uint16* pnStartCalcuRow, VEHICLE_LENGTHINFO* pVehicleLengthInfo, ALGCFGS *pCfgs, ALGPARAMS *pParams)
//{
//	int nContinueRow = 0, nStartRow, nEndRow,  nRow;
//	unsigned int uThreshold;
//	int khcount=0;
//	uThreshold = 1;//1
	
//	nStartRow = 0;
//	nEndRow = 0;
	
//	if( pCfgs->m_iHeight - (*(pnStartCalcuRow)) < 30 )//5
//		return 0;
	
//	for( nRow = (*(pnStartCalcuRow)); nRow < pCfgs->m_iHeight-1 ; nRow++ )
//	{
//		if( GetImagePointValue( pCfgs->m_iWidth -1 , nRow , pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage) > uThreshold )
//		{
//			if( nStartRow == 0 )
//				nStartRow = nRow;
//			nContinueRow++;
//			nEndRow = nRow;//0;
//			khcount=0;
//		}
		/*else if( (nStartRow != 0) && (nContinueRow > VEHICLE_MIN_ROW) )
		{
		nEndRow = nRow - 1;
		pVehicleLengthInfo->uEndRow = nStartRow;
		pVehicleLengthInfo->fLengthSum += (float)(nEndRow - nStartRow);
		pVehicleLengthInfo->uLengthCount++;
		
		  return nEndRow;
		  }
		  else if( nStartRow != 0 )
		nEndRow = nRow;*/
//		else 
//		{
//			khcount++;
//		}
//		if ( (nStartRow != 0) &&(khcount>5))
//		{
//			nEndRow = nRow - 1;
//			pVehicleLengthInfo->uEndRow = nStartRow;
//			pVehicleLengthInfo->fLengthSum += (float)(nEndRow - nStartRow);
//			pVehicleLengthInfo->uLengthCount++;
			
//			return nEndRow;
//		}
//	}
//	pVehicleLengthInfo->uEndRow = nStartRow;
//	pVehicleLengthInfo->fLengthSum += (float)(nEndRow - nStartRow);
//	pVehicleLengthInfo->uLengthCount++;
//	return nEndRow;
	
//}

/*void LengthInfoAdd(struct cSpeedDetectStruct* pSpeedDetectStruct, VEHICLE_LENGTHINFO* pVehicleLengthInfo)
{
	pSpeedDetectStruct->nLengthInfoTop++;
	if( pSpeedDetectStruct->nLengthInfoTop > (LENGTHINFO_MAXSIZE - 1) )
	{
		pSpeedDetectStruct->nLengthInfoTop = LENGTHINFO_MAXSIZE - 1;
		return;
	}
	//SourceAddress = (Uint16*)pVehicleLengthInfo;
	//TargetAddress = (Uint16*)&pSpeedDetectStruct->VehicleLengthInfo[pSpeedDetectStruct->nLengthInfoTop];
	//Elements = VEHICLE_LENGTHINFO_LENGTH;
	//DataExchange();

	//memcpy((void *)&pSpeedDetectStruct->VehicleLengthInfo[pSpeedDetectStruct->nLengthInfoTop], (void *)pVehicleLengthInfo, sizeof(VEHICLE_LENGTHINFO));

	memcpy((void *)&pSpeedDetectStruct->VehicleLengthInfo[pSpeedDetectStruct->nLengthInfoTop], (void *)pVehicleLengthInfo, sizeof(VEHICLE_LENGTHINFO));
}*/

//void LengthGetVehicleInInfo( Uint8* puSobelImage, struct cSpeedDetectStruct* pSpeedDetectStruct, ALGCFGS *pCfgs, ALGPARAMS *pParams)
//{
//	unsigned uThreshold;//, uMaxChangedRow;
//	Int16 i, nRow, nValidLineCount;//, nInvalidLineCount, nChangedRow;
//	bool bGetEmptyRow;
//	Int16 nStartAddInRow = 5;

//	uThreshold = 1;

//	bGetEmptyRow = FALSE;
//	if( pSpeedDetectStruct->uLengthBeginInRows > 10 )  //5
//	{
//		bGetEmptyRow = TRUE;
//	}
//	else
//		pSpeedDetectStruct->uLengthBeginInRows = 0;

//	for( nRow = 1 + nStartAddInRow ; nRow < 45 + nStartAddInRow ; nRow++ )  //8
//	{
//		if( (GetImagePointValue( pCfgs->m_iWidth - 1, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage ) > uThreshold) &&
//		        (bGetEmptyRow == FALSE) )
//		{
//			if( nRow == 1 + nStartAddInRow )
//				pSpeedDetectStruct->uLengthBeginInRows = 1;
//			else if( pSpeedDetectStruct->uLengthBeginInRows == 0 )
//				return;
//			else
//				pSpeedDetectStruct->uLengthBeginInRows++;
//		}
//		else if( (GetImagePointValue( pCfgs->m_iWidth - 1 , nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage ) < uThreshold) &&
//		         (bGetEmptyRow == TRUE) ) /*??隆矛??隆矛?3?3??????隆矛?????隆矛????∶????隆矛??隆矛?????隆矛??隆矛?o??隆矛????隆矛???2a?????o??隆矛??2?DD???????DD*/
//		{
//			pSpeedDetectStruct->uLengthEndEmptyRow++;
//			if( pSpeedDetectStruct->uLengthEndEmptyRow >= 10 ) //VEHICLE_MIN_ROW ) /*/??隆矛??隆矛?1??22?3?????DD????? ?????篓篓?????篓篓??隆矛???????篓篓?D?????1??????3??????隆矛???3????隆矛???隆矛??隆矛*/
//			{
//				nValidLineCount = 0;  /*??隆矛??D3??????隆矛????????DD????隆矛??y*/
//				for( i = nRow; i < (Int16)pCfgs->m_iHeight; i++ )
//				{
//					if( GetImagePointValue( pCfgs->m_iWidth - 1, i, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage ) > uThreshold )
//					{
//						nValidLineCount++;
//						pSpeedDetectStruct->uLengthBeginInRows = i;
//					}
//					else if( nValidLineCount != 0 )
//						break;
//				}
//				if( pSpeedDetectStruct->uLengthBeginInRows >= (pCfgs->m_iHeight - 10) )
//				{
					//pSpeedDetectStruct->uLengthEndEmptyRow = 0;
					//pSpeedDetectStruct->uLengthBeginInRows = 0;
//				}
//			}
//		}
//	}
//}

/*void LengthInfoRemoveAt( struct cSpeedDetectStruct* pSpeedDetectStruct, Int16 nIndex )
{
	Int16 i;
	if( nIndex <= pSpeedDetectStruct->nLengthInfoTop )
	{
		for( i = nIndex; i <= (pSpeedDetectStruct->nLengthInfoTop - 1); i++ )
		{
			//SourceAddress = (Uint16*)&pSpeedDetectStruct->VehicleLengthInfo[i+1];
			//TargetAddress = (Uint16*)&pSpeedDetectStruct->VehicleLengthInfo[i];
			//Elements = VEHICLE_LENGTHINFO_LENGTH;
			//DataExchange();

			//memcpy((void *)&pSpeedDetectStruct->VehicleLengthInfo[i], (void *)&pSpeedDetectStruct->VehicleLengthInfo[i+1], sizeof(VEHICLE_LENGTHINFO));
			memcpy((void *)&pSpeedDetectStruct->VehicleLengthInfo[i], (void *)&pSpeedDetectStruct->VehicleLengthInfo[i+1], sizeof(VEHICLE_LENGTHINFO));
		}
		pSpeedDetectStruct->VehicleLengthInfo[pSpeedDetectStruct->nLengthInfoTop].uEndRow = 0;
		pSpeedDetectStruct->VehicleLengthInfo[pSpeedDetectStruct->nLengthInfoTop].uLengthCount = 0;
		pSpeedDetectStruct->VehicleLengthInfo[pSpeedDetectStruct->nLengthInfoTop].fLengthSum = 0;
		pSpeedDetectStruct->VehicleLengthInfo[pSpeedDetectStruct->nLengthInfoTop].uThreshold = 0;
		pSpeedDetectStruct->nLengthInfoTop--;
	}
}

Uint16 uGetMaxUnEmptyRow(Uint8* puImage, Int16 nStartLine,Int16 nEndLine, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	Int16 nRow;

	for( nRow = nEndLine - 1; nRow >= nStartLine; nRow-- )
	{
		if( GetImagePointValue( pCfgs->m_iWidth - 1, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puImage ) != 0 )
			return  (Uint16)nRow;
	}
	return 1;
}*/


bool bIsSobleBlockUnChanged(Uint8 *pucSobelCurrentImage, int nStartLine, int nEndLine,int flag, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	int nSpeedStartLine = nStartLine;
	int nSpeedEndLine = nEndLine;
	int nRow;
	int k;
	int nUnChangedRow = 0;
	Uint16 value;
	Uint16 ucThreshold;

	if (flag == 0)
	{
		//ucThreshold = 10;
		ucThreshold = pCfgs->CameraCfg.uInThreshold;  // by david 20130910
		k = 1;
	}
	else
	{
		ucThreshold =1;

		k = 1;

	}
////20140113	
	nUnChangedRow = 0;
	value =0;
	if (flag==1)//night
	{
		for(  nRow = 0; nRow <pCfgs->m_iHeight/2; nRow++ )
		{
			value = GetImagePointValue( pCfgs->m_iWidth -1 , nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, pucSobelCurrentImage );
			if( value < ucThreshold )
			{
				nUnChangedRow++;
			}
			if( (nUnChangedRow * k) >= pCfgs->m_iHeight/2-3 )//3
			{
				return TRUE;
			}
		}
	}
	else
	{
		for(  nRow = nSpeedStartLine; nRow < nSpeedEndLine; nRow++ )
		{
			value = GetImagePointValue( pCfgs->m_iWidth -1 , nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, pucSobelCurrentImage );
			if( value < ucThreshold )
			{
				nUnChangedRow++;
			}
			if( nUnChangedRow * k >= (nSpeedEndLine - nSpeedStartLine-10) )//3
			{
				return TRUE;
			}
		}
	}
	
	
	return FALSE;
}


bool bIsSobleBlockChanged(Uint8 *pucSobelCurrentImage, Int16 nStartLine, Int16 nEndLine, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
//	Int16 nSpeedStartLine = nStartLine;
//	Int16 nSpeedEndLine = nEndLine;
	Int16 nRow;
	Uint16 ucThreshold = 1;
	Int16 nUnChangedRow = 0;
//	Int16 nChangedRow = 0;
	Uint16 value = 0;

	for(  nRow = nStartLine; nRow < nEndLine; nRow++ )
	{
		value = GetImagePointValue( pCfgs->m_iWidth -1 , nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, pucSobelCurrentImage );

		if( value < ucThreshold )
		{
			nUnChangedRow++;
		}
		if( nUnChangedRow * 4 > (nEndLine - nStartLine) )
			//if( nUnChangedRow*2 > (nEndLine - nStartLine) )
		{
			return FALSE;
		}

		/*
		if( GetImagePointValue( pCfgs->m_iWidth -1 , nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, pucSobelCurrentImage ) > ucThreshold )
		{
			nChangedRow++;
			printf("nChangedRow  ->  %d\n",nChangedRow);

		}

		if( nChangedRow*200 > (nEndLine - nStartLine) )
		{
			printf("return TRUE  ->\n");
			return TRUE;
		}
		*/

	}

	return TRUE;
}

/*double dAverageGreyInBlock(Uint8 *puImage, Int16 nFromBlockID, Int16 nToBlockID, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	Int16 nPointSum=0, nStartRow=0, nEndRow=0, nStartCol=0, nEndCol=0, iRow=0, iCol=0;
	double 	gdAverGrey=0.0,gdSum=0.0;

	nStartRow = nFromBlockID;
	nEndRow = nToBlockID;
	nStartCol = 1;
	nEndCol = pCfgs->m_iWidth-1;
	gdSum = 0;


	for( iRow = nStartRow; iRow < nEndRow; iRow++ )
	{
		for( iCol = nStartCol; iCol < nEndCol; iCol++ )
		{
			gdAverGrey = (double)GetImagePointValue(iCol, iRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puImage );
			gdSum += gdAverGrey;
			nPointSum++;
		}
	}
	return (double)(gdSum/nPointSum);
}*/

//Int16 nSpeedGetVehicleHeadRow( Uint8* puSobelImage, Uint16* pnStartCalcuRow, VEHICLE_SPEEDINFO* pVehicleSpeedInfo, int flag, ALGCFGS *pCfgs, ALGPARAMS *pParams)
//{
//	int nContinueRow = 0;
//	int nStartRow, nEndRow,  nRow;
//	unsigned int uThreshold;
//	int khcount=0;
	
//	uThreshold = 5;//1;
//	nStartRow = 0;
//	nEndRow = 0;
	
	
//	if( pCfgs->m_iHeight - (*(pnStartCalcuRow)) < 5 )//5
//		return 0;
//	if (flag==0)
//	{
//		for( nRow = (*(pnStartCalcuRow)); nRow < pCfgs->m_iHeight-1 ; nRow++ )
//		{
//			if( GetImagePointValue( pCfgs->m_iWidth -1 , nRow , pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage) > uThreshold )
//			{
//				if( nStartRow == 0 )
//					nStartRow = nRow;
//				nContinueRow++;
//				nEndRow = nRow;//0;
//				khcount=0;
//			}
//			else if(nContinueRow>0)
//			{
//				khcount++;
//			}
			
//			if( (nContinueRow > 10) &&(khcount>4) ) //???3D?D?3?????????3????DD??隆矛??y?????????篓篓??隆矛???隆矛???khcount?D???????D?3????3???????隆矛????隆矛?????????隆矛??o??隆矛??????篓篓??隆矛??
//			{
//				nEndRow = nRow - 1;///MOVE ADDED HEAD?
				//pVehicleSpeedInfo->uHeadRow = nStartRow;
//				return nEndRow;
//			}
//		}
//	}
//	else
//	{
//		uThreshold = pCfgs->m_iWidth / 5;
//		for( nRow = (*(pnStartCalcuRow)); nRow <pCfgs->m_iHeight-1; nRow++ )
//		{
//			if (nRow>250)
//			{
//				int a=GetImagePointValue( pCfgs->m_iWidth -1 , nRow , pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage);
//			}
//			if( GetImagePointValue( pCfgs->m_iWidth -1 , nRow , pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage) > uThreshold )
//			{
//				if( nStartRow == 0 )
//					nStartRow = nRow;
//				nContinueRow++;
//				nEndRow = nRow;//0;
//				khcount=0;
//			}
//			else if( nStartRow != 0 ) 
//			{
//				nEndRow = nRow-1;///MOVE ADDED HEAD?
				//pVehicleSpeedInfo->uHeadRow = nStartRow;
//				return nEndRow;
//			}
//		}
//	}
	/*	for( nRow =m_iHeight-2 ; nRow > (*(pnStartCalcuRow)) ; nRow-- )
	{
	if( GetImagePointValue( m_iWidth -1 , nRow , m_iHeight, m_iWidth, puSobelImage) > uThreshold )
	{
	if( nEndRow == 0 )
	nEndRow= nRow;
	nContinueRow++;
	nStartRow= 0;//0;
	khcount=0;
	}
	else
	{
	if( (nStartRow != 0) &&(nContinueRow>15) ) 
	{
				//nEndRow = nRow - 1;///MOVE ADDED HEAD?
				//pVehicleSpeedInfo->uHeadRow = nStartRow;
				return nEndRow;
				}
				else
				nContinueRow=0;
				}
}*/
//	return nEndRow;
//}

/*bool bVehicleHeadIn(Uint8* puSourceImage, Uint8* puTargetImage, Int16 nStartLine,Int16 nEndLine, Int16 nRowThreshold, Int16 nColThreshold, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	Int16 nValidRowCount = 0, nValidColPtCount = 0;
	Int16 nRow, nCol;
	bool bVehicleCome = FALSE;

	for( nRow = nStartLine; nRow < nEndLine; nRow++ )
	{
		nValidColPtCount = 0;
		for( nCol = 0; nCol < pCfgs->m_iWidth; nCol++ )
		{
			if( ( (*(puSourceImage + nRow*pCfgs->m_iWidth + nCol)) * (*(puTargetImage + nRow*pCfgs->m_iWidth + nCol)) ) == 0 )
				SetImagePointValue( nCol, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, 0, puTargetImage );
			else///setpoint?
				nValidColPtCount++;
		}
		if( nValidColPtCount < nColThreshold )
		{
			if( nValidRowCount < nRowThreshold )
				nValidRowCount = 0;
		}
		else
			nValidRowCount++;
	}
	if( nValidRowCount >= nRowThreshold )
		bVehicleCome = TRUE;

	if( bVehicleCome == FALSE )
	{
		for( nRow = nStartLine; nRow < nEndLine; nRow++ )
		{
			for( nCol = 0; nCol < pCfgs->m_iWidth; nCol++ )
			{
				SetImagePointValue( nCol, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, 0, puTargetImage );
			}
		}
	}
	return bVehicleCome;
}

void SpeedInfoAdd(struct cSpeedDetectStruct* pSpeedDetectStruct, VEHICLE_SPEEDINFO* pVehicleSpeedInfo)
{
	pSpeedDetectStruct->nSpeedInfoTop++;

	if( pSpeedDetectStruct->nSpeedInfoTop > (SPEEDINFO_MAXSIZE - 1) )
	{
		pSpeedDetectStruct->nSpeedInfoTop = SPEEDINFO_MAXSIZE - 1;
		return;
	}

	//SourceAddress = (Uint16*)pVehicleSpeedInfo;
	//TargetAddress = (Uint16*)&pSpeedDetectStruct->VehicleSpeedInfo[pSpeedDetectStruct->nSpeedInfoTop];
	//Elements = VEHICLE_SPEEDINFO_LENGTH;
	//DataExchange();

	//memcpy((void *)&pSpeedDetectStruct->VehicleSpeedInfo[pSpeedDetectStruct->nSpeedInfoTop], (void *)pVehicleSpeedInfo, sizeof(VEHICLE_SPEEDINFO));
	memcpy((void *)&pSpeedDetectStruct->VehicleSpeedInfo[pSpeedDetectStruct->nSpeedInfoTop], (void *)pVehicleSpeedInfo, sizeof(VEHICLE_SPEEDINFO));

}

Int16 nSpeedInfoGetSize(struct cSpeedDetectStruct* pSpeedDetectStruct)
{
	return (Int16)( pSpeedDetectStruct->nSpeedInfoTop + 1 );
}

void SpeedInfoRemoveAt( struct cSpeedDetectStruct* pSpeedDetectStruct, Int16 nIndex )
{
	pSpeedDetectStruct->nSpeedInfoTop = -1;
}

void SpeedGetVehicleInInfo( Uint8* puSobelImage, struct cSpeedDetectStruct* pSpeedDetectStruct, int flag, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	unsigned uThreshold;
	int i, nRow, nInvalidLineCount;
	bool bGetEmptyRow;
	int nStartAddInRow = 5;//30
	
	uThreshold = 5;//1
	
	bGetEmptyRow = FALSE;
	if (flag==0)
	{
		if( pSpeedDetectStruct->uSpeedBeginInRows > 10 )//5
		{
			bGetEmptyRow = TRUE;
		}
		else
			pSpeedDetectStruct->uSpeedBeginInRows = 0;
		
		for( nRow = nStartAddInRow + 1; nRow < nStartAddInRow + 48; nRow++ )//8,31-78,48
		{
			if( (GetImagePointValue( pCfgs->m_iWidth - 1, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage ) > uThreshold) &&
				(bGetEmptyRow == FALSE) )
//			{
//				if( nRow == 1 + nStartAddInRow )
//					pSpeedDetectStruct->uSpeedBeginInRows = 1;
//				else if( pSpeedDetectStruct->uSpeedBeginInRows == 0 )
//					return;
//				else
//					pSpeedDetectStruct->uSpeedBeginInRows++;
//			}
			//else if( (GetImagePointValue( m_iWidth - 1 , nRow, m_iHeight, m_iWidth, puSobelImage ) < uThreshold) &&
			//	(bGetEmptyRow == TRUE) ) 
//			else if( (GetImagePointValue( pCfgs->m_iWidth - 1 , nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage )> uThreshold) &&
//				(bGetEmptyRow == TRUE) ) 
//			{
//				pSpeedDetectStruct->uSpeedEndEmptyRow++;
//				if( pSpeedDetectStruct->uSpeedEndEmptyRow >= 10 )//VEHICLE_MIN_ROW /*??隆矛??隆矛?1??22?3?????DD????? ?????篓篓?????篓篓??隆矛???????篓篓???隆矛??D?????1??????3??????隆矛???3????隆矛???隆矛??隆矛*/
//				{
//					nInvalidLineCount = 0;
//					for( i = nRow; i< pCfgs->m_iHeight - 1; i++ )
//					{
//						if( GetImagePointValue( pCfgs->m_iWidth - 1, i, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage ) > uThreshold )
//						{
//							nInvalidLineCount++;
							//pSpeedDetectStruct->uSpeedBeginInRows = i;
//							pSpeedDetectStruct->uSpeedalsoInRows=i;
//						}
//						else if( nInvalidLineCount != 0 )
//							break;
//					}
					
//					if( pSpeedDetectStruct->uSpeedalsoInRows >= (pCfgs->m_iHeight - 10) ){//10
//						pSpeedDetectStruct->uSpeedEndEmptyRow = 0;
//						pSpeedDetectStruct->uSpeedalsoInRows = 0;
						//pSpeedDetectStruct->uSpeedBeginInFrame=0;
//					}
//				}
//			}
			//	else if ((GetImagePointValue( m_iWidth - 1 , nRow, m_iHeight, m_iWidth, puSobelImage )<uThreshold) &&
			//	(pSpeedDetectStruct->uSpeedEndEmptyRow>0))
			//	{
			//	pSpeedDetectStruct->uSpeedEndEmptyRow=0;
			//}
//		}
//	}
//	else
//	{
//		uThreshold=5;
//		if( pSpeedDetectStruct->uSpeedBeginInRows > 5 )//5
//		{
//			bGetEmptyRow = TRUE;
//		}
//		else
//			pSpeedDetectStruct->uSpeedBeginInRows = 0;
//		for( nRow = nStartAddInRow + 1; nRow < nStartAddInRow + 48; nRow++ )//8,31-78,48
//		{
//			if( (GetImagePointValue( pCfgs->m_iWidth - 1, nRow, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage ) > uThreshold) &&
//				(bGetEmptyRow == FALSE) ) 
//			{
//				if( nRow == 1 + nStartAddInRow )
//					pSpeedDetectStruct->uSpeedBeginInRows = 1;
//				else if( pSpeedDetectStruct->uSpeedBeginInRows == 0 )
//					return;
//				else
//					pSpeedDetectStruct->uSpeedBeginInRows++;
//			}
//			else if((bGetEmptyRow == TRUE) ) 
//			{
//				for( i = nRow; i< pCfgs->m_iHeight - 1; i++ )
//				{
//					if( GetImagePointValue( pCfgs->m_iWidth - 1, i, pCfgs->m_iHeight, pCfgs->m_iWidth, puSobelImage ) > uThreshold )
//					{
//						pSpeedDetectStruct->uSpeedEndEmptyRow++;
//						nInvalidLineCount++;
//						pSpeedDetectStruct->uSpeedBeginInRows = i;
//						pSpeedDetectStruct->uSpeedalsoInRows=i;
//					}
//					else if( nInvalidLineCount != 0 )
//						break;
//				}
				
//				if( pSpeedDetectStruct->uSpeedalsoInRows >= (pCfgs->m_iHeight - 10) ){//10
//					pSpeedDetectStruct->uSpeedEndEmptyRow = 0;
//					pSpeedDetectStruct->uSpeedalsoInRows = 0;
					//pSpeedDetectStruct->uSpeedBeginInFrame=0;
//				}
//			}
//		}
//	}
//}

bool bIsSpeedCounterChanged( Uint8* puSobelImage, Uint16 uSpeedCounterStartRow, SPEED_COUNTER_STATUS* pSpeedCounterStatus,int flag,  ALGCFGS *pCfgs, ALGPARAMS *pParams )
{
//	int nLeftBlockColumn = pCfgs->m_iWidth / 3, nRightBlockColumn = pCfgs->m_iWidth * 2 / 3;
//	int nPosSum = 0, nRowCount = 0;
	int nRow;
	int nUnChangedRow = 0;
  //	int startrow = 0;
//	int nullRowNum = 0, maxNullRow = 0;
//	int  maxNullRowPos = 0;
	unsigned int uThreshold ;//1
	int k;
	unsigned char* ptr = puSobelImage + (uSpeedCounterStartRow + 2) * pCfgs->m_iWidth + pCfgs->m_iWidth - 1;
	
	if (flag == 0)
	{
		//uThreshold = 2;  //by david 20130905 from 5 to 20
		uThreshold = pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold;  //by david 20130905 
		k = 2;
	//	startrow = uSpeedCounterStartRow;
	

		for( nRow = uSpeedCounterStartRow + 2; nRow < pCfgs->m_iHeight -1; nRow++, ptr += pCfgs->m_iWidth)
		{
			if(*ptr < uThreshold )
			{
				//nullRowNum++;
				nUnChangedRow++;
			}
			//else
			//{
			//	if (nullRowNum > maxNullRow) {
			//		maxNullRow = nullRowNum;
			//		pParams->NormalDetectCfg.cSpeedDesc.position1 = nRow-maxNullRow+1;
			//		pParams->NormalDetectCfg.cSpeedDesc.position2 = nRow;
			//	}
			//	nullRowNum=0;
			//}
		
		}
	
	//if( nUnChangedRow * k > pParams->NormalDetectCfg.cSpeedDesc.uVehicleHeadMinRow )
		if( (nUnChangedRow * k) > (pCfgs->m_iHeight - uSpeedCounterStartRow - 2) ) 
		{
			//pSpeedCounterStatus->bRightDestroyed = FALSE;
			//pSpeedCounterStatus->bLeftDestroyed = FALSE;
			return FALSE;	
		}
	}
	else
	{
////20140110 zcj
		uThreshold=pCfgs->m_iWidth/5;
	////    uThreshold = pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold;   
	
		k=1;
		for( nRow = uSpeedCounterStartRow + 2; nRow < pCfgs->m_iHeight -1; nRow++, ptr += pCfgs->m_iWidth )
		{
			if(*ptr < uThreshold )
				nUnChangedRow++;
		}
		//startrow=m_iHeight-100;
		if( nUnChangedRow*k > (pCfgs->m_iHeight-uSpeedCounterStartRow-5))//
		{
			return FALSE;
		}
		//startrow=m_iHeight-100;
		
	}
	return TRUE;
}

float GetLenOfTwoPoint(CPoint* Cmp_Dot1, CPoint* Cmp_Dot2)
{

	float i,j;
	j = 0;
	i=-1;
	i = (float)((Cmp_Dot1->x - Cmp_Dot2->x) * (Cmp_Dot1->x - Cmp_Dot2->x));
	i = (float)( i + (Cmp_Dot1->y - Cmp_Dot2->y) * (Cmp_Dot1->y - Cmp_Dot2->y));

	j = (float) ( sqrt( i ) );
	return (float)j;
}

CPoint ptGetDot(CPoint* ptUpLeft, CPoint* ptUpRight, CPoint* ptDownRight, CPoint* ptDownLeft, Int16 nColNum, Uint32 * ptStorePlace, int step)
{
	CPoint ptRowAndCol, ptTemp, ptOne, ptTwo;
	Uint16	MaxX,MaxY,/*cNewX,*/ uMaxLines;
	int iRow_Num, iRow_Num1 = 0;
	int iRow, iCol;
	float fSlope;
	float temp1, temp2;
	float	fBDivder,fDivder;
	Uint32 	ImageAddr;
	Uint8 	cState;
	//bit7=1 
	//bit6=1 



	cState=0;
	ptOne.x = ptUpLeft->x;
	ptOne.y = ptUpLeft->y;
	ptTwo.x = ptUpRight->x;
	ptTwo.y = ptUpRight->y;

	temp1 =GetLenOfTwoPoint( &ptOne, &ptTwo );

	ptOne.x = ptDownLeft->x;
	ptOne.y = ptDownLeft->y;
	ptTwo.x = ptDownRight->x;
	ptTwo.y = ptDownRight->y;
	temp2 =GetLenOfTwoPoint( &ptOne, &ptTwo );

	if	(temp1 >= temp2)
	{
		cState =0x80;
	};
	if	((cState & 0x80) ==0x80)
	{
		if	((ptUpLeft->x - ptUpRight->x) ==0)
		{
			fSlope = 2.0;
		}
		else
		{
//			fSlope =(float)((float)(ptUpLeft->y - ptUpRight->y)/(float)(ptUpLeft->x - ptUpRight->x));
			if( abs(ptUpLeft->y - ptUpRight->y) > abs(ptUpLeft->x - ptUpRight->x) )
				fSlope = 1.5;
			else
				fSlope = 0;

		};

		ptOne.x = ptDownLeft->x;
		ptOne.y = ptDownLeft->y;
		ptTwo.x = ptDownRight->x;
		ptTwo.y = ptDownRight->y;

		temp1 =GetLenOfTwoPoint( &ptOne, &ptTwo );

		// by david
		if ( (temp1 > 0) && (temp1 < (nColNum - 1)))
		{
			nColNum = (int)( temp1 + 1 );
		};

		// if (temp1 > 0)
		// {
			//nColNum = (int)( temp1 + 1 );
			// nColNum = (int)( temp1 );// by david 20130910
		// }
	}
	else
	{
		if	((ptDownRight->x - ptDownLeft->x) ==0)
		{
			fSlope = 2.0;
		}
		else
		{
//			fSlope =(float)((float)(ptDownLeft->y - ptDownRight->y)/(float)(ptDownLeft->x - ptDownRight->x));
			if( abs(ptDownLeft->y - ptDownRight->y) > abs(ptDownLeft->x - ptDownRight->x) )
				fSlope = 1.5;
			else
				fSlope = 0;
		};

		ptOne.x = ptUpLeft->x;
		ptOne.y = ptUpLeft->y;
		ptTwo.x = ptUpRight->x;
		ptTwo.y = ptUpRight->y;

		temp1 =GetLenOfTwoPoint( &ptOne, &ptTwo );

		// by david
		if ( (temp1 > 0) && (temp1 < (nColNum -1)))
		{
			nColNum = (int)( temp1 + 1 );
		};
		// if (temp1 > 0)
		// {
			//nColNum = (int)( temp1 + 1 );
			// nColNum = (int)( temp1 );//by david 20130910
		// }
	};

	uMaxLines = (Uint16)( MaxDotsInDetect / nColNum );
    cState =0x80;
	/*if	(abs(fSlope) >= 1)
	{
		iRow_Num =abs(ptUpLeft->x -ptDownLeft->x);
		if	(iRow_Num >= abs(ptUpRight->x - ptDownRight->x))
		{
			iRow_Num =abs(ptUpRight->x - ptDownRight->x);
			cState =cState +0x40;
		};
	}
	else*/
	{
		iRow_Num =abs(ptUpLeft->y -ptDownLeft->y);
		if	(iRow_Num >= abs(ptUpRight->y - ptDownRight->y))
		{
			iRow_Num =abs(ptUpRight->y - ptDownRight->y);
			cState =cState +0x40;
		};
	};
	//iRow_Num++;    //by david
	if( iRow_Num > uMaxLines )
		iRow_Num = uMaxLines;

	switch (cState &0xc0)
	{
	case	0x00:
		fBDivder=(((float)ptDownRight->y -(float)ptUpLeft->y)*((float)ptDownRight->x -(float)ptDownLeft->x)*((float)ptUpRight->x -(float)ptDownRight->x)\
		          + ((float)ptDownRight->y -(float)ptDownLeft->y)*((float)ptUpRight->x -(float)ptDownRight->x)*(float)ptUpLeft->x \
		          - ((float)ptUpRight->y -(float)ptDownRight->y)*((float)ptDownRight->x -(float)ptDownLeft->x)*(float)ptDownRight->x);
		fDivder=(((float)ptDownRight->y -(float)ptDownLeft->y)*((float)ptUpRight->x -(float)ptDownRight->x) - ((float)ptUpRight->y -(float)ptDownRight->y)*((float)ptDownRight->x -(float)ptDownLeft->x));
		ptUpRight->x =(unsigned int)(fBDivder/fDivder);

		fBDivder=(((float)ptDownRight->x -(float)ptUpLeft->x)*((float)ptDownRight->y -ptDownLeft->y)*((float)ptUpRight->y -(float)ptDownRight->y)\
		          + ((float)ptDownRight->x -(float)ptDownLeft->x)*((float)ptUpRight->y -(float)ptDownRight->y)*(float)ptUpLeft->y \
		          - ((float)ptUpRight->x -(float)ptDownRight->x)*((float)ptDownRight->y -(float)ptDownLeft->y)*(float)ptDownRight->y);
		fDivder=(((float)ptDownRight->x -(float)ptDownLeft->x)*((float)ptUpRight->y -(float)ptDownRight->y) - ((float)ptUpRight->x -(float)ptDownRight->x)*((float)ptDownRight->y -(float)ptDownLeft->y));
		ptUpRight->y =(Uint16)(fBDivder/fDivder);
		break;
	case	0x40:
		fBDivder=(((float)ptDownLeft->y -(float)ptUpRight->y)*((float)ptDownRight->x -(float)ptDownLeft->x)*((float)ptDownLeft->x -(float)ptUpLeft->x)\
		          + ((float)ptDownRight->y -(float)ptDownLeft->y)*((float)ptDownLeft->x -(float)ptUpLeft->x)*(float)ptUpRight->x \
		          - ((float)ptDownLeft->y -(float)ptUpLeft->y)*((float)ptDownRight->x -(float)ptDownLeft->x)*(float)ptDownLeft->x);
		fDivder=(((float)ptDownRight->y -(float)ptDownLeft->y)*((float)ptDownLeft->x -(float)ptUpLeft->x) - ((float)ptDownLeft->y -(float)ptUpLeft->y)*((float)ptDownRight->x -(float)ptDownLeft->x));
		ptUpLeft->x =(unsigned int)(fBDivder/fDivder);

		fBDivder=(((float)ptDownLeft->x -(float)ptUpRight->x)*((float)ptDownRight->y -(float)ptDownLeft->y)*((float)ptDownLeft->y -(float)ptUpLeft->y)\
		          + ((float)ptDownRight->x -(float)ptDownLeft->x)*((float)ptDownLeft->y -(float)ptUpLeft->y)*(float)ptUpRight->y \
		          - ((float)ptDownLeft->x -(float)ptUpLeft->x)*((float)ptDownRight->y -(float)ptDownLeft->y)*(float)ptDownLeft->y);
		fDivder=(((float)ptDownRight->x -(float)ptDownLeft->x)*((float)ptDownLeft->y -(float)ptUpLeft->y) - ((float)ptDownLeft->x -(float)ptUpLeft->x)*((float)ptDownRight->y -(float)ptDownLeft->y));
		ptUpLeft->y =(Uint16)(fBDivder/fDivder);

		break;
	case	0x80:
		fBDivder=(((float)ptUpRight->y -(float)ptDownLeft->y)*((float)ptUpRight->x -(float)ptUpLeft->x)*((float)ptDownRight->x -(float)ptUpRight->x)\
		          + ((float)ptUpRight->y -(float)ptUpLeft->y)*((float)ptDownRight->x -(float)ptUpRight->x)*(float)ptDownLeft->x \
		          - ((float)ptDownRight->y -(float)ptUpRight->y)*((float)ptUpRight->x -(float)ptUpLeft->x)*(float)ptUpRight->x);
		fDivder=(((float)ptUpRight->y -(float)ptUpLeft->y)*((float)ptDownRight->x -(float)ptUpRight->x) - ((float)ptDownRight->y -(float)ptUpRight->y)*((float)ptUpRight->x -(float)ptUpLeft->x));
		ptDownRight->x =(Uint16)(fBDivder/fDivder);

		fBDivder=(((float)ptUpRight->x -(float)ptDownLeft->x)*((float)ptUpRight->y -(float)ptUpLeft->y)*((float)ptDownRight->y -(float)ptUpRight->y)\
		          + ((float)ptUpRight->x -(float)ptUpLeft->x)*((float)ptDownRight->y -(float)ptUpRight->y)*(float)ptDownLeft->y \
		          - ((float)ptDownRight->x -(float)ptUpRight->x)*((float)ptUpRight->y -(float)ptUpLeft->y)*(float)ptUpRight->y);
		fDivder=(((float)ptUpRight->x -(float)ptUpLeft->x)*((float)ptDownRight->y -(float)ptUpRight->y) - ((float)ptDownRight->x -(float)ptUpRight->x)*((float)ptUpRight->y -(float)ptUpLeft->y));
		ptDownRight->y =(Uint16)(fBDivder/fDivder);
		break;
	default:
		fBDivder=(((float)ptUpLeft->y -(float)ptDownRight->y)*((float)ptUpRight->x -(float)ptUpLeft->x)*((float)ptDownLeft->x -(float)ptUpLeft->x)\
		          + ((float)ptUpRight->y -(float)ptUpLeft->y)*((float)ptDownLeft->x -(float)ptUpLeft->x)*(float)ptDownRight->x \
		          - ((float)ptDownLeft->y -(float)ptUpLeft->y)*((float)ptUpRight->x -(float)ptUpLeft->x)*(float)ptUpLeft->x);
		fDivder=(((float)ptUpRight->y -(float)ptUpLeft->y)*((float)ptDownLeft->x -(float)ptUpLeft->x) - ((float)ptDownLeft->y -(float)ptUpLeft->y)*((float)ptUpRight->x -(float)ptUpLeft->x));
		ptDownLeft->x =(Uint16)(fBDivder/fDivder);

		fBDivder=(((float)ptUpLeft->x -(float)ptDownRight->x)*((float)ptUpRight->y -(float)ptUpLeft->y)*((float)ptDownLeft->y -(float)ptUpLeft->y)\
		          + ((float)ptUpRight->x -(float)ptUpLeft->x)*((float)ptDownLeft->y -(float)ptUpLeft->y)*(float)ptDownRight->y \
		          - ((float)ptDownLeft->x -(float)ptUpLeft->x)*((float)ptUpRight->y -(float)ptUpLeft->y)*(float)ptUpLeft->y);
		fDivder=(((float)ptUpRight->x -(float)ptUpLeft->x)*((float)ptDownLeft->y -(float)ptUpLeft->y) - ((float)ptDownLeft->x -(float)ptUpLeft->x)*((float)ptUpRight->y -(float)ptUpLeft->y));
		ptDownLeft->y =(Uint16)(fBDivder/fDivder);

		break;
	};
	//MaxX=384;
	//MaxY=288;
	//MaxX=384 * 2;
	//MaxY=288 * 2;

	MaxX=FULL_COLS;
	MaxY=FULL_ROWS;

	/*	cNewX=ProgControlWord.ControlWord&0x0c000;
		switch(cNewX){
			case 0x0000 :
					MaxX=384;
					MaxY=288;
					break;
			case 0x4000:
					MaxX=360;
					MaxY=288;
					break;
			case 0x8000:
					MaxX=360;
					MaxY=240;
					break;
			case 0x0c000:
					MaxX=320;
					MaxY=240;
					break;
		} */
	for( iRow = 0; iRow < iRow_Num; iRow++)
	{
		iRow_Num1++;
		for( iCol = 0; iCol < nColNum ; iCol++ )
		{
		/*	ptTemp.x=(Uint16)( (float)ptUpLeft->x + ((float)ptDownLeft->x - (float)ptUpLeft->x)*iRow/(iRow_Num -1)\
			                   +((float)ptUpRight->x - (float)ptUpLeft->x)*iCol/(nColNum -1)\
			                   +((float)ptUpLeft->x + (float)ptDownRight->x - (float)ptUpRight->x - (float)ptDownLeft->x)*iCol/((iRow_Num -1) *(nColNum -1)) );
			ptTemp.y=(Uint16)( (float)ptUpLeft->y + ((float)ptDownLeft->y - (float)ptUpLeft->y)*iRow/(iRow_Num -1)\
			                   +((float)ptUpRight->y -(float)ptUpLeft->y)*iCol/(nColNum -1)\
			                   +((float)ptUpLeft->y + (float)ptDownRight->y - (float)ptUpRight->y - (float)ptDownLeft->y)*iCol/((iRow_Num -1) *(nColNum -1)) );

		*/
			temp1=(float)ptUpRight->x + ((float)ptDownRight->x-(float)ptUpRight->x)*iRow/(iRow_Num -1);
			temp2=(float)ptUpLeft->x + ((float)ptDownLeft->x - (float)ptUpLeft->x)*iRow/(iRow_Num -1);
			ptTemp.x=(Uint16)( temp2+(temp1-temp2)*iCol/(nColNum-1)+((float)ptUpLeft->x + (float)ptDownRight->x - (float)ptUpRight->x - (float)ptDownLeft->x)*iCol/((iRow_Num -1) *(nColNum -1)));
			ptTemp.y=(Uint16)( (float)ptUpLeft->y + ((float)ptDownLeft->y - (float)ptUpLeft->y)*iRow/(iRow_Num -1)\
				+((float)ptUpRight->y -(float)ptUpLeft->y)*iCol/(nColNum -1)\
				+((float)ptUpLeft->y + (float)ptDownRight->y - (float)ptUpRight->y - (float)ptDownLeft->y)*iCol/((iRow_Num -1) *(nColNum -1)) );

			
			if(ptTemp.x>MaxX||ptTemp.y>MaxY)
			{
				break;
			}
			
			ImageAddr=((Uint32)ptTemp.y*(Uint32)MaxX)+((Uint32)ptTemp.x);
			//ImageAddr=(((ImageAddr&0x18000)<<1)|(ImageAddr&0x7fff))|0x88000;
			/*
			if((unsigned int)(ptStorePlace+iRow*nColNum*2+2*iCol+1)<0x8000){
				ptRowAndCol.x = 0;
				ptRowAndCol.y = 0;
				return	ptRowAndCol;
			}
			(*(ptStorePlace+iRow*nColNum*2+2*iCol)) =(unsigned int)(ImageAddr>>16);
			(*(ptStorePlace+iRow*nColNum*2+2*iCol+1)) =(unsigned int)ImageAddr;
			*/
			
			*(ptStorePlace+iRow*nColNum+iCol) = 2 * ImageAddr + 1;

		}
	}


	ptRowAndCol.x = (Uint16)nColNum;
	ptRowAndCol.y = iRow_Num1;

	return ptRowAndCol;
}


///////////////////////////////////////////////
//  end
///////////////////////////////////////////////

bool DayorNight(unsigned char* puNewImage,int nWidth,int nHight, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	int row,col,count = 0;
	double sum = 0;
	int cmax = 0, cmin = 0;
	//double hsum = 0, lsum = 0;
	long long hsum = 0, lsum = 0;
	int hcount = 0, lcount = 0;
	
	for (row=0;row<nHight;row++)
	{
		for(col=0;col<nWidth;col++)
		{
			count++;
			sum+=GetImagePointValue(col,row,pCfgs->m_iHeight,pCfgs->m_iWidth,puNewImage);

			
			if (cmax==0)
			{
				cmax=GetImagePointValue(col,row,nHight,nWidth,puNewImage);
			}
			else if (cmax<GetImagePointValue(col,row,nHight,nWidth,puNewImage))
			{
				cmax=GetImagePointValue(col,row,nHight,nWidth,puNewImage);
			}
			if (cmin==0)
			{
				cmin=GetImagePointValue(col,row,nHight,nWidth,puNewImage);
			}
			else if (cmin>GetImagePointValue(col,row,nHight,nWidth,puNewImage))
			{
				cmin=GetImagePointValue(col,row,nHight,nWidth,puNewImage);
			}
		}
	}
	
	if (count)
	{
		sum=(double)(sum/count);
	}

	for (row=0;row<nHight;row++)
	{
		for (col=0;col<nWidth;col++)
		{
			if (GetImagePointValue(col,row,nHight,nWidth,puNewImage)>(cmax+cmin)/2)
			{
				hsum+=GetImagePointValue(col,row,nHight,nWidth,puNewImage);
				hcount++;
			}
			else
			{
				lsum+=GetImagePointValue(col,row,nHight,nWidth,puNewImage);
				lcount++;
			}
		}
	}
	
	if (lcount&&hcount)
	{
		hsum /= hcount;
		lsum /= lcount;
	}
	//pCfgs->guTest1=sum;
	//pCfgs->guTest2=hsum-lsum;
	if ((hsum-lsum)>60&&sum<100)//
	{
		return FALSE;
	}
		
	return TRUE;
}

void NightLightDetect(unsigned char* puNewImage, unsigned char* puTempImage,ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	
	int i,j,l,k,t,s; //n,n1,n2,Color=0
	int col,row;
	unsigned char* MidImage;

	int dwArrayCurrentSize, dwArraySize, dwProcessedSize;		

   	//int lthreold=10,rthreold=pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold1;//200;
   	int lthreold=pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold,rthreold=pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold1;//600
	CRect rtConnect;
	CRect fore;
	
	int Mean= 0;
	int area;
	double Arearate=0.0;
	int AreaA,AreaB;
//	int vertical=0,recover=0;
	int dy=0,dx=0;
	int center;

	int position=0;
	int position1=0;
	int dwForegroundCount;

		
	ArrayInit((Array *)pParams->ForegroundArray, sizeof(CForeground_Array), FOREGROUND_ARRAY_CAPACITY);////?????
	pCfgs->thresholdValue=pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold2;
	pCfgs->thresholdValue=(pCfgs->thresholdValue<150)? 150:pCfgs->thresholdValue;
	memset(puTempImage, 0, sizeof(unsigned char) * pCfgs->m_iWidth * pCfgs->m_iHeight);
	for(row = 0; row < pCfgs->m_iHeight; ++row)
	{
		for(col = 0; col < pCfgs->m_iWidth; ++col)
		{
			////?????
			//Mean= 0;
			/*for(k=0; k<3; k++)
			{
			for(l=0; l<3; l++)
			{
			if((row+l) < pCfgs->m_iHeight && (col+k) < pCfgs->m_iWidth)
			{
			Mean+=GetImagePointValue(col+k,row+l,pCfgs->m_iHeight,pCfgs->m_iWidth,puNewImage);//puNewImage
			}
			}
			}*/
			Mean =GetImagePointValue(col,row,pCfgs->m_iHeight,pCfgs->m_iWidth,puNewImage);
			if (Mean>pCfgs->thresholdValue)
			{
				Mean= 0;
				for(k = 0; k < 3; ++k)
				{
					for(l = 0; l < 3; ++l)
					{
						if((row+l) < pCfgs->m_iHeight && (col+k) < pCfgs->m_iWidth)
						{
							Mean+=GetImagePointValue(col+k,row+l,pCfgs->m_iHeight,pCfgs->m_iWidth,puNewImage);//puNewImage
						}
					}
				}
				Mean/=9;
				if (Mean>pCfgs->thresholdValue)
				{
					SetImagePointValue(col,row,pCfgs->m_iHeight,pCfgs->m_iWidth,255,puTempImage);
				}
			}

		}
	}
	
	MidImage=pParams->puPointNextImage;
	memset(MidImage, 0, sizeof(unsigned char) * pCfgs->m_iWidth *pCfgs->m_iHeight);
	memset(pParams->m_pPointArray,0,sizeof(CPoint) * pCfgs->m_iWidth *pCfgs->m_iHeight);
	
	//pCfgs->guTest2=0;

	
	for( row=0; row<pCfgs->m_iHeight; row+=2 )
	{
		for( col=0; col<pCfgs->m_iWidth; col+=2 )
		{
			if(GetImagePointValue(col,row,pCfgs->m_iHeight,pCfgs->m_iWidth,puTempImage) 
			   &&(!GetImagePointValue(col,row,pCfgs->m_iHeight,pCfgs->m_iWidth,MidImage))
			   )
			{

				// Got a start point
				pParams->m_pPointArray[0].x =col;
				rtConnect.left = col;
				rtConnect.right = col;
				
				pParams->m_pPointArray[0].y = row;
				rtConnect.top = row;
				rtConnect.bottom = row;
				
				dwArrayCurrentSize = 1;
				dwProcessedSize = 0;
				do
				{
					dwArraySize = dwArrayCurrentSize;
					for(l=dwProcessedSize; l<dwArraySize; ++l)
					{
						rtConnect.left=( rtConnect.left < pParams->m_pPointArray[l].x )? rtConnect.left : pParams->m_pPointArray[l].x;
						rtConnect.right=( rtConnect.right > pParams->m_pPointArray[l].x )? rtConnect.right : pParams->m_pPointArray[l].x;
						rtConnect.top=( rtConnect.top < pParams->m_pPointArray[l].y )? rtConnect.top : pParams->m_pPointArray[l].y;
						rtConnect.bottom=( rtConnect.bottom > pParams->m_pPointArray[l].y )? rtConnect.bottom : pParams->m_pPointArray[l].y;
						for( t=-1; t<=1; ++t)
						{
							for( s=-1; s<=1; ++s)
							{
								k = pParams->m_pPointArray[l].y + t;
								j = pParams->m_pPointArray[l].x + s;
								if((k >= 0)
								    &&(k < pCfgs->m_iHeight)
								    &&(j >= 0)
								    &&(j < pCfgs->m_iWidth)
									)
								{
									if(GetImagePointValue(j,k,pCfgs->m_iHeight,pCfgs->m_iWidth,puTempImage) && 
										!GetImagePointValue(j,k,pCfgs->m_iHeight,pCfgs->m_iWidth,MidImage))
									{
										SetImagePointValue(j,k,pCfgs->m_iHeight,pCfgs->m_iWidth,1,MidImage);
										pParams->m_pPointArray[dwArrayCurrentSize].x = j;
										pParams->m_pPointArray[dwArrayCurrentSize].y = k;
										dwArrayCurrentSize++;						
									}
								}
							}
						}
					}
					dwProcessedSize = dwArraySize;
				}while(dwArrayCurrentSize > dwArraySize);
						
				
				area=RECT_Width(rtConnect)*RECT_Height(rtConnect)+1;
			    		

				//if(area!=0)
				//{
				// Add to foreground array
					if(((RECT_Width(rtConnect) + 1) * (RECT_Height(rtConnect) + 1))>lthreold
					     &&((RECT_Width(rtConnect) + 1) * (RECT_Height(rtConnect) + 1))<rthreold
					     &&((RECT_Height(rtConnect) + 1)<(int)(pCfgs->m_iHeight/2))
					     &&(int)((RECT_Height(rtConnect) + 1)/(RECT_Width(rtConnect) + 1))<3//3
						 &&(int)((RECT_Width(rtConnect) + 1)/(RECT_Height(rtConnect) + 1))<3
						 &&((double)((double)dwArraySize/(double)area)>0.3)
						 )
					{
						fore.left=rtConnect.left;
						fore.right=rtConnect.right;
						fore.top=rtConnect.top;
						fore.bottom=rtConnect.bottom;
						ArrayAdd((Array *)pParams->ForegroundArray,(void*)&fore);
						//pCfgs->guTest2++;	
							
					}
					//else if (((RECT_Width(rtConnect) + 1) * (RECT_Height(rtConnect) + 1))>rthreold 
					         // &&((RECT_Height(rtConnect) + 1)>(pCfgs->m_iHeight/5))  
					         // &&((double)(dwArraySize/area)>0.5)                            
						     // &&(int)((RECT_Width(rtConnect) + 1)/(RECT_Height(rtConnect) + 1))<5   
						     // &&(int)((RECT_Height(rtConnect) + 1)/(RECT_Width(rtConnect) + 1))<5//5  
						     // &&((int)((rtConnect.left+rtConnect.right)/2)>(int)(pCfgs->m_iWidth/4))      
						     // &&(((rtConnect.left+rtConnect.right)/2)<(0.8*pCfgs->m_iWidth))
						     // )
				//	{
					//	fore.left=rtConnect.left;
					//	fore.right=rtConnect.right;
					//	fore.top=rtConnect.top;
					//	fore.bottom=rtConnect.bottom;
					//	
					//	ArrayRemoveAll((Array *)pParams->ForegroundArray);
					//	ArrayAdd((Array *)pParams->ForegroundArray,(void*)&fore);	
					//	pParams->NormalDetectCfg.cSpeedDesc.fitflag=TRUE;
                        
						 //pCfgs->guTest1++;
				//	}
				//}
			}
		}
	}

	dwForegroundCount = ArrayGetSize((Array *)pParams->ForegroundArray);
	
	if (dwForegroundCount>1)
	{
			//pCfgs->guTest = dwForegroundCount;
		for (i = 0; i < dwForegroundCount - 1; ++i)
		{
			for (j = i + 1; j < dwForegroundCount; ++j)
			{
		    	Arearate=0.0;
				AreaA=0;
				AreaB=0;
				AreaA=(RECT_Width(pParams->ForegroundArray->array[i].m_rtConnect)+1)*(RECT_Height(pParams->ForegroundArray->array[i].m_rtConnect)+1);
				AreaB=(RECT_Width(pParams->ForegroundArray->array[j].m_rtConnect)+1)*(RECT_Height(pParams->ForegroundArray->array[j].m_rtConnect)+1);
				
				Arearate=(double)((double)min(AreaA,AreaB)/(double)max(AreaB,AreaA));
			//	recover=0;
			//	vertical=max(pParams->ForegroundArray->array[i].m_rtConnect.top,pParams->ForegroundArray->array[j].m_rtConnect.top)-
			//		min(pParams->ForegroundArray->array[i].m_rtConnect.bottom,pParams->ForegroundArray->array[j].m_rtConnect.bottom);
			/*	if(min(RECT_Height(pParams->ForegroundArray->array[i].m_rtConnect), RECT_Height(pParams->ForegroundArray->array[j].m_rtConnect))!=0)
				{
					recover=-vertical/min(RECT_Height(pParams->ForegroundArray->array[i].m_rtConnect) ,RECT_Height(pParams->ForegroundArray->array[j].m_rtConnect));
				}
				*/
				dy=0;
				dx=0;
				dy=max(pParams->ForegroundArray->array[i].m_rtConnect.bottom,pParams->ForegroundArray->array[j].m_rtConnect.bottom)-
					min(pParams->ForegroundArray->array[i].m_rtConnect.top,pParams->ForegroundArray->array[j].m_rtConnect.top);
				dx=max(pParams->ForegroundArray->array[i].m_rtConnect.right,pParams->ForegroundArray->array[j].m_rtConnect.right)-
					min(pParams->ForegroundArray->array[i].m_rtConnect.left,pParams->ForegroundArray->array[j].m_rtConnect.left);
				center=abs(pParams->ForegroundArray->array[i].m_rtConnect.top-pParams->ForegroundArray->array[j].m_rtConnect.top);
				
				
				

				if (Arearate>0.2              
				    &&dx>(int)(pCfgs->m_iWidth/4)    
				    &&dy<(int)(pCfgs->m_iHeight/2)
				    &&dy<8*min(RECT_Height(pParams->ForegroundArray->array[i].m_rtConnect),RECT_Height(pParams->ForegroundArray->array[j].m_rtConnect)))
				{
					if (pParams->NormalDetectCfg.cSpeedDesc.fitflag==FALSE)
					{
						pParams->ForegroundArray->array[i].m_rtConnect.bottom=max(pParams->ForegroundArray->array[i].m_rtConnect.bottom,pParams->ForegroundArray->array[j].m_rtConnect.bottom);
						pParams->ForegroundArray->array[i].m_rtConnect.right=max(pParams->ForegroundArray->array[i].m_rtConnect.right,pParams->ForegroundArray->array[j].m_rtConnect.right);
						pParams->ForegroundArray->array[i].m_rtConnect.left=min(pParams->ForegroundArray->array[i].m_rtConnect.left,pParams->ForegroundArray->array[j].m_rtConnect.left);
						pParams->ForegroundArray->array[i].m_rtConnect.top=min(pParams->ForegroundArray->array[i].m_rtConnect.top,pParams->ForegroundArray->array[j].m_rtConnect.top);
						
						//ForegroundArray.RemoveAt(j);
						ArrayRemoveAt((Array *)pParams->ForegroundArray,j);
						
						position=i;
						dwForegroundCount--;
						pParams->NormalDetectCfg.cSpeedDesc.fitflag=TRUE;

						//	pCfgs->guTest3=6;
					}
					else
					{
						pParams->ForegroundArray->array[i].m_rtConnect.bottom=max(pParams->ForegroundArray->array[i].m_rtConnect.bottom,pParams->ForegroundArray->array[j].m_rtConnect.bottom);
						pParams->ForegroundArray->array[i].m_rtConnect.right=max(pParams->ForegroundArray->array[i].m_rtConnect.right,pParams->ForegroundArray->array[j].m_rtConnect.right);
						pParams->ForegroundArray->array[i].m_rtConnect.left=min(pParams->ForegroundArray->array[i].m_rtConnect.left,pParams->ForegroundArray->array[j].m_rtConnect.left);
						pParams->ForegroundArray->array[i].m_rtConnect.top=min(pParams->ForegroundArray->array[i].m_rtConnect.top,pParams->ForegroundArray->array[j].m_rtConnect.top);
						//ForegroundArray.RemoveAt(j);
						ArrayRemoveAt((Array *)pParams->ForegroundArray,j);
						position1=i;
						dwForegroundCount--;

					//	pCfgs->guTest3= 8;
					}
				}
			}
		}
	}
	
	//非车灯区域置0
	memset(puTempImage, 0, sizeof(unsigned char) * pCfgs->m_iWidth * pCfgs->m_iHeight);
	if (pParams->NormalDetectCfg.cSpeedDesc.fitflag==TRUE)
	{

		if (position1!=0                    
			&&position1<ArrayGetSize((Array *)pParams->ForegroundArray)    
			&&position<ArrayGetSize((Array *)pParams->ForegroundArray)     
			)
		{
			if (pParams->ForegroundArray->array[position1].m_rtConnect.top<pParams->ForegroundArray->array[position].m_rtConnect.top)
			{
				position=position1;
			}
		}
		pParams->NormalDetectCfg.cSpeedDesc.fitflag=FALSE;

		for(row = pParams->ForegroundArray->array[position].m_rtConnect.top; row <= pParams->ForegroundArray->array[position].m_rtConnect.bottom; ++row)
		{
			for(col = 0; col < pCfgs->m_iWidth; ++col)
			{
				SetImagePointValue(col,row,pCfgs->m_iHeight,pCfgs->m_iWidth,255,puTempImage);

			}

		}
	}
	else
	{
		if (dwForegroundCount>0)
		{
			for (i=0;i<dwForegroundCount;i++)
			{
				if (RECT_Width(pParams->ForegroundArray->array[i].m_rtConnect)*RECT_Height(pParams->ForegroundArray->array[i].m_rtConnect)>lthreold&&//pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold60
					RECT_Width(pParams->ForegroundArray->array[i].m_rtConnect)/(RECT_Height(pParams->ForegroundArray->array[i].m_rtConnect)+1)<3
					&&RECT_Height(pParams->ForegroundArray->array[i].m_rtConnect)/(RECT_Width(pParams->ForegroundArray->array[i].m_rtConnect)+1)<3//3
					&&RECT_Height(pParams->ForegroundArray->array[i].m_rtConnect)<pCfgs->m_iHeight/2)
				{
					for(row = pParams->ForegroundArray->array[i].m_rtConnect.top; row <= pParams->ForegroundArray->array[i].m_rtConnect.bottom; ++row)
					{
						for(col = 0; col < pCfgs->m_iWidth; ++col)
						{
							SetImagePointValue(col,row,pCfgs->m_iHeight,pCfgs->m_iWidth,255,puTempImage);

						}

					}
					position=10;
					break;
				}
			}
		}

	}
}

int AveGray(unsigned char* puNewImage,int nWidth,int nHight,ALGCFGS *pCfgs)
{
	int row,col,count = 0;
	int sum = 0;
	for (row=0;row<nHight;row+=5)
	{
		unsigned char* ptr = puNewImage + row * nWidth;
		for(col=0;col<nWidth;col+=2)
		{
			count++;
			sum+=ptr[col];	
		}
	}
	sum = (count > 0)? sum / count : 0;
	return sum;
}

void SobelCalculate(unsigned char *puPointNewImage,unsigned char *puPointSobelImage,int threshold,int width,int height)
{
	unsigned char *in;   
	unsigned char *out;            

	int H, O, V, i, j;
	int i00, i01, i02;
	int i10,      i12;
	int i20, i21, i22;

	in = puPointNewImage;
	out = puPointSobelImage;
	for (i = 1; i < height - 1; i++)
	{
		in = puPointNewImage + i * width;
		out = puPointSobelImage + i * width;
		for(j = 1; j < width - 1; j++)
		{
			in = in + j;
			i00=*(in - width - 1);
			i01=*(in - width);
			i02=*(in - width + 1);
			i10=*(in - 1);
			i12=*(in + 1);
			i20=*(in + width - 1);
			i21=*(in + width);
			i22=*(in + width + 1);

			H = -   i00 - 2*i01 -   i02 +
				+   i20 + 2*i21 +   i22;

			V = -   i00         +   i02
				- 2*i10         + 2*i12
				-   i20         +   i22;

			O = abs(H) + abs(V);
			O = (O < threshold)? 0:255;
			*(out+j) = O;

		}

	}
}

////20140102 array struct start

void ArrayInit(Array *nArray, int sizeOfStruct, int capacity)
{
	nArray->sizeOfStruct = sizeOfStruct;
	nArray->sizeOfArray = 0;
	nArray->capacity = capacity;
}

int ArrayGetCapaticy(Array *nArray)
{
	return nArray->capacity;
}

int ArrayGetSize(Array *nArray)
{
	return nArray->sizeOfArray;
}

int ArrayAdd(Array *nArray, void* elem)
{
	int size;
//	ObjTracking_Array *pObjTrackArray = NULL;
//	ObjTracking_Array2 *pObjTrackArray2 = NULL;
//	ObjectMultiInfo_Array *pObjectMultiInfoArray = NULL;
	CForeground_Array *pCForegroundArray = NULL;

	if( nArray->sizeOfArray > (nArray->capacity - 1))
	{
		nArray->sizeOfArray  = nArray->capacity - 1;
		return nArray->sizeOfArray;
	}

	size = nArray->sizeOfStruct;
	switch(size)
	{
/*	case sizeof(ObjTracking_Array):
		pObjTrackArray = (ObjTracking_Array *)nArray;
		memcpy((void*)&pObjTrackArray->array[nArray->sizeOfArray], (void*)elem, sizeof(POINT));
		nArray->sizeOfArray++;
		break;
	case sizeof(ObjTracking_Array2):
		pObjTrackArray2 = (ObjTracking_Array2 *)nArray;
		memcpy((void*)&pObjTrackArray2->array[nArray->sizeOfArray], (void*)elem, sizeof(Point_Array));
		nArray->sizeOfArray++;
		break;
	case sizeof(ObjectMultiInfo_Array):
		pObjectMultiInfoArray = (ObjectMultiInfo_Array *)nArray;
		memcpy((void*)&pObjectMultiInfoArray->array[nArray->sizeOfArray], (void*)elem, sizeof(struct ObjectMultiInfo));
		nArray->sizeOfArray++;
		break;
	*/
	case sizeof(CForeground_Array):
		pCForegroundArray = (CForeground_Array *)nArray;
		memcpy((void*)&pCForegroundArray->array[nArray->sizeOfArray], (void*)elem, sizeof(CRect));
		nArray->sizeOfArray++;
		break;
	default:
		break;
	}

	return nArray->sizeOfArray;

}

void ArrayRemoveAt( Array *nArray, int nIndex )
{
	int i;
	int size;
/*	ObjTracking_Array *pObjTrackArray = NULL;
	ObjTracking_Array2 *pObjTrackArray2 = NULL;
	ObjectMultiInfo_Array *pObjectMultiInfoArray = NULL;
*/
	CForeground_Array *pCForegroundArray = NULL;
	CRect* SourceAddress;
	CRect* TargetAddress;
	
	if( nIndex <= (nArray->sizeOfArray - 1) )
	{
		size = nArray->sizeOfStruct;
		switch(size)
		{
/*		case sizeof(ObjTracking_Array):
			pObjTrackArray = (ObjTracking_Array *)nArray;
			for( i = nIndex; i <= (nArray->sizeOfArray - 2); i++ )
			{
				SourceAddress = (char*)&pObjTrackArray->array[i+1];
				TargetAddress = (char*)&pObjTrackArray->array[i];
				memcpy((void*)TargetAddress, (void*)SourceAddress, sizeof(POINT));
			}
			nArray->sizeOfArray--;
			break;
		case sizeof(ObjTracking_Array2):
			pObjTrackArray2 = (ObjTracking_Array2 *)nArray;
			for( i = nIndex; i <= (nArray->sizeOfArray - 2); i++ )
			{
				SourceAddress = (char*)&pObjTrackArray2->array[i+1];
				TargetAddress = (char*)&pObjTrackArray2->array[i];
				memcpy((void*)TargetAddress, (void*)SourceAddress, sizeof(Point_Array));
			}
			nArray->sizeOfArray--;
			break;
		case sizeof(ObjectMultiInfo_Array):
			pObjectMultiInfoArray = (ObjectMultiInfo_Array *)nArray;
			for( i = nIndex; i <= (nArray->sizeOfArray - 2); i++ )
			{
				SourceAddress = (char*)&pObjectMultiInfoArray->array[i+1];
				TargetAddress = (char*)&pObjectMultiInfoArray->array[i];
				memcpy((void*)TargetAddress, (void*)SourceAddress, sizeof(struct ObjectMultiInfo));
			}
			nArray->sizeOfArray--;
			break;
	*/
		case sizeof(CForeground_Array):
			pCForegroundArray = (CForeground_Array *)nArray;
			for( i = nIndex; i <= (nArray->sizeOfArray - 2); i++ )
			{
				SourceAddress = (CRect *)&pCForegroundArray->array[i+1];
				TargetAddress = (CRect *)&pCForegroundArray->array[i];
				memcpy((void*)TargetAddress, (void*)SourceAddress, sizeof(CRect));
			}
			nArray->sizeOfArray--;
			break;
		default:
			break;
		}
		
	}
}


void ArraySetSize(Array *nArray, int size)
{
	nArray->sizeOfArray = size;
}

void ArrayRemoveAll(Array *nArray)
{
	nArray->sizeOfArray = 0;
}
////end array struct

int StandardDeviation (unsigned char* puNewImage,int width,int height,int start_line,int stop_line)
{

	int x=0,y=0;
	Int32 average = 0;
	Int32 variance = 0;
	int num=0;
	for (y=start_line;y <stop_line;y+=5)
	{
		unsigned char* ptr = puNewImage + y * width;
		for (x=0;x <width;x+=2)
		{
			average += ptr[x];
			num++;
		}
	} 
	average = (num > 0)? average / num : 0;

	
	for (y=start_line;y <stop_line;y+=5)
	{
		unsigned char* ptr = puNewImage + y * width;
		for (x=0;x <width;x+=2)
		{
			int temp = ptr[x];
			variance += (temp - average)*(temp - average);
		}
	}
	variance = (num > 0)? variance / num : 0;
    return (int)(sqrt((float)variance)+0.5);
}

////20140709
double TeamLength1(unsigned char* puNewImage,ALGCFGS *pCfgs,ALGPARAMS *pParams,CPoint *LineUp,bool inLaneCar,Uint16  LaneID)
{

	int i,j,row,col,k,t,s,l,p,count=0;
	int dwArrayCurrentSize, dwArraySize, dwProcessedSize;
	int width = 0, height = 0;
	CRect rtConnect;

	int light=0;
	int maxValue=0,minValue=-1;
	double percent1=0;

	unsigned char* MidImage = pParams->puSobelTempImageX;
	pCfgs->thresholdValue=pParams->NormalDetectCfg.cSpeedDesc.uSpeedCounterChangedThreshold2;
	pCfgs->thresholdValue=(pCfgs->thresholdValue<180)? 180:pCfgs->thresholdValue;
	memset(MidImage,0,sizeof(unsigned char) * pCfgs->team_width *pCfgs->team_height);

	for (p=0;p<10;p++)
	{
		for (row=(pCfgs->team_height*(10-p)/10-1);row>=(pCfgs->team_height*(9-p)/10);row-= 2)
		{
			for (col=0;col<pCfgs->team_width;col+=2)
			{
				if(GetImagePointValue(col,row,pCfgs->team_height,pCfgs->team_width,puNewImage)>pCfgs->thresholdValue
					&&(!GetImagePointValue(col,row,pCfgs->team_height,pCfgs->team_width,MidImage))
					)
				{

					// Got a start point
					pParams->m_pPointArray[0].x =col;
					rtConnect.left = col;
					rtConnect.right = col;

					pParams->m_pPointArray[0].y = row;
					rtConnect.top = row;
					rtConnect.bottom = row;

					dwArrayCurrentSize = 1;
					dwProcessedSize = 0;
					do
					{
						dwArraySize = dwArrayCurrentSize;
						for(l=dwProcessedSize; l<dwArraySize; ++l)
						{
							rtConnect.left=( rtConnect.left < pParams->m_pPointArray[l].x )? rtConnect.left : pParams->m_pPointArray[l].x;
							rtConnect.right=( rtConnect.right > pParams->m_pPointArray[l].x )? rtConnect.right : pParams->m_pPointArray[l].x;
							rtConnect.top=( rtConnect.top < pParams->m_pPointArray[l].y )? rtConnect.top : pParams->m_pPointArray[l].y;
							rtConnect.bottom=( rtConnect.bottom > pParams->m_pPointArray[l].y )? rtConnect.bottom : pParams->m_pPointArray[l].y;
							for( t=-1; t<=1; ++t)
							{
								for( s=-1; s<=1; ++s)
								{
									k = pParams->m_pPointArray[l].y + t;
									j = pParams->m_pPointArray[l].x + s;
									if((k >= 0)
										&&(k < pCfgs->team_height)
										&&(j >= 0)
										&&(j < pCfgs->team_width)
										)
									{
										if(GetImagePointValue(j,k,pCfgs->team_height,pCfgs->team_width,puNewImage)>pCfgs->thresholdValue && 
											!GetImagePointValue(j,k,pCfgs->team_height,pCfgs->team_width, MidImage))
										{
											SetImagePointValue(j,k,pCfgs->team_height,pCfgs->team_width,255, MidImage);
											pParams->m_pPointArray[dwArrayCurrentSize].x = j;
											pParams->m_pPointArray[dwArrayCurrentSize].y = k;
											dwArrayCurrentSize++;
										}
									}
								}
							}
						}
						dwProcessedSize = dwArraySize;
					}while(dwArrayCurrentSize > dwArraySize);
					width = rtConnect.right - rtConnect.left + 1;
					height = rtConnect.bottom - rtConnect.top + 1;
					if (width/height<3&&height/width<(6 - p / 2)&& width * height>(40-2 * p))
					{
						pParams->NormalDetectCfg.cSpeedDesc.position[count]=(rtConnect.bottom+rtConnect.top)/2;
						count++;
					}
				}
			}
		}
		for(i=0;i<count-1;i++)
		{
			if (abs(pParams->NormalDetectCfg.cSpeedDesc.position[i]-pParams->NormalDetectCfg.cSpeedDesc.position[i+1])<(20+2*(4-p)))
			{
				for (j=i+1;j<count-1;j++)
				{
					pParams->NormalDetectCfg.cSpeedDesc.position[j]=pParams->NormalDetectCfg.cSpeedDesc.position[j+1];
				}
				count--;
				i--;
			}
		}
	}

	//no light
	for(i=0;i<count;i++)
	{
		if (pParams->NormalDetectCfg.cSpeedDesc.recount)
		{
			for (j=0;j<pParams->NormalDetectCfg.cSpeedDesc.recount;j++)
			{
				if (abs(pParams->NormalDetectCfg.cSpeedDesc.position[i]-pParams->NormalDetectCfg.cSpeedDesc.preposition[j])<15&&
					pParams->NormalDetectCfg.cSpeedDesc.position[i]>(0.6*pCfgs->team_height))//在图像的下半部分
				{
					light++;
					if (maxValue < pParams->NormalDetectCfg.cSpeedDesc.position[i])
					{
						maxValue = pParams->NormalDetectCfg.cSpeedDesc.position[i];
						if(minValue == -1)//给minValue一个值
						{
							minValue=pParams->NormalDetectCfg.cSpeedDesc.position[i];
						}
					}
					if(minValue > pParams->NormalDetectCfg.cSpeedDesc.position[i] && (minValue - pParams->NormalDetectCfg.cSpeedDesc.position[i] < 0.5 * pCfgs->team_height))
						minValue=pParams->NormalDetectCfg.cSpeedDesc.position[i];
				}	
				else if (abs(pParams->NormalDetectCfg.cSpeedDesc.position[i]-pParams->NormalDetectCfg.cSpeedDesc.preposition[j])<5&&
					pParams->NormalDetectCfg.cSpeedDesc.position[i]<(0.6*pCfgs->team_height))
				{
					light++;
					if(minValue > pParams->NormalDetectCfg.cSpeedDesc.position[i] && (minValue - pParams->NormalDetectCfg.cSpeedDesc.position[i] < 0.5 * pCfgs->team_height))
						minValue=pParams->NormalDetectCfg.cSpeedDesc.position[i];
				}
			}
		}
	}
	memcpy(pParams->NormalDetectCfg.cSpeedDesc.preposition,pParams->NormalDetectCfg.cSpeedDesc.position,sizeof(int)*count);
	pParams->NormalDetectCfg.cSpeedDesc.recount=count;

	if (maxValue>0.5*pCfgs->team_height&&(maxValue - minValue) > 0.2 * pCfgs->team_height &&light>1 && inLaneCar/*&&pParams->NormalDetectCfg.cSpeedDesc.position[2]>0.3*pCfgs->team_height*/)
	{

		percent1=1-(double)minValue/pCfgs->team_height;
		pParams->NormalDetectCfg.cSpeedDesc.lxtime++;

	}
	/*else if (count>7&&pParams->NormalDetectCfg.cSpeedDesc.position[2]>0.4*pCfgs->team_height)
	{
		pParams->NormalDetectCfg.cSpeedDesc.percent=(double)count/10;
	}*/
	else
	{
		percent1 = 0;
		pParams->NormalDetectCfg.cSpeedDesc.lxtime = 0;
	}
	//if (percent<0.7&&light>2)//&&max>0.6*team_height&&min<0.3*team_height
	//{
	//	percent=1-(double)position[count-1]/team_height;
	//}
	if(pParams->NormalDetectCfg.cSpeedDesc.percent>0.6&&percent1<0.3 && inLaneCar)
		pParams->NormalDetectCfg.cSpeedDesc.percent=pParams->NormalDetectCfg.cSpeedDesc.percent-0.1;
	else 
	{
		if(percent1 > 0 && pParams->NormalDetectCfg.cSpeedDesc.lxtime <= 1 && pParams->NormalDetectCfg.cSpeedDesc.percent < 0.05)//防止误检
			pParams->NormalDetectCfg.cSpeedDesc.percent = 0;
		else
			pParams->NormalDetectCfg.cSpeedDesc.percent=percent1;
	}
	if (pParams->NormalDetectCfg.cSpeedDesc.percent>1)
		pParams->NormalDetectCfg.cSpeedDesc.percent=1;
    //pCfgs->guTestB[LaneID] = light;
	//pCfgs->guTestC[LaneID] = pParams->NormalDetectCfg.cSpeedDesc.percent * 100;
	return pParams->NormalDetectCfg.cSpeedDesc.percent;
}

double QueueLengthDetect(unsigned char* OriginImage,unsigned char* puSubImage,unsigned char* puSobelImage, unsigned char* puSobelConnectImage,ALGCFGS *pCfgs, ALGPARAMS *pParams,Uint16 LaneID)
{
	bool IsFullQueue=FALSE;
	bool IsPartQueue[8]={FALSE};
	int headline=0;
	int num=0;
	int quenum=0;
	int topy=0;
	int continuous_que = 0;
	int aveheight=pCfgs->team_height/8;
	float EXPlength=0.0;
	if(pCfgs->team_height <= 0)
		return 0;
	//IsFullQueue = IsVehicleQueue(puSubImage, puSobelImage,0,pCfgs->team_height, pCfgs, pParams);//

	for(headline=pCfgs->team_height;(headline-aveheight)>=0;headline=(headline-aveheight))
	{

		IsPartQueue[num] = IsVehicleQueue(puSubImage, puSobelImage,headline-aveheight,headline, pCfgs, pParams);//

		if (IsPartQueue[num] ==TRUE)
		{
			quenum++;
			if (num > 0 && IsPartQueue[num-1] == TRUE)
			{
				continuous_que++;
				if(num * aveheight - topy < aveheight + 1)
				{
					topy = aveheight * (num + 1);
				}
				else if(continuous_que >=2 && (num-continuous_que)*aveheight-topy<2*aveheight)
				{
					topy = aveheight * (num + 1);
				}
				else if(continuous_que >=2 && (num-continuous_que)*aveheight-topy<=2*aveheight&&num == 4)
				{
					topy = aveheight * (num + 1);
				}
			}

		}
		else
		{
			continuous_que = 0;
		}
		num++;


	}

	/*if ((quenum>=3)||(IsFullQueue ==TRUE))
	{
		topy=aveheight*(lasttruepart+1);
	}*/

	if (topy<=5)
	{
		topy=0;
	}
	//if (quenum>1 &&(IsPartQueue[1]==TRUE||IsPartQueue[0]==TRUE) )
	//if ((IsFullQueue ==TRUE && IsPartQueue[0] && IsPartQueue[1])||((quenum > 0) && (IsPartQueue[0] == TRUE || IsPartQueue[1] == TRUE)) )
	if ((quenum > 0) && (IsPartQueue[0] == TRUE || IsPartQueue[1] == TRUE))
	{
		pParams->NormalDetectCfg.cSpeedDesc.percent=(float)topy/pCfgs->team_height;



	} 
	else
	{
		pParams->NormalDetectCfg.cSpeedDesc.percent=0.0;

	}
	//pCfgs->guTestB[LaneID] = quenum;
	//pCfgs->guTestC[LaneID] = (pParams->NormalDetectCfg.cSpeedDesc.percent * 100);

	//没有建立背景前，判断排队更严格
	if(pParams->NormalDetectCfg.cSpeedDesc.getQueback_flag == FALSE && quenum < 6)
	{
		pParams->NormalDetectCfg.cSpeedDesc.percent = 0.0;
	}

	EXPlength=0.4*pParams->NormalDetectCfg.cSpeedDesc.pastlength[3]+0.3*pParams->NormalDetectCfg.cSpeedDesc.pastlength[2]+0.2*pParams->NormalDetectCfg.cSpeedDesc.pastlength[1]+0.1*pParams->NormalDetectCfg.cSpeedDesc.pastlength[0];
	pParams->NormalDetectCfg.cSpeedDesc.pastlength[0]=pParams->NormalDetectCfg.cSpeedDesc.pastlength[1];
	pParams->NormalDetectCfg.cSpeedDesc.pastlength[1]=pParams->NormalDetectCfg.cSpeedDesc.pastlength[2];
	pParams->NormalDetectCfg.cSpeedDesc.pastlength[2]=pParams->NormalDetectCfg.cSpeedDesc.pastlength[3];
	pParams->NormalDetectCfg.cSpeedDesc.pastlength[3]=pParams->NormalDetectCfg.cSpeedDesc.percent;
	if ((EXPlength-pParams->NormalDetectCfg.cSpeedDesc.percent)>0.5)
	{
		pParams->NormalDetectCfg.cSpeedDesc.percent=EXPlength;
	}


	return pParams->NormalDetectCfg.cSpeedDesc.percent;

}

bool IsVehicleQueue(Uint8* puSubImage, Uint8* puSobelImage,Uint16 tail_line,Uint16 head_line, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	int nRow, nCol;
	float subnum = 0.0, sobelnum = 0.0;
	Uint32 offset = 0;
	unsigned char* sobelPtr;
	unsigned char* subPtr;
	for( nRow =tail_line ; nRow < head_line; nRow++ )
	{
		offset = nRow * pCfgs->team_width;
		sobelPtr = puSobelImage + offset;
		subPtr = puSubImage + offset;
		for (nCol =0 ; nCol < pCfgs->team_width; nCol++)
		{
			if (subPtr[nCol])
			{
				subnum++;
			}
			if (sobelPtr[nCol])
			{
				sobelnum++;
			}
		}	
	}

	if(head_line != tail_line)
	{
		subnum = subnum / (pCfgs->team_width * (head_line - tail_line));
		sobelnum = sobelnum / (pCfgs->team_width*(head_line - tail_line));

	}      

	//if ((overlapnum>=0.7)&&((subnum<0.1&&sobelnum>=0.3)||(subnum<=0.01&&sobelnum>=0.2)))
	if ((subnum < 0.15 && sobelnum >= 0.5) || (subnum <= 0.1 && sobelnum >= 0.3) || (subnum < 0.01 && sobelnum > 0.2))
	{
		return TRUE;
	} 
	else
	{
		return FALSE;
	}

}

//连通域
void SobelConectivity(unsigned char* puSobelImage,unsigned char* puSobelConnectImage,ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	int dwArrayCurrentSize, dwArraySize, dwProcessedSize,k,j,dwForegroundCount;
	CRect rtConnect;
	//unsigned char* out;
	int connectnum = 1;
	int row,col,l,t,s;
	int area,width,height;
	int frow,fcol;
	unsigned char* out;
	
	//ArrayInit((Array *)pParams->ForegroundArray, sizeof(CForeground_Array), FOREGROUND_ARRAY_CAPACITY);
	out=puSobelConnectImage; //new unsigned char[pCfgs->team_width*pCfgs->team_height];
	memset(out,0,sizeof(unsigned char) * pCfgs->team_width *pCfgs->team_height);

	for( row = 0; row < pCfgs->team_height; row++)
	{
		for(col = 0; col < pCfgs->team_width; col++)
		{
			if(GetImagePointValue(col,row,pCfgs->team_height,pCfgs->team_width,puSobelImage) 
				&&(!GetImagePointValue(col,row,pCfgs->team_height,pCfgs->team_width,out))
				)
			{

				// Got a start point
				pParams->m_pPointArray[0].x =col;
				rtConnect.left = col;
				rtConnect.right = col;

				pParams->m_pPointArray[0].y = row;
				rtConnect.top = row;
				rtConnect.bottom = row;

				dwArrayCurrentSize = 1;
				dwProcessedSize = 0;

				do
				{
					dwArraySize = dwArrayCurrentSize;
					for(l = dwProcessedSize; l < dwArraySize; ++l)
					{
						rtConnect.left = (rtConnect.left < pParams->m_pPointArray[l].x) ? rtConnect.left : pParams->m_pPointArray[l].x;
						rtConnect.right = (rtConnect.right > pParams->m_pPointArray[l].x) ? rtConnect.right : pParams->m_pPointArray[l].x;
						rtConnect.top = (rtConnect.top < pParams->m_pPointArray[l].y) ? rtConnect.top : pParams->m_pPointArray[l].y;
						rtConnect.bottom = (rtConnect.bottom > pParams->m_pPointArray[l].y) ? rtConnect.bottom : pParams->m_pPointArray[l].y;					
						for( t = -1; t <= 1; ++t)
						{
							for( s =-1; s <= 1; ++s)
							{
								k = pParams->m_pPointArray[l].y + t;
								j = pParams->m_pPointArray[l].x + s;
								if((k >= 0)
									&&(k < pCfgs->team_height)
									&&(j >= 0)
									&&(j < pCfgs->team_width)
									)
								{
									if(GetImagePointValue(j,k,pCfgs->team_height,pCfgs->team_width,puSobelImage) && 
										!GetImagePointValue(j,k,pCfgs->team_height,pCfgs->team_width,out))
									{
										SetImagePointValue(j,k,pCfgs->team_height,pCfgs->team_width,connectnum,out);
										pParams->m_pPointArray[dwArrayCurrentSize].x = j;
										pParams->m_pPointArray[dwArrayCurrentSize].y = k;
										dwArrayCurrentSize++;
									}
								}
							}
						}
					}
					dwProcessedSize = dwArraySize;
				}while(dwArrayCurrentSize > dwArraySize);
				width = RECT_Width(rtConnect) + 1;
				height = RECT_Height(rtConnect) + 1;
				area = width * height;
				if((area >= 100)&&( width > 10)&&(height > 10))
				{	
					;
					/*
					ArrayAdd((Array *)pParams->ForegroundArray,(void*)&rtConnect);*/				

				}
				else
				{
					for (frow = rtConnect.top; frow <= rtConnect.bottom; frow++)
					{
						for (fcol = rtConnect.left; fcol <= rtConnect.right; fcol++)
						{
							if (GetImagePointValue(fcol,frow,pCfgs->team_height,pCfgs->team_width,out) == connectnum)
							{
								SetImagePointValue(fcol,frow,pCfgs->team_height,pCfgs->team_width,0,puSobelImage);
							}							
						}
					}
				}
				
				connectnum = (connectnum >= 255)?  1 : connectnum + 1; 
			}
		}
	}

	//dwForegroundCount = ArrayGetSize((Array *)pParams->ForegroundArray);


}
Uint16 overlapRatio(const CvRect r1,const CvRect r2)
{
	Uint16 ratio=0;
	Uint16 area_r1,area_r2,area_intersection;
	Uint16 x1   =   MAX(r1.x, r2.x);
	Uint16 y1   =   MAX(r1.y, r2.y);
	Uint16 x2   =   MIN(r1.x+r1.width, r2.x+r2.width);
	Uint16 y2   =   MIN(r1.y+r1.height, r2.y+r2.height);
	if(x1<x2 && y1<y2) //intersect
	{
		area_r1=r1.width*r1.height;
		area_r2=r2.width*r2.height;
		area_intersection=(x2-x1)*(y2-y1);
		ratio = area_intersection*100/(area_r1+area_r2-area_intersection);
	}
	return ratio;
}
Uint16 overlapRatio1(const CvRect r1,const CvRect r2)
{
	Uint16 ratio=0;
	Uint16 area_r1,area_r2,area_intersection;
	Uint16 x1   =   MAX(r1.x, r2.x);
	Uint16 y1   =   MAX(r1.y, r2.y);
	Uint16 x2   =   MIN(r1.x+r1.width, r2.x+r2.width);
	Uint16 y2   =   MIN(r1.y+r1.height, r2.y+r2.height);
	if(x1<x2 && y1<y2) //intersect
	{
		area_r1=r1.width*r1.height;
		area_r2=r2.width*r2.height;
		area_intersection=(x2-x1)*(y2-y1);
		ratio = area_intersection*100/MAX(1,MIN(area_r1,area_r2));
	}
	return ratio;
}
void merge_overlapped_rect(CvRect* candidate_roi,Uint16* candidate_num,Uint16 ratio_threshold)
{
	CvRect rect_temp[100];
	Uint16 temp_num=0;
	Uint16 rect_num = *candidate_num;
	Uint16 i=0,j=0;
	Uint16 x11=0,y11=0,x22=0,y22=0;
	for(i=0; i<rect_num; ++i)
	{
		temp_num=0;
		for(j=1; j<*candidate_num; ++j)
		{
			if(overlapRatio1(candidate_roi[0],candidate_roi[j])>ratio_threshold)
			{
				x11   =   MIN(candidate_roi[0].x, candidate_roi[j].x);
				y11   =   MIN(candidate_roi[0].y, candidate_roi[j].y);
				x22   =   MAX(candidate_roi[0].x+candidate_roi[0].width, candidate_roi[j].x+candidate_roi[j].width);
				y22   =   MAX(candidate_roi[0].y+candidate_roi[0].height, candidate_roi[j].y+candidate_roi[j].height);
				candidate_roi[0].x=x11;
				candidate_roi[0].y=y11;
				candidate_roi[0].width=x22-x11;
				candidate_roi[0].height=y22-y11;
				rect_num--;
			}	
			else
			{
				rect_temp[temp_num]=candidate_roi[j];
				temp_num++;
			}
		}
		rect_temp[temp_num]=candidate_roi[0];
		temp_num++;
		for(j=0;j<temp_num;++j)
		{
			candidate_roi[j] = rect_temp[j];
		}
		*candidate_num=temp_num;
		if(*candidate_num == 1)
			break;
	}
}
int merge_overlapped_target(CTarget* targets,Uint16* targets_size,Uint16 ratio_threshold)
{
	CTarget target_temp[100];
	Uint16 temp_num=0;
	Uint16 object_num = *targets_size;
	Uint16 i=0,j=0;
	CvRect rect0,rect1;
	Uint16 idx=101;
	bool flow_count0=FALSE,flow_count1=FALSE;
	Uint16 num_detected0=0,num_detected1=0;
	if(*targets_size<=1)
	{
		return 0;
	}
	for(i=0; i<object_num; ++i)
	{
		temp_num=0;
		rect0=targets[0].vehicle;
		flow_count0=targets[0].flow_count;
		for(j=1; j<*targets_size; ++j)
		{
			idx=101;
			rect1=targets[j].vehicle;
			flow_count1=targets[j].flow_count;
			if(overlapRatio1(rect0,rect1)>ratio_threshold&&((flow_count0&flow_count1)==FALSE))
			{
				if(flow_count0)
				{
					idx=0;
				}
				else if(flow_count1)
				{
					idx=j;
				}
				else
				{
					num_detected0=targets[0].num_detected;
					num_detected1=targets[j].num_detected;
					idx=(num_detected0>num_detected1)? 0:j;
				}
				targets[0]=targets[idx];
				object_num--;
			}	
			else
			{
				target_temp[temp_num]=targets[j];
				temp_num++;
			}
		}
		target_temp[temp_num]=targets[0];
		temp_num++;
		for(j=0;j<temp_num;++j)
		{
			targets[j] = target_temp[j];
		}
		*targets_size=temp_num;
		if(*targets_size == 1)
			break;
	}
	return 1;
}
Uint16 find_nearest_rect(CvRect vehicle_roi,CTarget* targets,Uint16 targets_size,Uint16* overlap_ratio1,Uint16* overlap_ratio2)
{
	Uint16 i=0,idx=0;
	Uint16 ratio=0;
	CTarget* it_max=&targets[0];
	*overlap_ratio1=0;
	*overlap_ratio2=0;
	if(targets_size<1)
	{
		return 101;
	}
	for(i=0;i<targets_size;++i)
	{
		ratio=MAX(overlapRatio(vehicle_roi,targets[i].vehicle_next),overlapRatio(vehicle_roi,targets[i].vehicle));
		if(ratio>*overlap_ratio1/*&&(vehicle_roi.y>=targets[i].vehicle_detected.y)*/)
		{
			*overlap_ratio1=ratio;
			it_max=&targets[i];
			idx=i;
		}
	}
	*overlap_ratio2=MAX(overlapRatio1(vehicle_roi,it_max->vehicle_next),overlapRatio1(vehicle_roi,it_max->vehicle));
	if(*overlap_ratio1>15)
	{
		return idx;
	}
	return 101;
}
void get_candidate(unsigned char* currFrame,unsigned char* prevFrame,unsigned char* tempFrame,Uint16 rows,Uint16 cols,ALGCFGS *pCfgs,ALGPARAMS *pParams)
{
	int scale=2;
    int loopDirect[8][2] = {{-2, 2}, {0, 2}, {2, 2}, {2, 0},{2, -2}, {0, -2}, {-2, -2}, {-2, 0}};
	int i=0,j=0,l=0,t=0/*,s=0*/;
	int row,col;
	unsigned char* curr_data=currFrame;
	unsigned char* prev_data=prevFrame;
	unsigned char* temp_data=tempFrame;
	unsigned char* fg_data1=prevFrame;
	unsigned char* temp_data1=tempFrame;
	int dwArrayCurrentSize=0,dwArraySize,dwProcessedSize;
	CRect rtConnect;
	int width,height,width_extend,height_extend;
	pCfgs->candidateROI_num=0;
	if(rows&&cols)
	{
	    int offset2 = scale*cols;
		memset(tempFrame,0,rows*cols*sizeof(unsigned char));
		for(j=0;j<rows;j+=scale)
		{
            int offset1 = j * cols;
			curr_data=currFrame + offset1;
			prev_data=prevFrame + offset1;
			temp_data=tempFrame + offset1;
			for(i=0;i<cols;i+=scale)
			{
				if((curr_data[i]-prev_data[i]>10)||(prev_data[i]-curr_data[i]>10))
				{
					temp_data[i-offset2]=255;
					temp_data[i+offset2]=255;
					temp_data[i-2*offset2]=255;
					temp_data[i+2*offset2]=255;
					temp_data[i]=255;
				}
			}
		}
		memcpy(prevFrame,tempFrame,cols*rows*sizeof(unsigned char));
		memset(tempFrame,0,rows*cols*sizeof(unsigned char));
		memset(pParams->m_pPointArray,0,rows*cols*sizeof(CPoint));
		for(j=0; j<rows; j+=3*scale)
		{
			unsigned char* fg_data=prevFrame+j*cols;
			unsigned char* temp_data=tempFrame+j*cols;
			for(i=0; i<cols; i+=3*scale)
			{
				if(fg_data[i]==255 && temp_data[i]==0)
				{
					pParams->m_pPointArray[0].x = rtConnect.left = rtConnect.right = i;
					pParams->m_pPointArray[0].y = rtConnect.top = rtConnect.bottom = j;
					dwArrayCurrentSize = 1;
					dwProcessedSize = 0;
					do
					{
						dwArraySize = dwArrayCurrentSize;
						for(l=dwProcessedSize; l<dwArraySize; ++l)
						{
							rtConnect.left=(rtConnect.left < pParams->m_pPointArray[l].x)? rtConnect.left:pParams->m_pPointArray[l].x;
							rtConnect.right=(rtConnect.right > pParams->m_pPointArray[l].x)? rtConnect.right:pParams->m_pPointArray[l].x;
							rtConnect.top=(rtConnect.top < pParams->m_pPointArray[l].y)? rtConnect.top:pParams->m_pPointArray[l].y;
							rtConnect.bottom=(rtConnect.bottom > pParams->m_pPointArray[l].y)? rtConnect.bottom:pParams->m_pPointArray[l].y;
							for (t = 0; t < 8; ++t)
							{
								row = pParams->m_pPointArray[l].y + loopDirect[t][1];
								col = pParams->m_pPointArray[l].x + loopDirect[t][0];
								if(row >= 0 && row < rows && 
									col >= 0 && col < cols)
								{
									fg_data1=prevFrame+row*cols;
									temp_data1=tempFrame+row*cols;
									if(fg_data1[col]==255 && temp_data1[col]==0)
									{
										temp_data1[col]= 255;
										pParams->m_pPointArray[dwArrayCurrentSize].x = col;
										pParams->m_pPointArray[dwArrayCurrentSize].y = row;
										dwArrayCurrentSize++;
									}
								}
							}
						}
						dwProcessedSize = dwArraySize;
					}while(dwArrayCurrentSize > dwArraySize);
					width=rtConnect.right-rtConnect.left+1;
					height=rtConnect.bottom-rtConnect.top+1;
					if(width*height >100&&width>30&&height>40)
					{
                        width_extend = (width * 205) >> 10; // 0.2 * width
                        height_extend = (height * 103) >> 10; // 0.2 * height
						rtConnect.left=rtConnect.left-width_extend;
						rtConnect.top=rtConnect.top-5;
						rtConnect.right=rtConnect.right+width_extend;
						rtConnect.bottom=rtConnect.bottom+height_extend;
						rtConnect.left=(rtConnect.left>0)? rtConnect.left:0;
						rtConnect.top=(rtConnect.top>0)? rtConnect.top:0;
						rtConnect.right=(rtConnect.right>(cols-1))? (cols-1):rtConnect.right;
						rtConnect.bottom=(rtConnect.bottom>(rows-1))? (rows-1):rtConnect.bottom; 
						pCfgs->candidateROI[pCfgs->candidateROI_num].x=rtConnect.left;
						pCfgs->candidateROI[pCfgs->candidateROI_num].y=rtConnect.top;
						pCfgs->candidateROI[pCfgs->candidateROI_num].width=rtConnect.right-rtConnect.left+1;
						pCfgs->candidateROI[pCfgs->candidateROI_num].height=rtConnect.bottom-rtConnect.top+1;
						pCfgs->candidateROI_num++;
					}
				}
			}
		}
		merge_overlapped_rect(pCfgs->candidateROI,&(pCfgs->candidateROI_num),50);
	}
}
inline bool IsRectIncluded(const CvRect* smallRect, const CvRect* bigRect)
{
    if ((smallRect->x >= bigRect->x) && (smallRect->y >= bigRect->y) && 
        (smallRect->x + smallRect->width <= bigRect->x + bigRect->width) &&
        (smallRect->y + smallRect->height <= bigRect->y + bigRect->height))
    {
        return TRUE;
    }
    return FALSE;
}
void SetEnhancedMask(const CvRect* candidateROI, const ALGCFGS* pCfgs, Uint8* maskData)
{
    CvRect vehicle;
	int startY,endY,halfWidth,startX;
	int y;
	Uint8* ptr;
	int i;
    for (i = 0; i < pCfgs->targets_size; ++i)
    {
        vehicle = pCfgs->targets[i].vehicle;
        vehicle.x -= pCfgs->detectROI.x;
        vehicle.y -= pCfgs->detectROI.y;
        if (pCfgs->targets[i].detected && IsRectIncluded(&vehicle, candidateROI))
        {
            vehicle.x -= candidateROI->x;
            vehicle.y -= candidateROI->y;
            startY = vehicle.y + (vehicle.height >> 1);
            endY = vehicle.y + vehicle.height;
            halfWidth = vehicle.width >> 1;
            startX = vehicle.x + halfWidth;
            for (y = startY; y < endY; ++y)
            {
                ptr = maskData + y * candidateROI->width;
                memset(ptr + startX, 0, halfWidth);
            }
            break;
        }
    }
}
void vehicle_detection(unsigned char* currFrame,unsigned char* prevFrame,unsigned char* maskImage,unsigned char* tempFrame,Uint16 width,Uint16 height,ALGCFGS *pCfgs,ALGPARAMS *pParams,Uint16 flag)
{
	Uint16 i=0,j=0/*,k=0*/;
	Uint16 detect_num=0;
	int vehicle_num[1]={0};
	if(flag==0)
	{
		pCfgs->bClassifierInit=alg_opencv_processPC(currFrame,maskImage,width, height,pCfgs->vehicleROI,vehicle_num,pCfgs->bClassifierInit,pCfgs->CameraCfg.uEnvironmentStatus);
		pCfgs->vehicleROI_num=*vehicle_num;
	}
	else
	{
		detect_num=0;
		get_candidate(currFrame,prevFrame,tempFrame,height,width,pCfgs,pParams);
        for (i = 0; i < pCfgs->candidateROI_num; ++i)
		{
            CvRect roi = pCfgs->candidateROI[i];
            roi.width = ((roi.width + 3) >> 2 ) << 2;
			for(j=0;j<pCfgs->candidateROI[i].height;++j)
			{
                int offset1 = j * roi.width;
                int offset2 = (j + roi.y) * width + roi.x;
                memcpy(tempFrame + offset1, currFrame + offset2, roi.width);
                memcpy(pParams->puSobelTempImageY + offset1, maskImage + offset2, roi.width);
			}
			detect_num=pCfgs->vehicleROI_num;
			*vehicle_num=pCfgs->vehicleROI_num;
            pCfgs->bClassifierInit=alg_opencv_processPC(tempFrame, pParams->puSobelTempImageY, roi.width, roi.height, pCfgs->vehicleROI, vehicle_num,pCfgs->bClassifierInit,pCfgs->CameraCfg.uEnvironmentStatus);
			pCfgs->vehicleROI_num=*vehicle_num;
			if(pCfgs->vehicleROI_num>detect_num)
			{
				for(j=detect_num;j<pCfgs->vehicleROI_num;j++)
				{
                    pCfgs->vehicleROI[j].x += roi.x;
                    pCfgs->vehicleROI[j].y += roi.y;
				}
			}
		}
	}
	merge_overlapped_rect(pCfgs->vehicleROI,&(pCfgs->vehicleROI_num),50);
	for(i=0;i<pCfgs->vehicleROI_num;++i)
	{
		pCfgs->vehicleROI[i].x += pCfgs->detectROI.x;
		pCfgs->vehicleROI[i].y += pCfgs->detectROI.y;
	}
}
Int16 get_speed(Int16* arr,int num)
{
	Uint16 i,j;
    Int16 array_temp[5]={0};
    Int16 temp=0;
	if(num==0)
    {
		return 0;
    }
	for(i=0;i<5;++i)
	{
        array_temp[i] = arr[i];
	}
	for(i=0;i<=num/2;i++)
    {
		for(j=i+1;j<num;j++)
		{
			if(array_temp[i]>array_temp[j])
			{
				temp=array_temp[i];
				array_temp[i]=array_temp[j];
				array_temp[j]=temp;
			}
		}
    }
		return array_temp[num/2];
}
void get_target(unsigned char* currFrame,unsigned char* prevFrame,unsigned char* maskImage,unsigned char* tempFrame,Uint16 width,Uint16 height,ALGCFGS *pCfgs,ALGPARAMS *pParams,Uint16 flag)
{
	Uint16 i=0,j=0,k=0;
	Uint16 overlap_ratio1[1]={0};
	Uint16 overlap_ratio2[1]={0};
	Uint16 idx1=101,idx2=101;
	Int16 vx=0,vy=0;
	CTarget* t=&pCfgs->targets[0];
	unsigned char* p1=currFrame;
	unsigned char* p2=prevFrame;
	int num=0;
	if (pParams->NormalDetectCfg.cSpeedDesc.SpeedDetectorWorkMode.bNight) // 夜间
	{
		//pCfgs->guTestA[1]=1;
		// 计算两帧差，膨胀后得到待检测的区域，生成待检测的子图像，在子图像上进行车辆检测，计算车辆位置(prevFrame被用来保存帧差后的膨胀图像)
		DetectVehicleInNight(currFrame, maskImage, width, height, prevFrame, tempFrame, pCfgs, pParams);

		// 标记targets
		SetTrackLabel(prevFrame, width, pCfgs->detectROI.x, pCfgs->detectROI.y, pCfgs->targets_size, pCfgs->targets);

		// 标记candidate_targets
		SetTrackLabel(prevFrame, width, pCfgs->detectROI.x, pCfgs->detectROI.y, pCfgs->candidate_targets_size, pCfgs->candidate_targets);
	}
	else // 白天
	{
		for(i=0;i<pCfgs->targets_size;i++)
		{
			Uint16 startY = pCfgs->targets[i].vehicle.y - pCfgs->detectROI.y;
			Uint16 endY = startY + pCfgs->targets[i].vehicle.height;
			Uint16 startX = pCfgs->targets[i].vehicle.x - pCfgs->detectROI.x;
			Uint16 endX = startX + pCfgs->targets[i].vehicle.width;
			pCfgs->targets[i].tracked=FALSE;
			pCfgs->targets[i].overlap_ratio_detected=0;
			num=0;
			for (j = startY;j < endY; ++j)
			{
				int offset = j * width;
				p1 = currFrame + offset;
				p2 = prevFrame + offset;
				for (k = startX; k < endX; ++k)
				{
					if((p1[k]-p2[k]>10)||(p2[k]-p1[k])>10)
					{
						num++;
					}
				}
			}
			if (num < ((pCfgs->targets[i].vehicle.height * pCfgs->targets[i].vehicle.width * 51) >> 10))
			{
				pCfgs->targets[i].vehicle_next=pCfgs->targets[i].vehicle;
				pCfgs->targets[i].tracked=TRUE;
			}
		}
		for(i=0;i<pCfgs->candidate_targets_size;++i)
		{
			Uint16 startY = pCfgs->candidate_targets[i].vehicle.y - pCfgs->detectROI.y;
			Uint16 endY = startY + pCfgs->candidate_targets[i].vehicle.height;
			Uint16 startX = pCfgs->candidate_targets[i].vehicle.x - pCfgs->detectROI.x;
			Uint16 endX = startX + pCfgs->candidate_targets[i].vehicle.width;
			pCfgs->candidate_targets[i].tracked=FALSE;
			pCfgs->candidate_targets[i].overlap_ratio_detected=0;
			num=0;
			for (j = startY;j < endY; ++j)
			{
				int offset = j * width;
				p1 = currFrame + offset;
				p2 = prevFrame + offset;
				for (k = startX; k < endX; ++k)
				{
					if((p1[k]-p2[k]>10)||(p2[k]-p1[k])>10)
					{
						num++;
					}
				}
			}
			if(num < ((pCfgs->candidate_targets[i].vehicle.height * pCfgs->candidate_targets[i].vehicle.width * 51) >> 10))
			{
				pCfgs->candidate_targets[i].vehicle_next=pCfgs->candidate_targets[i].vehicle;
				pCfgs->candidate_targets[i].tracked=TRUE;
			}
		}
		vehicle_detection(currFrame,prevFrame,maskImage,tempFrame,width,height,pCfgs,pParams,1);
	}
	for(i=0;i<pCfgs->targets_size;i++)
	{
		pCfgs->targets[i].detected=FALSE;
	}
	for(i=0;i<pCfgs->candidate_targets_size;i++)
	{
		pCfgs->candidate_targets[i].detected=FALSE;
	}
	width+=pCfgs->detectROI.x;
	height+=pCfgs->detectROI.y;
	for(i=0;i<pCfgs->vehicleROI_num;++i)
	{
		idx1=101;
		idx2=101;
		if((pCfgs->vehicleROI[i].y+pCfgs->vehicleROI[i].height)>height)
			continue;
		idx1=find_nearest_rect(pCfgs->vehicleROI[i],pCfgs->targets,pCfgs->targets_size,overlap_ratio1,overlap_ratio2);
		if(idx1>100)
		{
			idx2=find_nearest_rect(pCfgs->vehicleROI[i],pCfgs->candidate_targets,pCfgs->candidate_targets_size,overlap_ratio1,overlap_ratio2);
			if(idx2<=100)
			{
				t=&(pCfgs->candidate_targets[idx2]);
			}
		}
		else
		{
			t=&(pCfgs->targets[idx1]);
		}
		if((idx1>100)&&(idx2>100))
		{
			if((pCfgs->vehicleROI[i].y+pCfgs->vehicleROI[i].height)>(pCfgs->detectPosition+10))
				continue;
			pCfgs->candidate_targets[pCfgs->candidate_targets_size].vehicle =pCfgs->vehicleROI[i];
			pCfgs->candidate_targets[pCfgs->candidate_targets_size].vehicle_next =pCfgs->vehicleROI[i];
			pCfgs->candidate_targets[pCfgs->candidate_targets_size].detected=TRUE;
			pCfgs->candidate_targets[pCfgs->candidate_targets_size].tracked=FALSE;
			pCfgs->candidate_targets[pCfgs->candidate_targets_size].flow_count=FALSE;
			pCfgs->candidate_targets[pCfgs->candidate_targets_size].vehicle_id=0;
			pCfgs->candidate_targets[pCfgs->candidate_targets_size].num_detected=1;
			pCfgs->candidate_targets[pCfgs->candidate_targets_size].speed_num=0;
			pCfgs->candidate_targets[pCfgs->candidate_targets_size].num_lost_detected=0;
			pCfgs->candidate_targets[pCfgs->candidate_targets_size].out_flag=0;
			pCfgs->candidate_targets[pCfgs->candidate_targets_size].exist_num=0;
			pCfgs->candidate_targets[pCfgs->candidate_targets_size].overlap_ratio_detected=0;
			memset(pCfgs->candidate_targets[pCfgs->candidate_targets_size].vx,0,5*sizeof(Int16));
			memset(pCfgs->candidate_targets[pCfgs->candidate_targets_size].vy,0,5*sizeof(Int16));
			pCfgs->candidate_targets[pCfgs->candidate_targets_size].continue_num=0;
			pCfgs->candidate_targets_size++;
		}
		else if(*overlap_ratio1>t->overlap_ratio_detected)
		{
			if(t->tracked==FALSE)
			{
				vx = (pCfgs->vehicleROI[i].x + pCfgs->vehicleROI[i].width / 2 - (t->vehicle.x + t->vehicle.width / 2)) / (t->num_lost_detected + 1);
				vy = (pCfgs->vehicleROI[i].y + pCfgs->vehicleROI[i].height / 2 - (t->vehicle.y + t->vehicle.height / 2)) / (t->num_lost_detected + 1);
				if(vy > -30 && vy < 40)
				{
					if(t->speed_num==5)
					{
						for(j=0;j<4;j++)
						{
							t->vx[j]=t->vx[j+1];
							t->vy[j]=t->vy[j+1];
						}
						t->vx[4] = vx;
						t->vy[4] = vy;
					}
					else
					{
						t->vx[t->speed_num] = vx;
						t->vy[t->speed_num] = vy;
						t->speed_num++;
					}
				}
			}
			t->num_lost_detected=0;
			t->detected=TRUE;
			t->num_detected++;
			t->vehicle_next=pCfgs->vehicleROI[i];
			t->overlap_ratio_detected=*overlap_ratio1;
		}
	}
	for(i=0;i<pCfgs->targets_size;++i)
	{
		if(pCfgs->targets[i].flow_count)
			pCfgs->targets[i].exist_num++;
		vx=get_speed(pCfgs->targets[i].vx,pCfgs->targets[i].speed_num);
		vy=get_speed(pCfgs->targets[i].vy,pCfgs->targets[i].speed_num);
        if (vy <= 0)
        {
            vy = 0;
            vx = 0;
        }
		//vy=(vy<0)? 0:vy;
		if(pCfgs->targets[i].detected==FALSE)
		{
			if (pCfgs->targets[i].tracked==FALSE)
			{
				pCfgs->targets[i].vehicle_next.x=pCfgs->targets[i].vehicle.x+vx;
				pCfgs->targets[i].vehicle_next.y=pCfgs->targets[i].vehicle.y+vy;
			}
			else
			{
				// 未检测到目标且无帧差无运动，将速度置0.
				memset(pCfgs->targets[i].vx, 0, 5 * sizeof(Int16));
				memset(pCfgs->targets[i].vy, 0, 5 * sizeof(Int16));
			}
			pCfgs->targets[i].num_lost_detected++;
		}
		pCfgs->targets[i].vehicle=pCfgs->targets[i].vehicle_next;
		if((pCfgs->targets[i].vehicle.x+pCfgs->targets[i].vehicle.width)>width||(pCfgs->targets[i].vehicle.y+pCfgs->targets[i].vehicle.height)>height||pCfgs->targets[i].vehicle.x<pCfgs->detectROI.x ||pCfgs->targets[i].vehicle.y<pCfgs->detectROI.y)
		{
			if(pCfgs->targets[i].out_flag)
			{
				DeleteTarget(&pCfgs->targets_size, &i, pCfgs->targets);
				continue;
			}
			else
				pCfgs->targets[i].out_flag=1;
		}
		if(pCfgs->targets[i].continue_num>5000)
		{
			DeleteTarget(&pCfgs->targets_size, &i, pCfgs->targets);
			continue;
		}
		pCfgs->targets[i].vehicle_next.x = pCfgs->targets[i].vehicle.x + vx;
		pCfgs->targets[i].vehicle_next.y = pCfgs->targets[i].vehicle.y + vy;
		pCfgs->targets[i].vehicle_next.width=pCfgs->targets[i].vehicle.width;
		pCfgs->targets[i].vehicle_next.height=pCfgs->targets[i].vehicle.height;
		pCfgs->targets[i].continue_num++;
	}
	merge_overlapped_target(pCfgs->targets,&(pCfgs->targets_size),90);
	for(i=0;i<pCfgs->candidate_targets_size;++i)
	{
		vx=get_speed(pCfgs->candidate_targets[i].vx,pCfgs->candidate_targets[i].speed_num);
		vy=get_speed(pCfgs->candidate_targets[i].vy,pCfgs->candidate_targets[i].speed_num);
		vy=(vy<0)? 0:vy;
		if(pCfgs->candidate_targets[i].detected==FALSE&&pCfgs->candidate_targets[i].tracked==FALSE)
		{
			pCfgs->candidate_targets[i].vehicle_next.x=pCfgs->candidate_targets[i].vehicle.x+vx;
			pCfgs->candidate_targets[i].vehicle_next.y=pCfgs->candidate_targets[i].vehicle.y+vy;
			pCfgs->candidate_targets[i].num_lost_detected++;
		}
		pCfgs->candidate_targets[i].vehicle=pCfgs->candidate_targets[i].vehicle_next;
		if((pCfgs->candidate_targets[i].vehicle.x+pCfgs->candidate_targets[i].vehicle.width)>=width||(pCfgs->candidate_targets[i].vehicle.y+pCfgs->candidate_targets[i].vehicle.height)>=height||pCfgs->candidate_targets[i].vehicle.x<pCfgs->detectROI.x||pCfgs->candidate_targets[i].vehicle.y<pCfgs->detectROI.y||pCfgs->candidate_targets[i].continue_num>50)
		{
			DeleteTarget(&pCfgs->candidate_targets_size, &i, pCfgs->candidate_targets);
			continue;
		}
		pCfgs->candidate_targets[i].vehicle_next.x = pCfgs->candidate_targets[i].vehicle.x + vx;
		pCfgs->candidate_targets[i].vehicle_next.y = pCfgs->candidate_targets[i].vehicle.y + vy;
		pCfgs->candidate_targets[i].vehicle_next.width=pCfgs->candidate_targets[i].vehicle.width;
		pCfgs->candidate_targets[i].vehicle_next.height=pCfgs->candidate_targets[i].vehicle.height;
		pCfgs->candidate_targets[i].continue_num++;
		if((/*(pCfgs->candidate_targets[i].num_detected>=1&&pCfgs->candidate_targets[i].vehicle.width>=60)||*/pCfgs->candidate_targets[i].num_detected>=2)&&(pCfgs->candidate_targets[i].vehicle.y+pCfgs->candidate_targets[i].vehicle.height)<(pCfgs->detectPosition+10)/*(height-10)*/)
		{
			pCfgs->targets[pCfgs->targets_size]=pCfgs->candidate_targets[i];
			pCfgs->targets[pCfgs->targets_size].vehicle_id = pCfgs->target_id++;
			if(pCfgs->target_id>1000)
				pCfgs->target_id=0;
			pCfgs->targets_size++;
            DeleteTarget(&pCfgs->candidate_targets_size, &i, pCfgs->candidate_targets);
			continue;
		}
		// 候选目标与目标存在重叠，删除候选目标
		for (j = 0; j < pCfgs->targets_size; ++j)
		{
			// 两区域的重叠面积与两区域中最小面积的比值
			int ratio = overlapRatio1(pCfgs->targets[j].vehicle, pCfgs->candidate_targets[i].vehicle);
			if (ratio > 80)
			{

				DeleteTarget(&pCfgs->candidate_targets_size, &i, pCfgs->candidate_targets);
				break;
			}

		}
	}
}
void updateMinStd(int standard_deviation,ALGPARAMS *pParams,bool bCarExist)
{
	if(pParams->NormalDetectCfg.cSpeedDesc.min_standard_deviation==0)
		pParams->NormalDetectCfg.cSpeedDesc.min_standard_deviation=100;
	if(pParams->NormalDetectCfg.cSpeedDesc.lmin_standard_deviation==0)
		pParams->NormalDetectCfg.cSpeedDesc.lmin_standard_deviation=100;
	if (pParams->NormalDetectCfg.cSpeedDesc.begin_update_standard_deviation==FALSE)
	{
		if (standard_deviation<pParams->NormalDetectCfg.cSpeedDesc.min_standard_deviation&&bCarExist==FALSE&&pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged==FALSE&&pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged1==FALSE&&pParams->NormalDetectCfg.cSpeedDesc.num_cal_gmin==0)
		{
			pParams->NormalDetectCfg.cSpeedDesc.min_standard_deviation=standard_deviation;
		}
	}		
	if (standard_deviation<40&&pParams->NormalDetectCfg.cSpeedDesc.begin_update_gmin_deviation==FALSE)
	{
		if(bCarExist==FALSE&&pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged==FALSE&&pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged1==FALSE)
		{
			pParams->NormalDetectCfg.cSpeedDesc.begin_update_gmin_deviation=TRUE;
			pParams->NormalDetectCfg.cSpeedDesc.num_cal_lmin=0;
		}
	}
	if(pParams->NormalDetectCfg.cSpeedDesc.begin_update_gmin_deviation==TRUE)
	{
		if(bCarExist==FALSE&&pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged==FALSE&&pParams->NormalDetectCfg.cSpeedDesc.bSpeedCounterChanged1==FALSE)		
		{
			if (pParams->NormalDetectCfg.cSpeedDesc.lmin_standard_deviation>standard_deviation)
			{
				pParams->NormalDetectCfg.cSpeedDesc.lmin_standard_deviation=standard_deviation;
			}
		}
		pParams->NormalDetectCfg.cSpeedDesc.num_cal_lmin++;
		if(pParams->NormalDetectCfg.cSpeedDesc.num_cal_lmin>100)
		{
			if(pParams->NormalDetectCfg.cSpeedDesc.num_cal_gmin==0)
			{
				pParams->NormalDetectCfg.cSpeedDesc.gmin_standard_deviation=0;
			}
			pParams->NormalDetectCfg.cSpeedDesc.gmin_standard_deviation+=pParams->NormalDetectCfg.cSpeedDesc.lmin_standard_deviation;
			pParams->NormalDetectCfg.cSpeedDesc.num_cal_gmin++;
			if(pParams->NormalDetectCfg.cSpeedDesc.begin_update_standard_deviation==FALSE)
				pParams->NormalDetectCfg.cSpeedDesc.min_standard_deviation=pParams->NormalDetectCfg.cSpeedDesc.gmin_standard_deviation/pParams->NormalDetectCfg.cSpeedDesc.num_cal_gmin;
			pParams->NormalDetectCfg.cSpeedDesc.begin_update_gmin_deviation=FALSE;
			pParams->NormalDetectCfg.cSpeedDesc.lmin_standard_deviation=100;
		}
	}
	if(pParams->NormalDetectCfg.cSpeedDesc.num_cal_gmin>20)
	{
		pParams->NormalDetectCfg.cSpeedDesc.gmin_standard_deviation/=pParams->NormalDetectCfg.cSpeedDesc.num_cal_gmin;
		pParams->NormalDetectCfg.cSpeedDesc.min_standard_deviation=(pParams->NormalDetectCfg.cSpeedDesc.min_standard_deviation+pParams->NormalDetectCfg.cSpeedDesc.gmin_standard_deviation) >> 1;
		pParams->NormalDetectCfg.cSpeedDesc.gmin_standard_deviation=0;
		pParams->NormalDetectCfg.cSpeedDesc.num_cal_gmin=0;
		pParams->NormalDetectCfg.cSpeedDesc.begin_update_standard_deviation=TRUE;
	}
}
inline void DeleteTarget(Uint16* size, Uint16* startIdx, CTarget* target)
{
	Uint16 sz = *size;
	Uint16 idx = *startIdx;
	Uint16 j=0;
	for(j = idx + 1; j < sz; j++)
	{
		target[j-1] = target[j];
	}
	*size = sz - 1;
	*startIdx = idx - 1;
}
int AverageRegion(const CvRect* vehicleRegion, const CvRect* inputImgRegion, int step, const unsigned char* inputImg)
{
	int x = 0, y = 0;
	int startX = vehicleRegion->x - inputImgRegion->x;
	int startY = vehicleRegion->y - inputImgRegion->y;
	int endX = startX + vehicleRegion->width;
	int endY = startY + vehicleRegion->height;
	int avgVal = 0;
	int pixCnt = 0;
	startX = startX > 0 ? startX : 0;
	startY = startY > 0 ? startY : 0;
	endX = endX > inputImgRegion->width ? inputImgRegion->width : endX;
	endY = endY > inputImgRegion->height ? inputImgRegion->height : endY;
	for (y = startY; y < endY; y += step)
	{
		const unsigned char* ptr = inputImg + y * inputImgRegion->width;
		for (x = startX; x < endX; x += step)
		{
			avgVal += ptr[x];
			pixCnt++;
		}
	}
	return avgVal / pixCnt;
}
int AverageImage(int width, int height, int step, const unsigned char* inputImg)
{
	int x = 0, y = 0;
	int avgVal = 0;
	int pixCnt = 0;
	for (y = 0; y < height; y += step)
	{
		const unsigned char* ptr = inputImg + y * width;
		for (x = 0; x < width; x += step)
		{
			avgVal += ptr[x];
			pixCnt++;
		}
	}
	return avgVal / pixCnt;
}

void DilateImage(const unsigned char* input, int* lastY, int width, int height, int xSize, int ySize, unsigned char* output)
{
	const int size = width * height;
	int minX = 0, minY = 0, maxX = 0, maxY = 0;
	int i = 0, j = 0, m = 0, n = 0;
	int diff = 0, offset = 0, lastX = 0, idx;

	unsigned char* curInput = NULL;
	unsigned char* curOutput = NULL;

	memset(output, 0, size);

	xSize = (xSize - 1) / 2;
	ySize = (ySize - 1) / 2;

	// 不修改第一行、第一列、最后一行、最后一列，使其仍然为0.
	minX = xSize + 1;
	maxX = width - 2 - xSize;
	minY = ySize + 1;
	maxY = height - 2 - ySize;

#ifdef NOT_USE_OPTIMIZE_DILATE
	for (i = minY; i < maxY; ++i)
	{
		curInput = (unsigned char*)(input + i * width);
		for (j = minX; j < maxX; j += 2) // 为了提高速度，x步长为2.
		{
			if (curInput[j] == 255)
			{
				for (int m = -ySize; m <= ySize; ++m)
				{
					curOutput = output + (i + m) * width;
					for (int n = -xSize; n <= xSize; ++n)
					{
						curOutput[j + n] = 255;
					}
				}
			}
		}
	}
#else
	diff = width - (maxX - minX + 1);
	memset(lastY, 0, width * sizeof(int));

	for (i = minY, offset = minY * width + minX; i <= maxY; ++i, offset += diff)
	{
		lastX = 0; // 标记前一个位置是否为真
		for (j = minX; j <= maxX; j += 2, offset += 2) // 为了提高速度，x步长为2.
		{
			if (input[offset] == 255)
			{
				if (lastX)
				{
					if (lastY[j])
					{
						// 前一个x为真，且上一行该位置也为真，则只需将当前位置偏移ySize行、xSize列的位置设为255
						idx = offset + ySize * width + xSize;
						output[idx] = 255;
						output[idx - 1] = 255;
					}
					else
					{
						// 前一个x为真，但上一行该位置为假，则需将当前位置偏移xSize列的位置对应行都设为255
						idx = offset - ySize * width + xSize;
						for (m = -ySize; m <= ySize; m++, idx += width)
						{
							output[idx] = 255;
							output[idx - 1] = 255;
						}
						lastY[j] = 1;
					}
				}
				else
				{
					if (lastY[j])
					{
						// 前一个x为假，但上一行该位置为真，则需将当前位置偏移ySize行的位置对应列都设为255
						idx = offset + ySize * width;
						for (m = -xSize; m <= xSize; ++m)
						{
							output[idx + m] = 255;
						}
					}
					else
					{
						// 前一个x为假，且上一行该位置为假，则需将当前位置ySize行、xSize列的邻域位置都设为255
						curOutput = output + (i - ySize) * width + j;
						for (m = -ySize; m <= ySize; ++m, curOutput += width)
						{
							for (n = -xSize; n <= xSize; ++n)
							{
								curOutput[n] = 255;
							}
						}

						lastY[j] = 1;
					}

					lastX = 1;
				}
			}
			else
			{
				lastX = 0;
				lastY[j] = 0;
			}
		}
	}
#endif
}

#define RegionStack(pixels, curIdx, sp, stack) \
	if (pixels[curIdx] == 255) \
{\
	stack[sp++] = curIdx;\
	pixels[curIdx] = 1;\
}

// 区域生长，计算出的连通域都大于10个像素值
static int GrowingRegion(unsigned char* pixels, int width, int offset, int* stack, Box* output)
{
	const int minPixNum = 10; // 连通域最小像素个数
	int sp = 0;
	int pixelNum = 0;
	int curIdx = 0;

	int y = offset / width;
	int x = offset - y * width; // ie: offset % width
	int left = x;
	int top = y;
	int right = x;
	int bottom = y;

	stack[sp++] = offset;
	pixels[offset] = 1;

	while (sp--)
	{
		offset = stack[sp];

#ifndef USE_8_COMPONENT	
		// 4-连通域，为了提高速度，使用4连通域
		// x 4 x
		// 3 0 1
		// x 2 x
		curIdx = offset + 1;
		RegionStack(pixels, curIdx, sp, stack)

			curIdx = offset + width;
		RegionStack(pixels, curIdx, sp, stack)

			curIdx = offset - 1;
		RegionStack(pixels, curIdx, sp, stack)

			curIdx = offset - width;
		RegionStack(pixels, curIdx, sp, stack)
#else		
		// 8-连通域
		// 4 3 2
		// 5 0 1
		// 6 7 8
		curIdx = offset + 1;
		RegionStack(pixels, curIdx, sp, stack)

			curIdx = offset - width + 1;
		RegionStack(pixels, curIdx, sp, stack)

			curIdx = offset - width;
		RegionStack(pixels, curIdx, sp, stack)

			curIdx = offset - width - 1;
		RegionStack(pixels, curIdx, sp, stack)

			curIdx = offset - 1;
		RegionStack(pixels, curIdx, sp, stack)

			curIdx = offset - 1 + width;
		RegionStack(pixels, curIdx, sp, stack)

			curIdx = offset + width;
		RegionStack(pixels, curIdx, sp, stack)

			curIdx = offset + width + 1;
		RegionStack(pixels, curIdx, sp, stack)
#endif		
			pixelNum++;

		y = offset / width;
		x = offset - y * width; // ie: offset % width

		left = (x < left) ? x : left;
		top = (y < top) ? y : top;
		right = (x > right) ? x : right;
		bottom = (y > bottom) ? y : bottom;
	}

	if (pixelNum < minPixNum)
	{
		return -1;
	}

	output->left = left;
	output->top = top;
	output->right = right;
	output->bottom = bottom;
	output->isValid = 1;

	return pixelNum;
}

int FindRegion(const unsigned char* input, unsigned char* inputCopy, int* stack, int width, int height, Box* output)
{
	int regionNum = 0;
	int size = width * height;
	int offset = 0;
	int i = 0;

	//memset(stack, 0, size * sizeof(int));
	memcpy(inputCopy, input, size * sizeof(unsigned char));

	// 注意：第一行、最后一行、第一列、最后一列应该为0；若不为0，则需在此处将其改为0.
#ifdef HAVE_NOT_BEEN_ZERO
	// first row.
	memset(inputCopy, 0, width);
	// last row.
	memset(inputCopy + (height - 1) * width, 0, width);
	// first column and last column.
	for (i = 1; i < height - 1; ++i)
	{
		offset = i * width;
		inputCopy[offset] = 0;
		inputCopy[offset + width - 1] = 0;
	}
#endif

	for (i = 0; i < size; ++i)
	{
		if (inputCopy[i] == 255)
		{
			if (GrowingRegion(inputCopy, width, i, stack, &output[regionNum]) != -1)
			{
				regionNum++;
			}
		}
	}

	return regionNum;
}

void FrameDiff(const unsigned char* currFrame, const unsigned char* prevFrame, int width, int height, 
			   int threshold, unsigned char* absDiffFrame, int* nonZeroNumSum)
{
	const int sz = width * height;
	int i = 0, j = 0;
	int offset = 0;
	int lineCnt = 0; // 每行非零个数
	int diff = 0;
	unsigned char* currPtr = NULL;
	unsigned char* prevPtr = NULL;
	unsigned char* diffPtr  = NULL;
	int* currSumPtr = NULL;
	int* prevSumPtr = NULL;

	memset(absDiffFrame, 0, sz);
	memset(nonZeroNumSum, 0, sz * sizeof(int));

	// 这里忽略图像边框
	for (i = 1; i < height - 1; i++)
	{
		offset = i * width;
		lineCnt = 0;

		currPtr = (unsigned char*)(currFrame + offset);
		prevPtr = (unsigned char*)(prevFrame + offset);
		diffPtr = absDiffFrame + offset;
		currSumPtr = nonZeroNumSum + offset; // 非零个数积分图像
		prevSumPtr = nonZeroNumSum + offset - width; // 非零个数积分图像

		for (j = 1; j < width - 1; j++)
		{
			diff = currPtr[j] - prevPtr[j];
			if (abs(diff) > threshold)
			{
				diffPtr[j] = 255;
				lineCnt++;
			}
			currSumPtr[j] = prevSumPtr[j] + lineCnt;
		}
	}
}

void AbsDiff(const unsigned char* currImg, const unsigned char* prevImg, int width, int height, int threshold, unsigned char* absDiffImg)
{
	const int sz = width * height;
	int i = 0, j = 0;
	int offset = 0;
	int diff = 0;
	unsigned char* currPtr = (unsigned char*)(currImg) + width;
	unsigned char* prevPtr = (unsigned char*)(prevImg) + width;
	unsigned char* diffPtr = absDiffImg + width;

	memset(absDiffImg, 0, sz);

	// 这里忽略图像边框
	for (i = 1; i < height - 1; i++, currPtr += width, prevPtr += width, diffPtr += width)
	{
		for (j = 1; j < width - 1; j++)
		{
			diff = currPtr[j] - prevPtr[j];
			if (abs(diff) > threshold)
			{
				diffPtr[j] = 255;
			}
		}
	}
}

int CountNonZero(const unsigned char* currFrame, int startX, int startY, int endX, int endY, int width, int threshold)
{
	int nonZeroNum = 0;
	int i = 0, j = 0;
	unsigned char* ptr = (unsigned char*)(currFrame) + startY * width;

	for (i = startY; i <= endY; ++i, ptr += width)
	{
		for (j = startX; j <= endX; ++j)
		{
			if (ptr[j] > threshold)
			{
				nonZeroNum++;
			}
		}
	}
	return nonZeroNum;
}

int GetCandidateRoiInNight(int width, int regionCnt, const unsigned char* currFrame, Box* regionBnd, CvRect* candidateRoi)
{
	Uint16 candidateNum = 0;
	int i = 0;
	int nonZeroNum = 0;

	// 去除长、宽不满足条件的区域
	for (i = 0; i < regionCnt; ++i)
	{
		if ((regionBnd[i].right - regionBnd[i].left) < 30 || (regionBnd[i].bottom - regionBnd[i].top) < 40)
		{
			regionBnd[i].isValid = 0;
		}
	}

	// 去除亮度不满足条件的区域(因车灯为亮区域，故亮像素至少要大于阈值)
	for (i = 0; i < regionCnt; ++i)
	{
		if (regionBnd[i].isValid)
		{
			nonZeroNum = CountNonZero(currFrame, regionBnd[i].left, regionBnd[i].top, regionBnd[i].right, regionBnd[i].bottom, width, 150);
			if (nonZeroNum < 60)
			{
				regionBnd[i].isValid = 0;
			}
		}
	}

	// 将初步筛选的区域放入candidateRoi
	for (i = 0; i < regionCnt; ++i)
	{
		if (regionBnd[i].isValid)
		{
			candidateRoi[candidateNum].x = regionBnd[i].left;
			candidateRoi[candidateNum].y = regionBnd[i].top;
			candidateRoi[candidateNum].width = regionBnd[i].right - regionBnd[i].left + 1;
			candidateRoi[candidateNum].height = regionBnd[i].bottom - regionBnd[i].top + 1;
			candidateNum++;
		}
	}

	// 合并重叠的候选区域
	merge_overlapped_rect(candidateRoi, &candidateNum, 50);

	return candidateNum;
}

void DetectVehicleInNight(const unsigned char* currImg, const unsigned char* maskImg, int width, int height, 
						  unsigned char* prevImg, unsigned char* swapBuf, ALGCFGS *pCfgs, ALGPARAMS *pParams)
{
	Box regionBnd[128];
	int regionNum = 0;
	int detectNum = 0;
	int vehicleNum = 0;
	int i = 0, j = 0;
	int offset1 = 0, offset2 = 0;
	CvRect roi;

	unsigned char* roiSrcImg = NULL;
	unsigned char* roiMaskImg = NULL;
	unsigned char* diffImg = swapBuf;
	unsigned char* dilateImg = prevImg;
	// 两帧绝对差，并二值化
	AbsDiff(currImg, prevImg, width, height, pCfgs->CameraCfg.uVehicleHeadMinRow, diffImg);

	// 膨胀帧差得到二值图像(diffBinImg)，返回膨胀二值图像(dilateBinImg)
	DilateImage(diffImg, g_intStack, width, height, 17, 23, dilateImg); // dsp中丢帧，故x/y核大小可相对小些（19x23）

	// 计算膨胀二值图像(dilateBinImg)的各连通域的外接矩形，并返回连通域个数
	regionNum = FindRegion(dilateImg, pParams->puSobelTempImageY, g_intStack, width, height, regionBnd);

	// 传入区域分割得到的区域，舍去不满足长宽条件、不满足亮度条件、最后合并重叠的区域
	pCfgs->candidateROI_num = GetCandidateRoiInNight(width, regionNum, currImg, regionBnd, pCfgs->candidateROI);

	memset(pParams->puSobelTempImageX, 0, width * height * sizeof(unsigned char));
	memset(pParams->puSobelTempImageY, 0, width * height * sizeof(unsigned char));
	roiSrcImg = pParams->puSobelTempImageX;
	roiMaskImg = pParams->puSobelTempImageY;

	// 在候选区域中检测车辆
	for (i = 0; i < pCfgs->candidateROI_num; i++)
	{
		roi = pCfgs->candidateROI[i];
		roi.width = ((roi.width + 3) >> 2 ) << 2; // 子图像宽度为4的倍数
		// 拷贝子图像
		for(j = 0; j < pCfgs->candidateROI[i].height; j++)
		{
			offset1 = j * roi.width;
			offset2 = (j + roi.y) * width + roi.x;
			memcpy(roiSrcImg + offset1, currImg + offset2, roi.width);
			memcpy(roiMaskImg + offset1, maskImg + offset2, roi.width);
		}

		// 检测车辆
		pCfgs->bClassifierInit=alg_opencv_processPC(roiSrcImg, roiMaskImg, roi.width, roi.height, pCfgs->vehicleROI, &vehicleNum, pCfgs->bClassifierInit, pCfgs->CameraCfg.uEnvironmentStatus);
		pCfgs->vehicleROI_num = vehicleNum;

		if(pCfgs->vehicleROI_num > detectNum)
		{
			for(j = detectNum; j < pCfgs->vehicleROI_num; j++)
			{
				pCfgs->vehicleROI[j].x += roi.x;
				pCfgs->vehicleROI[j].y += roi.y;
			}
			detectNum = pCfgs->vehicleROI_num;
		}
	}

	// 合并重叠的车辆区域
	merge_overlapped_rect(pCfgs->vehicleROI, &(pCfgs->vehicleROI_num), 50);

	// 加上最外层的偏移位置，detectROI为用户在界面设置的检测范围除以2后得到的区域
	for(i = 0; i < pCfgs->vehicleROI_num; i++)
	{
		pCfgs->vehicleROI[i].x += pCfgs->detectROI.x;
		pCfgs->vehicleROI[i].y += pCfgs->detectROI.y;
	}
}

void SetTrackLabel(const unsigned char* srcImg, int width, int offsetX, int offsetY, int targetSize, CTarget* targets)
{
	int i = 0;
	int startX = 0, endX = 0, startY = 0, endY = 0;
	int nonZeroNum = 0;

	for (i = 0; i < targetSize; i++)
	{
		targets[i].tracked = FALSE;
		targets[i].overlap_ratio_detected = 0;

		startX = targets[i].vehicle.x - offsetX;
		endX = startX + targets[i].vehicle.width;
		startY = targets[i].vehicle.y - offsetY;
		endY = startY + targets[i].vehicle.height;

		nonZeroNum = CountNonZero(srcImg, startX, startY, endX, endY, width, 0);

		if (nonZeroNum < ((targets[i].vehicle.height * targets[i].vehicle.width * 100) >> 10))
		{
			targets[i].vehicle_next = targets[i].vehicle;
			targets[i].tracked = TRUE;
		}
	}
}
//能见度计算
int fuzzy(unsigned char* puNewImage,int nWidth,int nHeight)//
{
	int degree=0;
	int i,j;
	unsigned char x1,x2,x3;
	int temp=0;
	int count=0;
	int width= FULL_COLS << 1;//720*2
	unsigned char* p=puNewImage;
	for(i=0;i<nHeight-1;i+=8)  
	{  
		p = puNewImage + i * width + 1;
		for(j = 100;j<(width-100);j+=16)  
		{  
			x1=*(p+j);
			x2=*(p+width+j);
			x3=*(p+j+2);
			degree=(x2-x1)*(x2-x1)+(x3-x1)*(x3-x1);
			temp+=sqrt((float)degree);
			temp+=abs(x2-x1)+abs(x3-x1);
			count++;
		}  
	}  
	degree=temp/count;
	return degree;
}
bool visible_judge(Uint16 *a,int visib_length,int threshold)
{
	int i=0,num=0;
	for (i=0;i<visib_length;++i)
	{
		if (a[i]<threshold)
		{
			num++;
		}
		else
		{
			break;
		}

	}
	if (num>(visib_length/2))
	{
		return TRUE;
	} 
	else
	{
		return FALSE;
	}

}
//计算视频图像异常
bool Color_deviate(unsigned char* puNewImage,int width,int height,ALGCFGS *pCfgs)
{	
	float ave_a=0,ave_b=0,std_a=0,std_b=0;
	int i,j;
	float color_deviate=0;
	int count=0;
	int temp_a,temp_b;
	unsigned char* p=puNewImage;
	width=FULL_COLS << 1;//720*2
	for(i=10;i<height-10;i+=8)  
	{  
		p = puNewImage + i * width;
		for(j=100;j<(width-100);j+=16)  
		{  
			ave_a += *(p + j);
			ave_b += *(p + j + 2);
			count++;
		}
	}
	ave_a /= count;
	ave_b /= count;

	for(i=10;i<height-10;i+=8)  
	{  
		p = puNewImage + i * width;
		for(j=100;j<(width-100);j+=16)  
		{  
			temp_a = *(p + j);
			temp_b = *(p + j + 2);
			//temp_a = (temp_a >= 0)? temp_a : (-1*temp_a);
			//temp_b = (temp_b >= 0)? temp_b : (-1*temp_b);
			//std_a += temp_a;
			//std_b += temp_b;
			std_a += (temp_a - ave_a)*(temp_a - ave_a);
			std_b += (temp_b - ave_b)*(temp_b - ave_b);
		}
	}
	std_a /= count;
	std_b /= count;

	ave_a -= 128;
	ave_b -= 128;
	color_deviate=sqrt(ave_a*ave_a+ave_b*ave_b)/sqrt(std_a+std_b);
	//color_deviate=sqrt(ave_a*ave_a+ave_b*ave_b)/sqrt(std_a*std_a+std_b*std_b);
	//pCfgs->guTest3 = (int)(color_deviate*1000);
	//pCfgs->guTestD[0] = (int)(ave_a);
	//pCfgs->guTestD[1] = (int)(ave_b);
	//pCfgs->guTestD[2] = (int)(std_a);
	//pCfgs->guTestD[3] = (int)(std_b);
	if (color_deviate>=3||color_deviate<0.05||(std_a+std_b)<10)
	{
		return TRUE;
	} 
	else
	{
		return FALSE; 
	}

}
//对点进行排序
void sort_point(int array[][2],int length)
{
	int i=0,j=0;
	int temp[2];
	for(i=0;i<length-1;i++)
	{
		for(j=i+1;j<length;j++)
		{
			if(array[i][1]>array[j][1])
			{
				temp[0]=array[i][0];
				temp[1]=array[i][1];
				array[i][0]=array[j][0];
				array[i][1]=array[j][1];
				array[j][0]=temp[0];
				array[j][1]=temp[1];
			}
		}
	}
	if(array[0][0]>array[1][0])
	{
		temp[0]=array[0][0];
		temp[1]=array[0][1];
		array[0][0]=array[1][0];
		array[0][1]=array[1][1];
		array[1][0]=temp[0];
		array[1][1]=temp[1];
	}
	if(array[2][0]>array[3][0])
	{
		temp[0]=array[2][0];
		temp[1]=array[2][1];
		array[2][0]=array[3][0];
		array[2][1]=array[3][1];
		array[3][0]=temp[0];
		array[3][1]=temp[1];
	}

}
//对图像数据进行标定
static void camera_calibration(int base_line[][2],float base_length,int calibration_point[][2],float near_point_length,int laneNum,ALGCFGS *pCfgs)
{
	int i=0,j=0;
	//标定区域直线0——1——3——2——0
	float k01=0,b01=0;
	float k13=0,b13=0;
	float k32=0,b32=0;
	float k20=0,b20=0;
	//标定区域两条线的交点0-2 1-3
	float x0=0,y0=0;
	float dis=1;
	//检测车道中心线斜率
	float k_road=0,b_road=0;
	//float k_base=0,b_base=0;
	float pt_x=0,pt_y=0,pt1_x=0,pt1_y=0,pt2_x=0,pt2_y=0,pt3_x=0,pt3_y=0;

	int temp;
	int start_point=0,end_point=0;
	float calibration_base_line[FULL_ROWS]={0};
	float dis_pixel[FULL_ROWS]={0};
	float temp1=0,temp2=0;
	//对标定区域点进行排序
	sort_point(calibration_point,4);
	//求标定区域直线斜率和截距
	if(calibration_point[0][0]!=calibration_point[1][0])
	{
		k01=1.0*(calibration_point[0][1]-calibration_point[1][1])/(calibration_point[0][0]-calibration_point[1][0]);
		b01=calibration_point[0][1]-k01*calibration_point[0][0];
	}
	else
	{
		k01=1e+6;
	}
	if(calibration_point[1][0]!=calibration_point[3][0])
	{
		k13=1.0*(calibration_point[1][1]-calibration_point[3][1])/(calibration_point[1][0]-calibration_point[3][0]);
		b13=calibration_point[1][1]-k13*calibration_point[1][0];
	}
	else
	{
		k13=1e+6;
	}
	if(calibration_point[3][0]!=calibration_point[2][0])
	{
		k32=1.0*(calibration_point[3][1]-calibration_point[2][1])/(calibration_point[3][0]-calibration_point[2][0]);
		b32=calibration_point[3][1]-k32*calibration_point[3][0];
	}
	else
	{
		k32=1e+6;
	}
	if(calibration_point[2][0]!=calibration_point[0][0])
	{
		k20=1.0*(calibration_point[2][1]-calibration_point[0][1])/(calibration_point[2][0]-calibration_point[0][0]);
		b20=calibration_point[2][1]-k20*calibration_point[2][0];
	}
	else
	{
		k20=1e+6;
	}
	if(base_line[0][1]>base_line[1][1])
	{
		start_point=base_line[0][1];
		end_point=base_line[1][1];
	}

	else
	{
		start_point=base_line[1][1];
		end_point=base_line[0][1];
	}
	//两条车道线是垂直线
	if(k20>1e+5&&k13>1e+5)
	{
		dis=base_length/(start_point-end_point+1);
		calibration_base_line[FULL_ROWS-1]=near_point_length;
		for(i=FULL_ROWS-2;i>=0;i--)
		{
			calibration_base_line[i]=near_point_length+dis;
			dis_pixel[i]=dis;
		}
	}
	else
	{
		if(k20>1e+5)
		{
			x0=calibration_point[0][0];
			y0=k13*x0+b13;
		}
		else if(k13>1e+5)
		{
			x0=calibration_point[1][0];
			y0=k20*x0+b20;
		}
		else
		{
			x0=(b13-b20)/(k20-k13);
			y0=k13*x0+b13;
		}
		//对基准线上进行标定
		temp1=0,temp2=0;
		temp1=(float)(FULL_ROWS-1-end_point)/(float)(end_point-y0);
		temp2=(float)(FULL_ROWS-1-start_point)/(float)(start_point-y0);
		dis=(float)((base_line[1][0]-base_line[0][0])*(base_line[1][0]-base_line[0][0]));
		dis=(float)(dis+(base_line[1][1]-base_line[0][1])*(base_line[1][1]-base_line[0][1]));
		dis=sqrt(dis);
		base_length=base_length*(float)(start_point-end_point+1)/dis;
		dis=base_length/(temp1-temp2);
		calibration_base_line[FULL_ROWS-1]=near_point_length;
		for(i=FULL_ROWS-2;i>=0;i--)
		{
			calibration_base_line[i]=near_point_length+dis*(float)(FULL_ROWS-1-i)/(float)(i-y0);
			dis_pixel[i]=calibration_base_line[i]-calibration_base_line[i+1];
		}
	}
	//每个车道进行标定
	/*if(base_line[1][0]!=base_line[0][0])
	{
	k_base=1.0*(base_line[1][1]-base_line[0][1])/(base_line[1][0]-base_line[0][0]);
	b_base=base_line[1][1]-k_base*base_line[1][0];
	pt_x=(b01-b_base)/(k_base-k01);
	pt_y=k01*pt_x+b01;

	}
	else
	{
	pt_x=base_line[0][0];
	pt_y=k01*pt_x+b01;
	}*/
	pt_x=x0;
	pt_y=k01*pt_x+b01;
	for(i=0;i<laneNum;i++)
	{

		pt1_x=(pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQ[0].x+pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQ[1].x)/2;
		pt1_y=(pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQ[0].y+pCfgs->DownSpeedCfg.SpeedEachLane[i].ptCornerQ[1].y)/2;
		pt2_x=(pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[2].x+pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[3].x)/2;
		pt2_y=(pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[2].y+pCfgs->DownSpeedCfg.SpeedEachLane[i].ptFourCorner[3].y)/2;
		if(pt1_x!=pt2_x)
		{
			k_road=1.0*(pt1_y-pt2_y)/(pt1_x-pt2_x);
			b_road=pt1_y-k_road*pt1_x;
			//求交点
			pt3_x=(b01-b_road)/(k_road-k01);
			pt3_y=k01*pt3_x+b01;
		}
		else
		{
			//求交点
			pt3_x=pt1_x;
			pt3_y=k01*pt3_x+b01;
		}
		temp=(int)(pt3_y+0.5);
		pCfgs->actual_distance[i][temp]=calibration_base_line[(int)(pt_y+0.5)];
		for(j=temp+1;j<FULL_ROWS;j++)
		{
			pCfgs->actual_distance[i][j]=pCfgs->actual_distance[i][j-1]-dis_pixel[j-1];
		}
		for(j=temp-1;j>=0;j--)
		{
			pCfgs->actual_distance[i][j]=pCfgs->actual_distance[i][j+1]+dis_pixel[j];
		}
	}
}