#include "global.h"
#include "convert_file_op.h"
#ifdef CONFIG_TPD_MSTAR_TEST
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/time.h>
#include <linux/rtc.h>
#endif

extern MutualMpTest_t * ptMutualMpTest;
extern MutualMpTestResult_t * ptMutualMpTestResult;
extern struct _TP_INFO tpinfo;

#ifdef CONFIG_TPD_MSTAR_TEST
static int mstar_test_get_save_filename(char *filename, int len)
{	
	char* board_idFile = {"/persist/factoryinfo/board_id"};
	struct file *pfile = NULL;
	struct inode *inode = NULL;
	//unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	loff_t pos = 0;
	mm_segment_t old_fs;
	char board_id[20];
	struct timespec ts;
	struct rtc_time tm;

	//get rtc time
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", board_idFile); 

	if (NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}

	if (IS_ERR(pfile)) {
		MSTAR_TEST_DBG("error occured while opening file %s.",  filepath);
		snprintf(filename, len, "test_data%04d%02d%02d",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
		return 0;
	}

	inode = pfile->f_dentry->d_inode;
	//magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, board_id, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	//snprintf(filename, len, "TP_test_data%s_%02d%02d%02d", board_id, tm.tm_hour, tm.tm_min, tm.tm_sec);
	snprintf(filename, len, "TP_test_data%s", board_id);
	return 0;
}
#endif

void save_test_data(void)
{
	int i = 0, j = 0, max_channel, test_Interval = 3, testCount = 0, failCount = 0;
//	char *pbuf = NULL;
	char sResult[10];
	char *SetCsvData = NULL, CsvPATHName[256] = "";
	struct file *f=NULL;
	mm_segment_t fs;

#ifdef CONFIG_TPD_MSTAR_TEST
	char filename[64];
	char procfilename[64];
#endif

	fs = get_fs();
	set_fs(KERNEL_DS);

	SetCsvData = kmalloc(1024,GFP_KERNEL);

	if (ptMutualMpTestResult->nShortResult == ITO_TEST_OK &&
			 ptMutualMpTestResult->nOpenResult == ITO_TEST_OK)
	{
		strcpy(sResult, "PASS");

	} 
	else
	{
		if (testCount >= test_Interval) 
		{
			failCount++;
			if (failCount == test_Interval)
				strcpy(sResult, "FAIL");
			else 
				strcpy(sResult, "FAIL");
		}
		strcpy(sResult, "FAIL");
	}

#ifdef CONFIG_TPD_MSTAR_TEST

	mstar_test_get_save_filename(filename,56);
	strncpy(procfilename, filename, 56);
	strncpy(sResult, filename, 56);

	
#endif	
	
	sprintf(CsvPATHName,"/cache/TPtest/%s.csv", sResult);
	printk("CSV:%s\n", CsvPATHName);
	if(f == NULL)
		f = filp_open(CsvPATHName, O_CREAT | O_RDWR , 0644);
	if(IS_ERR(f))
	{
		printk("Failed to open csv file %s\n", CsvPATHName);
		goto fail_open;
	}
	
	//sprintf(SetCsvData, "Golden 0 Max,,");
	strcpy(SetCsvData, "Golden 0 Max,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for(i = 0; i < MAX_MUTUAL_NUM; i++)
	{
		//sprintf(SetCsvData, "%1f,", ptMutualMpTestResult->pGolden_CH_Max[i]);
		sprintf(SetCsvData, "%d,", ptMutualMpTestResult->pGolden_CH_Max[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	
	//sprintf(SetCsvData, "Golden 0,,");
	strcpy(SetCsvData, "Golden 0,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	for(i = 0; i < MAX_MUTUAL_NUM; i++)
	{
		//sprintf(SetCsvData, "%1f,", ptMutualMpTestResult->pGolden_CH[i]);
		sprintf(SetCsvData, "%d,", ptMutualMpTestResult->pGolden_CH[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	}
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	
	//sprintf(SetCsvData, "Golden 0 Min,,");
	strcpy(SetCsvData, "Golden 0 Min,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	for(i = 0; i < MAX_MUTUAL_NUM; i++)
	{
		//sprintf(SetCsvData, "%1f,", ptMutualMpTestResult->pGolden_CH_Min[i]);
		sprintf(SetCsvData, "%1d,", ptMutualMpTestResult->pGolden_CH_Min[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "\n");	
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	
	//sprintf(SetCsvData, "test_0_deltaC,,");
	strcpy(SetCsvData, "test_0_deltaC,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	for(i = 0; i < MAX_MUTUAL_NUM; i++)
	{
		sprintf(SetCsvData, "%d,", ptMutualMpTestResult->pOpenResultData[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	
	//sprintf(SetCsvData, "test_0_ratio,[%1$.2f~%2$.2f],", ptMutualMpTestResult->nRatioAvg_max, ptMutualMpTestResult->nRatioAvg_min);
	sprintf(SetCsvData, "test_0_ratio,[%1d~%2d],", ptMutualMpTestResult->nRatioAvg_max, ptMutualMpTestResult->nRatioAvg_min);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	for(i = 0; i < MAX_MUTUAL_NUM; i++)
	{
		//sprintf(SetCsvData, "%1f,", ptMutualMpTestResult->pGolden_CH_Max_Avg[i]);
		sprintf(SetCsvData, "%1d,", ptMutualMpTestResult->pGolden_CH_Max_Avg[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	
	//sprintf(SetCsvData, "test_border_ratio,[%1$.2f~%2$.2f] ", ptMutualMpTestResult->nBorder_RatioAvg_max, ptMutualMpTestResult->nBorder_RatioAvg_min);
	sprintf(SetCsvData, "test_border_ratio,[%1d~%2d] ", ptMutualMpTestResult->nBorder_RatioAvg_max, ptMutualMpTestResult->nBorder_RatioAvg_min);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	for(i = 0; i < MAX_MUTUAL_NUM; i++)
	{
		sprintf(SetCsvData, "%s", ",");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "Platform Version :%s\n", tpinfo.PlatformVersion);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);		
	sprintf(SetCsvData, "Device Driver Version : %s\n", DEVICE_DRIVER_RELEASE_VERSION);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	sprintf(SetCsvData, "FW Version : %s\n", tpinfo.FwVersion);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	sprintf(SetCsvData, "Main Block FW Version : %s\n", tpinfo.MainBlockFWVersion);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	sprintf(SetCsvData, "Info Block FW Version : %s\n", tpinfo.InfoBlockFWVersion);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	sprintf(SetCsvData, "ANA_Version : %s\n", ptMutualMpTest->ana_version);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	sprintf(SetCsvData, "SupportIC : %s\n", ptMutualMpTest->UIConfig.sSupportIC);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	sprintf(SetCsvData, "Project name : %s\n", ptMutualMpTest->project_name);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
	sprintf(SetCsvData, "Mapping table name : %s\n", ptMutualMpTestResult->mapTbl_sec);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	//sprintf(SetCsvData, "DC_Range=%u\n", ptMutualMpTest->ToastInfo.persentDC_VA_Range);
	sprintf(SetCsvData, "DC_Range=%d\n", ptMutualMpTest->ToastInfo.persentDC_VA_Range);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	//sprintf(SetCsvData, "DC_Range_up=%hu\n", ptMutualMpTest->ToastInfo.persentDC_VA_Range_up);
	sprintf(SetCsvData, "DC_Range_up=%d\n", ptMutualMpTest->ToastInfo.persentDC_VA_Range_up);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	//sprintf(SetCsvData, "DC_Ratio=%u\n", ptMutualMpTest->ToastInfo.persentDC_VA_Ratio);
	sprintf(SetCsvData, "DC_Ratio=%d\n", ptMutualMpTest->ToastInfo.persentDC_VA_Ratio);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	//sprintf(SetCsvData, "DC_Border_Ratio=%u\n", ptMutualMpTest->ToastInfo.persentDC_Border_Ratio);
	sprintf(SetCsvData, "DC_Border_Ratio=%d\n", ptMutualMpTest->ToastInfo.persentDC_Border_Ratio);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	//sprintf(SetCsvData, "DC_Ratio_up=%hu\n\n", ptMutualMpTest->ToastInfo.persentDC_VA_Ratio_up);
	sprintf(SetCsvData, "DC_Ratio_up=%d\n\n", ptMutualMpTest->ToastInfo.persentDC_VA_Ratio_up);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
 
	//sprintf(SetCsvData, "Golden,,");
	strcpy(SetCsvData, "Golden,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);		
	for(i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++)
	{
		sprintf(SetCsvData, "D%d,", i+1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);		
	for(j = 0; j < ptMutualMpTest->sensorInfo.numSen; j++)
	{
		sprintf(SetCsvData, ",S%d,", j+1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		for(i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++)
		{
			if (ptMutualMpTestResult->pGolden_CH[j * ptMutualMpTest->sensorInfo.numDrv + i] == NULL_DATA) //for mutual key
			{
				sprintf(SetCsvData, "%s", ",");
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
			}
			else
			{
				//sprintf(SetCsvData, "%1$.2f,", ptMutualMpTestResult->pGolden_CH[j * ptMutualMpTest->sensorInfo.numDrv + i]);
				sprintf(SetCsvData, "%.2d,", ptMutualMpTestResult->pGolden_CH[j * ptMutualMpTest->sensorInfo.numDrv + i]);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
			}
		}
		sprintf(SetCsvData, "\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);		
	
	//sprintf(SetCsvData, "Golden_Max,,");
	strcpy(SetCsvData, "Golden_Max,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);		
	for(i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++)
	{
		sprintf(SetCsvData, "D%d,", i+1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);		
	for(j = 0; j < ptMutualMpTest->sensorInfo.numSen; j++)
	{
		sprintf(SetCsvData, ",S%d,", j+1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		for(i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++)
		{
			if (ptMutualMpTestResult->pGolden_CH[j * ptMutualMpTest->sensorInfo.numDrv + i] == NULL_DATA) //for mutual key
			{
				sprintf(SetCsvData, "%s", ",");
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
			}
			else
			{
				//sprintf(SetCsvData, "%1$.2f,", ptMutualMpTestResult->pGolden_CH_Max[j * ptMutualMpTest->sensorInfo.numDrv + i]);
				sprintf(SetCsvData, "%.2d,", ptMutualMpTestResult->pGolden_CH_Max[j * ptMutualMpTest->sensorInfo.numDrv + i]);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
			}
		}
		sprintf(SetCsvData, "\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	
	//sprintf(SetCsvData, "Golden_Min,,");
	strcpy(SetCsvData, "Golden_Min,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);		
	for(i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++)
	{
		sprintf(SetCsvData, "D%d,", i+1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);		
	for(j = 0; j < ptMutualMpTest->sensorInfo.numSen; j++)
	{
		sprintf(SetCsvData, ",S%d,", j+1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		for(i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++)
		{
			if (ptMutualMpTestResult->pGolden_CH[j * ptMutualMpTest->sensorInfo.numDrv + i] == NULL_DATA) //for mutual key
			{
				sprintf(SetCsvData, "%s", ",");
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
			}
			else
			{
				//sprintf(SetCsvData, "%1$.2f,", ptMutualMpTestResult->pGolden_CH_Min[j * ptMutualMpTest->sensorInfo.numDrv + i]);
				sprintf(SetCsvData, "%.2d,", ptMutualMpTestResult->pGolden_CH_Min[j * ptMutualMpTest->sensorInfo.numDrv + i]);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
			}
		}
		sprintf(SetCsvData, "\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	
	//sprintf(SetCsvData, "DeltaC,,");
	strcpy(SetCsvData, "DeltaC,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);		
	for(i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++)
	{
		sprintf(SetCsvData, "D%d,", i+1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);		
	for(j = 0; j < ptMutualMpTest->sensorInfo.numSen; j++)
	{
		sprintf(SetCsvData, ",S%d,", j+1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		for(i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++)
		{
			if (ptMutualMpTestResult->pOpenResultData[j * ptMutualMpTest->sensorInfo.numDrv + i] == NULL_DATA) //for mutual key
			{
				sprintf(SetCsvData, "%s", ",");
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
			}
			else
			{
				//sprintf(SetCsvData, "%1$d,", ptMutualMpTestResult->pOpenResultData[j * ptMutualMpTest->sensorInfo.numDrv + i]);
				sprintf(SetCsvData, "%1d,", ptMutualMpTestResult->pOpenResultData[j * ptMutualMpTest->sensorInfo.numDrv + i]);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
			}
		}
		sprintf(SetCsvData, "\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	if(ptMutualMpTestResult->nOpenResult == 1)
	{
		sprintf(SetCsvData, "DeltaC_Result:PASS\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	else
	{
		if(ptMutualMpTestResult->pCheck_Fail[0] == 1)
		{
			sprintf(SetCsvData, "DeltaC_Result:FAIL\nFail Channel:");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			for (i = 0; i < MAX_MUTUAL_NUM; i++)
			{
				if (ptMutualMpTestResult->pOpenFailChannel[i] == PIN_NO_ERROR)
					continue;
					//sprintf(SetCsvData,"D%1$d.S%2$d", ptMutualMpTestResult->pOpenFailChannel[i] % 100, ptMutualMpTestResult->pOpenFailChannel[i] / 100);
					sprintf(SetCsvData,"D%1d.S%2d", ptMutualMpTestResult->pOpenFailChannel[i] % 100, ptMutualMpTestResult->pOpenFailChannel[i] / 100);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			}
			sprintf(SetCsvData, "\n");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		}
		else
		{
			sprintf(SetCsvData, "DeltaC_Result:PASS\n");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		}
	}
	
	//sprintf(SetCsvData, "\nRatio,,");
	strcpy(SetCsvData, "\nRatio,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);		
	for(i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++)
	{
		sprintf(SetCsvData, "D%d,", i+1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);		
	for(j = 0; j < ptMutualMpTest->sensorInfo.numSen; j++)
	{
		sprintf(SetCsvData, ",S%d,", j+1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		for(i = 0; i < ptMutualMpTest->sensorInfo.numDrv; i++)
		{
			if (ptMutualMpTestResult->pOpenResultData[j * ptMutualMpTest->sensorInfo.numDrv + i] == NULL_DATA) //for mutual key
			{
				sprintf(SetCsvData, "%s", ",");
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
			}
			else
			{
				//sprintf(SetCsvData, "%1$f,", ptMutualMpTestResult->pGolden_CH_Max_Avg[j * ptMutualMpTest->sensorInfo.numDrv + i]);
				sprintf(SetCsvData, "%1d,", ptMutualMpTestResult->pGolden_CH_Max_Avg[j * ptMutualMpTest->sensorInfo.numDrv + i]);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
			}
		}
		sprintf(SetCsvData, "\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	if(ptMutualMpTestResult->nOpenResult == 1)
	{
		sprintf(SetCsvData, "Ratio_Result:PASS\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	else
	{
		if(ptMutualMpTestResult->pCheck_Fail[0] == 1)
		{
			sprintf(SetCsvData, "Ratio_Result:FAIL\nFail Channel:");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			for (i = 0; i < MAX_MUTUAL_NUM; i++)
			{
				if (ptMutualMpTestResult->pOpenFailChannel[i] == PIN_NO_ERROR)
					continue;
					//sprintf(SetCsvData,"D%1$d.S%2$d", ptMutualMpTestResult->pOpenRatioFailChannel[i] % 100, ptMutualMpTestResult->pOpenRatioFailChannel[i] / 100);
					sprintf(SetCsvData,"D%1d.S%2d", ptMutualMpTestResult->pOpenRatioFailChannel[i] % 100, ptMutualMpTestResult->pOpenRatioFailChannel[i] / 100);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);
			}
			sprintf(SetCsvData, "\n");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		}
		else
		{
			sprintf(SetCsvData, "Ratio_Result:PASS\n");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		}
	}
	sprintf(SetCsvData, "\n\nShortValue=%d\n\n", ptMutualMpTest->sensorInfo.thrsShort);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	
	sprintf(SetCsvData, "ICPinShort=%d\n\n", ptMutualMpTest->sensorInfo.thrsICpin);
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	

	//sprintf(SetCsvData, "Pin Number,,");
	strcpy(SetCsvData, "Pin Number,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	max_channel = MAX_CHANNEL_NUM_28XX;
	if (tpinfo.ChipType == CHIP_TYPE_MSG28XXA)
		max_channel = MAX_CHANNEL_NUM_30XX;

	for (i = 0; i < max_channel; i ++) {
		if (ptMutualMpTestResult->pICPinChannel[i] == 0) {
			continue;
		}
		//logHeader[i+1-j] = "P" + Integer.toString(ptMutualMpTestResult->pICPinChannel[i]);
		sprintf(SetCsvData, "P%d,",ptMutualMpTestResult->pICPinChannel[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	//sprintf(SetCsvData, "\ndeltaR,,");
	strcpy(SetCsvData, "\ndeltaR,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	for (i = 0; i < max_channel; i ++) {
		if (ptMutualMpTestResult->pICPinChannel[i] == 0) {
			continue;
		}
		//sprintf(SetCsvData, "%1$.1fM,",ptMutualMpTestResult->pICPinShortRData[i]);
		sprintf(SetCsvData, "%1dM,",ptMutualMpTestResult->pICPinShortRData[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}

	//sprintf(SetCsvData, "\nresultData,,");
	strcpy(SetCsvData, "\nresultData,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	for (i = 0; i < max_channel; i ++) {
		if (ptMutualMpTestResult->pICPinChannel[i] == 0) {
			continue;
		}
		//sprintf(SetCsvData, "%1$d,",ptMutualMpTestResult->pICPinShortResultData[i]);
		sprintf(SetCsvData, "%d,",ptMutualMpTestResult->pICPinShortResultData[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	
	if (ptMutualMpTestResult->nShortResult == ITO_TEST_OK) {
		sprintf(SetCsvData, "\nICPin Short Test:PASS\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	else {
		if (ptMutualMpTestResult->pCheck_Fail[2] == 1) {
			sprintf(SetCsvData, "\nICPin Short Test:FAIL\nFail Channel:,,");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
			for (i = 0; i < max_channel; i++) {
				if (ptMutualMpTestResult->pICPinShortFailChannel[i] == 0) {
					continue;
				}
				//sprintf(SetCsvData, "P%1$d,",ptMutualMpTestResult->pICPinShortFailChannel[i]);
				sprintf(SetCsvData, "P%d,",ptMutualMpTestResult->pICPinShortFailChannel[i]);
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
			}
		} else {

			sprintf(SetCsvData, "\nICPin Short Test:PASS\n");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		}
	}

	//sprintf(SetCsvData, "\ndeltaR,,");
	strcpy(SetCsvData, "\ndeltaR,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	for (i = 0; i < 10; i ++) {	    						
		sprintf(SetCsvData, "%d,", i+1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}	
	for (i = 0; i < (ptMutualMpTest->sensorInfo.numSen); i ++) {
		if ((i % 10) == 0) {
			sprintf(SetCsvData, "\n,S%d,", i);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		}
		//sprintf(SetCsvData, "%1$.1fM,", ptMutualMpTestResult->pShortRData[i]);
		sprintf(SetCsvData, "%dM,", ptMutualMpTestResult->pShortRData[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	for (i = 0; i < (ptMutualMpTest->sensorInfo.numDrv); i ++) {
		if ((i % 10) == 0) {
			sprintf(SetCsvData, "\n,D%d,", i);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		}
		//sprintf(SetCsvData, "%1$.1fM,",  ptMutualMpTestResult->pShortRData[i + ptMutualMpTest->sensorInfo.numSen]);
		printk(" pshortdata = %d \n",  ptMutualMpTestResult->pShortRData[i + ptMutualMpTest->sensorInfo.numSen]);
		sprintf(SetCsvData, "%1dM,",  ptMutualMpTestResult->pShortRData[i + ptMutualMpTest->sensorInfo.numSen]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	
	sprintf(SetCsvData, "\n");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);				
	for (i = 0; i < ptMutualMpTest->sensorInfo.numGr; i ++) {
		if ((i % 10) == 0) {
			sprintf(SetCsvData, "\n,GR%d,", i);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		}
		//sprintf(SetCsvData, "%1$d,",  ptMutualMpTestResult->pShortResultData[i + ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv]);
		sprintf(SetCsvData, "%d,",  ptMutualMpTestResult->pShortResultData[i + ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	
	if (ptMutualMpTestResult->nShortResult == ITO_TEST_OK) {
		//sprintf(SetCsvData, "\nITO Short Test:PASS,");
		strcpy(SetCsvData, "\nITO Short Test:PASS,");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	else {
		if (ptMutualMpTestResult->pCheck_Fail[3] == 1) {
			if (testCount >= test_Interval) {
				if (failCount == test_Interval) {
					//sprintf(SetCsvData, "\nITO Short Test:FAIL\nFail Channel:,,");
					strcpy(SetCsvData, "\nITO Short Test:FAIL\nFail Channel:,,");
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
					j = 0;
					for (i = 0; i < ptMutualMpTest->sensorInfo.numSen; i++) {
						if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
							continue;
						//sprintf(SetCsvData, "S%1$d,", i + 1);
						sprintf(SetCsvData, "S%d,", i + 1);
						f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
					}
					for (; i < ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv; i++) {
						if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
							continue;
						//sprintf(SetCsvData, "D%1$d,", i + 1 - ptMutualMpTest->sensorInfo.numSen);
						sprintf(SetCsvData, "D%d,", i + 1 - ptMutualMpTest->sensorInfo.numSen);
						f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
					}
					for (; i < ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv + ptMutualMpTest->sensorInfo.numGr; i++) {
						if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
							continue;
						//sprintf(SetCsvData, "GR%1$d", i + 1 - ptMutualMpTest->sensorInfo.numSen - ptMutualMpTest->sensorInfo.numDrv);
						sprintf(SetCsvData, "GR%d", i + 1 - ptMutualMpTest->sensorInfo.numSen - ptMutualMpTest->sensorInfo.numDrv);
						f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
					}
				} else {
					//sprintf(SetCsvData, "\nITO Short Test:PASS,");
					strcpy(SetCsvData, "\nITO Short Test:PASS,");
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
				}
			} else {
				//sprintf(SetCsvData, "\nITO Short Test:FAIL\nFail Channel:,,");
				strcpy(SetCsvData, "\nITO Short Test:FAIL\nFail Channel:,,");
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
				j = 0;
				for (i = 0; i < ptMutualMpTest->sensorInfo.numSen; i++) {
					if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
						continue;
					//sprintf(SetCsvData, "S%1$d,", i + 1);
					sprintf(SetCsvData, "S%d,", i + 1);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
				}
				for (; i < ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv; i++) {
					if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
						continue;
					//sprintf(SetCsvData, "D%1$d,", i + 1 - ptMutualMpTest->sensorInfo.numSen);
					sprintf(SetCsvData, "D%d,", i + 1 - ptMutualMpTest->sensorInfo.numSen);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
				}
				for (; i < ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv + ptMutualMpTest->sensorInfo.numGr; i++) {
					if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
						continue;
					//sprintf(SetCsvData, "GR%1$d", i + 1 - ptMutualMpTest->sensorInfo.numSen - ptMutualMpTest->sensorInfo.numDrv);
					sprintf(SetCsvData, "GR%d", i + 1 - ptMutualMpTest->sensorInfo.numSen - ptMutualMpTest->sensorInfo.numDrv);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
				}
			}
		}
		else
		{
			//sprintf(SetCsvData, "\nITO Short Test:PASS,");
			strcpy(SetCsvData, "\nITO Short Test:PASS,");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		}
	}

	//sprintf(SetCsvData, "\nresultData,,");
	strcpy(SetCsvData, "\nresultData,,");
	f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	for (i = 0; i < 10; i ++) {	    						
		sprintf(SetCsvData, "%d,", i+1);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	for (i = 0; i < (ptMutualMpTest->sensorInfo.numSen); i ++) {
		if ((i % 10) == 0) {
			sprintf(SetCsvData, "\n,S%d,", i);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		}
		//sprintf(SetCsvData, "%1$d,",  ptMutualMpTestResult->pShortResultData[i]);
		sprintf(SetCsvData, "%d,",  ptMutualMpTestResult->pShortResultData[i]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	
	for (i = 0; i < (ptMutualMpTest->sensorInfo.numDrv); i ++) {
		if ((i % 10) == 0) {
			sprintf(SetCsvData, "\n,D%d,", i);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		}
		//sprintf(SetCsvData, "%1$d,",  ptMutualMpTestResult->pShortResultData[i + ptMutualMpTest->sensorInfo.numSen]);
		sprintf(SetCsvData, "%d,",  ptMutualMpTestResult->pShortResultData[i + ptMutualMpTest->sensorInfo.numSen]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	
	for (i = 0; i < (ptMutualMpTest->sensorInfo.numGr); i ++) {
		if ((i % 10) == 0) {
			sprintf(SetCsvData, "\n,GR%d,", i);
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		}
		//sprintf(SetCsvData, "%1$d,",  ptMutualMpTestResult->pShortResultData[i + ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv]);
		sprintf(SetCsvData, "%d,",  ptMutualMpTestResult->pShortResultData[i + ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv]);
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	
	if (ptMutualMpTestResult->nShortResult == ITO_TEST_OK) {
		//sprintf(SetCsvData, "\nITO Short Test:PASS\n");
		strcpy(SetCsvData, "\nITO Short Test:PASS\n");
		f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
	}
	else {
		if (ptMutualMpTestResult->pCheck_Fail[3] == 1) {
			if (testCount >= test_Interval) {
				if (failCount == test_Interval) {
					//sprintf(SetCsvData, "\nITO Short Test:FAIL\nFail Channel:,,");
					strcpy(SetCsvData, "\nITO Short Test:FAIL\nFail Channel:,,");
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
					j = 0;
					for (i = 0; i < ptMutualMpTest->sensorInfo.numSen; i++) {
						if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
							continue;
						//sprintf(SetCsvData, "S%1$d,", i + 1);
						sprintf(SetCsvData, "S%d,", i + 1);
						f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
					}
					for (; i < ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv; i++) {
						if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
							continue;
						//sprintf(SetCsvData, "D%1$d,", i + 1 - ptMutualMpTest->sensorInfo.numSen);
						sprintf(SetCsvData, "D%d,", i + 1 - ptMutualMpTest->sensorInfo.numSen);
						f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
					}
					for (; i < ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv + ptMutualMpTest->sensorInfo.numGr; i++) {
						if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
							continue;
						//sprintf(SetCsvData, "GR%1$d", i + 1 - ptMutualMpTest->sensorInfo.numSen - ptMutualMpTest->sensorInfo.numDrv);
						sprintf(SetCsvData, "GR%d", i + 1 - ptMutualMpTest->sensorInfo.numSen - ptMutualMpTest->sensorInfo.numDrv);
						f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
					}
				} else {
					//sprintf(SetCsvData, "\nITO Short Test:PASS\n");
					strcpy(SetCsvData, "\nITO Short Test:PASS\n");
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
				}
			} else {
				//sprintf(SetCsvData, "\nITO Short Test:FAIL\nFail Channel:,,");
				strcpy(SetCsvData, "\nITO Short Test:FAIL\nFail Channel:,,");
				f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
				j = 0;
				for (i = 0; i < ptMutualMpTest->sensorInfo.numSen; i++) {
					if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
						continue;
					//sprintf(SetCsvData, "S%1$d,", i + 1);
					sprintf(SetCsvData, "S%d,", i + 1);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
				}
				for (; i < ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv; i++) {
					if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
						continue;
					//sprintf(SetCsvData, "D%1$d,", i + 1 - ptMutualMpTest->sensorInfo.numSen);
					sprintf(SetCsvData, "D%d,", i + 1 - ptMutualMpTest->sensorInfo.numSen);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
				}
				for (; i < ptMutualMpTest->sensorInfo.numSen + ptMutualMpTest->sensorInfo.numDrv + ptMutualMpTest->sensorInfo.numGr; i++) {
					if (ptMutualMpTestResult->pShortFailChannel[i] == PIN_UN_USE)
						continue;
					//sprintf(SetCsvData, "GR%1$d", i + 1 - ptMutualMpTest->sensorInfo.numSen - ptMutualMpTest->sensorInfo.numDrv);
					sprintf(SetCsvData, "GR%d", i + 1 - ptMutualMpTest->sensorInfo.numSen - ptMutualMpTest->sensorInfo.numDrv);
					f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
				}
			}
		}
		else
		{
			//sprintf(SetCsvData, "\nITO Short Test:PASS,");
			strcpy(SetCsvData, "\nITO Short Test:PASS,");
			f->f_op->write(f, SetCsvData, strlen(SetCsvData) * sizeof(char), &f->f_pos);	
		}
	}

	filp_close(f, NULL);
fail_open:
	set_fs(fs);
	kfree(SetCsvData);
	return;		
}
