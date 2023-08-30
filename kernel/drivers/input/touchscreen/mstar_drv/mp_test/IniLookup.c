#include "global.h"
#include "iniparser.h"

static dictionary * ini = NULL;

char index_t[100];
int IniGetInt(char * section, char * key)
{
    if(ini == NULL){
        DBG(0,"*** %s(), ini handle error! ***\n", __func__);
    	return 0;
    }

    if(section == NULL){
        sprintf(index_t, "%s", key);
    }else{
    sprintf(index_t, "%s:%s", section, key);
    }

	printk("%s: index_t = %s \n", __func__, index_t);

    return iniparser_getint(ini, index_t, 0);
}

char * IniGetString(char * section, char * key)
{
	char index[100];

    if(ini == NULL){
        printk("*** %s(), ini handle error! ***\n", __func__);
    	return NULL;
    }

    if(section == NULL){
        sprintf(index, "%s", key);
    }else{
    	sprintf(index, "%s:%s", section, key);
    }

    return iniparser_getstring(ini, index, 0);
}

int IniGetU16Array(const char * section, const char * key, u16 * pBuf)
{
	char * s;
	char index[100];
	char * pToken;
	int nCount = 0;
	int res;
	long s_to_long;

    if(ini == NULL){
        printk("*** %s(), ini handle error! ***\n", __func__);
        return 0;
    }

    if(section == NULL){
        sprintf(index, "%s", key);
    }else{
        sprintf(index, "%s:%s", section, key);
    }

    s = iniparser_getstring(ini, index, NULL);
	while((pToken = strsep(&s, ",")) != NULL){
        res = kstrtol(pToken, 0, &s_to_long);
        if(res == 0)
        	pBuf[nCount] = s_to_long;
        else
            printk("%s: convert string to long error %d\n", __func__, res);
		nCount++;
	}
	return nCount;
}

#ifndef DISABLE_DOUBLE
int IniGetDoubleArray(char * section, char * key, double * pBuf)
#else
int IniGetDoubleArray(char * section, char * key, int * pBuf)
#endif
{
	char * s;
	char index[100];
	char * pToken;
	int nCount = 0;
#ifdef DISABLE_DOUBLE
	int res;
	long s_to_long;
#endif

    if(ini == NULL){
        printk("*** %s(), ini handle error! ***\n", __func__);
        return 0;
    }

    if(section == NULL){
        sprintf(index, "%s", key);
    }else{
    	sprintf(index, "%s:%s", section, key);
    }

    s = iniparser_getstring(ini, index, NULL);
	printk(" %s \n ", s);
	while((pToken = strsep(&s, ",")) != NULL){
#ifdef DISABLE_DOUBLE
        res = kstrtol(pToken, 0, &s_to_long);
        if(res == 0)
        	pBuf[nCount] = s_to_long;
        else
            printk("%s: convert string to long error %d\n", __func__, res);
		nCount++;
#endif		
	}
	return nCount;
}

int  IniGetIntArray(char * section, char * key, int * pBuf)
{
	char * s;
	char index[100];
	char * pToken;
	int nCount = 0;
	int res;
    long s_to_long;

    if(ini == NULL){
        printk("*** %s(), ini handle error! ***\n", __func__);
        return 0;
    }

    if(section == NULL){
        sprintf(index, "%s", key);
    }else{
    sprintf(index, "%s:%s", section, key);
    }

    s = iniparser_getstring(ini, index, NULL);
	while((pToken = strsep(&s, ",")) != NULL){
        res = kstrtol(pToken, 0, &s_to_long);
        if(res == 0)
        	pBuf[nCount] = s_to_long;
        else
            printk("%s: convert string to long error %d\n", __func__, res);
		nCount++;
	}
	return nCount;
}

int IniSplitU8Array(char * section, char * key, u8 * pBuf)
{
	char * s;
	char index[100];
	char * pToken;
	int nCount = 0;
	int i, res;
    long s_to_long = 0;

    if(ini == NULL){
        printk("*** %s(), ini handle error! ***\n", __func__);
        return 0;
    }

    if(section == NULL){
        sprintf(index, "%s", key);
    }else{
    sprintf(index, "%s:%s", section, key);
    }

    s = iniparser_getstring(ini, index, NULL);
	while((pToken = strsep(&s, ".")) != NULL){
        res = kstrtol(pToken, 0, &s_to_long);
        if(res == 0)
        	pBuf[nCount] = s_to_long;
        else
            printk("%s: convert string to long error %d\n", __func__, res);
		nCount++;
	}
	for(i=0; i<nCount; i++){
        printk("%d\n", pBuf[i]);
	}
    
	return nCount;
}

int IniSecKeysTo2DArray2(const char * pFile, char * pSection, u16 pArray[][2])
{
	struct file *f;
	char *s;
	char szLine[100], szSection[100];
	char * pToken;
	int nInSection = 0;
	int nCount = 0;
	int nKeyNum = 0;
	char EOF[10];

    printk("*** %s *** \n", __func__);
	
	strcpy(EOF, "&^*EOF");

	f = file_open(pFile, O_RDONLY, 0, NULL);
	if (f == NULL){
		printk("Cannot open file: %s\n", pFile);
		return 0;
	}

	sprintf(szSection, "[%s]", pSection);
	while (kgets(szLine, 100, f) != NULL) {
		if(strncmp(szLine, EOF, 5) == 0)             
	 	    break;

	    if (strstr(szLine, szSection) != NULL) {
			nInSection = 1;
			continue;
		}

		if(nInSection == 1){
			if(szLine != NULL){
				if (strstr(szLine, "[") != NULL)
				{
					printk("szLine = %s\n", szLine);
					break;
				}
				s = kstrdup(szLine, GFP_KERNEL);
				while((pToken = strsep(&s, "=,")) != NULL){
					if(nCount == 1){
						pArray[nKeyNum][0] = katoi(pToken);
					}else if(nCount == 2){
						pArray[nKeyNum][1] = katoi(pToken);
					}
					nCount++;
				}
				//printk("%s: %d,%d\n", pSection, pArray[nKeyNum][0], pArray[nKeyNum][1]);
				s = NULL;
				nCount = 0;
				nKeyNum++;
			}else{
				nInSection = 0;
				printk("nKeyNum : %d", nKeyNum);
				break;
			}
		}
	}

	kfree(s);
	file_close(f);
	return nKeyNum;
}

int IniLoad(char * pFile)
{
	ini = iniparser_load(pFile);
    if (ini == NULL) {
        DBG(0, "*** %s : iniPath = %s *** \n", __func__, pFile);
        return 0;
    }

    return 1;
}

void IniFree(void)
{
	if(ini != NULL){
        iniparser_freedict(ini);
	}
}
