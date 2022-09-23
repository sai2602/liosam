/*------------------------------------------------------------------------------
*
* Subset of ublox.c
*
*   from ANAVS_RTK_software/rtklib/rcv/ublox.c
*
* authors : Robert Bensch, ANavS GmbH
* history : 2018/04/18 - Initial file creation
*-----------------------------------------------------------------------------*/

# include <string.h>

/* get fields (little-endian) ------------------------------------------------*/
static unsigned short U2(unsigned char *p) {unsigned short u; memcpy(&u,p,2); return u;}
static unsigned int   U4(unsigned char *p) {unsigned int   u; memcpy(&u,p,4); return u;}

/* set fields (little-endian) ------------------------------------------------*/
static void setU2(unsigned char *p, unsigned short u) {memcpy(p,&u,2);}
static void setU4(unsigned char *p, unsigned int   u) {memcpy(p,&u,4);}

/* checksum ------------------------------------------------------------------*/
/*
static int checksum(unsigned char *buff, int len)
{
    unsigned char cka=0,ckb=0;
    int i;

    for (i=2;i<len-2;i++) {
        cka+=buff[i];
        ckb+=cka;
    }
    return cka==buff[len-2]&&ckb==buff[len-1];
}
static void setcs(unsigned char *buff, int len)
{
    unsigned char cka=0,ckb=0;
    int i;

    for (i=2;i<len-2;i++) {
        cka+=buff[i]; ckb+=cka;
    }
    buff[len-2]=cka;
    buff[len-1]=ckb;
}
*/
