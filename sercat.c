#include <errno.h>
#include <string.h>

#include <sys/fcntl.h>
#include <sys/ioctl.h>
//#include <sys/stat.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
//#include <stropts.h> // deprecated


#ifdef __linux__
#include <asm/ioctls.h>
//#include <asm/termios.h> // use termbits instead so there is no redefinition hell
#include <asm/termbits.h>
#else
#include <termios.h>
#endif /* of __linux__ */

#include <sys/select.h>
#include <sys/time.h>
#include <stdarg.h>


#define BAUDRATE 115200
#define BAUDRATE_ESPBOOT 74880
#define DEFAULT_TIMESTAMP 0
#define DEFAULT_PULSETIME 1000

//#include <termios.h>

//extern int tcgetattr (int __fd, struct termios *__termios_p) __THROW;
//extern int tcsetattr (int __fd, int __optional_actions,
  //                    const struct termios *__termios_p) __THROW;


/* source : http://stackoverflow.com/a/6947758 */
// https://gist.github.com/jerome-labidurie/5dde0ec105fe88aaf5fa8f3c54f4b07d


int isfirst=1;
int quiet=0;
int fd_tty;
int baudrate=BAUDRATE;

int espboot=0;
int espbootts=0;
int espbootln=0;

int olfcrlf=0; // 0=no mod, 1=cr, 2=crlf
int ilfcrlf=0;


#define LLEN 32
char lline[LLEN+2];
int llinelen=0;

void printtimestamp();

int printset(int isrts,int state){
    if(!isfirst)printf("\n");
    //printtimestamp();
    printf(" *** setting ");
    if(isrts)printf("RTS");else printf("DTR");
    printf(" to ");
    if(state)printf("HIGH");else printf("LOW");
    printf(" ***\n");
    isfirst=1;
}

int xioctl(int fd,int req,void*param,char*name){
    int r;
    //fprintf(stderr,"[[IOCTL:%s]]",name);
    r=ioctl(fd,req,param);
    if(r<0){fprintf(stderr,"ERROR: IOCTL %s failed: %i, errno=%i\n",name,r,errno);}
}

int setrts(int fd,int state){
    int flag;
    if(!quiet)printset(1,state);
    flag = TIOCM_RTS;
    if(state)xioctl(fd,TIOCMBIS,&flag,"TIOCMBIS (RTS)");//Set RTS pin
    else     xioctl(fd,TIOCMBIC,&flag,"TIOCMBIC (RTS)");//clear RTS pin
}

int setdtr(int fd,int state){
    int flag;
    if(!quiet)printset(0,state);
    flag = TIOCM_DTR;
    if(state)xioctl(fd,TIOCMBIS,&flag,"TIOCMBIS (DTR)");//Set RTS pin
    else     xioctl(fd,TIOCMBIC,&flag,"TIOCMBIC (DTR)");//clear RTS pin
}


int term_set_baudrate (int fd, int baud_rate)
{
    int rval, i;
    struct termios2 tio;

    rval = 0;

//    do { /* dummy */

//        tio = term.nexttermios[i];
    xioctl(fd,TCGETS2,&tio,"TCGETS2");

    /* drivers/tty/tty_xioctl.c :: tty_termios_baud_rate() */
    tio.c_cflag = (tio.c_cflag & ~CBAUD) | BOTHER;
    tio.c_ispeed = baud_rate;
    tio.c_ospeed = baud_rate;

    /* drivers/tty/tty_xioctl.c :: tty_termios_input_baud_rate() */
    /* If CIBAUD == B0, use output baud rate. Setting tio.c_ispeed is
     * unneeded */
    tio.c_cflag = (tio.c_cflag & ~(CBAUD << IBSHIFT)) | (B0 << IBSHIFT);
    xioctl(fd,TCSETS2,&tio,"TCSETS2");

//    }while (0);

    return rval;
}



int set_interface_attribs (int fd, int speed, int parity,int rts,int dtr) {
    struct termios2 tty;
    memset (&tty, 0, sizeof tty);

    if(dtr>=0)setdtr(fd,dtr);
    if(rts>=0)setrts(fd,rts);

    xioctl(fd,TCGETS2,&tty,"TCGETS2");

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;     // disable break processing; disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars
    tty.c_iflag &= ~(INLCR | ICRNL | ISTRIP | IXON | BRKINT); // raw mode; don't translate NL to CR or CR to NL on input, get all 8 bits of input, disable xon/xoff flow control on output, no interrupt on break signal
    tty.c_lflag = 0;        // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;        // no remapping, no delays
    tty.c_cc[VMIN]  = 0;        // read doesn't block
    tty.c_cc[VTIME] = 5;        // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if(speed>0){
      tty.c_cflag = (tty.c_cflag & ~CBAUD) | BOTHER;
      tty.c_ospeed = speed;
      tty.c_ispeed = speed;
    }

    xioctl(fd,TCSETS2,&tty,"TCSETS2");

    xioctl(fd,TCGETS2,&tty,"TCGETS2");
    if(!quiet)if(speed>=0)fprintf(stderr," *** device speed is %i bps ***\n",tty.c_ispeed);
    //fprintf(stderr,"fd:%i,%i\n",fd,fd_tty);
    return 0;
}

void set_blocking (int fd, int should_block) {
    struct termios2 tty;
    memset (&tty, 0, sizeof tty);
    xioctl(fd,TCGETS2,&tty,"TCGETS2");
    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 2;        // 0.2 seconds read timeout
    xioctl(fd,TCSETS2,&tty,"TCSETS2");
}

/** usage :
...
char *portname = "/dev/ttyUSB1"
 ...
int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
if (fd < 0)
{
    perror ("error %d opening %s: %s", errno, portname, strerror (errno));
    return;
}
set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
set_blocking (fd, 0);        // set no blocking
write (fd, "hello!\n", 7);       // send 7 character greeting
usleep ((7 + 25) * 100);         // sleep enough to transmit the 7 plus
                     // receive 25:  approx 100 uS per char transmit
char buf [100];
int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
*/

int isfilterchar(int c){
    if(c>0x1f)if(c<0x80)return 0;
    if(c==0x0d)return 0;
    if(c==0x0a)return 0;
    return 1;
}

void filterbuf(char*s,int n){
    int t;
    for(t=0;t<n;t++){
      if(isfilterchar(s[t]))s[t]='_';
    }
}

struct timeval tstart;
struct timeval tlast;

int timestamptype=DEFAULT_TIMESTAMP;

long timediff(struct timeval *t1,int printdiff){
  struct timeval t2;
  gettimeofday(&t2,0);
  long secdiff=t2.tv_sec-t1->tv_sec;
  long msecdiff=(t2.tv_usec-t1->tv_usec)/1000;
  if(msecdiff<0){msecdiff+=1000;secdiff--;}
  if(printdiff){printf("[%d.%03d]",secdiff,msecdiff);}
  return secdiff*1000+msecdiff;
//  fprintf(stderr,"[[%d,%d,%d]]",secdiff,msecdiff,(secdiff*1000+msecdiff));
}

long totinlines=1;
long totalmaxlines=0;

void printtimestamp(){
//    timestamptype=7;
    if( (!espboot) || (espbootts) || espbootln ){
      if(timestamptype&0x04)printf("[%d]",totinlines);
      if(timestamptype&0x02)timediff(&tlast,1);
      if(timestamptype&0x01)timediff(&tstart,1);
    }
    gettimeofday(&tlast,0);
}


void processline(){
    lline[llinelen]=0;
    //printf("{{%s}}",lline);
    //if(lline[0]=='~')printf("@@@@@@");
    if(espboot){
      if(strstr(lline,"~ld")){
        printf(" *** bootloader end detected ***\n");
        set_interface_attribs (fd_tty, baudrate, 0, -1,-1);
        espboot=0;
      }
    }
}


void printhexbuf(char*s,int n,int filter,int nohex){
    int t;
    for(t=0;t<n;t++){
      if(llinelen<LLEN){lline[llinelen]=s[t];llinelen++;}
      if(isfirst)if((totalmaxlines==0)||(totinlines<=totalmaxlines)){printtimestamp();isfirst=0;}
      if(!nohex){if(isfilterchar(s[t])){fprintf(stdout,"[%02x]",s[t]);continue;}}
      else if(filter){if(isfilterchar(s[t]))s[t]='_';}
      fprintf(stdout,"%c",s[t]);
      if(s[t]==0x0a){
         fflush(stdout);
         if((!espboot)||(espbootln))totinlines++;
         isfirst=1;
         processline();
         llinelen=0;
      }
    }
}

#define fd_stdin 0
#define fd_stdout 1
#define fd_stderr 2
#define OBUFSIZE 2048

char o[OBUFSIZE];
int ostart=0;
int oend=0;

int ogetend(){
    if(oend>=ostart)return oend;
    return oend+OBUFSIZE;
}

int ogetlen(){
    return ogetend()-ostart;
}

int oput(int c){
    o[oend]=c;
    oend++;
}

int oget(){
    int c;
    c=o[ostart];ostart++;
    return c;
}

int isofull(){
    if(ogetend()-ostart > OBUFSIZE-2)return 1;
    return 0;
}

int isoempty(){
    if(ostart==oend)return 1;
    return 0;
}

int dumpobuf(){
    fprintf(stderr,"[[ostart=%i,oend=%i,o='",ostart,oend);
    int t;
    for(t=ostart;t<ogetend();t++)fprintf(stderr,"%c",o[t%OBUFSIZE]);
    fprintf(stderr,"']]\n");

}


void help(){
    printf("sercat (compiled "__DATE__", "__TIME__")\n");
    printf("cat for serial ports, with RTS/DTR control, for arduino, ESP modules and microcontrollers\n");
    printf("\n");
    printf("Usage: sercat </dev/ttySomething> [-b speed] [-dtr] [-rts] [...]\n");
    printf("  -b <baudrate>      default 115200, 0=leave as is\n");
    printf("  -dtr               set DTR high (default: low)\n");
    printf("  -rts               set RTS high (default: low, unavoidably pulses high on port open due to linux kernel\n");
    printf("  -dtrp              pulse DTR high on start, for 500 ms\n");
    printf("  -rtsp              pulse RTS high on start, for 500 ms\n");
    printf("  -pt <n>            pulse time in ms (default: %i)\n",DEFAULT_PULSETIME);
    printf("  -raw               no filtering of unprintable characters (default: replace with '_')\n");
    printf("  -hex               show unprintables as [hex], replace with '_'\n");
    printf("  -notty             don't set RTS/DTR and baudrate, for timestamping only\n");
    printf("  -nl <n>            exit after n input lines\n");
    printf("  -ns <n>            exit after n seconds (todo)\n");
    printf("  -sd <n>            delay sending stdin for n milliseconds\n");
    printf("  -sl <n>            delay sending stdin for n input lines\n");
    printf("  -t <n>             line timestamp format, 0..7 (4=line number, 2=line time difference, 1=line time)\n");
    printf("  -q                 quiet, no state changes\n");
    printf("  -espboot           show ESP bootloader (start at 74880 bps until ~ld line, then switch to -b baudate\n");
    printf("  -espboott          show ESP bootloader, with timestamps if -t is set\n");
    printf("  -olfcr             translate terminal input LF to CR\n");
    printf("  -olfcrlf           translate terminal input LF to CRLF\n");
    printf("  -ilfcrlf           translate remote LF to CRLF\n");
    printf("\n");
}

/*
bootloader baudrate: 74880

 ets Jan  8 2013,rst cause:2, boot mode:(3,6)

load 0x4010f000, len 3584, room 16
tail 0
chksum 0xb0
csum 0xb0
v3969889e
~ld
*/


int main(int argc,char*argv[]){
    int rts=0,dtr=0,setrts=0,setdtr=0;
    int baudratestart;
    struct timeval tv;
    fd_set rfd,wfd;
    int resetmsec=DEFAULT_PULSETIME;
    int totin=0;

    char portname[255]="";
    int rtsp=0,dtrp=0;
    int filterout=1;
    int nohexout=1;
    long senddelay=0;
    int senddelaychars=0;
    int senddelaylines=0;
    int istty=1;

    totalmaxlines=0;
    timestamptype=7;
    long totalmaxmsec=0;

    int t;

    if(argc<2){help();return 0;}


    for(t=1;t<argc;t++){
      if(argv[t][0]!='-'){strncpy(portname,argv[t],254);portname[254]=0;continue;}
      if(!strcmp(argv[t],"-nl")){t++;totalmaxlines=atol(argv[t]);continue;}
      if(!strcmp(argv[t],"-ns")){t++;totalmaxmsec=1000*atoi(argv[t]);continue;}
      if(!strcmp(argv[t],"-sd")){t++;senddelay=atol(argv[t]);continue;}
      if(!strcmp(argv[t],"-sl")){t++;senddelaylines=atoi(argv[t]);continue;}
      if(!strcmp(argv[t],"-t")){t++;timestamptype=atoi(argv[t]);continue;}
      if(!strcmp(argv[t],"-b")){t++;baudrate=atoi(argv[t]);continue;}
      if(!strcmp(argv[t],"-rts")){setrts=1;continue;}
      if(!strcmp(argv[t],"-dtr")){setdtr=1;continue;}
      if(!strcmp(argv[t],"-rtsp")){rtsp=1;continue;}
      if(!strcmp(argv[t],"-dtrp")){dtrp=1;continue;}
      if(!strcmp(argv[t],"-pt")){t++;resetmsec=atoi(argv[t]);continue;}
      if(!strcmp(argv[t],"-raw")){filterout=0;nohexout=1;continue;}
      if(!strcmp(argv[t],"-hex")){nohexout=0;continue;}
      if(!strcmp(argv[t],"-notty")){istty=0;continue;}
      if(!strcmp(argv[t],"-q")){quiet=1;continue;}
      if(!strcmp(argv[t],"-espboot")){espboot=1;continue;}
      if(!strcmp(argv[t],"-espboott")){espboot=1;espbootts=1;espbootln=1;continue;}
      if(!strcmp(argv[t],"-olfcr")){olfcrlf=1;continue;}
      if(!strcmp(argv[t],"-olfcrlf")){olfcrlf=2;continue;}
      if(!strcmp(argv[t],"-ilfcrlf")){ilfcrlf=2;continue;}

      if(!strcmp(argv[t],"-h")){help();return 0;}
//      if(!strcmp(argv[t],""){;continue;}
    }

    gettimeofday(&tstart,0);
    gettimeofday(&tlast,0);

    baudratestart=baudrate;
    if(espboot)baudratestart=BAUDRATE_ESPBOOT;

    //strncpy(portname,argv[1],254);portname[254]=0;
    fd_tty = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_tty < 0)
    {
        perror ("error opening port");
        printf ("error %d opening %s: %s", errno, portname, strerror (errno));
        return 1;
    }
    rts=setrts;dtr=setdtr;
    if(rtsp)rts=!setrts;
    if(dtrp)dtr=!setdtr;
    if(istty)set_interface_attribs (fd_tty, baudratestart, 0, rts, dtr);  // set speed to 115,200 bps, 8n1 (no parity), RTS low (will pulse on port open), DTR low
    if(istty)set_blocking (fd_tty, 0);        // set no blocking
//    usleep ((7 + 25) * 100);         // sleep enough to transmit the 7 plus
                      // receive 25:  approx 100 uS per char transmit

    if(istty)if(rtsp||dtrp){
      if(!quiet)fprintf(stderr," *** sleeping for %i msec, with RTS=%i, DTR=%i ***\n",resetmsec,rts,dtr);
      usleep(1000*resetmsec);
      rts=setrts;dtr=setdtr;
      set_interface_attribs(fd_tty,-1,0,rts,dtr);
    }

    gettimeofday(&tstart,0);
    gettimeofday(&tlast,0);

    tv.tv_sec=1;
    tv.tv_usec=0;
    char buf [100];
    int sel;
    int bufdumped=0;
    //printtimestamp(1);
    while(1){
      if((totalmaxlines>0) && (totinlines>totalmaxlines))break;
      if((totalmaxmsec) && (timediff(&tstart,0)>totalmaxmsec))break;

      FD_ZERO(&rfd);
      FD_SET(fd_stdin,&rfd);
      FD_SET(fd_tty,&rfd);
      FD_ZERO(&wfd);
      FD_SET(fd_tty,&wfd);

      switch(sel=select(fd_tty+1,&rfd,&wfd,NULL,&tv)){
      case -1: perror("select");return EXIT_FAILURE;
      case 0: usleep(100);break;//puts("select() timeout");break;
      default:

        // input from port
        if(FD_ISSET(fd_tty,&rfd)){
          int n = read (fd_tty, buf, (sizeof buf)-1);  // read up to 100 characters if ready to read
          totin=totin+n;
          printhexbuf(buf,n,filterout,nohexout);
        }

        if(FD_ISSET(fd_tty,&wfd)){
          if(!isoempty()){
            if( (timediff(&tstart,0)>=senddelay) && (totin>=senddelaychars) && (totinlines>=senddelaylines) ){ // we got enough time from start, and enough characters from input
              char s[2];
              s[0]=oget();
              if(!quiet){
                if(s[0]==0x0a)fprintf(stderr,"{\\n}");else
                if(s[0]==0x0d)fprintf(stderr,"{\\r}");else
                fprintf(stderr,"{%c}",s[0]);
                if(isoempty())fprintf(stderr,"\n");
                if(s[0]=='\n' && olfcrlf){
                  if(olfcrlf==1)write(fd_tty,"\r",1);
                  else if(olfcrlf==2)write(fd_tty,"\r\n",2);
                }
                else
                  write(fd_tty,s,1);
              }
            }
          }
        }

        // input from stdin, output to port
        if(FD_ISSET(fd_stdin,&rfd)){
          if(!isofull()){
            char s[2];
            if(read(fd_stdin,s,1)){
              if(s[0]=='r'&0x1f)printf("[RESET]");
              if(s[0]=='\n' && ilfcrlf)oput('\r');
              oput(s[0]);
            }
            // TODO: ctrl-R, ctrl-I for RTS, DTR pulse
          }
        }
      }
    }
    return 0;
}


