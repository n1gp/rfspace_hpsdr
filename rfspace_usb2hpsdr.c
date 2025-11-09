/* Copyright (C)
*   10/2025 - Rick Koch, N1GP
*   Wrote rfspace_usb2hpsdr using various open sources on the internet.
*     https://github.com/kgarrels/SoapySDR14
*     Ian (G7GHH), sdr-14 code examples
*     Christoph, https://github.com/dl1ycf/pihpsdr
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
*/

/*
 * This program simulates an HPSDR Hermes board using USB data from
 * an RFSPACE SDR-14 or SDR-IQ radio.
 */

#include "rfspace_usb2hpsdr.h"

// Defines for some of the AD6620 registers
#define ADR_CIC2SCALE 0x305     //8 bits, range 0 to 6 each count==6dB atten
#define ADR_CIC2M 0x306         //8 bits, 0 to 15( decimation of 1 to 16 )
#define ADR_CIC5SCALE 0x307     //8 bits, range 0 to 20 each count == 6dB
#define ADR_CIC5M 0x308         //0 to 31(decimation 1 to 32)
#define ADR_RCFCTRL 0x309       // 8 bits
#define ADR_RCFM 0x30A  //8 bits 0 to 31 (decimation 1 to 32)
#define ADR_RCFOFFSET 0x30B     //8 bit filter coef offset for RCF filter
#define ADR_TAPS 0x30C          //8 bits 0 to 255(number of RCF taps 1 to 256)
#define ADR_RESERVED 0x30D      //must be zero

struct ftdi_context *ftdi;
char serialNumber[64] = {0};
int fd;
unsigned char broadcastReply[60];
int active = 0;
uint32_t last_sequence_number = 0;
uint32_t last_seqnum = 0, seqnum;
float complex iqBuffer[((RTL_READ_COUNT / 2) + (IQ_FRAME_DATA_LEN * 2))];

static int num_copy_rcvrs = 0, do_exit = 0;
static int copy_rcvr[8];
static u_char last_num_rcvrs = 0;
static u_char last_rate = 0;
static int last_freq[8] = { 0 };
static bool _running, _keep_running;

static size_t resp_size;
static unsigned char resp_data[1024*10];
static pthread_mutex_t resp_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t resp_cond = PTHREAD_COND_INITIALIZER;

struct sockaddr_in myaddr;				// our address
struct sockaddr_in remaddr;				// remote address
socklen_t addrlen = sizeof(remaddr);	// length of addresses

#define SYNC 0x7F
unsigned char hpsdrdata[1032];
uint8_t header_hpsdrdata[4] = { 0xef, 0xfe, 1, 6 };
uint8_t sync_hpsdrdata[8] = { SYNC, SYNC, SYNC, 0, 0, 0, 0, 0};
static int sock_udp;

static void process_ep2(uint8_t *frame);
static void *handler_ep6(void *arg);

static int oldnew = 1;  // 1: only P1, 2: only P2, 3: P1 and P2,
static int active_thread = 0;
static int enable_thread = 0;
static bool _run_usb_read_task;

struct main_cb mcb;

static pthread_mutex_t iqready_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t iqready_cond = PTHREAD_COND_INITIALIZER;
static u_int rcvr_flags = 0;
static pthread_mutex_t send_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t send_cond = PTHREAD_COND_INITIALIZER;
static u_int send_flags = 0;
static pthread_mutex_t done_send_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t done_send_cond = PTHREAD_COND_INITIALIZER;

static pthread_t hpsdrsim_sendiq_thr[MAX_RCVRS];
static pthread_t usb_read_task;

static int nsamps_packet[8] = { 126, 72, 50, 38, 30, 26, 22, 20 };
static int frame_offset1[8] = { 520, 520, 516, 510, 496, 510, 500, 516 };
static int frame_offset2[8] = { 1032, 1032, 1028, 1022, 1008, 1022, 1012, 1028 };

static int running = 0;

static u_char buffer[MAX_BUFFER_LEN];
static u_char payload[HPSDR_FRAME_LEN];

static struct sockaddr_in addr_new;
static struct sockaddr_in addr_old;

// using clock_nanosleep of librt
extern int clock_nanosleep(clockid_t __clock_id, int __flags,
                           __const struct timespec *__req,
                           struct timespec *__rem);

uint64_t get_posix_clock_time_us()
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
        return (uint64_t)(ts.tv_sec * 1000000 + ts.tv_nsec / 1000);
    } else {
        return 0; // Error handling
    }
}

bool transaction( const unsigned char *cmd, size_t size, unsigned char *response )
{
    size_t rx_bytes = 0;
    int ret=0;

    memset(response, 0, 64);

    if ( (ret = ftdi_write_data(ftdi, cmd, size))  ) {
        if (ret != (int)size) return false;
    }

    pthread_mutex_lock (&resp_lock);
    pthread_cond_wait (&resp_cond, &resp_lock);

    rx_bytes = resp_size;
    memcpy( response, &resp_data[0], rx_bytes );

#if 0
    printf("%d> ", rx_bytes);
    for (size_t i = 0; i < rx_bytes; i++)
        printf("%02x ", (unsigned char) resp_data[i]);
    printf("\n");
#endif
    pthread_mutex_unlock (&resp_lock);

    return true;
}

bool resp_transaction( const unsigned char *cmd, size_t size )
{
    unsigned char response[64];

    if ( ! transaction( cmd, size, response ) )
        return false;

    // comparing the contents is not really feasible due to protocol
    if ( resp_size == size ) // check response size against request
        return true;

    return false;
}

// Send a value to a register on the AD6620
int SetAD6620Register(unsigned int address,unsigned long long data)
{
    unsigned char data_out[9] = { 0x09,0xa0, 0x00,0x00, 0x00,0x00,0x00,0x00,0x00 };
    unsigned char response[64];
    // AD6620 address
    data_out[2] = address & 0xff;
    data_out[3] = (address & 0xff00) >> 8;
    // Data
    data_out[4] = data & 0xff;
    data_out[5] = (data & 0xff00) >> 8;
    data_out[6] = (data & 0xff0000) >> 16;
    data_out[7] = (data & 0xff000000) >> 24;
    data_out[8] = (data & 0xff00000000) >> 32;

    if ( ! transaction((unsigned char *)&data_out, 9, response)) {
        t_print("SetAD6620Register() failed.\n");
        return -1;
    }
    return 1;
}

// Convert to twos compliment encoded number to a normal number
// Convert a number to two's compliment format
unsigned long long ConvertToTwosCompliment(int raw)
{
    if (raw >= 0) {
        return (unsigned long long)raw;
    }
    // Convert to twos compliment
    raw = abs(raw);
    raw = ~raw;
    return ((unsigned long long)(raw + 1) & 0xffffffff);
}

bool start()
{
    _running = true;
    _keep_running = false;

    unsigned char start[] = { 0x08, 0x00, 0x18, 0x00, 0x81, 0x02, 0x00, 0x01 };

    // set if gain
    unsigned char ifgain[] = { 0x06, 0x00, 0x40, 0x00, 0x00, 0x18 };
    resp_transaction( ifgain, sizeof(ifgain) );

    return resp_transaction( start, sizeof(start) );
}

bool stop()
{
    if ( ! _keep_running )
        _running = false;
    _keep_running = false;

    unsigned char stop[] = { 0x08, 0x00, 0x18, 0x00, 0x81, 0x01, 0x00, 0x00 };
    return resp_transaction( stop, sizeof(stop) );
}

void sdr14_sighandler (int signum)
{
    t_print ("Signal caught, exiting!\n");
    enable_thread = 0;
    stop();
    while (active_thread) usleep(1000);
    do_exit = 1;
    if (_running) _running = false;
    if (running) running = 0;
    hpsdrsim_stop_threads ();
    resamp_crcf_destroy(mcb.q);
}

// Set the AD6620s RCF filters from tapCount coefficients stored in the filterCoeff array
int SetAD6620Filter(int* filterCoeff,unsigned int tapCount)
{
    unsigned int a;
    // Step through each of the tapCount addresses which start at zero
    for (a = 0; a< tapCount; a++) {
        // The coefficient values need to be converted to twos compliment format if they are negative
        // A test case for this would be that the number -1272 gets converted to 0x08FBFFFF
        unsigned long long dout = ConvertToTwosCompliment(filterCoeff[a]);
        // Now set the memory in the AD6620
        if (SetAD6620Register(a,dout) != 1) return -1;
    }
    return 1;
}

// Set the capture bandwidth of the SDR-14
int SetBandwidth(unsigned long bandwidth)
{
    unsigned int cic2Rate,cic2Scale,cic5Rate,cic5Scale,rcfRate,rcfScale,rcfTaps;
    int filterCoeffs[256];

    if (bandwidth == 50000) {
        cic2Rate = 8;
        cic2Scale = 4;
        cic5Rate = 30;
        cic5Scale = 20;
        rcfRate = 5;
        rcfScale = 0;
        rcfTaps = 256;
        int filt50[256] = {-115,572,286,894,855,1164,1031,971,601,278,-156,-424,-607,-546,-363,-34,269,511,560,445,158,-174,-474,-610,-558,-300,62,431,663,689,471,88,-355,-694,-821,-668,-282,228,686,938,880,519,-44,-622,-1023,-1093,-794,-202,491,1058,1291,1097,511,-282,-1025,-1457,-1414,-880,-14,908,1568,1727,1299,401,-689,-1602,-2017,-1759,-880,353,1536,2258,2242,1452,113,-1343,-2423,-2728,-2110,-724,997,2479,3195,2849,1494,-465,-2390,-3612,-3661,-2442,-292,2109,3946,4541,3597,1330,-1574,-4154,-5488,-5011,-2740,688,4177,6519,6787,4690,724,-3918,-7684,-9155,-7544,-3033,3174,9131,12703,12256,7260,-1366,-11320,-19399,-22305,-17564,-4306,16319,41233,66039,85987,97095,97095,85987,66039,41233,16319,-4306,-17564,-22305,-19399,-11320,-1366,7260,12256,12703,9131,3174,-3033,-7544,-9155,-7684,-3918,724,4690,6787,6519,4177,688,-2740,-5011,-5488,-4154,-1574,1330,3597,4541,3946,2109,-292,-2442,-3661,-3612,-2390,-465,1494,2849,3195,2479,997,-724,-2110,-2728,-2423,-1343,113,1452,2242,2258,1536,353,-880,-1759,-2017,-1602,-689,401,1299,1727,1568,908,-14,-880,-1414,-1457,-1025,-282,511,1097,1291,1058,491,-202,-794,-1093,-1023,-622,-44,519,880,938,686,228,-282,-668,-821,-694,-355,88,471,689,663,431,62,-300,-558,-610,-474,-174,158,445,560,511,269,-34,-363,-546,-607,-424,-156,278,601,971,1031,1164,855,894,286,572,-115};
        size_t size_in_bytes = rcfTaps * sizeof(int);
        memcpy(filterCoeffs,filt50,size_in_bytes);
        // Below calculated with 66666666.6667/1200.0
        //sampleRate = 55555;
    } else if (bandwidth == 100000) {
        cic2Rate = 5;
        cic2Scale = 3;
        cic5Rate = 30;
        cic5Scale = 20;
        rcfRate = 4;
        rcfScale = 0;
        rcfTaps = 256;
        int filt100[256] = {111,108,197,150,69,-150,-405,-679,-847,-866,-689,-380,-23,244,337,222,-27,-290,-429,-371,-131,176,402,426,227,-107,-406,-515,-361,-8,374,586,504,149,-309,-639,-655,-326,199,657,801,531,-39,-630,-929,-758,-173,545,1024,994,435,-394,-1071,-1227,-742,170,1054,1438,1085,131,-956,-1608,-1449,-509,765,1716,1818,959,-467,-1739,-2170,-1474,53,1653,2479,2038,481,-1436,-2717,-2634,-1140,1064,2851,3238,1922,-515,-2847,-3823,-2822,-237,2665,4354,3835,1218,-2256,-4793,-4956,-2465,1563,5093,6183,4030,-501,-5191,-7525,-6007,-1062,5000,9018,8574,3371,-4365,-10755,-12118,-6953,2955,12982,17627,13228,184,-16444,-28453,-27679,-9595,24220,65916,103759,126224,126224,103759,65916,24220,-9595,-27679,-28453,-16444,184,13228,17627,12982,2955,-6953,-12118,-10755,-4365,3371,8574,9018,5000,-1062,-6007,-7525,-5191,-501,4030,6183,5093,1563,-2465,-4956,-4793,-2256,1218,3835,4354,2665,-237,-2822,-3823,-2847,-515,1922,3238,2851,1064,-1140,-2634,-2717,-1436,481,2038,2479,1653,53,-1474,-2170,-1739,-467,959,1818,1716,765,-509,-1449,-1608,-956,131,1085,1438,1054,170,-742,-1227,-1071,-394,435,994,1024,545,-173,-758,-929,-630,-39,531,801,657,199,-326,-655,-639,-309,149,504,586,374,-8,-361,-515,-406,-107,227,426,402,176,-131,-371,-429,-290,-27,222,337,244,-23,-380,-689,-866,-847,-679,-405,-150,69,150,197,108,111};
        size_t size_in_bytes = rcfTaps * sizeof(int);
        memcpy(filterCoeffs,filt100,size_in_bytes);
        // Below calculated with 66666666.6667/600.0
        //sampleRate = 111111;
    } else if (bandwidth == 150000) {
        cic2Rate = 5;
        cic2Scale = 3;
        cic5Rate = 28;
        cic5Scale = 20;
        rcfRate = 3;
        rcfScale = 0;
        rcfTaps = 256;
        int filt150[256] = {80,-1272,-1501,-2506,-1995,-1237,225,985,1065,218,-585,-897,-355,403,811,432,-300,-777,-504,221,771,585,-145,-776,-676,61,783,777,36,-784,-884,-149,774,996,279,-749,-1108,-427,706,1217,591,-641,-1321,-772,553,1415,968,-437,-1496,-1177,293,1560,1398,-119,-1602,-1628,-87,1620,1863,325,-1608,-2101,-598,1562,2339,905,-1478,-2571,-1247,1350,2793,1624,-1175,-3001,-2036,947,3190,2484,-660,-3353,-2967,307,3484,3486,118,-3575,-4042,-626,3619,4637,1227,-3606,-5273,-1936,3523,5956,2775,-3354,-6694,-3770,3080,7501,4965,-2669,-8398,-6426,2076,9422,8256,-1227,-10635,-10642,-11,12151,13929,1901,-14192,-18856,-5052,17262,27312,11251,-22720,-45866,-28628,35237,119172,178193,178193,119172,35237,-28628,-45866,-22720,11251,27312,17262,-5052,-18856,-14192,1901,13929,12151,-11,-10642,-10635,-1227,8256,9422,2076,-6426,-8398,-2669,4965,7501,3080,-3770,-6694,-3354,2775,5956,3523,-1936,-5273,-3606,1227,4637,3619,-626,-4042,-3575,118,3486,3484,307,-2967,-3353,-660,2484,3190,947,-2036,-3001,-1175,1624,2793,1350,-1247,-2571,-1478,905,2339,1562,-598,-2101,-1608,325,1863,1620,-87,-1628,-1602,-119,1398,1560,293,-1177,-1496,-437,968,1415,553,-772,-1321,-641,591,1217,706,-427,-1108,-749,279,996,774,-149,-884,-784,36,777,783,61,-676,-776,-145,585,771,221,-504,-777,-300,432,811,403,-355,-897,-585,218,1065,985,225,-1237,-1995,-2506,-1501,-1272,80};
        size_t size_in_bytes = rcfTaps * sizeof(int);
        memcpy(filterCoeffs,filt150,size_in_bytes);
        // Below calculated with 66666666.6667/420.0
        //sampleRate = 158730;
    } else if (bandwidth == 190000) {
        cic2Rate = 10;
        cic2Scale = 5;
        cic5Rate = 17;
        cic5Scale = 16;
        rcfRate = 2;
        rcfScale = 0;
        rcfTaps = 256;
        int filt190[256] = {3447,-2833,-1187,-2836,784,-262,-551,-1155,655,341,-484,-824,627,573,-493,-814,597,739,-495,-905,556,899,-472,-1036,498,1065,-419,-1189,417,1238,-335,-1355,308,1416,-219,-1527,167,1595,-66,-1700,-8,1771,124,-1870,-221,1939,355,-2031,-475,2096,628,-2177,-772,2235,946,-2304,-1114,2350,1310,-2404,-1504,2435,1723,-2470,-1944,2482,2188,-2494,-2436,2483,2707,-2468,-2984,2429,3285,-2381,-3593,2308,3925,-2221,-4268,2107,4635,-1974,-5017,1808,5426,-1618,-5854,1390,6314,-1128,-6799,819,7322,-465,-7882,50,8492,429,-9154,-993,9888,1653,-10702,-2438,11626,3378,-12684,-4524,13926,5947,-15415,-7763,17256,10157,-19614,-13468,22786,18349,-27322,-26280,34392,41334,-46742,-79378,69315,263870,263870,69315,-79378,-46742,41334,34392,-26280,-27322,18349,22786,-13468,-19614,10157,17256,-7763,-15415,5947,13926,-4524,-12684,3378,11626,-2438,-10702,1653,9888,-993,-9154,429,8492,50,-7882,-465,7322,819,-6799,-1128,6314,1390,-5854,-1618,5426,1808,-5017,-1974,4635,2107,-4268,-2221,3925,2308,-3593,-2381,3285,2429,-2984,-2468,2707,2483,-2436,-2494,2188,2482,-1944,-2470,1723,2435,-1504,-2404,1310,2350,-1114,-2304,946,2235,-772,-2177,628,2096,-475,-2031,355,1939,-221,-1870,124,1771,-8,-1700,-66,1595,167,-1527,-219,1416,308,-1355,-335,1238,417,-1189,-419,1065,498,-1036,-472,899,556,-905,-495,739,597,-814,-493,573,627,-824,-484,341,655,-1155,-551,-262,784,-2836,-1187,-2833,3447};
        size_t size_in_bytes = rcfTaps * sizeof(int);
        memcpy(filterCoeffs,filt190,size_in_bytes);
        // Below calculated with 66666666.6667/340.0
        //sampleRate = 196078;
    } else return -2;

    // Set these registers
    if (SetAD6620Register(ADR_CIC2M,cic2Rate - 1) != 1) return -1;
    if (SetAD6620Register(ADR_CIC2SCALE,cic2Scale) != 1) return -1;
    if (SetAD6620Register(ADR_CIC5M,cic5Rate - 1) != 1) return -1;
    if (SetAD6620Register(ADR_CIC5SCALE,cic5Scale) != 1) return -1;
    if (SetAD6620Register(ADR_RCFM,rcfRate - 1) != 1) return -1;
    if (SetAD6620Register(ADR_RCFOFFSET,rcfScale) != 1) return -1;
    if (SetAD6620Register(ADR_TAPS,rcfTaps - 1) != 1) return -1;
    // Set the AD6620s RCF filter
    if (SetAD6620Filter(filterCoeffs,rcfTaps) != 1) return -1;

    return 1;
}

int SetRFGain( const double gain )
{
    unsigned char atten[] = { 0x06, 0x00, 0x38, 0x00, 0x00, 0x00 };

    if ( gain <= -20 )
        atten[sizeof(atten)-1] = 0xE2;
    else if ( gain <= -10 )
        atten[sizeof(atten)-1] = 0xEC;
    else if ( gain < 0 )
        atten[sizeof(atten)-1] = 0xF6;
    else // +10 dB
        atten[sizeof(atten)-1] = 0x00;

    resp_transaction( atten, sizeof(atten) );
    return 1;
}

int SetIFGain( const double gain)
{
    unsigned char atten[] = { 0x06, 0x00, 0x40, 0x00, 0x00, 0x00};
    atten[sizeof(atten)-1] = (unsigned char)gain;

    resp_transaction( atten, sizeof(atten) );
    return 1;
}

int setFrequency(const double frequency)
{
    //t_print("setFrequency: %f kHz\n", frequency/1000);

    uint32_t u32_freq = (uint32_t)frequency;

    // SDR14 4.2.3 Receiver Frequency
    unsigned char tune[] = { 0x0A, 0x00, 0x20, 0x00, 0x00, 0xb0, 0x19, 0x6d, 0x00, 0x01 };

    tune[sizeof(tune)-5] = u32_freq >>  0;
    tune[sizeof(tune)-4] = u32_freq >>  8;
    tune[sizeof(tune)-3] = u32_freq >> 16;
    tune[sizeof(tune)-2] = u32_freq >> 24;
    tune[sizeof(tune)-1] = 0;

    resp_transaction( tune, sizeof(tune) );
    return 1;
}

size_t read_bytes( unsigned char *data_in, size_t size, bool run )
{
    size_t nbytes = 0;

    while ( nbytes < size && run ) {
        int nread=0;

        nread = ftdi_read_data( ftdi, (unsigned char *)&data_in[nbytes], 1);
        if (nread < 0 ) t_print("nread %d != 1\n", nread);

        if ( nread == 0 )
            continue;

        if ( nread < 0 )
            break;

        nbytes++;
    }
    return nbytes;
}

bool ack()
{
    unsigned char ack[] = { 0x03, 0x60, 0x00 };
    return ftdi_write_data(ftdi, ack, 3);
}

void *usb_read_task_func (void *arg)
{
    unsigned char data[1024*10];
    uint64_t time_us, last_time_us;

    t_print("Starting usb_read_task_func()\n");
    while ( _run_usb_read_task ) {
        size_t nbytes;
        nbytes = read_bytes( data, 2, _run_usb_read_task );
        if ( nbytes != 2 )
            continue;

        size_t length = ((data[1] << 8) | data[0]) & 0x1fff;
        if ( 0 == length ) // SDR-IQ 5.4.1 Output Data Item 0
            length = 1024*8 + 2;

        if ( length <= 2 )
            continue;

        length -= 2; // subtract header

        if ( length > sizeof(data) - 2 ) {
            _run_usb_read_task = false;
            continue;
        }

        nbytes = read_bytes( data + 2, length, _run_usb_read_task );

        if ( nbytes != length ) {
            continue;
        }

        if ( 1024*8 == length ) {
            time_us = get_posix_clock_time_us();
            long long milliseconds = (time_us - last_time_us) / 1000;

            // SDR-14 needs ack
            if(milliseconds > 2750) {
                ack();
                last_time_us = time_us;
            }

            size_t num_samples = length / 4;
            //t_print("num_samples:%ld\n", num_samples);

#define SCALE (20.0f)

            int16_t *sample = (int16_t *)(data + 2);

            for ( size_t i = 0; i < num_samples; i++ ) {
                //t_print("i:%d I:%f Q:%f ", i, *(sample+0) * SCALE_16, *(sample+1) * SCALE_16 );
                iqBuffer[i] = ((float)*(sample+0) * SCALE) + (((float)*(sample+1) * SCALE) * _Complex_I);
                // offset to the next I+Q sample
                sample += 2;
            }

            pthread_mutex_lock (&iqready_lock);
            rcvr_flags |= 1;
            //rcvr_flags |= rcb->rcvr_mask;
            pthread_cond_broadcast (&iqready_cond);
            pthread_mutex_unlock (&iqready_lock);
        } else {
            // copy response & signal transaction
            pthread_mutex_lock (&resp_lock);
            memset(resp_data, 0, 64);
            memcpy(resp_data, data, length + 2);
            resp_size = length + 2;
            pthread_cond_broadcast (&resp_cond);
            pthread_mutex_unlock (&resp_lock);
        }
    }
    return NULL;
}

int reset_usb_device(unsigned int vendor_id, unsigned int product_id)
{
    FILE *fp; 
    char line[64];
    int rc;

    // Construct the lsusb command to list all USB devices
    fp = popen("lsusb", "r");
    if (fp == NULL) {
        perror("Failed to run lsusb command");
        return -1;
    }    

    // Read the output of lsusb line by line
    while (fgets(line, sizeof(line), fp) != NULL) {
        int bus, device;
        unsigned int id_vendor, id_product;

        // Parse the line for Bus, Device, and ID
        // Expected format: "Bus 001 Device 001: ID 1d6b:0002 ..."
        if (sscanf(line, "Bus %d Device %d: ID %x:%x", &bus, &device, &id_vendor, &id_product) == 4) { 
            // Check if the IDs match the target device
            if (id_vendor == vendor_id && id_product == product_id) {
                sprintf(line, "/dev/bus/usb/%03d/%03d", bus, device);
                printf("Found %s\n", line);
                pclose(fp);
                fd = open(line, O_WRONLY);
                if (fd < 0) { 
                    perror("Error opening output file");
                    return -1;
                }

                printf("Resetting USB device %s\n", line);
                rc = ioctl(fd, USBDEVFS_RESET, 0);
                if (rc < 0) { 
                    perror("Error in ioctl");
                    close(fd);
                    return -1;
                }

                return 1;
            }
        }
    }

    pclose(fp);
    return -1; // Device not found
}

int find_usb_device(unsigned int vendor_id, unsigned int product_id)
{
    FILE *fp;
    char line[64];

    // Construct the lsusb command to list all USB devices
    fp = popen("lsusb", "r");
    if (fp == NULL) {
        t_perror("Failed to run lsusb command");
        return -1;
    }

    // Read the output of lsusb line by line
    while (fgets(line, sizeof(line), fp) != NULL) {
        int bus, device;
        unsigned int id_vendor, id_product;

        // Parse the line for Bus, Device, and ID
        // Expected format: "Bus 001 Device 001: ID 1d6b:0002 ..."
        if (sscanf(line, "Bus %d Device %d: ID %x:%x", &bus, &device, &id_vendor, &id_product) == 4) {
            // Check if the IDs match the target device
            if (id_vendor == vendor_id && id_product == product_id) {
                pclose(fp);
                return 1;
            }
        }
    }

    pclose(fp);
    return -1; // Device not found
}

int initSDR()
{
    int ret;
    unsigned int vid, pid;
    unsigned char response[64];

    if ((ret = find_usb_device(0x0403, 0xf728)) == 1) { // sdr-14
        vid = 0x0403;
        pid = 0xf728;
    } else if ((ret = find_usb_device(0x1234, 0x0009)) == 1) { // sdr-iq
        vid = 0x1234;
        pid = 0x0009;
    } else
        return -1;

    if ((ftdi = ftdi_new()) == 0) {
        t_print("ftdi_new failed\n");
        return -1;
    }

    if ((ret = ftdi_usb_open(ftdi, vid, pid)) < 0) {
        t_print("unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        ftdi_free(ftdi);
        return -1;
    }

    _run_usb_read_task = true;
    if ((ret = pthread_create (&usb_read_task, NULL, usb_read_task_func, NULL))) {
        t_print ("pthread_create failed on usb_read_task: ret=%d\n", ret);
        return (-1);
    }
    pthread_detach(usb_read_task);

    unsigned char name[] = { 0x04, 0x20, 0x01, 0x00 }; // SDR14 4.1.1 Target Name
    if ( transaction( name, sizeof(name), response ) )
        t_print("Using RFSPACE %s ", &response[sizeof(name)]);

    unsigned char sern[] = { 0x04, 0x20, 0x02, 0x00 }; // SDR14 4.1.2 Target Serial Number
    if ( transaction( sern, sizeof(sern), response ) )
        printf("SN %s\n", &response[sizeof(sern)]);

    // Set the SDR bandwidth
    if (SetBandwidth(190000) != 1) {
        t_print("Unable to set the SDR Bandwidth\n");
        return -1;
    }

    // Set the SDR frequency
    if (setFrequency(900000) != 1) {
        t_print("Unable to set the SDR Bandwidth\n");
        return -1;
    }

    // Set the SDR RF gain
    if (SetRFGain(10) != 1) {
        t_print("Unable to set the SDR RF Gain\n");
        return -1;
    }

    // default to 190000
    mcb.q = resamp_crcf_create(0.979202154f,13,0.45f,60.0f,64);
    resamp_crcf_print(mcb.q);

    return 1;
}

char *
time_stamp ()
{
    char *timestamp = (char *) malloc (sizeof (char) * 16);
    time_t ltime = time (NULL);
    struct tm *tm;

    tm = localtime (&ltime);
    sprintf (timestamp, "%02d:%02d:%02d", tm->tm_hour, tm->tm_min, tm->tm_sec);
    return timestamp;
}

void
load_packet (struct rcvr_cb *rcb)
{
    int b, i, j, k, copy_total = (mcb.active_num_rcvrs - 1) + num_copy_rcvrs;

    // if we need to copy a receiver we'll choose the last active 'real' one
    bool do_copy = ((num_copy_rcvrs > 0)
                    && (rcb->rcvr_num == mcb.active_num_rcvrs - 1));
    int offset1 = (num_copy_rcvrs > 0) ? frame_offset1[copy_total] : mcb.frame_offset1;
    int offset2 = (num_copy_rcvrs > 0) ? frame_offset2[copy_total] : mcb.frame_offset2;
    int offsetx;
    float complex *out_buf = &rcb->iqSamples[rcb->iqSample_offset];
    int IQData;

    i = 0;
    k = 0;
    b = 16;

    pthread_mutex_lock (&send_lock);

    // insert data in lower and upper bank for each of the receivers
    while (k++ < 2) {
        offsetx = (i == 0) ? offset1 : offset2;
        while (b < offsetx) {
            for (j = 0; j < mcb.active_num_rcvrs + num_copy_rcvrs; j++) {
                if ((j == rcb->rcvr_num) || (do_copy && copy_rcvr[j] == j)) {
                    IQData = (int)cimagf(out_buf[i]);
                    payload[b++] = IQData >> 16;
                    payload[b++] = IQData >> 8;
                    payload[b++] = IQData & 0xff;
                    IQData = (int)crealf(out_buf[i]);
                    payload[b++] = IQData >> 16;
                    payload[b++] = IQData >> 8;
                    payload[b++] = IQData & 0xff;

                    if (do_copy) {
                        if (j == copy_total)
                            i += 1;
                    } else {
                        i += 1;
                    }
                } else {
                    b += 6;
                }
            }
            b += 2;                // skip mic data
        }
        b = 528;
    }

    send_flags |= rcb->rcvr_mask;
    pthread_cond_broadcast (&send_cond);
    pthread_mutex_unlock (&send_lock);

    pthread_mutex_lock (&done_send_lock);
    while (send_flags & rcb->rcvr_mask) {
        pthread_cond_wait (&done_send_cond, &done_send_lock);
    }
    pthread_mutex_unlock (&done_send_lock);
}

void *
hpsdrsim_sendiq_thr_func (void *arg)
{
    int samps_packet;
    struct rcvr_cb *rcb = (struct rcvr_cb *) arg;

    rcb->iqSample_offset = rcb->iqSamples_remaining = 0;

    //t_print("ENTERING hpsdrsim_sendiq_thr_func() rcvr %d...\n", rcb->rcvr_num+1);
    while (!do_exit) {
        if (!running) rcb->iqSample_offset = rcb->iqSamples_remaining = 0;
        samps_packet = (num_copy_rcvrs > 0)
                       ? nsamps_packet[(mcb.active_num_rcvrs - 1) + num_copy_rcvrs]
                       : mcb.nsamps_packet;

        pthread_mutex_lock (&iqready_lock);
        while (!(rcvr_flags & rcb->rcvr_mask)) {
            pthread_cond_wait (&iqready_cond, &iqready_lock);
        }
        rcvr_flags &= ~rcb->rcvr_mask;
        pthread_mutex_unlock (&iqready_lock);

        // can happen when switching between rcvr numbers
        if (rcb->iqSamples_remaining < 0)
            rcb->iqSamples_remaining = 0;

        // resample starting at any remaining offset
        unsigned int ny;
        resamp_crcf_execute_block(mcb.q, &iqBuffer[0], (RTL_READ_COUNT / 4), &rcb->iqSamples[rcb->iqSamples_remaining], &ny);
        rcb->iqSamples_remaining += ny;

        while (rcb->iqSamples_remaining > samps_packet) {
            load_packet (rcb);
            rcb->iqSamples_remaining -= samps_packet;
            rcb->iqSample_offset += samps_packet;
        }

        // move remaining samples to beginning of buffer
        if ((rcb->iqSample_offset > 0) && (rcb->iqSamples_remaining > 0)) {
            memcpy (&(rcb->iqSamples[0]),
                    &(rcb->iqSamples[rcb->iqSample_offset]),
                    rcb->iqSamples_remaining * sizeof (float complex));
            rcb->iqSample_offset = 0;
        }
    }

    pthread_exit (NULL);
    t_print("EXITING hpsdrsim_sendiq_thr_func() rcvr_mask%d...\n", rcb->rcvr_mask);
}

void
hpsdrsim_stop_threads ()
{
    running = 0;
    pthread_cancel (usb_read_task);
#if 0   //Segmentation fault?
    for (i = 0; i < mcb.total_num_rcvrs; i++) {
        pthread_cancel (mcb.rcb[i].hpsdrsim_sendiq_thr);
    }

    pthread_mutex_lock (&send_lock);
    send_flags = mcb.rcvrs_mask;
    pthread_cond_broadcast (&send_cond);
    pthread_mutex_unlock (&send_lock);

    pthread_mutex_lock (&iqready_lock);
    rcvr_flags = mcb.rcvrs_mask;
    pthread_cond_broadcast (&iqready_cond);
    pthread_mutex_unlock (&iqready_lock);
#endif
}

void
format_payload (void)
{
    int i;
    u_char hpsdr_header[8] = { 0xEF, 0xFE, 1, 6, 0, 0, 0, 0 };
    u_char proto_header[8] =
    { 0x7f, 0x7f, 0x7f, 0, 0x1e, 0, 0, HERMES_FW_VER };

    for (i = 0; i < HPSDR_FRAME_LEN; i++)
        payload[i] = 0;

    for (i = 0; i < 8; i++)
        payload[i] = hpsdr_header[i];

    for (i = 8; i < 16; i++)
        payload[i] = proto_header[i - 8];

    for (i = 520; i < 528; i++)
        payload[i] = proto_header[i - 520];
}

int new_protocol_running()
{
    return 0;
}

int main (int argc, char *argv[])
{
    int r, count = 0;
    pthread_t thread;
    uint8_t id[4] = { 0xef, 0xfe, 1, 6 };
    struct sockaddr_in addr_udp;
    struct sockaddr_in addr_from;
    socklen_t lenaddr;
    struct timeval tv;
    int yes = 1;
    uint32_t i, last_seqnum = 0xffffffff, seqnum;  // sequence number of received packet
    int udp_retries = 0;
    int bytes_read;
    uint32_t code;
    uint32_t *code0 = (uint32_t *) buffer;  // fast access to code of first buffer
    const int MAC1 = 0x00;
    const int MAC2 = 0x1C;
    const int MAC3 = 0xC0;
    const int MAC4 = 0xA2;
    int MAC5 = 0x10;
    const int MAC6 = 0xDD;  // P1
    const int MAC6N = 0xDD; // P2
    int OLDDEVICE = ODEV_HERMES;
    int NEWDEVICE = NDEV_HERMES;

    struct sigaction sigact;

    mcb.output_rate = 50000;

    // Initialize per receiver config settings
    for (i = 0; i < MAX_RCVRS; i++) {
        mcb.agc_mode[i] = 0;
        mcb.last_agc_mode[i] = 0;
        mcb.gain[i] = 0;
        mcb.gain_mode[i] = 1;
        mcb.last_gain[i] = 0;

        copy_rcvr[i] = -1;
    }

    sigact.sa_handler = sdr14_sighandler;
    sigemptyset (&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigaction (SIGINT, &sigact, NULL);
    sigaction (SIGTERM, &sigact, NULL);
    sigaction (SIGQUIT, &sigact, NULL);
    sigaction (SIGPIPE, &sigact, NULL);

    format_payload ();

    pthread_mutex_init (&iqready_lock, NULL);
    pthread_cond_init (&iqready_cond, NULL);
    pthread_mutex_init (&send_lock, NULL);
    pthread_cond_init (&send_cond, NULL);
    pthread_mutex_init (&done_send_lock, NULL);
    pthread_cond_init (&done_send_cond, NULL);

    // enable the 1st rcvr until we get the active count
    mcb.rcb[0].rcvr_mask = 1;
    mcb.active_num_rcvrs = 1;
    mcb.total_num_rcvrs = 1;
    mcb.rcvrs_mask = 1;
    mcb.nsamps_packet = nsamps_packet[0];
    mcb.frame_offset1 = frame_offset1[0];
    mcb.frame_offset2 = frame_offset2[0];

    if ((r = initSDR()) != 1) {
        t_print("RFSpace USB device not found!\n");
        return (r);
    }

    for (i = 0; i < mcb.total_num_rcvrs; i++) {
        mcb.rcb[i].mcb = &mcb;
        mcb.rcb[i].new_freq = 0;
        mcb.rcb[i].output_rate = 50000;

        mcb.rcb[i].hpsdrsim_sendiq_thr = hpsdrsim_sendiq_thr[i];
        mcb.rcb[i].rcvr_num = i;

        if ((r = pthread_create (&hpsdrsim_sendiq_thr[i], NULL,
                                 hpsdrsim_sendiq_thr_func, &mcb.rcb[i]))) {
            t_print ("pthread_create failed on hpsdrsim_sendiq_thr: r=%d\n", r);
            return (r);
        }
    }

    if ((sock_udp = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        t_perror("socket");
        return EXIT_FAILURE;
    }

    setsockopt(sock_udp, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
    setsockopt(sock_udp, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
    tv.tv_sec = 0;
    tv.tv_usec = 1000;
    setsockopt(sock_udp, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));
    memset(&addr_udp, 0, sizeof(addr_udp));
    addr_udp.sin_family = AF_INET;
    addr_udp.sin_addr.s_addr = htonl(INADDR_ANY);
    addr_udp.sin_port = htons(1024);

    if (bind(sock_udp, (struct sockaddr *)&addr_udp, sizeof(addr_udp)) < 0) {
        t_perror("bind");
        return EXIT_FAILURE;
    }
    t_print("Waiting on Discovery...\n");

    while (!do_exit) {
        memcpy(buffer, id, 4);
        count++;
        lenaddr = sizeof(addr_from);
        bytes_read = recvfrom(sock_udp, buffer, HPSDR_FRAME_LEN, 0, (struct sockaddr *)&addr_from, &lenaddr);

        if (bytes_read > 0) {
            udp_retries = 0;
        } else {
            udp_retries++;
        }

        if (bytes_read < 0 && errno != EAGAIN) {
            t_perror("recvfrom");
            //return EXIT_FAILURE;
            continue;
        }

        // If nothing has arrived via UDP for some time, try to open TCP connection.
        // "for some time" means 10 subsequent un-successful UDP rcvmmsg() calls
        if (udp_retries > 10 && oldnew != 2) {
            //t_print("If nothing has arrived via UDP for some time, try to open TCP connection.\n");

            // This avoids firing accept() too often if it constantly fails
            udp_retries = 0;
        }

        if (bytes_read <= 0) {
            continue;
        }

        count = 0;
        code = *code0;

        switch (code) {
        // PC to SDR transmission via process_ep2
        case 0x0201feef:

            // processing an invalid packet is too dangerous -- skip it!
            if (bytes_read != HPSDR_FRAME_LEN) {
                t_print("InvalidLength: RvcMsg Code=0x%08x Len=%d\n", code, (int)bytes_read);
                break;
            }

            // sequence number check
            seqnum = ((buffer[4] & 0xFF) << 24) + ((buffer[5] & 0xFF) << 16) + ((buffer[6] & 0xFF) << 8) + (buffer[7] & 0xFF);

            if (seqnum != last_seqnum + 1 && last_seqnum != 0xffffffff) {
                t_print("SEQ ERROR: last %ld, recvd %ld\n", (long)last_seqnum, (long)seqnum);
            }

            last_seqnum = seqnum;
            //process_ep2(buffer + 11);
            process_ep2(buffer + 523);

            break;

        // respond to an incoming Metis detection request
        case 0x0002feef:
            if (oldnew == 2) {
                t_print("OldProtocol detection request from %s IGNORED.\n", inet_ntoa(addr_from.sin_addr));
                break;  // Swallow P1 detection requests
            }

            t_print( "Respond to an incoming Metis detection request from %s, code: 0x%08x\n", inet_ntoa(addr_from.sin_addr),
                     code);

            // processing an invalid packet is too dangerous -- skip it!
            if (bytes_read != 63) {
                t_print("InvalidLength: RvcMsg Code=0x%08x Len=%d\n", code, (int)bytes_read);
                break;
            }

            memset(buffer, 0, 60);
            buffer[0] = 0xEF;
            buffer[1] = 0xFE;
            buffer[2] = 0x02;
            buffer[3] = MAC1; // buffer[3:8] is MAC address
            buffer[4] = MAC2;
            buffer[5] = MAC3;
            buffer[6] = MAC4;
            buffer[7] = MAC5; // specifies type of radio
            buffer[8] = MAC6; // encodes old protocol
            buffer[2] = 2;

            if (new_protocol_running()) {
                buffer[2] = 3;
            }

            buffer[9] = HERMES_FW_VER; // software version
            buffer[10] = OLDDEVICE;

            sendto(sock_udp, buffer, 60, 0, (struct sockaddr *)&addr_from, sizeof(addr_from));

            break;

        // stop the SDR to PC transmission via handler_ep6, another start will stop and
        // start the handler_ep6 thread
        case 0x0004feef:
            t_print( "STOP the transmission via handler_ep6, code: 0x%08x\n", code);
            stop();


            // processing an invalid packet is too dangerous -- skip it!
            if (bytes_read != 64) {
                t_print("InvalidLength: RvcMsg Code=0x%08x Len=%d\n", code, bytes_read);
                break;
            }
            enable_thread = 0;

            while (active_thread) usleep(1000);

            break;

        case 0x0104feef:
        case 0x0204feef:
        case 0x0304feef:
            if (new_protocol_running()) {
                t_print("OldProtocol START command received but NewProtocol radio already running!\n");
                break;
            }

            // processing an invalid packet is too dangerous -- skip it!
            if (bytes_read != 64) {
                t_print("InvalidLength: RvcMsg Code=0x%08x Len=%d\n", code, bytes_read);
                break;
            }

            t_print( "START the PC-to-SDR handler thread, code: 0x%08x\n", code);
            enable_thread = 0;

            start();

            // thread is disabled so unblock send
            pthread_mutex_lock (&send_lock);
            send_flags = mcb.rcvrs_mask;
            pthread_cond_broadcast (&send_cond);
            pthread_mutex_unlock (&send_lock);

            while (active_thread) usleep(1000);

            memset(&addr_old, 0, sizeof(addr_old));
            addr_old.sin_family = AF_INET;
            addr_old.sin_addr.s_addr = addr_from.sin_addr.s_addr;
            addr_old.sin_port = addr_from.sin_port;
            enable_thread = 1;
            active_thread = 1;

            if (pthread_create(&thread, NULL, handler_ep6, NULL) < 0) {
                t_perror("create old protocol thread");
                return EXIT_FAILURE;
            }

            pthread_detach(thread);
            break;

        default:

            /*
             * Here we have to handle the following "non standard" cases:
             * NewProtocol "Discovery" packet   60 bytes starting with 00 00 00 00 02
             * NewProtocol "General"   packet   60 bytes starting with 00 00 00 00 00
             *                                  ==> this starts NewProtocol radio
             */
            if (code == 0 && buffer[4] == 0x02) {
                if (oldnew == 1) {
                    t_print("NewProtocol discovery packet from %s IGNORED.\n", inet_ntoa(addr_from.sin_addr));
                    break;
                }

                t_print("NewProtocol discovery packet received from %s\n", inet_ntoa(addr_from.sin_addr));
                // prepare response
                memset(buffer, 0, 60);
                buffer [4] = 0x02 + new_protocol_running();
                buffer [5] = MAC1;
                buffer[ 6] = MAC2;
                buffer[ 7] = MAC3;
                buffer[ 8] = MAC4;
                buffer[ 9] = MAC5; // specifies type of radio
                buffer[10] = MAC6N; // encodes new protocol
                buffer[11] = NEWDEVICE;
                buffer[12] = 38;
                buffer[13] = 19;
                buffer[20] = 2;
                buffer[21] = 1;
                buffer[22] = 3;

                sendto(sock_udp, buffer, 60, 0, (struct sockaddr *)&addr_from, sizeof(addr_from));
                break;
            }

            if (bytes_read == 60 && buffer[4] == 0x00) {
                if (oldnew == 1) {
                    t_print("NewProtocol General packet IGNORED.\n");
                    break;
                }

                // handle "general packet" of the new protocol
                memset(&addr_new, 0, sizeof(addr_new));
                addr_new.sin_family = AF_INET;
                addr_new.sin_addr.s_addr = addr_from.sin_addr.s_addr;
                addr_new.sin_port = addr_from.sin_port;
                //new_protocol_general_packet(buffer);
                break;
            } else {
                t_print("Invalid packet (len=%d) detected: ", bytes_read);

                for (i = 0; i < 16; i++) {
                    t_print("%02x ", buffer[i]);
                }

                t_print("\n");
            }

            break;
        }
    }

    close(sock_udp);

    return EXIT_SUCCESS;
}

void process_ep2(uint8_t *frame)
{
    int i = 0, j = 0;
    int c, offset;
    int freq, num_rcvrs;
    static int last_rfgain = 0;
    u_char C0_1, C0_2;

    // this is really attenuation
    if ((frame[1] == 0x40) && (last_rfgain != (frame[4] & 0x1F))) {
        SetRFGain(-1 * (frame[4] & 0x1F));
        last_rfgain = frame[4] & 0x1F;
    }

#if 0
    for (i = 0; i < HPSDR_FRAME_LEN; i++) {
        t_print ("%4d:%2x ", i, buffer[i]);

        if (!((i + 1) % 8))
            t_print ("\n");
    }
#endif

    // REMEMBER, 2 USB packets of data here (audio, C0, ...)
    C0_1 = buffer[11] & 0xFE;
    C0_2 = buffer[523] & 0xFE;

    for (c = 0; c < 2; c++) {
        freq = 0;

        if ((C0_1 >= 4 && c == 0)
            && (C0_1 <= (4 + ((mcb.total_num_rcvrs - 1) * 2)))) {
            offset = 0;
            freq = 1;
            j = (C0_1 - 4) / 2;
        } else if ((C0_2 >= 4 && c == 1)
                   && (C0_2 <= (4 + ((mcb.total_num_rcvrs - 1) * 2)))) {
            offset = 512;
            freq = 1;
            j = (C0_2 - 4) / 2;
        }

        if (freq) {
            freq = (int) buffer[12 + offset] << 24 | (int) buffer[13 + offset]
                   << 16 | (int) buffer[14 + offset] << 8 | (int) buffer[15 + offset];

            if (last_freq[j] != freq && 0 != freq) {
                if (j < mcb.total_num_rcvrs) {
                    mcb.rcb[j].new_freq = freq;
                }
                last_freq[j] = freq;
                if (setFrequency((unsigned long) freq) == -1)
                    t_print("Unable to set the SDR frequency\n");
            }
        }

        if ((C0_1 == 0x00) || (C0_2 == 0x00)) {
            offset = (C0_1 == 0x00) ? 0 : 512;
            if (last_rate != (buffer[12 + offset] & 3)) {
                if ( _running ) {
                    _keep_running = true;
                    stop();
                }
                usleep(200000);
                resamp_crcf_destroy(mcb.q);
                last_rate = (buffer[12 + offset] & 3);

                switch (last_rate) {
                case 3:
                    mcb.q = resamp_crcf_create(1.958404308f,13,0.45f,60.0f,64);
                    mcb.output_rate = 190000;
                    break;

                case 2:
                    mcb.q = resamp_crcf_create(0.979202154f,13,0.45f,60.0f,64);
                    mcb.output_rate = 190000;
                    break;

                case 1:
                    mcb.q = resamp_crcf_create(0.864000864f,13,0.45f,60.0f,64);
                    mcb.output_rate = 100000;
                    break;

                case 0:
                    mcb.q = resamp_crcf_create(0.864000864f,13,0.45f,60.0f,64);
                    mcb.output_rate = 50000;
                    break;

                default:
                    t_print ("WARNING: UNSUPPORTED RATE: %x!!!\n",
                             last_rate);
                }

                SetBandwidth(mcb.output_rate);

                if ( _running ) {
                    start();
                }

                t_print ("Setting hpsdr output rate to %d hz\n", mcb.output_rate);
            }

            if (last_num_rcvrs != (buffer[15 + offset] & 0x38)) {
                last_num_rcvrs = (buffer[15 + offset] & 0x38);
                num_rcvrs = (last_num_rcvrs >> 3) + 1;

                if (num_rcvrs <= mcb.total_num_rcvrs) {
                    num_copy_rcvrs = 0;
                    mcb.active_num_rcvrs = num_rcvrs;
                } else {
                    num_copy_rcvrs =
                        num_rcvrs - mcb.total_num_rcvrs;
                    mcb.active_num_rcvrs = mcb.total_num_rcvrs;
                }

                mcb.nsamps_packet =
                    nsamps_packet[mcb.active_num_rcvrs - 1];
                mcb.frame_offset1 =
                    frame_offset1[mcb.active_num_rcvrs - 1];
                mcb.frame_offset2 =
                    frame_offset2[mcb.active_num_rcvrs - 1];
                mcb.rcvrs_mask = 1;


                // disable all previous rcvrs except rcvr 1
                for (i = 1; i < mcb.total_num_rcvrs; i++) {
                    mcb.rcb[i].rcvr_mask = 0;
                }

                // now enable any new ones
                for (i = 1; i < mcb.active_num_rcvrs; i++) {
                    mcb.rcvrs_mask |= 1 << i;
                    mcb.rcb[i].rcvr_mask = 1 << i;
                    mcb.rcb[i].output_rate = 0;
                }

                t_print ("Requested %d Activated %d actual rcvr(s)\n",
                         num_rcvrs, mcb.active_num_rcvrs);

                if (num_copy_rcvrs > 0) {
                    for (i = mcb.active_num_rcvrs; i < num_rcvrs; i++) {
                        copy_rcvr[i] = i;
                    }

                    t_print ("Activated %d COPY(S) of rcvr %d\n",
                             num_copy_rcvrs, mcb.active_num_rcvrs);
                }
            }
        }
    }
}

void *handler_ep6(void *arg)
{
    uint32_t counter = 0;

    t_print("start handler_ep6()\n");
    while (enable_thread) {
        running = 1;
        // wait for all rcvrs to have formatted data ready before sending
        pthread_mutex_lock (&send_lock);
        while (send_flags != mcb.rcvrs_mask && enable_thread) {
            pthread_cond_wait (&send_cond, &send_lock);
        }

#if 0                           // dump the frame for analysis
        if (iloop++ == 100) {
            //iloop = 0;
            t_print ("rcvrs_mask:%x send_flags:%d\n", mcb.rcvrs_mask,
                     send_flags);

            for (int i = 0; i < HPSDR_FRAME_LEN; i++) {
                t_print ("%4d:%2x ", i, payload[i]);

                if (!((i + 1) % 8))
                    t_print ("\n");
            }
        }

//                      exit(0);
#endif
        // plug in sequence numbers
        *(uint32_t *)(payload + 4) = htonl(counter);
        ++counter;

        pthread_mutex_lock (&done_send_lock);
        sendto(sock_udp, payload, HPSDR_FRAME_LEN, 0, (struct sockaddr *)&addr_old, sizeof(addr_old));
        pthread_mutex_unlock (&send_lock);
        send_flags = 0;
        pthread_cond_broadcast (&done_send_cond);
        pthread_mutex_unlock (&done_send_lock);
    }

    active_thread = 0;
    running = 0;
    t_print("stop handler_ep6()\n");
    return NULL;
}

void t_print(const char *format, ...)
{
    va_list(args);
    va_start(args, format);
    struct timespec ts;
    double now;
    static double starttime;
    static int first = 1;
    char line[1024];
    clock_gettime(CLOCK_MONOTONIC, &ts);
    now = ts.tv_sec + 1E-9 * ts.tv_nsec;

    if (first) {
        first = 0;
        starttime = now;
    }

    //
    // After 11 days, the time reaches 999999.999 so we simply wrap around
    //
    if (now - starttime >= 999999.995) {
        starttime += 1000000.0;
    }

    //
    // We have to use vsnt_print to handle the varargs stuff
    // g_print() seems to be thread-safe but call it only ONCE.
    //
    vsnprintf(line, 1024, format, args);
    printf("%10.6f %s", now - starttime, line);
}

void t_perror(const char *string)
{
    t_print("%s: %s\n", string, strerror(errno));
}
