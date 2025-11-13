#ifndef _RFSPACE_HPSDR_H
#define _RFSPACE_HPSDR_H

/* Copyright (C)
*
*   10/2025 - Rick Koch, N1GP
*   Wrote rfspace_hpsdr using various open sources on the internet.
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

#include <sched.h>
#include <unistd.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <termios.h>
#include <libgen.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <netdb.h>
#include <complex.h>
#include <liquid/liquid.h>
#include <linux/usbdevice_fs.h>
#include <libftdi1/ftdi.h>

#define PRG_VERSION "1.0" // see ChangeLog for history

#define HERMES_FW_VER 32
#define MAX_BUFFER_LEN 2048
#define HPSDR_FRAME_LEN 1032
#define IQ_FRAME_DATA_LEN 63
#define SDR_READ_COUNT (8192)
#define MAX_RCVRS 1
#define IQ_FRAME_DATA_LEN 63
#define PORT 50000
#define DEFAULT_HOST "127.0.0.1"
#define MAXSTR 128

struct main_cb {
    int total_num_rcvrs;
    int active_num_rcvrs;
    u_int rcvrs_mask;
    int nsamps_packet;
    int frame_offset1;
    int frame_offset2;
    int output_rate;
    int up_xtal;
    char sound_dev[MAXSTR];
    char ip_addr[MAXSTR];
    char serialstr[MAXSTR];
    char ip[MAXSTR];
    resamp_crcf q;

    // the last array member is used to remember last settings
    int agc_mode[MAX_RCVRS + 1];
    int gain[MAX_RCVRS + 1];
    int freq_offset[MAX_RCVRS + 1];
    int center_freq[MAX_RCVRS + 1];
    int if_bw[MAX_RCVRS + 1];
    int gain_mode[MAX_RCVRS + 1];

    // Added to handle dynamic config file updates
    int last_agc_mode[MAX_RCVRS + 1];
    int last_gain[MAX_RCVRS + 1];
    int last_freq_offset[MAX_RCVRS + 1];
    int last_center_freq[MAX_RCVRS + 1];
    int last_if_bw[MAX_RCVRS + 1];
    int last_gain_mode[MAX_RCVRS + 1];


    struct rcvr_cb {
        float dest[4] __attribute__((aligned(16)));
        int rcvr_num;
        int new_freq;
        int curr_freq;
        int output_rate;
        u_int rcvr_mask;
        struct main_cb* mcb;
        pthread_t hpsdrsim_sendiq_thr;

        int iqSample_offset;
        int iqSamples_remaining;
        float complex iqSamples[((SDR_READ_COUNT / 2) + (IQ_FRAME_DATA_LEN * 2))];
    } rcb[MAX_RCVRS];
};

void format_payload(void);
void load_packet(struct rcvr_cb* rcb);
void sdr14_sighandler(int signum);

void* hpsdrsim_sendiq_thr_func(void* arg);
void hpsdrsim_stop_threads();

#define ODEV_NONE          999
#define ODEV_HERMES          1
#define ODEV_ORION           5

#define NDEV_NONE          999
#define NDEV_HERMES          1
#define NDEV_ORION           4

//##################### NEW PROTOCOL STUFF
#define LENNOISE 1536000
#define NOISEDIV (RAND_MAX / 768000)
#define OLDRTXLEN 64512
#define NEWRTXLEN 64320
#define IM3a  0.60
#define IM3b  0.20

//
// Forward declarations for new protocol stuff
//
void   new_protocol_general_packet(unsigned char *buffer);
int    new_protocol_running(void);

// using clock_nanosleep of librt
extern int clock_nanosleep(clockid_t __clock_id, int __flags,
                           __const struct timespec *__req,
                           struct timespec *__rem);

//
// message printing
//
#include <stdarg.h>
void t_print(const char *format, ...);
void t_perror(const char *string);

#endif // _RFSPACE_HPSDR_H
