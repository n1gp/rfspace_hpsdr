/* Copyright (C)
*   11/2025 - Rick Koch, N1GP
*   This module implements protocol2 for this project.
*   Most of the code in this module was derived from the work of
*   Christoph van Wüllen, DL1YCF
*   https://github.com/dl1ycf/pihpsdr - newhpsdrsim.c
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

#include "rfspace_hpsdr.h"

#define NUMRECEIVERS 4

extern struct sockaddr_in addr_new;
extern u_int send_flags;
extern pthread_mutex_t send_lock;
extern pthread_cond_t send_cond;
extern pthread_mutex_t done_send_lock;
extern pthread_cond_t done_send_cond;
extern struct main_cb mcb;
extern bool _running;
extern bool _keep_running;

static int bits = -1;
static int run = 0;
static long rxfreq[NUMRECEIVERS];
static int adcdither;
static int adcrandom;
static int ddcenable[NUMRECEIVERS];
static int rxrate[NUMRECEIVERS];
static int stepatt0 = -1;
static int ddc_port = 1025;
static int hp_port = 1027;
static int ddc0_port = 1035;
static float rxatt0;
static unsigned char pbuf[238*6];

static pthread_t highprio_thread_id = 0;
static pthread_t ddc_specific_thread_id;
static pthread_t rx_thread_id[NUMRECEIVERS];

void   *highprio_thread(void*);
void   *ddc_specific_thread(void*);
void   *rx_thread(void *);

int new_protocol_running()
{
    if (run) {
        return 1;
    } else {
        return 0;
    }
}

void load_packet2 (struct rcvr_cb *rcb)
{
    float complex *out_buf = &rcb->iqSamples[rcb->iqSample_offset];
    int i, j, IQData;

    pthread_mutex_lock (&send_lock);
    for (i = 0, j = 0; i < 238; i++, j+=6) {
        IQData = (int)cimagf(out_buf[i]);
        pbuf[j] = IQData >> 16;
        pbuf[j+1] = IQData >> 8;
        pbuf[j+2] = IQData & 0xff;
        IQData = (int)crealf(out_buf[i]);
        pbuf[j+3] = IQData >> 16;
        pbuf[j+4] = IQData >> 8;
        pbuf[j+5] = IQData & 0xff;
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

void new_protocol_general_packet(unsigned char *buffer)
{
    static unsigned long seqnum = 0;
    unsigned long seqold;
    seqold = seqnum;
    seqnum = (buffer[0] >> 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];

    if (seqnum != 0 && seqnum != seqold + 1 ) {
        t_print("GP: SEQ ERROR, old=%lu new=%lu\n", seqold, seqnum);
    }

    //
    // Start HighPrio thread if we arrive here for the first time
    // The HighPrio thread keeps running all the time.
    //
    if (!highprio_thread_id) {
        if (pthread_create(&highprio_thread_id, NULL, highprio_thread, NULL) < 0) {
            t_perror("***** ERROR: Create HighPrio thread");
        }
        pthread_detach(highprio_thread_id);
    }
}

void *highprio_thread(void *data)
{
    int sock;
    struct sockaddr_in addr;
    socklen_t lenaddr = sizeof(addr);
    unsigned long seqnum = 0, seqold;
    unsigned char buffer[2000];
    struct timeval tv;
    int yes = 1;
    int rc;
    long freq;
    int i;
    sock = socket(AF_INET, SOCK_DGRAM, 0);

    if (sock < 0) {
        t_perror("***** ERROR: HP: socket");
        return NULL;
    }

    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
    tv.tv_sec = 0;
    tv.tv_usec = 10000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(hp_port);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        t_perror("***** ERROR: HP: bind");
        close(sock);
        return NULL;
    }

    while (1) {
        //
        rc = recvfrom(sock, buffer, 1444, 0, (struct sockaddr *)&addr, &lenaddr);

        if (rc < 0 && errno != EAGAIN) {
            t_perror("***** ERROR: HighPrio thread: recvmsg");
            break;
        }

        if (rc < 0) {
            continue;
        }

        if (rc != 1444) {
            t_print("Received HighPrio packet with incorrect length %d\n", rc);
            break;
        }

        seqold = seqnum;
        seqnum = (buffer[0] >> 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];

        if (seqnum != 0 && seqnum != seqold + 1 ) {
            t_print("HP: SEQ ERROR, old=%lu new=%lu\n", seqold, seqnum);
        }

        for (i = 0; i < NUMRECEIVERS; i++) {
            freq = (buffer[ 9 + 4 * i] << 24) + (buffer[10 + 4 * i] << 16) + (buffer[11 + 4 * i] << 8) + buffer[12 + 4 * i];

            if (bits & 0x08) {
                freq = round(122880000.0 * (double) freq / 4294967296.0);
            }

            if (freq != rxfreq[i]) {
                rxfreq[i] = freq;
                setFrequency(rxfreq[i]);
                //t_print("HP: DDC%d freq: %lu\n", i, freq);
            }
        }

        rc = buffer[1443];
        if (rc != stepatt0) {
            stepatt0 = rc;
            rxatt0 = (float)(-stepatt0);
            SetRFGain(rxatt0);
            //t_print("HP: StepAtt0 = %d RxAtten0 = %f\n", stepatt0, rxatt0);
        }

        rc = (buffer[4] >> 0) & 0x01;
        if (rc != run) {
            run = rc;
            //t_print("HP: Run=%d\n", rc);

            // if run=0, wait for threads to complete, otherwise spawn them off
            if (run) {
                if (pthread_create(&ddc_specific_thread_id, NULL, ddc_specific_thread, NULL) < 0) {
                    t_perror("***** ERROR: Create DDC specific thread");
                }

                for (i = 0; i < NUMRECEIVERS; i++) {
                    if (pthread_create(&rx_thread_id[i], NULL, rx_thread, (void *) (uintptr_t) i) < 0) {
                        t_perror("***** ERROR: Create RX thread");
                    }
                }
                start();
            } else {
                // Clean-Up done below
                stop();
                break;
            }
        }

        rc = buffer[1443];
    }

    run = 0;

    t_print("HP thread terminating.\n");
    highprio_thread_id = 0;
    close(sock);
    highprio_thread_id = 0;
    return NULL;
}

void *ddc_specific_thread(void *data)
{
    int sock;
    struct sockaddr_in addr;
    socklen_t lenaddr = sizeof(addr);
    unsigned long seqnum, seqold;
    struct timeval tv;
    unsigned char buffer[2000];
    int yes = 1;
    int rc;
    int i;
    sock = socket(AF_INET, SOCK_DGRAM, 0);

    if (sock < 0) {
        t_perror("***** ERROR: RX specific: socket");
        return NULL;
    }

    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
    tv.tv_sec = 0;
    tv.tv_usec = 10000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(ddc_port);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        t_perror("***** ERROR: RX specific: bind");
        close(sock);
        return NULL;
    }

    seqnum = 0;

    t_print("ddc_specific_thread()\n");
    while (run) {
        rc = recvfrom(sock, buffer, 1444, 0, (struct sockaddr *)&addr, &lenaddr);

        if (rc < 0 && errno != EAGAIN) {
            t_perror("***** ERROR: DDC specific thread: recvmsg");
            break;
        }

        if (rc < 0) {
            continue;
        }

        if (rc != 1444) {
            t_print("RXspec: Received DDC specific packet with incorrect length");
            break;
        }

        seqold = seqnum;
        seqnum = (buffer[0] >> 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];

        if (seqnum != 0 && seqnum != seqold + 1 ) {
            t_print("RXspec: SEQ ERROR, old=%lu new=%lu\n", seqold, seqnum);
        }

        rc = buffer[5] & 0x01;

        if (rc != adcdither) {
            adcdither = rc;
            t_print("RX: ADC dither=%d\n", rc);
        }

        rc = buffer[6] & 0x01;

        if (rc != adcrandom) {
            adcrandom = rc;
            t_print("RX: ADC random=%d\n", rc);
        }

        for (i = 0; i < NUMRECEIVERS; i++) {
            int modified = 0;

            rc = (buffer[18 + 6 * i] << 8) + buffer[19 + 6 * i];

            if (rc != rxrate[i]) {
                rxrate[i] = rc;
                modified = 1;
                // these also worked, 380952 195121 96385 48192
                // 1.008001008 0.984004797f 0.996005603f 0.996015936f
                mcb.output_rate = rxrate[i] * 1000 / 0.96;
                setSampleRate(mcb.output_rate);
            }

            rc = (buffer[7 + (i / 8)] >> (i % 8)) & 0x01;

            if (rc != ddcenable[i]) {
                modified = 1;
                ddcenable[i] = rc;
            }

            if (modified) {
                t_print("RX: DDC%d Enable=%d Rate=%d\n",
                        i, ddcenable[i], rxrate[i]);
                rc = 0;
            }
        }
    }

    close(sock);
    return NULL;
}

void *rx_thread(void *data)
{
    int sock;
    struct sockaddr_in addr;
    // One instance of this thread is started for each DDC
    unsigned long seqnum;
    unsigned char buffer[1444];
    int myddc;
    int yes = 1;
    unsigned char *p;
    myddc = (int) (uintptr_t) data;

    if (myddc < 0 || myddc >= NUMRECEIVERS) {
        return NULL;
    }

    seqnum = 0;
    sock = socket(AF_INET, SOCK_DGRAM, 0);

    if (sock < 0) {
        t_perror("***** ERROR: RXthread: socket");
        return NULL;
    }

    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(ddc0_port + myddc);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        t_perror("***** ERROR: RXthread: bind");
        close(sock);
        return NULL;
    }

    t_print("rx_thread (%d)\n", myddc);
    while (run) {
        if (ddcenable[myddc] <= 0 || rxrate[myddc] == 0 || rxfreq[myddc] == 0) {
            usleep(500000);
            continue;
        }

        p = buffer;
        *p++ = (seqnum >> 24) & 0xFF;
        *p++ = (seqnum >> 16) & 0xFF;
        *p++ = (seqnum >>  8) & 0xFF;
        *p++ = (seqnum >>  0) & 0xFF;
        seqnum += 1;

        // no time stamps
        p += 9;

        *p++ = 24; // 24 bits per sample
        *p++ = 0;
        *p++ = 238; // samps per packet

        pthread_mutex_lock (&send_lock);
        while (send_flags != mcb.rcvrs_mask) {
            pthread_cond_wait (&send_cond, &send_lock);
        }

        memcpy(p, pbuf, 1428);

#if 0
        if (seqnum == 100) {
            t_print ("rcvrs_mask:%x send_flags:%d\n", mcb.rcvrs_mask, send_flags);

            for (int i = 0; i < 1444; i++) {
                printf("%4d:%2x ", i, buffer[i]);

                if (!((i + 1) % 8))
                    printf("\n");
            }
        }
#endif

        pthread_mutex_lock (&done_send_lock);
        if (sendto(sock, buffer, 1444, 0, (struct sockaddr * )&addr_new, sizeof(addr_new)) < 0) {
            t_perror("***** ERROR: RX thread sendto");
            break;
        }

        pthread_mutex_unlock (&send_lock);
        send_flags = 0;
        pthread_cond_broadcast (&done_send_cond);
        pthread_mutex_unlock (&done_send_lock);
    }

    close(sock);
    return NULL;
}
