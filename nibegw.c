/**
 * Copyright (c) 2010-2024 Contributors to the openHAB project
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * ----------------------------------------------------------------------------
 *
 *	This application listening data from various Nibe heat pumps (RS485 bus)
 *	and send valid frames to configurable IP/port address by UDP packets.
 *	Application also acknowledge the valid packets to heat pump.
 *
 *	Serial settings: 9600 baud, 8 bits, Parity: none, Stop bits 1
 *
 *	MODBUS module support should be turned ON from the heat pump.
 *
 *	Frame format:
 *	+----+------+------+-----+-----+----+----+-----+
 *	| 5C | ADDR | ADDR | CMD | LEN |  DATA   | CHK |
 *	+----+------+------+-----+-----+----+----+-----+
 *
 *	     |------------ CHK ------------------|
 *
 *	Address: 
 *		0x0016 = SMS40
 *		0x0019 = RMU40
 *		0x0020 = MODBUS40
 *
 *	Checksum: XOR
 *
 *	When valid data is received (checksum ok),
 *	 ACK (0x06) should be sent to the heat pump.
 *	When checksum mismatch,
 *	 NAK (0x15) should be sent to the heat pump.
 *
 *	If heat pump does not receive acknowledge in certain time period,
 *	pump will raise an alarm and alarm mode is activated.
 *
 *	Author: pauli.anttila@gmail.com
 *
 *	Build: gcc -std=gnu99 -o nibegw nibegw.c
 *
 *	3.2.2013    v1.00   Initial version
 *	5.2.2013    v1.01   
 *	4.11.2013   v1.02   Support cheksum and data special cases.
 *	20.12.2013  v1.03   Fixed compiling error.
 *	3.6.2014    v1.04   
 *	4.6.2014    v1.10   More options.
 *	10.9.2014   v1.20   Bidirectional support.
 *	30.6.2015   v1.21   Some fixes.
 *	20.2.2017   v1.22   Separated read and write token support.
 *	7.2.2021    v1.23   Fixed compile error in RasPi.
 *	19.11.2022  v1.30   Support 16-bit addressing.
 *	26.12.2022  v1.31   Fixed serial settings (for RPi Zero 2 W + Waveshare RS485 CAN hat)
 */

#include <signal.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>

#define VERSION	"1.31"

#define FALSE	0
#define TRUE	1

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

int verbose = 0;
int testmode = FALSE;

// Funktionsdeklrarationer
int udpPortSetup(int readPort, int writePort);
int initSerialPort(int fd, int hwflowctrl);
ssize_t readData(int fildes, void *buf, size_t nbyte);
//int checkMessage(unsigned char *message, int len); - denna rad verkar finnas på två ställen, se rad 333

//Funktionsdefinitioner
int udpPortSetup(int readPort, int writePort) {
    int sockfdRead, sockfdWrite;
    struct sockaddr_in serverAddrRead, serverAddrWrite;

    // Skapa och binda för läsport
    sockfdRead = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfdRead < 0) {
        perror("Failed to create socket for read");
        return -1;
    }
    memset(&serverAddrRead, 0, sizeof(serverAddrRead));
    serverAddrRead.sin_family = AF_INET;
    serverAddrRead.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddrRead.sin_port = htons(readPort);
    if (bind(sockfdRead, (struct sockaddr *)&serverAddrRead, sizeof(serverAddrRead)) < 0) {
        perror("Failed to bind socket for read");
        return -1;
    }

    // Skapa och binda för skrivport
    sockfdWrite = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfdWrite < 0) {
        perror("Failed to create socket for write");
        return -1;
    }
    memset(&serverAddrWrite, 0, sizeof(serverAddrWrite));
    serverAddrWrite.sin_family = AF_INET;
    serverAddrWrite.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddrWrite.sin_port = htons(writePort);
    if (bind(sockfdWrite, (struct sockaddr *)&serverAddrWrite, sizeof(serverAddrWrite)) < 0) {
        perror("Failed to bind socket for write");
        return -1;
    }

    return 0;
}

int initSerialPort(int fd, int hwflowctrl) {
    struct termios options;

    // Få aktuella inställningar
    if (tcgetattr(fd, &options) < 0) {
        perror("Failed to get serial port attributes");
        return -1;
    }

    // Ställ in hastighet (9600 baudrate här som exempel)
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    // Ställ in 8N1 (8 data bits, no parity, 1 stop bit)
    
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; // 8 data bits

    if (hwflowctrl) {
	    options.c_cflag |= CRTSCTS;
    } else {
	    options.c_cflag &= ~CRTSCTS;
    }

	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST;
	


    // Tillämpa inställningar
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        perror("Failed to set serial port attributes");
        return -1;
    }

    return 0;
}



void signalCallbackHandler(int signum)
{
	if (verbose) printf("\nExit...caught by signal %d\n", signum);
	exit(1);
}

void printMessage(const unsigned char* const message, int msglen)
{
	printf("Data: ");
	for (int l = 0; l < msglen; l++)
		printf("%02X", message[l]);
	printf("\n");
}

int writeDataToSerialPort(int fd, const unsigned char* const message, int msglen)
{
	int retval = -1;
	
	if (verbose > 2) printf("Write data to serial port\n");
	if (verbose > 2) printMessage(message, msglen);
	
	if( write( fd, message, msglen) == msglen)
	{
		tcdrain (fd);
		retval = 0;
	}
	
	return retval;
}

int forwardUdpMsgToSerial(int udpfd, int serialfd)
{
	#define MAX_UDP_MSG_SIZE 50
	unsigned char udp_packet[MAX_UDP_MSG_SIZE];
	int udplen;
	
	if ((udplen = recv(udpfd, udp_packet, MAX_UDP_MSG_SIZE, 0)) > 0)
	{
		if (verbose > 1) printf("Received UDP message...relay message to serial port\n");
		if (verbose > 2) printMessage( udp_packet, udplen);
		
		writeDataToSerialPort(serialfd, udp_packet, udplen);
		return FALSE;
	}
	return TRUE;
}

int sendAck(int fd)
{
	unsigned char ack = 0x06;
	if (verbose > 1) printf("Send ACK (0x06)\n");
	return writeDataToSerialPort(fd, &ack, 1);
}

int sendNak(int fd)
{
	unsigned char nack = 0x15;
	if (verbose > 1) printf("Send NAK (0x15)\n");
	return writeDataToSerialPort(fd, &nack, 1);
}

char* getTimeStamp(char* buffer)
{
	struct timeval  tv;
	struct timezone tz;
	struct tm*      tm;
 
	gettimeofday(&tv, &tz);
	tm = localtime(&tv.tv_sec);
 
	sprintf(buffer, "%d.%d.%d %d:%02d:%02d:%ld", 
		   tm->tm_year + 1900,
		   tm->tm_mon + 1,
		   tm->tm_mday,
		   tm->tm_hour,
		   tm->tm_min, tm->tm_sec, tv.tv_usec
		   );
	return buffer;
}

// ny kod
//readData definition (innehålls debug-utskrifter)
ssize_t readData(int fildes, void *buf, size_t nbyte)
{
    if (testmode)
    {
        unsigned char testdata[] = 
            /* Junk */
            "\x01\x02" \
            
            /* Frame from MODBUS40 */
            "\x5C\x00\x20\x6B\x00\x4B" \
            
            /* Frame from RMU40 */
            "\x5C\x00\x19\x60\x00\x79" \
            
            /* Frame from RMU40 */
            "\x5C\x00\x19\x62\x18\x00\x80\x00\x80\x00\x00\x00\x00\x00"
            "\x80\x00\x00\x00\x00\x00\x0B\x0B\x00\x00\x00\x01\x00\x00"
            "\x05\xE7" \
            
            /* Data frame from MODBUS40 */
            "\x5C\x00\x20\x68\x50\x01\xA8\x1F\x01\x00\xA8\x64\x00\xFD" \
            "\xA7\xD0\x03\x44\x9C\x1E\x00\x4F\x9C\xA0\x00\x50\x9C\x78" \
            "\x00\x51\x9C\x03\x01\x52\x9C\x1B\x01\x87\x9C\x14\x01\x4E" \
            "\x9C\xC6\x01\x47\x9C\x01\x01\x15\xB9\xB0\xFF\x3A\xB9\x4B" \
            "\x00\xC9\xAF\x00\x00\x48\x9C\x0D\x01\x4C\x9C\xE7\x00\x4B" \
            "\x9C\x00\x00\xFF\xFF\x00\x00\xFF\xFF\x00\x00\xFF\xFF\x00" \
            "\x00\x45" \
            
            /* Token Frame from MODBUS40 */
            "\x5C\x00\x20\x69\x00\x49" \
            ;

        int len = sizeof(testdata);

        if (len > nbyte)
        {
            fprintf(stderr, "Too much test data, limiting %u to %u\n", len, (unsigned int)nbyte);
        }

        len = MIN(len, nbyte);
        memcpy(buf, testdata, len);

        static int delay = FALSE;
        if (delay)
            sleep(2); // slow down little bit after first round

        delay = TRUE;
        return len;
    }

    printf("Attempting to read %zu bytes from fd %d\n", nbyte, fildes);  // Debug-utskrift
    ssize_t bytesRead = read(fildes, buf, nbyte);
    if (bytesRead == -1)
    {
        perror("Failed to read data");
        return -1;
    }
    printf("Read %ld bytes\n", bytesRead);  // Debug-utskrift
    return bytesRead;
}






/*
 * Return:
 *	>0 if valid message received (return message len)
 *	 0 if OK but message not ready
 *	-1 if invalid message
 *	-2 if checksum fails
 */
int checkMessage(const unsigned char* const data, int len)
{
	if (len >= 1)
	{
		if (data[0] != 0x5C)
			return -1;
		
		if (len >= 6)
		{
			int datalen = data[4];
			
			if (len < datalen + 6)
				return 0;
			
			unsigned char calc_checksum = 0;
			
			// calculate XOR checksum
			for(int i = 1; i < (datalen + 5); i++)
				calc_checksum ^= data[i];
			
			unsigned char msg_checksum = data[datalen + 5];
			
			if (verbose) printf("(calc/recv checksum %02X/%02X = ", calc_checksum, msg_checksum);
			
			if (calc_checksum != msg_checksum)
			{
				// check special case, if checksum is 0x5C (start character), 
				// heat pump seems to send 0xC5 checksum
				if (calc_checksum != 0x5C && msg_checksum != 0xC5)
				{
					if (verbose) printf("ERROR)\n");
					return -2;
				}
			}
			
			if (verbose) printf("OK)\n");
			return datalen + 6;
		}
		
	}
	
	return 0;
}

void printUsage(char* appname)
{
	char* usage = "%s usage:\n\n" \
	"\t-h                 Print help\n" \
	"\t-v                 Print debug information\n" \
	"\t-d <device name>   Serial port device (default: /dev/ttyS0)\n" \
	"\t-a <address>       Remote host address (default: 127.0.0.1)\n" \
	"\t-p <port>          Remote UDP port (default: 9999)\n" \
	"\t-f                 Disable flow control (default: HW)\n" \
	"\t-r <address>       RS-485 address to listen (default: 0x20)\n" \
	"\t-i                 Send all messages by UDP (default: only modbus data)\n" \
	"\t-n                 Don't send acknowledge at all\n" \
	"\t-o                 Send acknowledge to all addresses\n" \
	"\t-t                 Test mode\n" \
	"\t-l <port>          Local UDP port for read commands (default: 10000)\n" \
	"\t-w <port>          Local UDP port for write commands (default: 10001)\n" \
	"\t-q                 Print data in log format\n" \
	;
	
	fprintf (stderr, usage, appname);
}
 
int main(int argc, char **argv)
{
    char *device = "/dev/ttyS0";
    char *remoteHost = "127.0.0.1";
    int remotePort = 9999;
    int localPort4readCmds = 12000;
    int localPort4writeCmds = 12001;
    unsigned char rs485addr = 0x20;
    int sendall = FALSE;
    int sendack = TRUE;
    int ackall = FALSE;
    int hwflowctrl = TRUE;
    int log = FALSE;

    int c;
    opterr = 0;

    while ((c = getopt (argc, argv, "hvd:a:p:r:infotql:w:")) != -1)
    {
        switch (c)
        {
        case 'v':
            verbose++;
            break;

        case 'i':
            sendall = TRUE;
            break;

        case 'n':
            sendack = FALSE;
            break;

        case 'f':
            hwflowctrl = FALSE;
            break;

        case 'o':
            ackall = TRUE;
            break;

        case 't':
            testmode = TRUE;
            break;

        case 'q':
            log = TRUE;
            break;

        case 'd':
            device = optarg;
            break;

        case 'a':
            remoteHost = optarg;
            break;

        case 'p':
            remotePort = atoi(optarg);
            break;

        case 'l':
            localPort4readCmds = atoi(optarg);
            break;

        case 'w':
            localPort4writeCmds = atoi(optarg);
            break;

        case 'r':
            rs485addr = atoi(optarg);
            break;

        case '?':
            if (optopt == 'd' || optopt == 'a' || optopt == 'p')
                fprintf (stderr, "Option -%c requires an argument.\n", optopt);
            else if (isprint (optopt))
                fprintf (stderr, "Unknown option `-%c'.\n", optopt);
            else
                fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
            return 1;

        case 'h':
        default:
            printUsage(argv[0]);
            return 1;
        }
    }

    if (verbose)
    {
        printf("NibeGW version:                    %s\n", VERSION);
        printf("Verbose level:                     %i\n", verbose);
        printf("Test mode:                         %s\n", testmode ? "TRUE" : "FALSE");
        printf("Serial port:                       %s\n", device);
        printf("Flow control:                      %s\n", hwflowctrl ? "HW" : "None");
        printf("remote UDP address:                %s:%u\n", remoteHost, remotePort);
        printf("server UDP address for read cmds:  %u\n", localPort4readCmds);
        printf("server UDP address for write cmds: %u\n", localPort4writeCmds);
        printf("RS-485 address to listen:          0x%02X\n", rs485addr);
        printf("Send all messages by UDP:          %s\n", sendall ? "TRUE" : "FALSE");
        printf("Send acknowledge:                  %s\n", sendack ? "TRUE" : "FALSE");
        printf("Send acknowledge to all addresses: %s\n", ackall ? "TRUE" : "FALSE");
    }

    // Install signal handlers
    signal(SIGINT, signalCallbackHandler);

    int serialport_fd = -1;
    int udp_fd = -1;
    int udp4writeCmds_fd = -1;

    // Initialize destination address
    struct sockaddr_in dest;
    memset((char *)&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_addr.s_addr = inet_addr(remoteHost);
    dest.sin_port = htons(remotePort);

// Öppna den seriella porten
	serialport_fd = open(device, O_RDWR | O_NOCTTY);
	if (serialport_fd < 0) {
		fprintf(stderr, "Failed to open %s: %s\n", device, strerror(errno));
		return 1;
	} 
	
// Initiera den seriella porten
	if (initSerialPort(serialport_fd, hwflowctrl) == -1) { 
		fprintf(stderr, "Failed to set serial port: %s\n", strerror(errno));
return 1;
	}
// Sätt den seriella porten till icke-blockerande läge
	int flags = fcntl(serialport_fd, F_GETFL, 0);
	fcntl(serialport_fd, F_SETFL, flags | O_NONBLOCK);

	
    // Anropa udpPortSetup för att binda UDP-portarna
    if (udpPortSetup(localPort4readCmds, localPort4writeCmds) < 0) {
        fprintf(stderr, "Failed to setup UDP ports\n");
        return 1;
    }

    int maxdatalen = 200;

    if (testmode)
    {
        maxdatalen = 1000;
    }

    unsigned char buffer[maxdatalen];
    unsigned char message[maxdatalen];

    for (;;)
    {

        if (testmode == FALSE && serialport_fd < 0)
        {
            if (strncmp(device, "stdin" , 5) == 0)
            {
                if (verbose) printf("Use stdin as virtual serial port\n");
                serialport_fd = STDIN_FILENO;
            }
            else
            {
                // Open the serial port
                if (verbose) printf("Open serial port: %s\n", device);
                serialport_fd = open(device, O_RDWR | O_NOCTTY); // | O_NDELAY

                if (serialport_fd < 0)
                {
                    fprintf(stderr, "Failed to open %s: %s\n", device, strerror(errno));
                }

                // Initialize serial port
                if (initSerialPort(serialport_fd, hwflowctrl) == -1)
                {
                    fprintf(stderr, "Failed to set serial port: %s\n", strerror(errno));
                }
            }
        }

        if (testmode || serialport_fd >= 0)
        {
            char timestamp[80];
            ssize_t len = 0;

            int startfound = FALSE;
            int index = 0;

            // read all available bytes from serial port

        while ((len = readData(serialport_fd, buffer, maxdatalen)) > 0) {
            for (int i = 0; i < len; i++) {
                if (log) printf("\\x%02X", buffer[i]);

                if (startfound == FALSE) {
                    if (verbose) printf("\n%s: ", getTimeStamp(timestamp));
                }

                if (verbose) printf("%02X ", buffer[i]);
                if (verbose > 3) printf("(%c) ", buffer[i]);

                if (startfound == FALSE && buffer[i] == 0x5C) {
                    startfound = TRUE;
                    index = 0;
                }

                if (startfound) {
                    if ((index+1) >= maxdatalen) {
                        // too long message, try to find new start char
                        startfound = FALSE;
                    } else {
                        message[index++] = buffer[i];

                        int msglen = checkMessage(message, index);

                        switch (msglen) {
                            case 0: // Message ok so far, but not ready
                                break;

                            case -1: // Invalid message
                                startfound = FALSE;
                                break;

                            case -2: // Checksum error
                                if (message[2] == rs485addr || ackall) {
                                    if (sendack) sendNak(serialport_fd);
                                }   
                                startfound = FALSE;
                                break;

                            default:
                                if (verbose > 1) printf("Valid message received, len=%u\n", msglen);

                                if (message[2] == rs485addr || ackall) {
                                    // send ack to nibe or read/write messages if token received
                                    int nothingToSend = TRUE;

                                    if (message[3] == 0x69 && message[4] == 0x00) {
                                        if (verbose > 1) printf("Read token received\n");
                                        nothingToSend = forwardUdpMsgToSerial(udp_fd, serialport_fd);
                                    } else if (message[3] == 0x6b && message[4] == 0x00) {
                                        if (verbose > 1) printf("Write token received\n");
                                        nothingToSend = forwardUdpMsgToSerial(udp4writeCmds_fd, serialport_fd);
                                    }

                                    if (nothingToSend) {
                                        if (verbose > 1) printf("Nothing to send...");
                                        if (sendack) sendAck(serialport_fd);
                                    }
                                }

                                if (message[2] == rs485addr || sendall) {
                                    // send message to remote
                                    if (verbose > 1) printf("Send UDP data to %s:%u\n", remoteHost, remotePort);
                                    if (verbose > 2) printMessage(message, msglen);

                                    if (sendto(udp_fd, message, msglen + 1, 0, (struct sockaddr *)&dest, sizeof(dest)) == -1) {
                                        fprintf(stderr, "Failed to send udp packet: %s\n", strerror(errno));
                                    }
                                }

                                startfound = FALSE;
                                break;
                        }   
                    }
                }
            }
        }

        if (len < 0 && errno != EAGAIN) {
            perror("Read error");
        }

        sleep(1); // Justera eller ta bort efter behov
    }

    // Stäng alla öppna filer och socketar
    close(serialport_fd);
    close(udp_fd);
    close(udp4writeCmds_fd);

    return 0;
    }
}
