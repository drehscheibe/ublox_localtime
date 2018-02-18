// Melchior FRANZ <mfranz@aon.at> 2018-01-19
//
#include <assert.h>
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>

#include "calendar.h"
#include "fletcher.h"

#define TIMEZONE 1  // A

#define UBX_SYNC1 0xb5
#define UBX_SYNC2 0x62

#define TARGET_I2C    0  // DDC
#define TARGET_UART1  1
#define TARGET_UART2  2
#define TARGET_USB    3
#define TARGET_SPI    4
#define TARGET_RSVD   5  // reserved

//-- ACK --
#define ACK           0x05
#define ACK_ACK       0x01 // 2 Answer Message Acknowledged
#define ACK_NAK       0x00 // 2 Answer Message Not-Acknowledged UBX Class AID AssistNow Aiding Messages

//-- AID --
#define AID           0x0b
#define AID_ALM       0x30 // 0 Poll Request Poll GPS Aiding Almanac Data
//                    0x30 // 1 Poll Request Poll GPS Aiding Almanac Data for a SV
//                    0x30 // (8) or (40) Input/Output Message GPS Aiding Almanac Input/Output Message
#define AID_ALPSRV    0x32 // 16 Output Message ALP client requests AlmanacPlus data from server
//                    0x32 // 16 + 1*dataSize Input Message ALP server sends AlmanacPlus data to client
//                    0x32 // 8 + 2*size Output Message ALP client sends AlmanacPlus data to server.
#define AID_ALP       0x50 // 0 + 2*N Input message ALP file data transfer to the receiver
//                    0x50 // 1 Input message Mark end of data transfer
//                    0x50 // 1 Output message Acknowledges a data transfer
//                    0x50 // 1 Output message Indicate problems with a data transfer
//                    0x50 // 24 Periodic/Polled Poll the AlmanacPlus status
#define AID_AOP       0x33 // 0 Poll request Poll AssistNow Autonomous data
//                    0x33 // 1 Poll request Poll AssistNow Autonomous data for one satellite
//                    0x33 // (48) or (192) Input/Output Message AssistNow Autonomous data
#define AID_DATA      0x10 // 0 Poll Polls all GPS Initial Aiding Data
#define AID_EPH       0x31 // 0 Poll Request Poll GPS Aiding Ephemeris Data
//                    0x31 // 1 Poll Request Poll GPS Aiding Ephemeris Data for a SV
//                    0x31 // (8) or (104) Input/Output Message GPS Aiding Ephemeris Input/Output Message
#define AID_HUI       0x02 // 0 Poll Request Poll GPS Health, UTC and ionosphere parameters
//                    0x02 // 72 Input/Output Message GPS Health, UTC and ionosphere parameters
#define AID_INI       0x01 // 0 Poll Request Poll GPS Initial Aiding Data
//                    0x01 // 48 Polled Aiding position, time, frequency, clock drift
#define AID_REQ       0x00 // 0 Virtual Sends a poll (AID-DATA) for all GPS Aiding Data UBX Class CFG Configuration Input Messages


//-- CFG --
#define CFG           0x06
#define CFG_ANT       0x13 // 0 Poll Request Poll Antenna Control Settings
//                    0x13 // 4 Get/Set Get/Set Antenna Control Settings
#define CFG_CFG       0x09 // (12) or (13) Command Clear, Save and Load configurations
#define CFG_DAT       0x06 // 0 Poll Request Poll Datum Setting
//                    0x06 // 2 Set Set Standard Datum
//                    0x06 // 44 Set Set User-defined Datum
//                    0x06 // 52 Get Get currently selected Datum
#define CFG_EKF       0x12 // 0 Poll Request Poll EKF Module Settings
//                    0x12 // 16 Get/Set Get/Set EKF Module Settings - LEA-6R
#define CFG_ESFGWT    0x29 // 44 Get/Set message Get/Set settings of gyro+wheel tick sol (GWT) - LEA-6
#define CFG_FXN       0x0e // 0 Poll Request Poll FXN configuration
//                    0x0e // 36 Command RXM FixNOW configuration.
#define CFG_INF       0x02 // 1 Poll Request Poll INF message configuration for one protocol
//                    0x02 // 0 + 10*N Set/Get Information message configuration
#define CFG_ITFM      0x39 // 8 Command Jamming/Interference Monitor configuration.
#define CFG_MSG       0x01 // 2 Poll Request Poll a message configuration
//                    0x01 // 8 Set/Get Set Message Rate(s)
//                    0x01 // 3 Set/Get Set Message Rate
#define CFG_NAV5      0x24 // 0 Poll Request Poll Navigation Engine Settings
//                    0x24 // 36 Get/Set Get/Set Navigation Engine Settings
#define CFG_NAVX5     0x23 // 0 Poll Request Poll Navigation Engine Expert Settings
//                    0x23 // 40 Get/Set Get/Set Navigation Engine Expert Settings
#define CFG_NMEA      0x17 // 0 Poll Request Poll the NMEA protocol configuration
//                    0x17 // 4 Set/Get Set/Get the NMEA protocol configuration
#define CFG_NVS       0x22 // 13 Command Clear, Save and Load non-volatile storage data
#define CFG_PM2       0x3b // 0 Poll Request Poll extended Power Management configuration
//                    0x3b // 44 Set/Get Extended Power Management configuration
#define CFG_PM        0x32 // 0 Poll Request Poll Power Management configuration
//                    0x32 // 24 Set/Get Power Management configuration
#define CFG_PRT       0x00 // 0 Poll Request Polls the configuration of the used I/O Port
//                    0x00 // 1 Poll Request Polls the configuration for one I/O Port
//                    0x00 // 20 Get/Set Get/Set Port Configuration for UART
//                    0x00 // 20 Get/Set Get/Set Port Configuration for USB Port
//                    0x00 // 20 Get/Set Get/Set Port Configuration for SPI Port
//                    0x00 // 20 Get/Set Get/Set Port Configuration for DDC Port
#define CFG_RATE      0x08 // 0 Poll Request Poll Navigation/Measurement Rate Settings
//                    0x08 // 6 Get/Set Navigation/Measurement Rate Settings
#define CFG_RINV      0x34 // 0 Poll Request Poll contents of Remote Inventory
//                    0x34 // 1 + 1*N Set/Get Set/Get contents of Remote Inventory
#define CFG_RST       0x04 // 4 Command Reset Receiver / Clear Backup Data Structures
#define CFG_RXM       0x11 // 0 Poll Request Poll RXM configuration
//                    0x11 // 2 Set/Get RXM configuration
#define CFG_SBAS      0x16 // 0 Poll Request Poll contents of SBAS Configuration
//                    0x16 // 8 Command SBAS Configuration
#define CFG_TMODE2    0x3d // 0 Poll Request Poll Time Mode Settings
//                    0x3d // 28 Get/Set Time Mode Settings 2
#define CFG_TMODE     0x1d // 0 Poll Request Poll Time Mode Settings
//                    0x1d // 28 Get/Set Time Mode Settings
#define CFG_TP5       0x31 // 0 Poll Request Poll Timepulse Parameters
//                    0x31 // 1 Poll Request Poll TimePulse Parameters
//                    0x31 // 32 Get/Set Get/Set TimePulse Parameters
#define CFG_TP        0x07 // 0 Poll Request Poll TimePulse Parameters
//                    0x07 // 20 Get/Set Get/Set TimePulse Parameters
#define CFG_USB       0x1b // 0 Poll Request Poll a USB configuration
//                    0x1b // 108 Get/Set Get/Set USB Configuration UBX Class ESF External Sensor Fusion Messages

//-- ESF --
#define ESF           0x10
#define ESF_MEAS      0x02 // (8 + 4*N) or (12 + Input/Output 4*N) Message External Sensor Fusion Measurements (LEA-6R)
#define ESF_STATUS    0x10 // 16 + 4*numSens Periodic/Polled Sensor Fusion Status Information (LEA-6R)
//                    0x10 // 16 + 4*numSens Periodic/Polled Sensor Fusion Status Information (LEA-6R) UBX Class INF Information Messages

//-- INF --
#define INF           0x04
#define INF_DEBUG     0x04 // 0 + 1*N Output ASCII String output, indicating debug output
#define INF_ERROR     0x00 // 0 + 1*N Output ASCII String output, indicating an error
#define INF_NOTICE    0x02 // 0 + 1*N Output ASCII String output, with informational contents
#define INF_TEST      0x03 // 0 + 1*N Output ASCII String output, indicating test output
#define INF_WARNING   0x01 // 0 + 1*N Output ASCII String output, indicating a warning UBX Class MON Monitoring Messages

//-- MON --
#define MON           0x0a
#define MON_HW2       0x0b // 28 Periodic/Polled Extended Hardware Status
#define MON_HW        0x09 // 68 Periodic/Polled Hardware Status
//                    0x09 // 68 Periodic/Polled Hardware Status
#define MON_IO        0x02 // 0 + 20*N Periodic/Polled I/O Subsystem Status
#define MON_MSGPP     0x06 // 120 Periodic/Polled Message Parse and Process Status
#define MON_RXBUF     0x07 // 24 Periodic/Polled Receiver Buffer Status
#define MON_RXR       0x21 // 1 Get Receiver Status Information
#define MON_TXBUF     0x08 // 28 Periodic/Polled Transmitter Buffer Status
#define MON_VER       0x04 // 70 + 30*N Answer to Poll Receiver/Software/ROM Version UBX Class NAV Navigation Results

//-- NAV --
#define NAV           0x01
#define NAV_AOPSTATUS 0x60 // 20 Periodic/Polled AssistNow Autonomous Status
#define NAV_CLOCK     0x22 // 20 Periodic/Polled Clock Solution
#define NAV_DGPS      0x31 // 16 + 12*numCh Periodic/Polled DGPS Data Used for NAV
#define NAV_DOP       0x04 // 18 Periodic/Polled Dilution of precision
#define NAV_EKFSTATUS 0x40 // 36 Periodic/Polled Dead Reckoning Software Status
#define NAV_POSECEF   0x01 // 20 Periodic/Polled Position Solution in ECEF
#define NAV_POSLLH    0x02 // 28 Periodic/Polled Geodetic Position Solution
#define NAV_SBAS      0x32 // 12 + 12*cnt Periodic/Polled SBAS Status Data
#define NAV_SOL       0x06 // 52 Periodic/Polled Navigation Solution Information
#define NAV_STATUS    0x03 // 16 Periodic/Polled Receiver Navigation Status
#define NAV_SVINFO    0x30 // 8 + 12*numCh Periodic/Polled Space Vehicle Information
#define NAV_TIMEGPS   0x20 // 16 Periodic/Polled GPS Time Solution
#define NAV_TIMEUTC   0x21 // 20 Periodic/Polled UTC Time Solution
#define NAV_VELECEF   0x11 // 20 Periodic/Polled Velocity Solution in ECEF
#define NAV_VELNED    0x12 // 36 Periodic/Polled Velocity Solution in NED UBX Class RXM Receiver Manager Messages

//-- RXM --
#define RXM           0x02
#define RXM_ALM       0x30 // 0 Poll Request Poll GPS Constellation Almanach Data
//                    0x30 // 1 Poll Request Poll GPS Constellation Almanach Data for a SV
//                    0x30 // (8) or (40) Poll Answer / Periodic GPS Aiding Almanach Input/Output Message
#define RXM_EPH       0x31 // 0 Poll Request Poll GPS Constellation Ephemeris Data
//                    0x31 // 1 Poll Request Poll GPS Constellation Ephemeris Data for a SV
//                    0x31 // (8) or (104) Poll Answer / Periodic GPS Aiding Ephemeris Input/Output Message
#define RXM_PMREQ     0x41 // 8 Input Requests a Power Management task
#define RXM_RAW       0x10 // 8 + 24*numSV Periodic/Polled Raw Measurement Data
#define RXM_SFRB      0x11 // 42 Periodic Subframe Buffer
#define RXM_SVSI      0x20 // 8 + 6*numSV Periodic/Polled SV Status Info UBX Class TIM Timing Messages

//-- TIM --
#define TIM           0x0d
#define TIM_SVIN      0x04 // 28 Periodic/Polled Survey-in data
#define TIM_TM2       0x03 // 28 Periodic/Polled Time mark data
#define TIM_TP        0x01 // 16 Periodic/Polled Timepulse Timedata
#define TIM_VRFY      0x06 // 20 Polled/Once Sourced Time Verification


struct nav_timeutc_t {
	uint32_t iTOW;
	uint32_t tAcc;
	int32_t nano;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t sec;
	uint8_t valid;
};


static bool done;
static int verbose;


void help()
{
	printf("Usage: ublox_localtime [-h]\n");
	printf("       ublox_localtime [-v] [-d <device>] [-b <baud-rate>]\n");
	printf("\n");
	printf("  -b, --baud                 set baud-rate (-b? shows list; default: 9600)\n");
	printf("  -d, --device               device file (default: /dev/gps0)\n");
	printf("  -h, --help                 this help screen\n");
	printf("  -q, --quiet                reset verbosity\n");
	printf("  -v, --verbose              verbose (up to five times: -vvvvv)\n");
}


static void fatal(const char *str)
{
	perror(str);
	exit(-1);
}


static void irq_handler(int sig)
{
	done = true;
}


static int gps_send_raw(int fd, const void *data, int len)
{
	if (verbose > 3) {
		fprintf(stderr, "> ");
		for (int i = 0; i < len; i++)
			fprintf(stderr, "\033[35m%02x\033[m ", ((const uint8_t *)data)[i]);
		fprintf(stderr, "\n");
	}

	int ret = write(fd, data, len);
	if (ret == -1) {
		perror("write");
		return ret;
	}
	if (ret != len) {
		fprintf(stderr, "gps_send_raw: bad length: %d\n", ret);
		return -2;
	}

	return 0;
}


static int gps_send_cmd(int fd, uint8_t cl, uint8_t id, const void *data, unsigned len)
{
	uint8_t buf[256], *p = buf;
	assert(len <= sizeof(buf) - 8);

	*p++ = UBX_SYNC1;
	*p++ = UBX_SYNC2;
	*p++ = cl;
	*p++ = id;
	*p++ = (uint8_t)len;
	*p++ = (uint8_t)(len >> 8);

	if (len) {
		memcpy(p, data, len);
		p += len;
	}

	struct fletcher8_t f = { 0 };
	fl8_adds(&f, buf + 2, len + 4);
	*p++ = f.a;
	*p++ = f.b;

	return gps_send_raw(fd, buf, p - buf);
}


static void ubx_nav_timeutc(const struct nav_timeutc_t *utc)
{
	uint16_t year = le16toh(utc->year);

	if (verbose > 1)
		fprintf(stderr, "\033[%dm%04u-%02u-%02u %02u:%02u:%02u UTC %+1.9lf s  acc %u ns  (%s)\033[m   ",
				(utc->valid & 0x4) ? 32 : 31,
				year, utc->month, utc->day,
				utc->hour, utc->minute, utc->sec,
				(int)le32toh(utc->nano) / 1000000000.0,
				(unsigned)le32toh(utc->tAcc),
				(utc->valid & 0x4) ? "valid" : "invalid");

	struct tm u = {
		.tm_year = year - 1900,
		.tm_mon = utc->month - JANUARY,
		.tm_mday = utc->day,
		.tm_hour = utc->hour + TIMEZONE,
		.tm_min = utc->minute,
		.tm_sec = utc->sec,
		.tm_isdst = is_dst(utc->hour, utc->day, utc->month, year),
	};
	time_t epoch = mktime(&u);
	if (epoch == -1)
		perror("mktime");
	if (verbose > 2)
		fprintf(stderr, "%lu   ", (long unsigned)epoch);

	struct tm l;
	localtime_r(&epoch, &l);

	static const char *wd[] = { "So", "Mo", "Di", "Mi", "Do", "Fr", "Sa" };
	fprintf(stderr, "\033[%dm%04u-%02u-%02u %02u:%02u:%02u %s %s W%u\033[m\n",
			utc->valid ? 32 : 31,
			l.tm_year + 1900, l.tm_mon + JANUARY, l.tm_mday,
			l.tm_hour, l.tm_min, l.tm_sec,
			l.tm_isdst ? "MESZ" : "MEZ", wd[l.tm_wday],
			calendar_week(l.tm_mday, l.tm_mon + JANUARY, l.tm_year + 1900));
}


static void ubx_nav_sol(const void *data)
{
	const struct nav_sol_t {
		uint32_t iTOW;      // GPS Millisecond Time of Week [ms]
		int32_t fTOW;       // Fractional Nanoseconds remainder of rounded ms above, range -500000 .. 500000 [ms]
		int16_t week;       // week - GPS week (GPS time)
		uint8_t gpsFix;     // GPSfix Type, range 0..5
		uint8_t flags;      // Fix Status Flags
		int32_t ecefX;      // ECEF X coordinate [cm]
		int32_t ecefY;      // ECEF Y coordinate [cm]
		int32_t ecefZ;      // ECEF Z coordinate [cm]
		uint32_t pAcc;      // 3D Position Accuracy Estimate [cm]
		int32_t ecefVX;     // ECEF X velocity [cm/s]
		int32_t ecefVY;     // ECEF Y velocity [cm/s]
		int32_t ecefVZ;     // ECEF Z velocity [cm/s]
		uint32_t sAcc;      // Speed Accuracy Estimate [cm/s]
		uint16_t pDOP;      // Position DOP [0.01]
		uint8_t reserved1;  // Reserved
		uint8_t numSV;      // Number of SVs used in Nav Solution
		uint32_t reserved2; // Reserved
	} *gps = (struct nav_sol_t *)data;

	static const char *gps_fix_name[] = { "No Fix", "Dead Reckoning only", "2D-Fix", "3D-Fix",
			"GPS + dead reckoning combined", "Time only fix" };

	fprintf(stderr, "#%u %d (%s) %s \t", gps->numSV, gps->gpsFix,
			gps->gpsFix > 5 ? "unknown" : gps_fix_name[gps->gpsFix],
			(gps->flags & 1) ? "OK" : "NOK");
}


static void ubx_nav_timegps(const void *data)
{
	const struct nav_timegps_t {
		uint32_t iTOW; // time of week [ms]
		int32_t fTOW;  // time of week remainder [ns]
		int16_t week;  // number of week since 06JAN1980 (Sunday)
		int8_t leapS;
		uint8_t valid;
		uint32_t tAcc;
	} *gps = (struct nav_timegps_t *)data;

	fprintf(stderr, "itow %u  ftow %d  week %d  leap %d  flags %x  acc %u\n",
			le32toh(gps->iTOW), le32toh(gps->fTOW), le16toh(gps->week),
			gps->leapS, gps->valid, le32toh(gps->tAcc));
}


static int ubx(uint8_t cl, uint8_t id, const uint8_t *data, int len)
{
	if (verbose > 2) {
		fprintf(stderr, "\n\033[94mUBX: class 0x%02x, id 0x%02x, len %d: ", cl, id, len);
		for (int i = 0; i < len; i++)
			fprintf(stderr, "0x%02x ", data[i]);
		fprintf(stderr, "\033[m\n");
	}

	switch (cl) {
	case ACK:
		switch (id) {
		case ACK_ACK:
			if (verbose > 2)
				fprintf(stderr, "\033[92mack\033[m\n");
			break;

		case ACK_NAK:
			if (verbose > 2)
				fprintf(stderr, "\033[91mnak\033[m\n");
			break;

		default:
			break;
		}
		break;

	case NAV:
		switch (id) {
		case NAV_TIMEUTC:
			if (len == sizeof(struct nav_timeutc_t))
				ubx_nav_timeutc((const struct nav_timeutc_t *)data);
			break;

		case NAV_TIMEGPS:
			if (len == 16)
				ubx_nav_timegps(data);
			break;

		case NAV_SOL:
			if (len == 52 && verbose > 0)
				ubx_nav_sol(data);
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}
	return 0;
}


#define B(a) { #a, B##a }
static const struct baudrate {
	const char *name;
	int number;
} baudrates[] = {
	B(50), B(75), B(110), B(134), B(150), B(200), B(300), B(600), B(1200), B(1800),
	B(2400), B(4800), B(9600), B(19200), B(38400), B(57600), B(115200), B(230400),
	B(460800), B(500000), B(576000), B(921600), B(1000000), B(1152000), B(1500000),
	B(2000000), B(2500000), B(3000000), B(3500000), B(4000000),
	{ 0, 0 }
};
#undef B


int main(int argc, char *argv[])
{
	const char *device = "/dev/gps0";
	int baud = B9600;
	int ret;

	while (true) {
		int c;
		int option_index = 0;

		static struct option long_options[] = {
			{ "baud",              required_argument, 0, 'b' },
			{ "device",            required_argument, 0, 'd' },
			{ "help",              no_argument,       0, 'h' },
			{ "quiet",             no_argument,       0, 'q' },
			{ "verbose",           no_argument,       0, 'v' },
			{ 0,                   0,                 0,  0  }
		};

		c = getopt_long(argc, argv, "b:d:hqv", long_options, &option_index);
		if (c == -1)
			break;

		switch (c) {
		case 'b': {
			const struct baudrate *b;
			for (b = baudrates; b->name; b++) {
				if (!strcmp(optarg, b->name)) {
					baud = b->number;
					break;
				}
			}
			if (!b->name) {
				if (strcmp(optarg, "?"))
					fprintf(stderr, "unsupported baudrate %s\n\n"
							"available rates are:\n", optarg);
				for (b = baudrates; b->name; b++) {
					fprintf(stderr, "%s", b->name);
					if (b[1].name)
						fprintf(stderr, ", ");
				}
				fprintf(stderr, "\n");
				exit(-1);
			}
			break;
		}

		case 'd':
			device = optarg;
			break;

		case 'h':
			help();
			exit(0);
			break;

		case 'q':
			verbose = 0;
			break;

		case 'v':
			verbose++;
			break;

		case '?':
			break;

		default:
			printf("?? getopt returned character code 0%o ??\n", c);
		}
	}

	int fd = open(device, O_RDWR | O_NOCTTY);
	if (fd < 0)
		fatal("open tty");

	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = irq_handler;
	sigaction(SIGINT, &sa, NULL);   // Ctrl-c
	sigaction(SIGQUIT, &sa, NULL);  // Ctrl-\  (prevent segfault)

	if (ioctl(fd, TIOCEXCL, NULL) < 0)
		fatal("ioctl TIOEXCL");

	struct termios restore;
	ret = tcgetattr(fd, &restore);
	if (ret != 0)
		fatal("tcgetattr1");

	struct termios ios;
	cfmakeraw(&ios);
	cfsetospeed(&ios, baud);
	cfsetispeed(&ios, baud);

	tcflush(fd, TCIOFLUSH);

	ret = tcsetattr(fd, TCSANOW, &ios);
	if (ret != 0)
		fatal("tcsetattr1");

	while (optind < argc)
		fprintf(stderr, "don't know what to do with %s\n", argv[optind++]);

	//                                  USB                                          was:  7 (in)  7 (out)
	if (gps_send_cmd(fd, CFG, CFG_PRT, "\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x01\x00\x00\x00\x00\x00", 20)) // turn off NMEA on USB
		perror("gps_send_cmd");

	if (gps_send_cmd(fd, CFG, CFG_MSG, "\x01\x04\x00\x00\x00\x00\x00\x00", 8)) // turn off NAV_DOP
		perror("gps_send_cmd");

	if (gps_send_cmd(fd, CFG, CFG_MSG, "\x01\x06\x00\x00\x00\x00\x00\x00", 8)) // turn off NAV_SOL
		perror("gps_send_cmd");

	if (gps_send_cmd(fd, CFG, CFG_MSG, "\x01\x20\x00\x00\x00\x00\x00\x00", 8)) // turn off NAV_TIMEGPS
		perror("gps_send_cmd");

	if (gps_send_cmd(fd, CFG, CFG_MSG, "\x01\x30\x00\x00\x00\x00\x00\x00", 8)) // turn off NAV_SVINFO
		perror("gps_send_cmd");

	if (gps_send_cmd(fd, CFG, CFG_MSG, "\x01\x32\x00\x00\x00\x00\x00\x00", 8)) // turn off NAV_SBAS
		perror("gps_send_cmd");

	if (gps_send_cmd(fd, CFG, CFG_MSG, "\x01\x20\x00\x00\x00\x00\x00\x00", 8)) // turn off NAV_TIMEGPS
		perror("gps_send_cmd");

	if (gps_send_cmd(fd, CFG, CFG_MSG, "\x01\x21\x00\x00\x00\x01\x00\x00", 8)) // turn on NAV_TIMEUTC on USB every 1 cycles
		perror("gps_send_cmd");

	if (gps_send_cmd(fd, CFG, CFG_MSG, "\x01\x06\x00\x00\x00\x01\x00\x00", 8)) // turn on NAV_SOL on USB every 1 cycles
		perror("gps_send_cmd");

	if (gps_send_cmd(fd, NAV, NAV_TIMEUTC, NULL, 0))
		perror("gps_send_cmd");

	enum {
		IDLE,
		NMEA,
		UBX_PREFIX,
		UBX_HEADER,
		UBX_PAYLOAD,
	} state = IDLE;

	uint8_t buf[2048], *p = buf;
	int garbage = 0;
	int overflow = 0;
	int corrupt = 0;
	uint8_t cl, id;
	uint16_t len;

	len = cl = id = 0; // shut up compiler warnings

	while (!done) {
		uint8_t c;
		int n = read(fd, &c, sizeof(c));
		if (n == -1) {
			if (errno == EINTR)
				break;
			else
				fatal("read");

		} else if (!n) { // device has been disconnected
			fprintf(stderr, "read returned zero\n");
			break;
		}

		if (verbose > 3)
			fprintf(stderr, "0x%02x ", c);

		switch (state) {
		case IDLE:
			p = buf;
			if (c == '$') {
				state = NMEA;

			} else if (c == UBX_SYNC1) {
				state = UBX_PREFIX;

			} else {
				garbage++;
			}
			break;

		case NMEA:
			if (c == '\n') {
				if (p > buf && p[-1] == '\r') { // end NMEA
					*p = '\0';
					if (verbose)
						fprintf(stderr, "\n\033[36mNMEA: %s\033[m\n\n", buf);
				} else {
					corrupt++;
				}
				state = IDLE;

			} else if (p - buf >= sizeof(buf)) {
				overflow++;
				state = IDLE;

			} else {
				*p++ = c;
			}
			break;

		case UBX_PREFIX:
			if (c == UBX_SYNC2) {
				state = UBX_HEADER;

			} else {
				corrupt++;
				state = IDLE;
			}
			break;

		case UBX_HEADER:
			*p++ = c;
			if (p - buf == 4) {
				cl = buf[0];
				id = buf[1];
				len = buf[2] | ((uint16_t)buf[3] << 8);
				state = UBX_PAYLOAD;
			}
			break;

		case UBX_PAYLOAD:
			*p++ = c;
			if (p - buf == 4 + len + 2) {
				struct fletcher8_t f = { 0 };
				fl8_adds(&f, buf, 4 + len);
				if (p[-2] == f.a && p[-1] == f.b)
					ubx(cl, id, buf + 4, len);
				else
					fprintf(stderr, "UBX: wrong checksum\n");
				state = IDLE;
			}
		}
	}

	fprintf(stderr, "garbage:     %d\n", garbage);
	fprintf(stderr, "overflow:    %d\n", overflow);
	fprintf(stderr, "corrupt:     %d\n", corrupt);

	tcsetattr(fd, TCSANOW, &restore);
	close(fd);
	return EXIT_SUCCESS;
}
