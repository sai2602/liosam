/*------------------------------------------------------------------------------
*
* UBX-like protocol type definitions
*
*   custom type definitions
*
* authors : Robert Bensch, ANavS GmbH
* history : 2018/04/18 - Initial file creation
*           2019/03/12 - Merged Julius extension of sUbxDataSolution
*-----------------------------------------------------------------------------*/

#ifndef UBX_TYPES_H
#define UBX_TYPES_H

//
// UBX message protocol header
//  see definitions in: ANAVS_RTK_software/rtklib/rcv/ublox.c
//
#define SYNC1_UBX      0xB5     // ubx message sync code 1
#define SYNC2_UBX      0x62     // ubx message sync code 2

#define CLSID_UBX      0x02     // ubx message class id

#define MSGID_IMU      0x49     // ubx message id: (0x02 0x49) IMU raw meas data (rev 1)
#define MSGID_IMUB     0x4A     // ubx message id: (0x02 0x4A) IMU raw meas data for optional IMU
#define MSGID_WHEEL    0xfd     // ubx message id: (0x02 0xfd) ODO raw meas data
#define MSGID_CAN      0xfe     // ubx message id: (0x02 0xfe) CAN odometry meas data
#define MSGID_SOL      0xE0     // ubx message id: (0x02 0xE0) Solution

// Header definitions
typedef struct {
    unsigned char HEAD1;
    unsigned char HEAD2;
    unsigned char CLASSID;
    unsigned char MSGID;
    int bPayloadLen;    // 2/4 bytes payload length field
} sUbxHeaderDef;

// Checksum definitions
typedef struct {
   unsigned char chkA;
   unsigned char chkB;
} fletcher_checksum_t;

// Decoding variables
typedef union {
    uint8_t timingInfo;
    unsigned char timingInfoByte[1];
} uint8_union;

typedef union {
   uint16_t uInt16;
   unsigned char bytes[2];
} uint16_union;

typedef union {
   uint32_t uInt32;
   unsigned char bytes[4];
} uint32_union;

typedef union {
    uint64_t towUs;
    unsigned char t_bytes[8];
} uint64_union;

typedef union {
  int16_t sInt16;
  unsigned char bytes[2];
} int16_union;

typedef union {
  float_t sFloat;
  unsigned char bytes[4];
} float_union;

typedef union {
  double_t sDouble;
  unsigned char bytes[8];
} double_union;

typedef struct {
    uint8_union timeInfo_help;
    uint64_union towGNSS;
    int16_union acc_x;
    int16_union acc_y;
    int16_union acc_z;
    int16_union gyro_x;
    int16_union gyro_y;
    int16_union gyro_z;
    int16_union magn_x;
    int16_union magn_y;
    int16_union magn_z;
} sUbxDataImu;

typedef struct {
    uint64_union towGNSS;
    int16_union left_rate;
    int16_union right_rate;
    int16_union left_current;
    int16_union right_current;
} sUbxDataWheel;

typedef struct {
    uint8_union rcv_id;
    uint8_union timingInfo;
    uint64_union towGNSS;
    float_union speedFl;
    float_union speedFr;
    float_union speedRl;
    float_union speedRr;
    float_union wheelHeading;
} sUbxDataCanOdom;

typedef struct {
    uint8_union id;
    uint16_union resCode;
    uint16_union week;
    double_union tow;
    uint16_union weekInit;
    double_union towInit;
    double_union lat;
    double_union lon;
    double_union height;
    double_union x_ECEF;
    double_union y_ECEF;
    double_union z_ECEF;
    double_union b_N;
    double_union b_E;
    double_union b_D;
    double_union b_N_std;
    double_union b_E_std;
    double_union b_D_std;
    double_union v_N;
    double_union v_E;
    double_union v_D;
    double_union v_N_std;
    double_union v_E_std;
    double_union v_D_std;
    double_union acc_X;
    double_union acc_Y;
    double_union acc_Z;
    double_union acc_X_std;
    double_union acc_Y_std;
    double_union acc_Z_std;
    double_union att_head;
    double_union att_pitch;
    double_union att_roll;
    double_union att_head_std;
    double_union att_pitch_std;
    double_union att_roll_std;
    double_union accuracy;
    double_union timingInfo_Elapsed_GNSS;
    double_union timingInfo_Elapsed_IMU;
    double_union timingInfo_Elapsed_BARO;
    double_union timingInfo_Elapsed_ODO;
    double_union gnss_rcptn;
    uint8_union num_sat;

} sUbxDataSolution;

typedef struct {
    int head1;
    int head2;
    int classId;
    int msgId;
    int bCount2;
    int sensor;
    uint16_union pl2;
    uint32_union pl4;
    int payloadData;
    unsigned int bPayload;
    unsigned int bCount;         // payload byte count
    sUbxDataImu imu;
    sUbxDataWheel wheel;
    sUbxDataCanOdom canOdom;
    sUbxDataSolution sol;
} sUbxDecodeState;

void reset( sUbxDecodeState& uS) {
    uS = {};    // initialize to zeros
}

#endif // UBX_TYPES_H
