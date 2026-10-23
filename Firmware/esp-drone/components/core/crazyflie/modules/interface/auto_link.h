/*
 * ESP-Drone Firmware
 *
 * auto_link.h - Autonomy link to the Mac on UDP 2391 (docs/autonomy-link.md)
 *
 * Stage A1: status and flow-camera photos. Stage A2b (docs/autonomy-a2b.md
 * section 3): the flight commands ARM, TAKEOFF, LAND and DISARM (and PHOTO
 * while autonomy is active) go to the autonomy state machine through
 * autonomy.h's mailbox, asynchronously: posted, then acknowledged with the
 * state machine's result when it arrives, while the task keeps serving every
 * other packet; ACK BUSY (never cached) on a timeout. KILL (any session, any
 * seq, valid tag; its own pre-auth budget) calls autonomyKillFromLink()
 * before anything else and is answered at the sender's address; only a HELLO
 * or a current-session packet sets the peer. AUTO_STATUS is pushed while
 * autonomy is active; photo sets the state machine starts are captured and
 * pushed in the air (after the take-off window). The link never writes the
 * setpoint, the motors (except the KILL stop) or the drift hold.
 *
 * The port is opened only if a 16-byte key is stored in NVS (USB console
 * 'autokey'); the key is read at boot, so a new key takes effect after a
 * reboot. A low-priority task owns the socket; its stack is in PSRAM (it
 * never writes flash: the key is read in autoLinkInit), internal RAM only if
 * PSRAM is missing. Photo frames are copied in the camera task (camera_flow.h
 * snapshot) into PSRAM.
 */
#ifndef AUTO_LINK_H_
#define AUTO_LINK_H_

#include <stdbool.h>
#include <stdint.h>

void autoLinkInit(void);

typedef struct {
  bool keyLoaded;          /* a key was read at boot */
  uint8_t keyFp[4];        /* first 4 bytes of SHA-256(key) */
  bool running;            /* task started */
  uint32_t stackFree;      /* task's lowest free stack, bytes */
  bool portOpen;           /* socket bound to UDP 2391 */
  uint32_t session;        /* 0 = none */
  bool havePeer;
  uint8_t peerIp[4];       /* where pushes go (AUTO_STATUS, META, CHUNK): set by a HELLO that opens or
                            * replays the session and by a new-seq packet of the current session; replies
                            * (ACK, HELLO_ACK, STATUS) go to each packet's sender */
  uint16_t peerPort;
  uint32_t rxDatagrams;    /* everything received on the port */
  uint32_t rateDrops;      /* over 60 packets/s, dropped before the tag check */
  uint32_t authFails;      /* bad magic, version, length or tag */
  uint32_t accepted;       /* valid tag, session and new seq (HELLO included) */
  uint32_t dups;           /* repeats answered from the ack cache */
  uint32_t oldSeq;         /* older seq, dropped */
  uint32_t badSession;
  uint32_t txPackets;
  uint32_t txErrors;
  uint32_t txNoMem;        /* sendto ENOMEM (Wi-Fi TX queue full); retried */
  uint32_t photos;         /* PHOTO commands accepted */
  bool photoBusy;          /* a photo set is held */
  uint16_t photoId;
  uint32_t hbAgeMs;        /* since the last HB, UINT32_MAX if none */
  uint8_t lastAckType;     /* last ACK sent: command type and result */
  uint8_t lastAckResult;
  bool anyAck;
  /* A2b (append only) */
  bool stackInPsram;       /* the task's stack is in PSRAM */
  uint32_t kills;          /* KILL packets acted on (valid tag, any session) */
  uint32_t autoCmds;       /* ARM/TAKEOFF/LAND/DISARM/PHOTO given to the state machine */
  uint32_t cmdTimeouts;    /* commands withdrawn unprocessed at 100 ms (ACK BUSY, not cached) */
  uint32_t autoStatusTx;   /* AUTO_STATUS packets sent */
  uint32_t autoCaptures;   /* photo sets started for the state machine */
  uint32_t photosPreempted;/* held A1 or earlier-sortie sets released to start a state-machine set */
  uint32_t cmdBusy;        /* commands answered BUSY at once: one already pending, or the mailbox slot held */
  uint32_t cmdTakenUnknown;/* commands answered BUSY with AUTO_DETAIL_TAKEN_UNKNOWN (taken, no result in 200 ms) */
  uint32_t photoHeldBusy;  /* PHOTO refused while a state-machine set is still held */
  uint32_t killRateDrops;  /* 0x13 datagrams over the KILL budget (AUTO_KILL_RATE_MAX_PER_S) */
} autoLinkStats_t;

void autoLinkGetStats(autoLinkStats_t *out);

/* Key fingerprint as shown by both sides: first 4 bytes of SHA-256(key). */
void autoLinkKeyFingerprint(const uint8_t key[16], uint8_t fp[4]);

/* Name of an ACK result code (auto_proto.h autoResult_t). */
const char *autoLinkResultName(uint8_t result);

#endif /* AUTO_LINK_H_ */
