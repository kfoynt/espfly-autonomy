/*
 * ESP-Drone Firmware
 *
 * auto_mailbox.h - The command mailbox between the autonomy link task and
 * the stabilizer task, and the flash-write interlock (stage A2b,
 * docs/autonomy-a2b.md sections 3.1 and 7.3). Step 0: frozen, owned by the
 * integrator; modules/src/autonomy.c (WP1, and the step-0 stub) is the only
 * drone caller, tools/tests/auto_link_host links the same file.
 *
 * Pure code, no ESP-IDF or FreeRTOS header. The caller holds ONE lock around
 * every call except autoMbPending (a portMUX spinlock in autonomy.c; nothing
 * in the single-threaded host tests). No call blocks, allocates or loops.
 * Host-tested by tools/tests/test_auto_mailbox.c (the race table).
 *
 * Contract:
 *   - One command at a time. The link posts, then polls autoMbTakeResult each
 *     loop while it keeps serving other packets (KILL, HB, photos); it never
 *     waits inside a call.
 *   - The stabilizer takes the command once (autoMbTake) and publishes its
 *     result after the FSM step that processed it (autoMbPublish). The slot
 *     is free again only at the publish (or a successful cancel).
 *   - Cancel before the take: the command is guaranteed never to be
 *     processed. Cancel after the take: TAKEN, the result follows and is
 *     either read or dropped by the next post.
 *   - Tickets are never 0 and never repeat within 2^32 posts; a result is
 *     only ever returned to the ticket that produced it.
 *   - Flash interlock: an NVS/flash write may start only while the FSM is in
 *     IDLE and no ARM is in the slot, and an ARM taken while a write is in
 *     progress is refused by the FSM (G1). Because both sides decide under
 *     the same lock, "no flash write while autonomy is active" holds without
 *     a check-then-act race.
 */
#ifndef AUTO_MAILBOX_H_
#define AUTO_MAILBOX_H_

#include <stdbool.h>
#include <stdint.h>

#include "auto_fsm.h"     /* autoCmd_t, autoState_t */
#include "auto_proto.h"   /* autoResult_t */

/* ACK detail of a BUSY whose command the FSM took but whose result did not
 * arrive within AUTONOMY_CMD_WAIT_HARD_MS: it may have been acted on. The Mac
 * reconciles through AUTO_STATUS (state, lastCmdType, lastCmdResult) before
 * sending again (docs/autonomy-a2b.md section 3.1). */
#define AUTO_DETAIL_TAKEN_UNKNOWN 0xFFFFFFFFu

/* The result of one command, as the link ACKs it (byte 6 = state). */
typedef struct {
  uint32_t ticket;         /* from autoMbPost */
  autoResult_t result;     /* autoFsmOut_t.cmdResult */
  uint32_t detail;         /* autoFsmOut_t.cmdDetail (ARM OK: armToken; GUARDS: bitmap; else the state) */
  autoState_t state;       /* the FSM state after the step that processed it */
} autoCmdResult_t;

typedef enum {
  AUTO_MB_POSTED = 0,      /* posted; *ticket identifies it */
  AUTO_MB_SLOT_BUSY,       /* the slot still holds a command (waiting, or taken and not yet published) */
} autoMbPost_t;

typedef enum {
  AUTO_MB_CANCELLED = 0,   /* withdrawn before the FSM took it: never processed */
  AUTO_MB_TAKEN,           /* the FSM took it and has not published yet: its result follows */
  AUTO_MB_DONE,            /* its result is waiting: read it with autoMbTakeResult */
  AUTO_MB_UNKNOWN,         /* not this mailbox's current ticket (already read, or superseded) */
} autoMbCancel_t;

typedef struct {
  uint32_t lastTicket;     /* the last ticket issued (0 before the first) */
  uint32_t cmdTicket;      /* the command in the slot, 0: empty */
  bool cmdTaken;           /* the FSM took it */
  autoCmd_t cmd;
  uint32_t resTicket;      /* the result waiting, 0: none */
  autoCmdResult_t res;
  uint8_t state;           /* autoState_t the glue last published (autoMbSetState / autoMbPublish) */
  uint8_t flashWriters;    /* NVS/flash writes in progress */
  /* statistics */
  uint32_t posts, slotBusy, cancels, staleResults, flashRefused, armsBlocked;
} autoMailbox_t;

void autoMbInit(autoMailbox_t *mb);   /* empty, state IDLE */

/* ---- link task ----------------------------------------------------------- */

/* Posts cmd (never AUTO_CMD_KILL or NONE: those return SLOT_BUSY untouched).
 * An unread result of an earlier ticket is dropped (staleResults++): the
 * link only posts after it stopped waiting for the previous one. */
autoMbPost_t autoMbPost(autoMailbox_t *mb, const autoCmd_t *cmd, uint32_t *ticket);
/* true once ticket's result is published: copies it and empties the result slot. */
bool autoMbTakeResult(autoMailbox_t *mb, uint32_t ticket, autoCmdResult_t *out);
autoMbCancel_t autoMbCancel(autoMailbox_t *mb, uint32_t ticket);

/* ---- stabilizer task ----------------------------------------------------- */

/* The command to step with, at most once per ticket. *flashBusy is true when
 * a flash write is in progress at the take: the glue then makes G1 fail
 * (in.motorsReady false) so an ARM is refused with GUARDS/LOCKS. */
bool autoMbTake(autoMailbox_t *mb, autoCmd_t *cmd, uint32_t *ticket, bool *flashBusy);
/* After the FSM step that processed the taken command: stores its result,
 * sets the published state and frees the slot. false (nothing stored) unless
 * ticket is the taken command. */
bool autoMbPublish(autoMailbox_t *mb, uint32_t ticket, autoResult_t result, uint32_t detail, autoState_t state);
/* Every FSM transition (also those without a command): the state the flash
 * interlock reads. */
void autoMbSetState(autoMailbox_t *mb, autoState_t state);

/* Lock-free hint for the IDLE fast path (an aligned word read): a command is
 * in the slot and not taken yet. */
static inline bool autoMbPending(const autoMailbox_t *mb)
{
  return __atomic_load_n(&mb->cmdTicket, __ATOMIC_ACQUIRE) != 0 && !__atomic_load_n(&mb->cmdTaken, __ATOMIC_ACQUIRE);
}

/* ---- flash writers (imu_level_cal_store.c write paths) -------------------- */

/* true: the write may start (flashWriters++). false (flashRefused++): the
 * FSM is not in IDLE or an ARM is in the slot. Every true is paired with one
 * autoMbFlashEnd after the write. */
bool autoMbFlashBegin(autoMailbox_t *mb);
void autoMbFlashEnd(autoMailbox_t *mb);

#endif /* AUTO_MAILBOX_H_ */
