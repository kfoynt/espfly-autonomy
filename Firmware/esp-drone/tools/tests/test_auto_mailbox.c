/*
 * Host test of the autonomy command mailbox and the flash-write interlock
 * (components/core/crazyflie/utils/src/auto_mailbox.c, docs/autonomy-a2b.md
 * sections 3.1 and 7.3): the race table, then random interleavings of the
 * link task, the stabilizer task and a flash writer, each call one locked
 * step, against the contract's invariants.
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_auto_mailbox.c components/core/crazyflie/utils/src/auto_mailbox.c \
 *      -lm -o /tmp/test_auto_mailbox && /tmp/test_auto_mailbox
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "auto_mailbox.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

static autoCmd_t cmdOf(autoCmdType_t t)
{
  autoCmd_t c;
  memset(&c, 0, sizeof(c));
  c.type = t;
  return c;
}

static void testTable(void)
{
  autoMailbox_t mb;
  autoCmd_t c, got;
  autoCmdResult_t r;
  uint32_t t1 = 0, t2 = 0, tk = 0;
  bool fb = true;

  /* 1. The normal path. */
  autoMbInit(&mb);
  CHECK(mb.state == AUTO_IDLE && !autoMbPending(&mb), "init: empty, IDLE");
  c = cmdOf(AUTO_CMD_ARM);
  CHECK(autoMbPost(&mb, &c, &t1) == AUTO_MB_POSTED && t1 == 1u && autoMbPending(&mb), "post: ticket 1, pending");
  CHECK(!autoMbTakeResult(&mb, t1, &r), "no result before the step");
  CHECK(autoMbTake(&mb, &got, &tk, &fb) && tk == t1 && got.type == AUTO_CMD_ARM && !fb && !autoMbPending(&mb),
        "take: the command, once, no flash write");
  CHECK(!autoMbTake(&mb, &got, &tk, &fb), "take: never twice");
  CHECK(autoMbPublish(&mb, t1, AUTO_R_OK, 0x1234u, AUTO_ARMED) && mb.state == AUTO_ARMED, "publish");
  CHECK(!autoMbTakeResult(&mb, t1 + 1u, &r) && !autoMbTakeResult(&mb, 0u, &r),
        "a waiting result is never returned to another ticket");
  CHECK(autoMbTakeResult(&mb, t1, &r) && r.ticket == t1 && r.result == AUTO_R_OK && r.detail == 0x1234u &&
        r.state == AUTO_ARMED, "result: fields");
  CHECK(!autoMbTakeResult(&mb, t1, &r), "result: read once");

  /* 2. Cancel before the take: never processed. */
  autoMbInit(&mb);
  c = cmdOf(AUTO_CMD_TAKEOFF);
  autoMbPost(&mb, &c, &t1);
  CHECK(autoMbCancel(&mb, t1) == AUTO_MB_CANCELLED && !autoMbPending(&mb), "cancel before take: CANCELLED");
  CHECK(!autoMbTake(&mb, &got, &tk, &fb), "cancelled: never taken");
  CHECK(!autoMbPublish(&mb, t1, AUTO_R_OK, 0, AUTO_TAKEOFF), "cancelled: no publish");
  CHECK(autoMbCancel(&mb, t1) == AUTO_MB_UNKNOWN, "cancel again: UNKNOWN");

  /* 3. Cancel after the take: the result follows. */
  autoMbInit(&mb);
  autoMbPost(&mb, &c, &t1);
  autoMbTake(&mb, &got, &tk, &fb);
  CHECK(autoMbCancel(&mb, t1) == AUTO_MB_TAKEN, "cancel after take: TAKEN");
  CHECK(autoMbPost(&mb, &c, &t2) == AUTO_MB_SLOT_BUSY, "post while taken and unpublished: SLOT_BUSY");
  CHECK(autoMbPublish(&mb, t1, AUTO_R_BAD_STATE, 3u, AUTO_ARMED), "publish after a TAKEN cancel");
  CHECK(autoMbCancel(&mb, t1) == AUTO_MB_DONE, "cancel after the publish: DONE");
  CHECK(autoMbTakeResult(&mb, t1, &r) && r.result == AUTO_R_BAD_STATE, "the result is the taken command's");

  /* 4. A stale result for an old ticket never reaches a new one. */
  autoMbInit(&mb);
  autoMbPost(&mb, &c, &t1);
  autoMbTake(&mb, &got, &tk, &fb);
  autoMbPublish(&mb, t1, AUTO_R_OK, 7u, AUTO_TAKEOFF);   /* the link had given up on t1 (BUSY, taken-unknown) */
  c = cmdOf(AUTO_CMD_LAND);
  CHECK(autoMbPost(&mb, &c, &t2) == AUTO_MB_POSTED && t2 != t1 && mb.staleResults == 1u,
        "post with an unread result: posted, the old result dropped");
  CHECK(!autoMbTakeResult(&mb, t1, &r) && !autoMbTakeResult(&mb, t2, &r), "neither ticket sees t1's result");
  autoMbTake(&mb, &got, &tk, &fb);
  CHECK(tk == t2 && got.type == AUTO_CMD_LAND, "the new command is taken");
  autoMbPublish(&mb, t2, AUTO_R_OK, 9u, AUTO_LAND);
  CHECK(autoMbTakeResult(&mb, t2, &r) && r.ticket == t2 && r.detail == 9u, "t2 gets its own result");

  /* 5. Post while a command waits; KILL and NONE are never posted. */
  autoMbInit(&mb);
  c = cmdOf(AUTO_CMD_ARM);
  autoMbPost(&mb, &c, &t1);
  CHECK(autoMbPost(&mb, &c, &t2) == AUTO_MB_SLOT_BUSY && mb.slotBusy == 1u, "post while waiting: SLOT_BUSY");
  autoMbInit(&mb);
  c = cmdOf(AUTO_CMD_KILL);
  CHECK(autoMbPost(&mb, &c, &t1) == AUTO_MB_SLOT_BUSY && mb.cmdTicket == 0u, "KILL is never posted");
  c = cmdOf(AUTO_CMD_NONE);
  CHECK(autoMbPost(&mb, &c, &t1) == AUTO_MB_SLOT_BUSY && mb.cmdTicket == 0u, "NONE is never posted");

  /* 6. Publish only the taken command. */
  autoMbInit(&mb);
  c = cmdOf(AUTO_CMD_DISARM);
  autoMbPost(&mb, &c, &t1);
  CHECK(!autoMbPublish(&mb, t1, AUTO_R_OK, 0, AUTO_IDLE), "publish before the take: refused");
  autoMbTake(&mb, &got, &tk, &fb);
  CHECK(!autoMbPublish(&mb, t1 + 1u, AUTO_R_OK, 0, AUTO_IDLE) && !autoMbPublish(&mb, 0u, AUTO_R_OK, 0, AUTO_IDLE),
        "publish with another ticket: refused");
  CHECK(!autoMbTakeResult(&mb, 0u, &r), "ticket 0 never has a result");

  /* 7. Ticket wrap: never 0. */
  autoMbInit(&mb);
  mb.lastTicket = 0xFFFFFFFFu;
  c = cmdOf(AUTO_CMD_LAND);
  CHECK(autoMbPost(&mb, &c, &t1) == AUTO_MB_POSTED && t1 == 1u, "wrap: 0xFFFFFFFF -> 1, never 0");

  /* 8. Flash interlock. */
  autoMbInit(&mb);
  CHECK(autoMbFlashBegin(&mb) && mb.flashWriters == 1u, "IDLE, empty slot: a write may start");
  c = cmdOf(AUTO_CMD_ARM);
  autoMbPost(&mb, &c, &t1);
  CHECK(!autoMbFlashBegin(&mb) && mb.flashRefused == 1u, "an ARM waiting in the slot: a second write is refused");
  CHECK(autoMbTake(&mb, &got, &tk, &fb) && fb && mb.armsBlocked == 1u, "an ARM taken during a write: flashBusy");
  autoMbPublish(&mb, t1, AUTO_R_GUARDS, AUTO_G_LOCKS, AUTO_IDLE);   /* G1 refused it */
  autoMbFlashEnd(&mb);
  autoMbFlashEnd(&mb);
  CHECK(mb.flashWriters == 0u, "end never goes below 0");
  autoMbPost(&mb, &c, &t1);
  autoMbTake(&mb, &got, &tk, &fb);
  CHECK(!fb && !autoMbFlashBegin(&mb), "an ARM taken, not published: a write is refused");
  autoMbPublish(&mb, t1, AUTO_R_OK, 1u, AUTO_ARMED);
  CHECK(!autoMbFlashBegin(&mb), "ARMED: a write is refused");
  autoMbSetState(&mb, AUTO_LAND);
  CHECK(!autoMbFlashBegin(&mb), "LAND: a write is refused");
  autoMbSetState(&mb, AUTO_IDLE);
  c = cmdOf(AUTO_CMD_LAND);
  autoMbPost(&mb, &c, &t1);
  CHECK(autoMbFlashBegin(&mb), "IDLE with a LAND in the slot (cannot leave IDLE): a write may start");
}

/* ---- random interleavings ------------------------------------------------- */

static uint32_t rs = 12345u;
static uint32_t rnd(uint32_t n)
{
  rs ^= rs << 13;
  rs ^= rs >> 17;
  rs ^= rs << 5;
  return rs % n;
}

typedef enum { L_IDLE, L_WAIT, L_WAIT_TAKEN } linkPhase_t;
typedef enum { S_READY, S_HAVE, S_STEPPED } stabPhase_t;

static void testInterleavings(void)
{
  const int runs = 2000, steps = 4000;
  int bad = 0, flashArmed = 0, wrongResult = 0, cancelledTaken = 0, lost = 0, unknownWhileWaiting = 0;
  uint32_t results = 0, cancelled = 0, abandoned = 0, armsAccepted = 0, writes = 0, blocked = 0;
  for (int run = 0; run < runs; run++) {
    autoMailbox_t mb;
    autoMbInit(&mb);
    linkPhase_t lp = L_IDLE;
    stabPhase_t sp = S_READY;
    uint32_t lt = 0, st = 0, waitTurns = 0;
    autoCmd_t scmd;
    autoCmdResult_t r;
    bool sFlash = false;
    autoState_t trueState = AUTO_IDLE, after = AUTO_IDLE;
    uint32_t lastCancelled = 0;
    int writing = 0;
    for (int k = 0; k < steps; k++) {
      switch (rnd(3)) {
        case 0:   /* link */
          if (lp == L_IDLE) {
            static const autoCmdType_t types[4] = { AUTO_CMD_ARM, AUTO_CMD_TAKEOFF, AUTO_CMD_LAND, AUTO_CMD_DISARM };
            autoCmd_t c = cmdOf(types[rnd(4)]);
            if (autoMbPost(&mb, &c, &lt) == AUTO_MB_POSTED) {
              lp = L_WAIT;
              waitTurns = 0;
            }
          } else {
            waitTurns++;
            if (autoMbTakeResult(&mb, lt, &r)) {
              wrongResult += r.ticket != lt;
              results++;
              lp = L_IDLE;
            } else if (waitTurns > 6 + rnd(6)) {   /* the 100 ms / 200 ms timeouts */
              const autoMbCancel_t cr = autoMbCancel(&mb, lt);
              if (cr == AUTO_MB_CANCELLED) {
                lastCancelled = lt;
                cancelled++;
                lp = L_IDLE;
              } else if (cr == AUTO_MB_TAKEN && lp == L_WAIT) {
                lp = L_WAIT_TAKEN;   /* wait to the hard limit */
                waitTurns = 0;
              } else if (cr == AUTO_MB_TAKEN) {
                abandoned++;         /* BUSY, taken-unknown */
                lp = L_IDLE;
              } else if (cr == AUTO_MB_DONE) {
                lost += !autoMbTakeResult(&mb, lt, &r);
                results++;
                lp = L_IDLE;
              } else {
                unknownWhileWaiting++;
                lp = L_IDLE;
              }
            }
          }
          break;
        case 1:   /* stabilizer: take, step (no lock), publish */
          if (rnd(4) == 0) {
            break;   /* a stalled loop */
          }
          if (sp == S_READY) {
            if (autoMbTake(&mb, &scmd, &st, &sFlash)) {
              cancelledTaken += st == lastCancelled;
              sp = S_HAVE;
            }
          } else if (sp == S_HAVE) {
            after = trueState;
            if (scmd.type == AUTO_CMD_ARM && trueState == AUTO_IDLE && !sFlash) {
              after = AUTO_ARMED;
              armsAccepted++;
            } else if (scmd.type == AUTO_CMD_ARM && sFlash) {
              blocked++;
            } else if ((scmd.type == AUTO_CMD_LAND || scmd.type == AUTO_CMD_DISARM) && trueState != AUTO_IDLE) {
              after = AUTO_IDLE;
            }
            trueState = after;   /* the FSM changed state in the step, before the publish */
            sp = S_STEPPED;
          } else {
            bad += !autoMbPublish(&mb, st, AUTO_R_OK, 0, trueState);
            sp = S_READY;
          }
          break;
        default:  /* flash writer */
          if (writing) {
            autoMbFlashEnd(&mb);
            writing = 0;
          } else if (autoMbFlashBegin(&mb)) {
            writing = 1;
            writes++;
          }
          break;
      }
      flashArmed += writing && trueState != AUTO_IDLE;
    }
  }
  CHECK(bad == 0, "interleavings: every taken command published exactly once (%d refused)", bad);
  CHECK(wrongResult == 0, "interleavings: a result only ever reaches its own ticket (%d)", wrongResult);
  CHECK(cancelledTaken == 0, "interleavings: a CANCELLED command is never taken (%d)", cancelledTaken);
  CHECK(lost == 0 && unknownWhileWaiting == 0, "interleavings: DONE always has the result (%d), UNKNOWN never while waiting (%d)",
        lost, unknownWhileWaiting);
  CHECK(flashArmed == 0, "interleavings: no flash write while the FSM is out of IDLE (%d steps)", flashArmed);
  CHECK(results > 0 && cancelled > 0 && abandoned > 0 && armsAccepted > 0 && writes > 0 && blocked > 0,
        "interleavings: every path ran (results %u, cancelled %u, abandoned %u, ARMs %u, writes %u, ARMs blocked %u)",
        results, cancelled, abandoned, armsAccepted, writes, blocked);
  printf("info: %d runs x %d steps: results %u, cancelled %u, taken-unknown %u, ARMs accepted %u, flash writes %u, ARMs blocked by a write %u\n",
         runs, steps, results, cancelled, abandoned, armsAccepted, writes, blocked);
}

int main(void)
{
  testTable();
  testInterleavings();
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
