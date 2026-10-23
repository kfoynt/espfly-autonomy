/*
 * ESP-Drone Firmware
 *
 * auto_mailbox.c - The autonomy command mailbox and flash-write interlock
 * (auto_mailbox.h, docs/autonomy-a2b.md sections 3.1 and 7.3). Pure code;
 * the caller holds the lock. Host-tested by tools/tests/test_auto_mailbox.c.
 */
#include <string.h>

#include "auto_mailbox.h"

void autoMbInit(autoMailbox_t *mb)
{
  memset(mb, 0, sizeof(*mb));
  mb->state = AUTO_IDLE;
}

autoMbPost_t autoMbPost(autoMailbox_t *mb, const autoCmd_t *cmd, uint32_t *ticket)
{
  if (cmd->type == AUTO_CMD_NONE || cmd->type == AUTO_CMD_KILL || mb->cmdTicket != 0) {
    mb->slotBusy++;
    return AUTO_MB_SLOT_BUSY;
  }
  if (mb->resTicket != 0) {
    mb->resTicket = 0;     /* the link stopped waiting for it */
    mb->staleResults++;
  }
  uint32_t t = mb->lastTicket + 1u;
  if (t == 0u) {
    t = 1u;
  }
  mb->lastTicket = t;
  mb->cmd = *cmd;
  mb->cmdTaken = false;
  __atomic_store_n(&mb->cmdTicket, t, __ATOMIC_RELEASE);
  mb->posts++;
  *ticket = t;
  return AUTO_MB_POSTED;
}

bool autoMbTakeResult(autoMailbox_t *mb, uint32_t ticket, autoCmdResult_t *out)
{
  if (ticket == 0u || mb->resTicket != ticket) {
    return false;
  }
  *out = mb->res;
  mb->resTicket = 0;
  return true;
}

autoMbCancel_t autoMbCancel(autoMailbox_t *mb, uint32_t ticket)
{
  if (ticket != 0u && mb->cmdTicket == ticket) {
    if (mb->cmdTaken) {
      return AUTO_MB_TAKEN;
    }
    __atomic_store_n(&mb->cmdTicket, 0u, __ATOMIC_RELEASE);
    mb->cancels++;
    return AUTO_MB_CANCELLED;
  }
  if (ticket != 0u && mb->resTicket == ticket) {
    return AUTO_MB_DONE;
  }
  return AUTO_MB_UNKNOWN;
}

bool autoMbTake(autoMailbox_t *mb, autoCmd_t *cmd, uint32_t *ticket, bool *flashBusy)
{
  *flashBusy = mb->flashWriters != 0;
  if (mb->cmdTicket == 0u || mb->cmdTaken) {
    return false;
  }
  __atomic_store_n(&mb->cmdTaken, true, __ATOMIC_RELEASE);
  *cmd = mb->cmd;
  *ticket = mb->cmdTicket;
  if (*flashBusy && cmd->type == AUTO_CMD_ARM) {
    mb->armsBlocked++;
  }
  return true;
}

bool autoMbPublish(autoMailbox_t *mb, uint32_t ticket, autoResult_t result, uint32_t detail, autoState_t state)
{
  if (ticket == 0u || mb->cmdTicket != ticket || !mb->cmdTaken) {
    return false;
  }
  mb->res.ticket = ticket;
  mb->res.result = result;
  mb->res.detail = detail;
  mb->res.state = state;
  mb->resTicket = ticket;
  mb->state = (uint8_t)state;
  mb->cmdTaken = false;
  __atomic_store_n(&mb->cmdTicket, 0u, __ATOMIC_RELEASE);
  return true;
}

void autoMbSetState(autoMailbox_t *mb, autoState_t state)
{
  mb->state = (uint8_t)state;
}

bool autoMbFlashBegin(autoMailbox_t *mb)
{
  const bool armInSlot = mb->cmdTicket != 0u && mb->cmd.type == AUTO_CMD_ARM;
  if (mb->state != AUTO_IDLE || armInSlot || mb->flashWriters == UINT8_MAX) {
    mb->flashRefused++;
    return false;
  }
  mb->flashWriters++;
  return true;
}

void autoMbFlashEnd(autoMailbox_t *mb)
{
  if (mb->flashWriters > 0) {
    mb->flashWriters--;
  }
}
