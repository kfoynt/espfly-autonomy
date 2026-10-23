# Autonomy link (Mac <-> drone), protocol v1

The Mac joins the drone's Wi-Fi access point (the drone is 192.168.43.42)
through its built-in Wi-Fi while it stays online through another interface.
The autonomy link is a separate UDP port next to the phone app's CRTP port:

| Port | Use |
|---|---|
| UDP 2390 | phone app (CRTP), unchanged |
| UDP 2391 | autonomy link (this document) |

Stage A1 carries photos and status. Stage A2b adds the flight commands (ARM,
TAKEOFF, LAND, DISARM), the Mac KILL and the AUTO_STATUS telemetry; the
state machine, the vertical controller and their guards are in
`docs/autonomy-a2.md`, the glue in `docs/autonomy-a2b.md`. The protocol
version stays 1: every A1 packet is unchanged, two A1 bytes that were
always 0 now carry the autonomy state (STATUS byte 7, ACK byte 6), and a
PHOTO refused `BUSY` now marks its detail as a held-set id (`0x03000000` |
id; the low 16 bits are the id, as in A1). The interactive hover (user
decision 2026-10-12, late; `docs/autonomy-a2b.md` section 14) uses the three
types stage C had reserved, YAW 0x30, MOVE 0x31 (stage C's STEP) and STOP
0x32 (its ABORT_MOVE), lets TAKEOFF carry the sortie's wait, and appends 32
bytes to AUTO_STATUS (96 bytes; the first 64 unchanged). Since 2026-10-14
(`docs/autonomy-a2b.md` section 21) a MOVE goes up to 4 m forward and
TAKEOFF can carry a planned move (18 bytes; the 12 of before unchanged).
Since 2026-10-15 ("Claude decides", `docs/autonomy-a2b.md` section 22) the
drone holds and reports instead of landing for the unusual: AUTO_STATUS
gains 40 bytes of report (136 bytes; the first 96 unchanged but `airLeftDs`,
now always 0: there is no air clock).

The link never writes the setpoint, the motors or the drift hold itself. It
posts commands to the state machine (autonomy.h's mailbox), and a KILL calls
`autonomyKillFromLink()` (motor stop if any duty is nonzero, then the state
machine goes to KILLED).

## Packet

All integers little-endian.

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | magic `0xE5` |
| 1 | 1 | version `1` |
| 2 | 1 | type |
| 3 | 1 | flags: bit0 `ACKREQ`, bit1 `RETX` (a retransmission) |
| 4 | 4 | session |
| 8 | 4 | seq |
| 12 | 2 | payload length (0..1100) |
| 14 | 2 | reserved, 0 |
| 16 | len | payload |
| 16+len | 16 | tag = first 16 bytes of HMAC-SHA256(key, bytes 0 .. 16+len-1) |

- **Key:** 16 random bytes made on the Mac (`~/.espfly/auto.key`, mode 0600)
  and installed over USB (`autokey <32 hex digits>`); it never crosses Wi-Fi.
  Without a key the drone does not open the port.
- **Key fingerprint:** first 4 bytes of SHA-256(key), shown by both sides.
- **Receiving:** packets with a bad magic, version, length or tag are dropped
  silently and counted (`authFails`). A Mac packet is at most 64 bytes in
  all (the drone's receive buffer; TAKEOFF with a planned move, the longest
  command, is 50, 44 without): a
  longer datagram is truncated, fails the length check and is never acted
  on.
- **Rate budgets** (fixed 1 s windows, counted before the tag check):
  - a datagram of at least 32 bytes whose clear-text type byte (offset 2) is
    `0x13` (KILL) counts against its own budget of 100 per second
    (`killRateDrops` above it);
  - everything else shares 60 per second (`rateDrops` above it).

  So packets of other types cannot use up the KILL's budget, and a KILL
  flood costs at most 100 tag checks a second. A real KILL burst is 10
  packets (20 with the `kill` client). The tag of a KILL is checked like any
  other: a `0x13` with a bad tag is counted in the KILL budget and dropped.
  Because both budgets are spent before the tag check, a station on the
  drone's Wi-Fi can use them up without the key: see "Known risk" under
  KILL.
- **What the budgets do not cover:** they share out only what the link task
  reads. Below it, lwIP's receive mailbox for the socket holds 6 datagrams
  (`CONFIG_LWIP_UDP_RECVMBOX_SIZE`); the task reads at most 4 per pass
  (`AUTO_RX_PER_LOOP`), and while it pushes photos it passes only every
  20 ms. A datagram that arrives while the mailbox is full is dropped inside
  lwIP, before any budget or tag check, a KILL included. So a burst of more
  than 6 datagrams queued ahead of a KILL (a flood from a station on the
  softAP, or a batch of the Mac's own `CHUNKS` re-requests) can drop that
  KILL. The 10-packet burst over 90 ms gets one through unless the mailbox
  stays full for the whole burst.
- **Sessions:** `HELLO` (session 0) starts a session; the drone answers
  `HELLO_ACK` with a new random session id and replaces any previous session.
  A repeat of the same HELLO (same nonce and seq, e.g. a retry) gets the same
  HELLO_ACK and keeps the session. Every later Mac packet carries that
  session; a wrong session gets `ACK` result `BAD_SESSION` (only if the tag is
  valid; its header carries the session the Mac sent). A new session while
  autonomy is active makes the state machine land (`LAND`, reason `SESSION`):
  the Mac tool never sends a new HELLO during a sortie.
- **Sequence numbers:** each side numbers its own packets from 1. The drone
  accepts a Mac `seq` only if it is greater than the last accepted one; a
  repeat of one of the last 8 accepted `seq` values is a duplicate: the
  drone re-sends the cached `ACK` payload (the same 12 bytes, in a new
  packet: its own header seq and tag) and does not act again. Older values
  are dropped without an answer. KILL is exempt (below).
- **Acks and retries:** the drone answers every packet that has `ACKREQ`
  with `ACK`. The Mac tool retries in two ways:
  - the one-shot commands without a daemon (`hello`, `status`, `photo`;
    `Link.command`): the same `seq` with `RETX` every 150 ms, up to 4 times;
  - the daemon: ARM, TAKEOFF and PHOTO are sent, repeated twice under the
    same `seq` with `RETX`, then twice under new seqs, 150 ms apart (5 sends;
    then "no ACK ... after 5 tries", and what an ARM may have started is
    stopped); LAND and DISARM go out under a new seq every 100 ms until
    confirmed (an ACK, or an AUTO_STATUS showing it), for up to 10 s; CHUNKS
    and PHOTO_FREE take a new seq on every retry. The heartbeat (no ACK)
    takes a new seq every 100 ms.

  Because the heartbeat's seq passes a lost command's within 100 ms, a `RETX`
  of a request that never arrived comes in as an old seq and is dropped with
  no answer: a same-seq `RETX` only recovers a lost ACK (the cached answer
  is replayed). A lost request gets its real second and third tries from the
  new-seq sends. The daemon reads the answer to such a send knowing the
  first copy may have acted (an ARM answered `BAD_STATE` in ARMED, a TAKEOFF
  `BAD_STATE` in TAKEOFF or HOVER, a PHOTO `BUSY` with a new set's id).
- **Where replies go:**
  - a reply (ACK, HELLO_ACK, STATUS, the AUTO_STATUS after a STATUS) goes to
    the address and port of the packet it answers;
  - pushes (AUTO_STATUS, photo META and CHUNK) go to the **peer**, which is
    set only by a HELLO that opens or replays the session and by a new (not
    duplicate) packet of the current session. A KILL never moves it, nor
    does a packet with another session or a replayed seq, so the `kill`
    client (session 0, its own socket) never redirects the daemon's pushes.
  - The one replay that does move the peer is the session's own HELLO (same
    nonce and seq), by design: a Mac whose HELLO_ACK was lost retries it,
    and the HELLO_ACK goes to the sender. A station that captured that HELLO
    and replays it from its own address therefore receives the pushes until
    the daemon's next new packet (its heartbeat, every 100 ms) moves the
    peer back; the Mac re-requests the chunks it missed with `CHUNKS`. This
    replay cannot change the session, act on anything, or redirect a reply.
  - A captured HELLO of an **earlier** session is not a replay to the drone:
    its nonce differs, so it opens a new session from the sender's address
    (A1 rule, unchanged) and the peer moves there for good. While autonomy is
    active a new session lands the sortie (`LAND`, reason `SESSION`), and the
    daemon's next packet is answered `BAD_SESSION`, so it stops. A station on
    the softAP (default password) can therefore end a sortie this way; it
    always ends in a LAND, never a cut or a climb.

## Types

| Type | Name | Dir | Payload |
|---|---|---|---|
| 0x01 | HELLO | M->D | u32 nonce |
| 0x02 | HB | M->D | u32 macMs (no ack); also the state machine's heartbeat (LAND `HB_LOST` after 1 s while active) |
| 0x03 | STATUS_REQ | M->D | empty; answered with STATUS, then AUTO_STATUS |
| 0x10 | ARM | M->D | u8 mode (0 FLIGHT, 1 DRY, 2 BENCH), u8 0, u16 airMaxMs (0: the NVS `autoair`, default 6000, 2000..8000; else 2000 up to the NVS `autoair`, which it can only lower: above it the ARM is refused `RANGE`) |
| 0x11 | TAKEOFF | M->D | u32 armToken, u16 heightCm (20..120 from the interactive hover, 20..60 before it, 20..40 before 2026-10-12 evening; the hover goes to 2.00 m by MOVEs up only since 2026-10-15, docs/autonomy-a2b.md 23.9: a LAND in the take-off climb falls at once), u8 photoN (0..3), u8 actWaitMs bits 16-23 (a pad that had to be 0 until 2026-10-14), u16 photoGapMs (150..1000), u16 actWaitMs bits 0-15. actWaitMs (u24): 0 = the NVS `autowait`; else this sortie's hover wait, 1000..120000 (1000..60000 until 2026-10-14). A wait <= 65535 ms is the same 12 bytes as before; a drone before 2026-10-14 refuses a nonzero byte 7 with `RANGE` (detail 0), never misreading the wait. **18 bytes** (`AUTO_TAKEOFF_PLAN_LEN`, the long-move build of 2026-10-14): + i16 planRightCm, i16 planFwdCm, u16 0 (reserved): the planned move, flown as a MOVE when the hover wait starts (below). The Mac sends 18 bytes only with a plan; an older drone refuses 18 bytes with `RANGE` (detail 0: not processed, no take-off), as it does any other length or a nonzero reserved field |
| 0x12 | LAND | M->D | empty; never refused by the state machine |
| 0x13 | KILL | M->D | 0..32 bytes, ignored (empty from the Mac tool); valid tag alone: any session, any seq |
| 0x14 | DISARM | M->D | empty; never refused by the state machine |
| 0x20 | PHOTO | M->D | u8 n (1..3), u8 0, u16 gapMs (IDLE: 0..1000; in the hover wait: 150..1000) |
| 0x21 | CHUNKS | M->D | u16 id, u8 idx, u8 count (0..8), u16 first; count 0 (first 0) re-sends the frame's META |
| 0x22 | PHOTO_FREE | M->D | u16 id |
| 0x30 | YAW | M->D | i16 deg (+ clockwise, to the right; 1..180 either way), u16 0 |
| 0x31 | MOVE | M->D | i16 rightCm, i16 fwdCm, i16 upCm (+ right, forward, up in the drone's frame when it is accepted; \|right\| <= 150, forward <= 400 (150 before 2026-10-14), back <= 150, sqrt(right^2 + fwd^2) <= 400, \|up\| <= 50 and a move with up != 0 goes at most 150 forward; not all 0), u16 0 |
| 0x32 | STOP | M->D | empty; never refused |
| 0x33 | SET_HEIGHT | M->D | i16 xCm (the judged true height, 10..250), i16 atCm (the drone's own height estimate when the judged photo was captured, cm; -32768 = INT16_MIN: the estimate now). 2026-10-15, below; an older drone answers `BAD_STATE` detail 0 (an unknown type). 2026-10-16: the 6-byte "go to the target" form appends i16 toCm (25..200; -32768: the commanded height); a drone before it answers `RANGE` detail 0 |
| 0x81 | HELLO_ACK | D->M | u32 session, u32 bootNonce, u32 echoNonce, u8 keyFp[4], char fw[24] (NUL padded) |
| 0x82 | ACK | D->M | u32 ackSeq, u8 ackType, u8 result, u8 state, u8 0, u32 detail |
| 0x83 | STATUS | D->M | see below |
| 0x84 | AUTO_STATUS | D->M | 136 bytes (96 from an interactive-hover drone before 2026-10-15, 64 from an A2b drone), see below |
| 0xA0 | META | D->M | see below |
| 0xA1 | CHUNK | D->M | u16 id, u8 idx, u8 0, u16 chunkIdx, u16 dataLen, data[dataLen <= 1024] |


`HELLO_ACK` is sent in the new session (its header carries the new session
id). `bootNonce` is random per boot: a change tells the Mac the drone reset,
and the Mac treats it as "mission aborted" (every boot starts in IDLE, and
after any reset FLIGHT and BENCH stay refused until a battery power-on or the
USB console's `auto unlock`; the link itself has no unlock).

### ACK results

| Code | Name |
|---|---|
| 0 | OK |
| 1 | BAD_STATE |
| 2 | BUSY |
| 3 | RANGE |
| 4 | TAKEOFF_WINDOW |
| 5 | CAM_UNAVAIL |
| 6 | LOW_BATT |
| 7 | APP_ACTIVE |
| 8 | KILL_LATCHED |
| 9 | POWER_DEFER |
| 10 | NO_PHOTO |
| 11 | MEM |
| 12 | GUARDS |
| 13 | STALE_FRAME |
| 14 | BAD_SESSION |

- **`state` (byte 6)** is the autonomy state (`autoState_t`: 0 IDLE, 1
  ARMED, 2 TAKEOFF, 3 HOVER, 4 LAND, 5 KILLED). For a flight command's
  result it is the state after the state-machine step that processed the
  command; for everything else (BUSY included) the state when the ACK is
  sent.
- **`detail`** depends on the command: the photo id for an accepted `PHOTO`
  (0 while autonomy is active means no set was started, see below); for a
  `PHOTO` refused `BUSY`, `HELD_SET` with the id of the set still held (the
  Mac may free it); for the flight commands, see below. Every `BUSY` the
  link itself answers carries its kind in the top byte (below).

## Flight commands (A2b)

ARM, TAKEOFF, LAND, DISARM, and PHOTO while autonomy is active (any state
but IDLE), go to the state machine. The link checks the payload length and
that every pad byte is 0 (`RANGE`, detail 0, otherwise; the values are the
state machine's to check), then posts the command and goes on serving every
other packet. The `ACK` follows when the state machine has processed it,
normally one stabilizer loop (1 ms) later. Only one command is in flight at
a time.

| Command | Result | detail |
|---|---|---|
| ARM | `OK`, `GUARDS`, `BAD_STATE`, `RANGE`, `KILL_LATCHED` | OK: the armToken (TAKEOFF must carry it); GUARDS: the failed-guard bitmap (`AUTO_G_*`, auto_fsm.h; G0 LINK .. G12 MODE); else the state |
| TAKEOFF | `OK`, `GUARDS`, `BAD_STATE`, `RANGE` | GUARDS: the bitmap (all guards are checked again); a token that is not this ARM's: `BAD_STATE`, detail 0; else the state before |
| LAND, DISARM | `OK` (never refused) | the state before |
| PHOTO (active) | `OK`, `BUSY`, `BAD_STATE`, `RANGE` | OK: the photo id of the set, or 0 (below); the state machine's BUSY and the rest: the state |

**A LAND after a contact** (user decision B, 2026-10-15, docs/autonomy-a2b.md 22.9, 22.10): once AUTO_STATUS `contacts`
(byte 126) is not 0 (a CONTACT or a COLLISION since this sortie's ARM), every LAND of the sortie is the blind one,
whatever started it: this LAND or a DISARM in the air, BATT_LOW and the other battery rules, the link, the phone, a
loop gap, a SINK, the controller's own. The blind LAND flies the ceiling alone from the measured hover (no loop on
the camera's height estimate, no fast LAND): a slow start, touchdowns up to ~1 m/s; at the floor its flat part ends
when the drone stops descending, so the props may run a few seconds there (a2b 22.10). The camera hold stays on to the
touchdown, except in a LAND begun in the contact's window (its hush, or before the camera height is back 0.5 s
after it). The Mac has no `approach` since the same day (decision P): a YAW, then a plain MOVE.

**PHOTO `OK` with detail 0** means: accepted by the state machine, but no
photo set was started (the camera was not running, or the PSRAM photo slots
could not be allocated). No META follows; AUTO_STATUS `capture` reads failed
(3) and the state machine lands (reason `CAPTURE_FAIL`). The Mac must not wait
for a photo id 0. (Photo ids start at 1 and skip 0, so 0 is never a set.)

The state machine's own rules are in `docs/autonomy-a2.md` section 3.3. The
Mac tool checks a nonzero `airMaxMs` before sending it: at most the
`autoair` of the last `espfly_diag.py auto get` (saved in
`~/.espfly/auto_get.json`), or 6000 ms (the default; 4500 before
2026-10-12 evening) without one. Its `--height` takes 20..120 cm (higher, to 2.00 m, by `move --up` from the hover) and
`--wait-s` 0 or 1..120 s (TAKEOFF `actWaitMs`; 1..60 until 2026-10-14). A
wait over 12 s needs `--height` >= 0.25 m + 0.01 m/s x min(wait + 8 s, 80 s)
(105 cm from 72 s).

### MOVE, YAW, STOP (the interactive hover)

They go to the state machine like the flight commands (one at a time, the
same `BUSY` kinds, the same caching). A MOVE or YAW is accepted only in
HOVER while waiting (after the first STABLE photo set) and while no motion
runs; an accepted one restarts the hover wait, which does not count down
while it runs. STOP ends a running motion where it is (the drone holds
there) and is never refused (with nothing running it is `OK` too).

| Result | When |
|---|---|
| `OK` | accepted (STOP: always) |
| `BAD_STATE` | not in the hover wait (`NOT_WAITING`), or a motion runs (`BUSY`: final, the Mac does not resend it) |
| `RANGE` | out of the limits (`RANGE`: the MOVE row of the type table; `autoMvRangeOk`), the height after the move outside 0.25..2.00 m (`HEIGHT`; 0.25..1.20 m until the 2.00 m cap of 2026-10-15; since 2026-10-15 one-sided: a move down below 0.25 m + min(0.30 m, 0.01 m/s x the air time), a move up above the cap), a forward move that could leave < 0.50 m to the scene ahead at the expansion scale 0.6 (`TOO_CLOSE`), sideways more than 0.5 x D (`TOO_WIDE`, 11), or the motion plus a LAND not fitting in the air left of the 120 s limit (80 s before 2026-10-14; `AIR`) |
| `CAM_UNAVAIL` | no scene distance D (`NO_DIST`), the camera hold not applied on a fresh view (`NO_VIEW`), a vertical move without the camera height estimate (`NO_CAM_HEIGHT`) |
| `LOW_BATT` | the battery-low condition (`BATT`) |

`detail` = state (bits 0-7) \| code << 8 \| motion seq << 16: for MOVE and
YAW the code is the refusal (`autoMvWhy_t`, auto_motion.h; 0 when accepted),
for STOP the motion state it found (`autoMvState_t`); the motion seq counts
the motions accepted this sortie (an accepted one carries its own).
AUTO_STATUS bytes 64..95 then show the motion's state, its progress and how
it ended.

### SET_HEIGHT (0x33, 2026-10-15; docs/autonomy-a2b.md 23)

"You are at X m": Claude judges the true height from a photo's level line and sends it with the drone's own estimate
when that photo was captured (`atCm`; the Mac takes it from the photo's JSON, `autonomy.h_est_capture_m`). The drone
takes the offset d = X - h_at (INT16_MIN: d = X - h_now), so moves made between the photo and the command cancel out.
On acceptance, on the same 1 kHz step: the controller's height estimate shifts by d (nothing else of its state), the
committed height offset (hTargetCm's reference) moves by d, and a vertical-only MOVE flies upM = min(h_now, cap) -
(h_now + d), normally -d, at <= 0.15 m/s back to the height the drone believed it held (now truly there): 0.10 m in
~1.4 s, 0.50 m in ~4.1 s, plus the settle (<= 3 s). STOP, a contact, a LAND or KILL during it keep the corrected
estimate (the reference where the drone stopped). One command at a time like the motions (the same `BUSY` kinds and
caching); the link refuses a length other than 4 or 6 or xCm < 0 with `RANGE` detail 0 (not processed).

**The target form** (6 bytes, 2026-10-16, docs/autonomy-a2b.md 27): i16 toCm appended, the height to fly to (25..200;
-32768: the commanded height, take-off height + the committed moves). The same shift by d (the true height X now);
then the flight upM = target - (h_now + d), the reference starting at the corrected estimate and committed at the
target by DONE (hTargetCm then reads the target): the drone ends truly at the target instead of ~0.1 m under the
commanded height. d may round to 0 cm (the flight alone), not both (RANGE). A flight up also needs the climb's thrust
headroom (`LOW_BATT` why 16 `UP_HEADROOM`: the ceiling in force less than min(4600, 10 % of T_hold) above T_hold, both
forms, checked after HEIGHT and before BATT). A drone before 2026-10-16 answers the 6-byte form `RANGE` detail 0 (the
link's length check, not processed); the Mac then sends the 4-byte form with X + h_now - target.

| Result | why (detail bits 8-15) | When (checked in this order) |
|---|---|---|
| `OK` | 0 | accepted |
| `BAD_STATE` | 1 `NOT_WAITING` | not HOVER in the wait with the controller in HOLD (IDLE, ARMED, TAKEOFF, LAND incl. SINK, KILLED, before the wait) |
| `BAD_STATE` | 2 `BUSY` | a motion runs (a MOVE, a YAW, a bob or re-view, a correction, the take-off's planned move) |
| `CAM_UNAVAIL` | 15 `SH_CONTACT` | a contact's hush or its window (the camera height not back 0.5 s after it) |
| `CAM_UNAVAIL` | 8 `NO_CAM_HEIGHT` | the camera height estimate not in use (open loop ignores the correction) |
| `CAM_UNAVAIL` | 7 `NO_VIEW` | the camera hold not applied on a fresh view |
| `BAD_STATE` | 14 `SH_RATE` | an accepted SET_HEIGHT less than 2 s ago |
| `RANGE` | 3 `RANGE` | xCm outside 10..250, or d rounds to 0 cm (the target form: a toCm outside 25..200, or d and the flight both 0 cm) |
| `RANGE` | 13 `SH_STEP` | \|d\| over 50 cm (in whole cm), or the flight \|upM\| over 0.55 m (an estimate above the cap, 2.00 m) |
| `RANGE` | 4 `HEIGHT` | the corrected estimate outside 0.10 m..the cap + 0.50 m; or (2026-10-16 review, the 6-byte form only) a descent whose target is under a MOVE down's floor, 0.25 m + min(0.30 m, 0.01 m/s x the air time) |
| `LOW_BATT` | 16 `UP_HEADROOM` | a flight up (upM > 0) without the climb's thrust headroom (2026-10-16; also a MOVE up) |
| `LOW_BATT` | 9 `BATT` | the battery-low condition (`battLow`: BATT_LOW held 0.25 s, or T_hold > 53400): no correction flies, up or down, as no MOVE (a descent's braking near the ceiling can end HOLD_CAPPED, a fast LAND: the 2026-10-15 review's drain sim) |

`detail` = state (bits 0-7) \| why << 8 \| the drone's estimate when the command came, cm, i16 (bits 16-31; 0 with the
controller inactive): `mvDetail`'s layout with the motion seq replaced by the old estimate. A refusal outside HOVER
(the state machine's `cmdOutside`) carries no estimate. This firmware never answers SET_HEIGHT with detail 0; an older
one answers the unknown type `BAD_STATE` detail 0, which the Mac reports as "firmware has no SET_HEIGHT".

AUTO_STATUS after an accepted one: lastCmdType 10, mvSeq + 1, mvKind MOVE with mvFlags bit 6 (0x40: on a MOVE, a
SET_HEIGHT's correction, not Claude's MOVE; on a YAW the blind turn), tgtUpCm = upM, progUpCm the estimate's change
since the correction started (0 on its first step), mvState RUN / SETTLE / DONE or STOPPED with mvEnd, hMm the
corrected estimate from the very next status, hTargetCm the reference ramping from old + d back to old. No event (the
trim learner skips sorties with one). The recorder's cvH (the camera observer's own height, not used for control) is
not shifted: after corrections it differs from vzH by their sum.

**The TAKEOFF's planned move** (the 18-byte TAKEOFF; `docs/autonomy-a2b.md`
21.4). The TAKEOFF itself checks the plan against the MOVE limits and that
the sortie has a hover wait (`actWaitMs`, or the NVS `autowait`, nonzero):
`RANGE` otherwise, no take-off. Every accepted TAKEOFF latches its plan (one
without a plan clears it). The plan starts as a MOVE on the step the hover
wait starts (STABLE 1 s plus the take-off photos: HOVER + ~1.5 s with one
photo), with every MOVE acceptance check on the live D then (`NO_DIST`,
`TOO_WIDE`, `TOO_CLOSE`, `NO_VIEW`, `BATT`, `AIR`; `TOO_CLOSE` at the Mac's
0.60 m floor, `AUTO_PLAN_MIN_FRONT_CM`, not the drone's 0.50 m: the Mac cannot
check the plan before the take-off, a2b 21.9). AUTO_STATUS shows it as a
Mac MOVE: **running** = mvSeq 1 and mvState RUN / PAUSED / SETTLE;
**refused** = waiting, mvSeq 0 and mvWhy != 0 (the drone hovers in the wait,
as after a refused Mac MOVE). A STOP before the wait (in the climb, or during
the take-off photos) cancels it; a LAND, a KILL or any landing before the wait
never reaches it. Since 2026-10-15 so does any event the drone reports before
the wait (a contact, NOT_STABLE, TAKEOFF_SLOW, CAPTURE_FAIL, the camera height
lost, a controller event, a loop gap, a HOVER a climb fault started): mvWhy 12
`PLAN_CANCELLED`, mvSeq 0, the drone hovers in the wait (each of those landed
until then, so the plan never flew after one). With `--plan-via mac` the Mac
does not send its MOVE when evCount (bytes 98-99) is not 0 at the wait. Before the wait a Mac MOVE gets `NOT_WAITING`; while the
plan runs, `BUSY`; a STOP ends it (`STOP_CMD`).

**BUSY from the link** carries its kind in the top byte of `detail` (bits
24-31; `AUTO_DETAIL_*` in auto_proto.h) and the kind's value in the low
bits. The state machine's own `BUSY` (a PHOTO while its set is still being
captured) keeps detail = the state, kind 0, and a `CHUNKS` answered `BUSY`
keeps detail 0 (A1). A detail is a photo id only when its kind is
`HELD_SET`.

| detail | Kind | Meaning | What the Mac does |
|---|---|---|---|
| `0x01000000` \| (seq & `0xFFFFFF`) | `PENDING` | another command is pending (the low 24 bits of its seq); this one was not processed | send it again with a new seq after that one's ACK |
| `0x02000000` \| state | `UNTAKEN` | not processed, guaranteed: the mailbox slot was still held when it arrived, or it was withdrawn untaken after 100 ms (the state, 0..5) | send it again with a new seq |
| `0x03000000` \| photo id | `HELD_SET` | a `PHOTO` refused while a set is held: in IDLE any set, while active a set the state machine started in this sortie (its id, 16 bits) | free that set (`PHOTO_FREE`), then ask again |
| `0xFFFFFFFF` (`AUTO_DETAIL_TAKEN_UNKNOWN`) | `0xFF` | the state machine took it but its result did not come within 200 ms: it **may have been acted on** | never re-send blindly: read AUTO_STATUS (`state`, `lastCmdType`, `lastCmdResult`) first |

`UNTAKEN` and `TAKEN_UNKNOWN` only happen with a stalled stabilizer, when the
motor watchdog has already stopped the motors (80-90 ms). A command is never
acted on twice, and a BUSY that is not `TAKEN_UNKNOWN` is never acted on
later.

The flight commands' `BUSY` answers (the four kinds above, `HELD_SET` while
active included) are never cached: a `RETX` of the same seq is a duplicate
with no cached answer and gets nothing. The IDLE `PHOTO`'s `HELD_SET` and
the `CHUNKS` `BUSY` are cached like every A1 answer.

Every other result (the state machine's, its `BUSY` included, and `RANGE`)
is cached as A1 caches answers: a duplicate seq gets the same ACK payload (in
a new packet, with its own header seq and tag), and the command is not
posted again. A result that arrives after a new
HELLO is still sent to the address and session of its command, and is not
cached in the new session.

A command without `ACKREQ` is processed the same way, with no ACK.

### KILL (0x13)

- Accepted with a valid tag alone: any session (0 from the `kill` client),
  any seq, a payload of 0 to 32 bytes (ignored; the Mac tool sends none). A
  KILL with a longer payload does not fit the 64-byte receive buffer: it is
  truncated, fails the length check (`authFails`) and is **not** acted on.
  The seq filter is neither checked nor updated, so a replayed or repeated
  KILL acts again (fail-safe).
- Acted on before anything else: `autonomyKillFromLink()` (the motor driver
  STOP if any duty is nonzero, then `killCount++`; the state machine goes to
  KILLED on its next step, or latches ARM off for 2 s in IDLE).
- With `ACKREQ`: `ACK` `OK`, sent to the sender's address in the sender's
  session; detail = the state read just before the KILL, byte 6 = the state
  read just after it. Both are read before the state machine's next 1 ms
  step, which is when it enters KILLED, so the first ACK of a burst usually
  still shows the state before the KILL (e.g. HOVER) and later ones show
  KILLED (in IDLE every ACK shows IDLE: the KILL only latches ARM off). The
  Mac tool reports KILLED if any ACK of its burst shows it; the daemon
  prefers an AUTO_STATUS whose `killCount` counts the KILL. Never cached;
  the peer does not move.
- The Mac sends 10 KILLs, 10 ms apart; each acts and restarts the 2 s latch.
- Latency: the task blocks on receive only while idle (it returns on
  arrival); while a command is pending it polls every 1 ms, while pushing
  photos every 20 ms, and while autonomy is active its receive timeout is
  20 ms. No command wait sits in front of a KILL. KILL to motor duty 0 is
  within 50 ms with a live (or blocked) stabilizer; with a stabilizer that
  busy-loops on core 0 the link (priority 1, core 0) cannot run, and the
  motor watchdog stops the motors at 80-90 ms.

**Known risk: anyone on the drone's Wi-Fi can block the Mac KILL.** The
drone's access point uses the stock ESP-Drone default password (`WIFI_PWD`
in wifi_esp32.c), so any station that knows the default can join it. The
KILL budget (100 per second) is spent before the tag is checked, so such a
station can send 100 forged `0x13` datagrams a second, without the key, and
every genuine Mac KILL in the rest of that second is dropped
(`killRateDrops` counts them); kept up, the flood blocks every Mac KILL.
A flood of any type that keeps the socket's 6-datagram receive mailbox full
drops the Mac KILLs inside lwIP as well, uncounted (see "Rate budgets").
This is accepted as designed: the budget exists to cap what a flood of
forged packets costs the drone (100 tag checks a second), and it can only
do that before the tag is checked. What still stops the drone in that case:

- the Mac heartbeat: while autonomy is active the state machine lands
  (`HB_LOST`) 1 s after the last Mac heartbeat, so stopping the daemon (or a
  flood of other forged packets, which uses up the shared 60/s budget the
  heartbeats need) brings it down;
- the phone: any input from the phone app makes the state machine land
  slowly;
- a Mac `LAND`, which travels in the shared budget, not the KILL budget;
- the air clock (`airMaxMs`): the state machine lands when it runs out
  (`AIR_TIME`).

None of these is a motor cut. Before any FLIGHT sortie, change the access
point password from the stock one, and keep other stations off the drone's
Wi-Fi.

### AUTO_STATUS (0x84), 136 bytes

The state machine and the vertical controller, as a snapshot taken by the
stabilizer task (`autonomyGetStatus`, `autoStatusMsg_t` in auto_proto.h).
Python: `struct.unpack("<IIII10B8H2h6H2BI", payload[:64])`, for bytes
64..95 (the interactive hover) `struct.unpack("<6BH6hHhhhHH", payload[64:96])`
and for bytes 96..135 (hold and report, 2026-10-15)
`struct.unpack("<BBHHBBHHHHhhHHHHhBBBBHBBH", payload[96:])`. An A2b drone
sends the first 64 bytes only, an interactive-hover drone of 2026-10-14 the
first 96; the Mac tool takes all three.

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | upMs (drone tick) |
| 4 | 4 | bootNonce (as HELLO_ACK and STATUS) |
| 8 | 4 | transitions since boot |
| 12 | 4 | failedGuards (`AUTO_G_*`): in IDLE the FLIGHT guards at 10 Hz, else the bitmap last checked |
| 16 | 1 | state (`autoState_t`) |
| 17 | 1 | mode (`autoMode_t`) of the current or last ARM |
| 18 | 1 | reason (`autoReason_t`) of the latest transition |
| 19 | 1 | vzLandReason (`vzReason_t`, when reason is VZ or a LAND ended) |
| 20 | 1 | landTrip (`autoReason_t`: TILT or RATE for TOUCHDOWN_TRIP) |
| 21 | 1 | vzPhase (`vzPhase_t`, 0 outside a sortie) |
| 22 | 1 | vzReason (`vzReason_t` of the controller's latest phase change) |
| 23 | 1 | flags, below |
| 24 | 1 | trips, below |
| 25 | 1 | capture (`autoCapture_t` of the set the state machine started: 0 none, 1 busy, 2 done, 3 failed) |
| 26 | 2 | photoId of that set (its META/CHUNK id), 0 none |
| 28 | 2 | stateMs (saturated at 65535) |
| 30 | 2 | airMs (the air clock, saturated) |
| 32 | 2 | stableMs (age of the STABLE streak, which survives dropouts < 150 ms each and < 300 ms in all; 0 without one; saturated) |
| 34 | 2 | waitLeftMs (hover wait left, 0 unless waiting; saturated at 65535 = "65.5 s or more" since waits reach 120 s) |
| 36 | 2 | hoverWaitMs (the wait latched at ARM, NVS `autowait`; saturated at 65535) |
| 38 | 2 | vbatMv (10 Hz) |
| 40 | 2 | vbatFastMinMv (lowest 1 kHz reading over the last 320 ms) |
| 42 | 2 | hMm (i16, the controller's height estimate) |
| 44 | 2 | vzMms (i16, its vertical speed estimate, up) |
| 46 | 2 | thrust (the controller's command, before the mode clamp) |
| 48 | 2 | tLo (thrust at lift-off, 0 before) |
| 50 | 2 | tHold (mean thrust in HOLD, 0 before) |
| 52 | 2 | tRef (LAND reference, 0 before LAND) |
| 54 | 2 | kPriorDiv10 (the sortie's stored hover gain / 10, NVS `vzK`) |
| 56 | 2 | khatDiv10 (the controller's hover gain estimate / 10) |
| 58 | 1 | lastCmdType (`autoCmdType_t` of the last command the state machine processed: 1 ARM, 2 TAKEOFF, 3 LAND, 4 KILL, 5 DISARM, 6 PHOTO, 7 MOVE, 8 YAW, 9 STOP, 10 SET_HEIGHT) |
| 59 | 1 | lastCmdResult (its result code) |
| 60 | 4 | killCount (Mac KILLs since boot) |
| 64 | 1 | mvState (`autoMvState_t`: 0 IDLE, 1 RUN, 2 PAUSED, 3 SETTLE, 4 DONE, 5 STOPPED) |
| 65 | 1 | mvKind (0 none, 1 MOVE, 2 YAW: the latest motion) |
| 66 | 1 | mvWhy (`autoMvWhy_t` of the latest refused MOVE / YAW / SET_HEIGHT; 12 PLAN_CANCELLED: the TAKEOFF's planned move not started after an event before the wait, 2026-10-15; SET_HEIGHT's 13 SH_STEP, 14 SH_RATE, 15 SH_CONTACT; 16 UP_HEADROOM, a climb without the thrust headroom, 2026-10-16) |
| 67 | 1 | mvEnd (`autoMvEnd_t`: 1 DONE, 2 STOP_CMD, 3 PROGRESS, 4 LEFT_HOVER, 5 RETRACK, 6 SHORT: a MOVE whose settle timed out short of its target or off its line, state DONE, the drone holds where it ended; user decision 2026-10-13, evening; 7 FRONT (2026-10-14): a forward MOVE stopped by the front guard, its measured progress past ln(D / 0.40 m) x 0.6 (the scene ahead got close), state STOPPED, the drone holds there; 2026-10-15, the drone stopped it and holds: 8 CONTACT (an in-air contact), 9 NO_VIEW (the view lost 2 s while a MOVE ran; a YAW turns back instead), 10 TILT, 11 UNDONE (a YAW turned back to its start heading and re-tracked there, state DONE), 12 LOOP_GAP) |
| 68 | 1 | mvFlags (`AUTO_MVF_*`: bit 0 D valid, 1 D stale after a turn or a long sideways move, 2 D of the sim, 3 camera height guess allowed, 4 re-track or re-view bob, 5 the forward D from the last forward / back move; 2026-10-15: 6 the YAW was accepted without a view (a blind turn, at most 90 deg), 7 the YAW is turning back; bit 6 with mvKind MOVE: a SET_HEIGHT's correction) |
| 69 | 1 | battLow (`AUTO_BATTLOW_*`: bit 0 HELD, the drone's BATT_LOW condition (the 10 Hz battery below 3.00 V, or the hover thrust T_hold over 57000) for 0.25 s: MOVE / YAW refused, `BATT_LOW` after 0.5 s in HOVER; bit 1 NOW, the condition holds now. The Mac acts on bit 0 only. A MOVE / YAW refused because T_hold is over 53400 alone shows only in its `LOW_BATT` reply. Until 2026-10-14: 3.30 V, 0.5 s / 1 s; autonomy-a2b.md section 16) |
| 70 | 2 | mvSeq (motions accepted this sortie) |
| 72 | 6 | progRightCm, progFwdCm, progUpCm (i16: the latest motion's measured progress; FLIGHT: the camera hold, DRY/BENCH: ideal. Once a MOVE has ended, whatever ended it, its measured travel at the end (before 2026-10-13 evening a STOP or PROGRESS restarted it at 0 with the hold's snap epoch)) |
| 78 | 6 | tgtRightCm, tgtFwdCm, tgtUpCm (i16: its command) |
| 84 | 2 | sceneDCm (the scene distance D a forward MOVE would use now; valid when mvFlags bit 0; a sideways one uses the larger of it and the observer's) |
| 86 | 2 | hTargetCm (i16: take-off height plus the vertical moves, commanded) |
| 88 | 2 | yawOffCdeg (i16: heading from the take-off heading, 0.01 deg, + clockwise) |
| 90 | 2 | yawTgtDeg (i16: the latest YAW's command) |
| 92 | 2 | mvPhaseMs (time in mvState, saturated) |
| 94 | 2 | airLeftDs: 0 since 2026-10-15 (no air clock). A 96-byte drone of 2026-10-14: until its 120 s air limit (80 s before; 0.1 s, at most 1200; 0 outside the air and at the limit); the Mac reads such a drone's air time from `airMs` below 65535, from `AIR_HARD_MS` - `airLeftDs` above, and checks `airMs` + 100 x `airLeftDs` (unsaturated) against its own limit |
| 96 | 1 | evCode (`autoEvent_t` of the latest event, below; 0 none this sortie) |
| 97 | 1 | evDetail (its detail, below) |
| 98 | 2 | evCount (events this sortie, wraps; the Mac logs one `drone_event` per change) |
| 100 | 2 | evAgeMs (since the latest event, saturated) |
| 102 | 1 | holdBits (`AUTO_HB_*`, below) |
| 103 | 1 | camBits (`AUTO_CB_*`: why the camera height went out, latched at its CAM_LOST, ORed while out) |
| 104 | 2 | camOffMs (with holdBits CAM_LOST: since the camera height was last in use, or the HOLD entry) |
| 106 | 2 | sinkInMs (with holdBits CAM_LOST: until the SINK; 0: now) |
| 108 | 2 | peakAccCg (the latest contact's peak \|acc\| over its hush, 0.01 g) |
| 110 | 2 | peakRateDps (its peak max(\|gx\|, \|gy\|), dps) |
| 112 | 4 | contactAxCg, contactAyCg (i16: its body-frame force at the \|acc\| peak, x forward, y left, 0.01 g; the obstacle is opposite: bearing atan2(Ay, -Ax), 0 ahead, +90 right) |
| 116 | 2 | sinceCmdDs (since the latest accepted MOVE, YAW, STOP or PHOTO, 0.1 s; 65535: none this sortie) |
| 118 | 2 | airDs (the air time, 0.1 s: past `airMs`'s 65.5 s) |
| 120 | 2 | khatPm (Khat / kPrior x 1000) |
| 122 | 2 | capQ (the controller's open-loop thrust bound: 0.995 x its camera-measured quiet hover thrust; 0 none yet) |
| 124 | 2 | contactYawCdeg (i16: `yawOffCdeg` at the contact: the bearing now is the one at the contact minus the turn since) |
| 126 | 1 | contacts (this sortie, saturating) |
| 127 | 1 | loopGaps (this sortie, saturating) |
| 128 | 4 | ev1Code, ev1Detail, ev1AgeMs (u8, u8, u16: the event before the latest) |
| 132 | 4 | ev2Code, ev2Detail, ev2AgeMs (the one before that) |

Events (`autoEvent_t`, auto_fsm.h; the detail in brackets): 1 CAM_LOST (camBits:
the camera height out 300 ms, the drone holds open loop under capQ), 2 CAM_BACK,
3 SINK (0 the 5 s budget ran out, 1 no camera-measured hover bound, 2 after a
contact or a climb fault: LAND SINK, the drone sinks to the ground and resumes if
the camera height comes back; a SINK begun in a contact's window does not resume,
and after any contact the SINK's descent is the blind LAND), 4 RESUME, 5 CONTACT (the controller's phase in
bits 0-5, 0x40 no hush (the chain bound), 0x80 a motion was stopped), 6 VZ (the
controller's `vzReason_t`: it held and reported), 7 NOT_STABLE (`AUTO_NS_*`), 8
TAKEOFF_SLOW, 9 CAPTURE_FAIL, 10 MOVE_STOPPED (`autoMvEnd_t`: NO_VIEW, TILT,
LOOP_GAP), 11 LOOP_GAP (its ms, 255 more), 12 RETRACK_UNDO (the cause's
`autoMvEnd_t`), 13 HOLD_NO_VIEW (1 coasting, 2 not applied, 3 stale frame), 14
VIEW_BACK, 15 HOLD_LIMIT (2026-10-16, information: the camera hold's roll or pitch correction at >= 1.3 deg for 8 s
outside motions; detail bit 0 roll, 1 pitch (the axes that rose), 2 roll negative, 3 pitch negative). `holdBits`: bit 0 CAM_LOST, 1 SINKING, 2 HUSH (a contact's hush runs),
3 K_CLAMP (Khat at its upper clamp or at / below 0.88 x kPrior), 4 CAPPED (the
thrust at a ceiling), 5 NO_VIEW, 6 NOT_STABLE, 7 OPEN_LOOP (HOLD without the
camera height, also the 1.5 s after a contact). `camBits`: bit 0 no accepted
frames for 2 s, 1 no G, 2 the CUSUM alarm, 3 the observer faulted or its
snapshot stale or paused, 4 the camera not RUNNING.

`flags`:

| Bit | Meaning |
|---|---|
| 0 | waiting (HOVER: photos done, waiting for the Mac) |
| 1 | STABLE conditions hold now |
| 2 | the controller uses the camera height observer this step |
| 3 | camera held awake through the take-off (no standby) |
| 4 | reset-locked (FLIGHT and BENCH refused until a battery power-on or the USB console's `auto unlock`) |
| 5 | pre-spin warning (motors start within 1 s) |
| 6 | the controller flies the sim plant (DRY, BENCH-sim) |
| 7 | camera drift hold applied |

`trips`, the conditions true now: bit 0 tilt > 30 deg, 1 rate > 500 dps,
2 |acc| < 0.4 g, 3 attitude error > 10 deg, 4 vbat < 3.05 V, 5 moved
(ARMED/pre-spin), 6 at rest, 7 the controller's thrust at a ceiling.

The enum values (states, modes, reasons, vz phases and reasons, guards) are
those of auto_fsm.h and vert_ctrl.h, which only ever append (2026-10-14:
reason 43 `PALM`, the palm cue; vz reason 28 `TOUCHDOWN_SUPPORT`, the support
probe's touchdown; docs/autonomy-a2b.md 18.3; 2026-10-15: reasons 44 `SINK`
and 45 `RESUME`, vz reasons 29 `RESUME` and 30 `CONTACT_HOLD`, motion ends 8
CONTACT to 12 LOOP_GAP, mvFlags bit 6 blind turn and 7 turning back; PALM and
the vz probe are no longer produced, HOVER_TIMEOUT, AIR_TIME, SORTIE_DONE,
HOVER_NO_CAM, MOVE_NO_CAM, MOVE_TILT neither: docs/autonomy-a2b.md 22).

**When it is sent** (pushes go to the peer, in the current session, and only
while there is one):

- after every STATUS (a STATUS_REQ, or its duplicate), to the sender;
- at once on every state-machine transition and every event (2026-10-15);
- every 100 ms while autonomy is active;
- every 500 ms for 2 s after returning to IDLE, so the Mac sees how the
  sortie ended.

Only an AUTO_STATUS that reaches the peer counts as the push: the answer to
a duplicate STATUS_REQ from another address (a replayed packet) goes there
as well, but the peer still gets the transition push and the next periodic
one on time.

The snapshot and its sequence number are read together: a push never
carries an older snapshot than the transition it reports. About 1 KB/s.

### STATUS (0x83), 48 bytes

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | upMs |
| 4 | 2 | vbatMv |
| 6 | 1 | camState (camera_flow.h cameraFlowState_t) |
| 7 | 1 | state (autonomy, `autoState_t`) |
| 8 | 2 | fps x10 |
| 10 | 2 | thrust (setpoint, 0..65535) |
| 12 | 4 | frameSeq (frames processed) |
| 16 | 2 | roll, 0.01 deg (i16) |
| 18 | 2 | pitch, 0.01 deg (i16) |
| 20 | 2 | yaw, 0.01 deg (i16) |
| 22 | 1 | lum |
| 23 | 1 | texture (capped at 255) |
| 24 | 1 | flags: bit0 motorsIdle, bit1 takeoffWindow, bit2 onGround, bit3 photoBusy |
| 25 | 1 | photo slots free |
| 26 | 2 | reserved |
| 28 | 4 | authFails |
| 32 | 4 | heap internal free |
| 36 | 4 | heap internal minimum |
| 40 | 4 | packets accepted |
| 44 | 4 | bootNonce |

### Photos

`PHOTO` asks for `n` frames of the flow camera, `gapMs` apart: 160x120,
row-major, in colour since 2026-10-15 (user request; opt-in, see below): YUV 4:2:2 as the sensor
sends it (YUYV: a pixel pair in 4 bytes Y0 U Y1 V, 320 bytes a row, 38400
bytes, 38 chunks; full-range BT.601, Y is the grey photo's value), or 8-bit
grey (19200 bytes, 19 chunks) with the drone's NVS `photocol` 0 (USB `auto set
photocol 0|1`, used from the next boot; default 0 since the review of 2026-10-15,
docs/autonomy-a2b.md 24.7). A grey frame's META is the
40 bytes of A1, unchanged; a colour frame's is 44 bytes with its pixel format
(below). A Mac tool from before 2026-10-15 cannot read a 44-byte META: set
`photocol 0` for it. The flow fit gets the frame's luma either way (docs/
autonomy-a2b.md 24): a photo costs the flight no frame.

**Always-on colour (2026-10-16, docs/autonomy-a2b.md 28).** With `photocol 1`
the sensor streams YUYV on every frame and the camera driver's own copy splits
it: the flow gets the Y bytes, and the frame a request arms (the first frame to
*start* after the request) is copied raw into the photo slot. **The wire format
is unchanged** (the 44-byte META, pixFmt 1, 38400 bytes, 38 chunks, the CRC over
the whole frame), so the Mac code is the same; only the timing moves: a colour
photo arrives **~70-190 ms later** than a grey one (0-67.5 ms to the next frame
start, 67.5 ms of capture, the flow finishing its frame and processing this one
for the meta), far inside the 1000 ms frame timeout. Every colour photo is a
whole frame of one capture; a frame the driver drops is replaced by the next
(re-armed on the drone, not visible here). Colour is never switched per photo:
one boot, one format. The daemon of 2026-10-16 also keeps a colour frame's bytes
as sent next to its PNGs (`<stamp>_<id>_<idx>.yuyv`, the photo reply's `raw`).

- **In IDLE** (stage A1) it is refused with `BUSY` while an earlier photo
  set is held (until `PHOTO_FREE` or 30 s; detail = `HELD_SET` | its id,
  `0x03000000` | id, where A1 sent the bare id: the low 16 bits are the
  same), with `CAM_UNAVAIL` unless the camera is running, and with
  `TAKEOFF_WINDOW` during a take-off window (Wi-Fi bursts at lift-off). An
  accepted `PHOTO` gets `ACK` OK with `detail` = photo id.
- **While autonomy is active** it goes to the state machine, which accepts
  it only in HOVER while waiting (it does not extend the wait); otherwise
  `BAD_STATE`. While a set the state machine started in this sortie is still
  held (not freed by `PHOTO_FREE` or the 30 s auto-free), the link refuses
  the PHOTO itself with `BUSY`, detail = `HELD_SET` | that set's id,
  uncached, and never passes it on: the frames the Mac has not received
  yet, or still re-requests with `CHUNKS`, are never released. Free the set,
  then ask again. An accepted PHOTO gets `OK` with `detail` = the new set's
  photo id, or 0: accepted, but no set started (the camera was not running
  or the photo slots could not be allocated). With 0 no META follows,
  AUTO_STATUS `capture` reads failed and the state machine lands
  (`CAPTURE_FAIL`); the Mac does not wait for a photo.
- **Sets the state machine starts on its own:** in every sortie with
  photoN > 0 it asks for one set at STABLE + 1 s. The drone captures and
  pushes it without a PHOTO: the Mac learns the id from the first META. The
  link releases any set still held to start it (an A1 set, or one left from
  an earlier sortie) and reports its progress to the state machine (busy,
  then done, or failed if the camera was not running or a frame is missing).

The drone copies each frame inside the camera task (between the flow fit and
returning the buffer, so the flow and the drift hold are not disturbed; in
colour the driver's copy task writes it, above) and
then pushes, per frame, one META and all CHUNKs (1024 bytes each, the last
shorter), 4 packets per 20 ms (about 200 KB/s: a grey frame in about 0.1 s, a
colour one in about 0.2 s). A
frame the camera could not supply is announced by a META with size 0 and
chunkCount 0. The Mac re-requests a lost META with `CHUNKS` count 0, then
missing chunks with `CHUNKS` (a frame with size 0 answers chunk requests
`NO_PHOTO`), and checks the CRC-32 (IEEE, as zlib) of the whole frame.

**Take-off window.** No photo is sent during a take-off window (ACKs, STATUS
and AUTO_STATUS still go out). In a manual take-off the camera sleeps through
it, so a capture waits for the window to end. In an autonomous take-off the
camera is held awake, so a set the state machine asks for is captured at
once; only its push waits. The window of an autonomous take-off ends 1.5 s
after the controller's detected lift-off, so the take-off photos go out in
HOVER, during the wait.

The held set is freed by `PHOTO_FREE`, 30 s after its push finished, or at
the latest 60 s after it started.

META (0xA0), 40 bytes (a grey frame, or a frame the camera could not supply),
or 44 (a colour frame):

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | id |
| 2 | 1 | idx (0..n-1) |
| 3 | 1 | n |
| 4 | 2 | width (160) |
| 6 | 2 | height (120) |
| 8 | 4 | size (19200 grey, 38400 colour; width x height x bytes per pixel) |
| 12 | 4 | crc32 |
| 16 | 4 | frameSeq |
| 20 | 4 | captureUpMs |
| 24 | 2 | roll, 0.01 deg (i16) |
| 26 | 2 | pitch, 0.01 deg (i16) |
| 28 | 2 | yaw, 0.01 deg (i16) |
| 30 | 2 | thrust |
| 32 | 1 | lum |
| 33 | 1 | texture |
| 34 | 2 | chunkSize (1024) |
| 36 | 2 | chunkCount (19 grey, 38 colour) |
| 38 | 2 | vbatMv |
| 40 | 1 | pixFmt, 44-byte META only: 1 = YUYV (auto_proto.h `AUTO_PIXFMT_YUYV`); 0 = grey (`AUTO_PIXFMT_GRAY8`, what a 40-byte META means); any other: the Mac cannot decode the frame (final) |
| 41 | 3 | 0 |

Camera geometry for bearings: horizontal focal length about 102.4 px at
160x120 (camera_flow.c `FLOW_KX_GZ`), so a pixel column x is at bearing
atan((x - 80) / 102.4), positive to the right.

## On the drone

- One low-priority task (priority 1, below the phone app's CRTP RX; core 0)
  owns the socket. Its 8 KB stack and the TX buffer are in PSRAM (internal
  RAM, 6 KB, only without PSRAM): about 7 KB of internal heap compared with
  stage A1. That is safe because the task never starts a flash operation,
  which would disable the cache and with it PSRAM: the key is read from NVS
  before the task starts, and everything the task calls is RAM work (the
  reasoning is in auto_link.c, `linkTaskStart`).
- USB `auto` prints the link counters: `linkStack=psram|internal`,
  `stackFree`, `kills`, `autoCmds`, `cmdTimeouts`, `cmdBusy`,
  `cmdTakenUnknown`, `autoStatusTx`, `autoCaptures`, `photosPreempted`,
  `photoHeldBusy`, `killRateDrops` (autoLinkStats_t, auto_link.h).

## Test vectors

`tools/tests/auto_proto_golden.json` (A1) and
`tools/tests/auto_proto_golden_a2.json` (A2b: ARM, ARM with airMaxMs,
TAKEOFF, LAND, DISARM, KILL, KILL in session 0, PHOTO in the hover, the ACKs
of an accepted and a GUARDS-refused ARM, AUTO_STATUS) and
`tools/tests/auto_proto_golden_ih.json` (the interactive hover: MOVE, YAW,
STOP, TAKEOFF with `actWaitMs`, a refused MOVE's ACK, the 96-byte
AUTO_STATUS; since 2026-10-14 also TAKEOFF with waits of 65536 and 120000 ms,
byte 7 carrying bits 16-23, and an AUTO_STATUS with the u16 ms fields
saturated; since the long-move build `takeoff_plan`, the 18-byte TAKEOFF with
left 80 cm, forward 250 cm and a 90 s wait; since 2026-10-15 `auto_status_hr`,
the 136-byte AUTO_STATUS of a drone holding after a contact with the camera
height lost; `set_height`, `set_height_now` and the ACKs `ack_set_height` and
`ack_set_height_refused` (SH_STEP), the SET_HEIGHT of the same day; since 2026-10-16 `set_height_to` and
`set_height_to_120`, its 6-byte target form; `meta_color`, a colour frame's 44-byte META,
with `frame_color_crc32` and its frame rule) hold packets made with the key
`000102...0f`; both the C codec
(`tools/tests/test_auto_proto.c`) and the Mac tool
(`tools/tests/test_espfly_auto.py`) must reproduce them byte for byte.
Regenerate with `python3 tools/tests/gen_auto_proto_golden.py`. No golden
packet is a `BUSY`, so the detail kinds left them unchanged;
`test_auto_proto` checks the kind encoding (`autoDetailBusy`) and its place
on the wire (ACK payload byte 11). The link task itself is tested on the
host by `tools/tests/auto_link_host`.
