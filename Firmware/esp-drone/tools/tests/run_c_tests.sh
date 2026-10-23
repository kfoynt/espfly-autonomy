#!/bin/bash
# The host C test runner (a copy in the tree since 2026-10-22; the scratchpad copy gets wiped). Usage: run_c_tests.sh [esp-drone_dir] [firmware-backups_dir]
# Compiles every tools/tests/test_*.c with its documented line and runs it; prints PASS/FAIL per run and a summary.
# Known failures carried since before (the data they want is not in this checkout): test_gyro_tempco, test_mahony_replay.
WT=${1:-$(cd "$(dirname "$0")/../.." && pwd)}
FB=${2:-../../firmware-backups}
BIN=${BIN:-$(cd "$(dirname "$0")" && pwd)/bin}
mkdir -p "$BIN"
cd "$WT" || exit 2
C=components/core/crazyflie
CAL=$FB/cal
pass=0; fail=0; failed=""
run() {   # run NAME "compile args" "run args"
  local name=$1 cargs=$2 rargs=$3
  if ! cc $cargs -o "$BIN/$name" 2> "$BIN/$name.cc.log"; then
    echo "FAIL $name (compile; $BIN/$name.cc.log)"; fail=$((fail+1)); failed="$failed $name"; return
  fi
  if "$BIN/$name" $rargs > "$BIN/$name.out" 2>&1; then
    echo "PASS $name $(tail -1 "$BIN/$name.out")"; pass=$((pass+1))
  else
    echo "FAIL $name $(tail -1 "$BIN/$name.out")"; fail=$((fail+1)); failed="$failed $name"
  fi
}
U=$C/utils/interface; M=$C/modules/interface; H=$C/hal/interface; S=$C/utils/src; MS=$C/modules/src
run test_accel_tempco "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_accel_tempco.c $S/accel_tempco.c $S/imu_level_cal.c -lm" "$CAL"
run test_att_trim_learn "-std=c99 -Wall -Wextra -I$U tools/tests/test_att_trim_learn.c $S/att_trim_learn.c -lm" "$CAL"
run test_auto_fsm "-std=c99 -Wall -Wextra -I$U tools/tests/test_auto_fsm.c $S/auto_fsm.c $S/auto_motion.c -lm" ""
run test_auto_glue_core "-std=c99 -O2 -Wall -Wextra -I$U -I$M -I$H tools/tests/test_auto_glue_core.c $S/auto_glue_core.c $S/auto_proto.c $S/cam_vz_obs.c $S/auto_motion.c -lm" ""
run test_auto_mailbox "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_auto_mailbox.c $S/auto_mailbox.c -lm" ""
run test_auto_motion "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_auto_motion.c $S/auto_motion.c -lm" ""
run test_auto_motion_native "-std=c99 -O2 -Wall -Wextra -DESPFLY_NATIVE_VGA=1 -I$U tools/tests/test_auto_motion_native.c $S/auto_motion.c -lm" ""
run test_scene_gate_replay "-std=c99 -O2 -Wall -Wextra -DESPFLY_NATIVE_VGA=1 -I$U tools/tests/test_scene_gate_replay.c $S/auto_motion.c $S/camera_hold.c -lm" "$CAL"
run test_slide_replay "-std=c99 -O2 -Wall -Wextra -DESPFLY_NATIVE_VGA=1 -I$U tools/tests/test_slide_replay.c $S/auto_motion.c $S/camera_hold.c -lm" ""
run test_auto_proto "-std=c99 -Wall -Wextra -I$U tools/tests/test_auto_proto.c $S/auto_proto.c" "tools/tests/auto_proto_golden.json tools/tests/auto_proto_golden_a2.json tools/tests/auto_proto_golden_ih.json"
run test_auto_sortie "-std=c99 -O2 -Wall -Wextra -I$U -I$M -I$H tools/tests/test_auto_sortie.c $S/auto_fsm.c $S/vert_ctrl.c $S/auto_motion.c -lm" ""
run test_autonomy_glue_host "-std=gnu99 -O1 -Wall -Wextra -Itools/tests/autonomy_glue_host/stub -I$U -I$M -I$H tools/tests/test_autonomy_glue_host.c $MS/autonomy.c $S/auto_glue_core.c $S/auto_mailbox.c $S/auto_fsm.c $S/vert_ctrl.c $S/auto_proto.c $S/auto_motion.c -lm" ""
run test_cam_clock "-std=c99 -Wall -Wextra -I$U tools/tests/test_cam_clock.c $S/cam_clock.c $S/cam_gnd.c -lm" ""
run test_cam_gnd "-std=c99 -Wall -Wextra -I$U tools/tests/test_cam_gnd.c $S/cam_gnd.c" ""
run test_cam_split "-std=gnu99 -O2 -Wall -Wextra -DCAM_SPLIT_TEST_HOOK -I$U tools/tests/test_cam_split.c $S/cam_split_core.c $S/optical_flow.c -lm" ""
run test_cam_vz_glue_host "-std=gnu99 -O1 -Wall -Wextra -Wno-unused-parameter -Wno-unused-function -Itools/tests/autonomy_glue_host/stub -I$U -I$M -I$H -I$MS tools/tests/test_cam_vz_glue_host.c $S/cam_vz_shadow.c $S/cam_vz_obs.c $S/auto_glue_core.c $S/auto_proto.c -lm" ""
run test_cam_vz_late "-std=c99 -O2 -Wall -Wextra -I$U -I$M tools/tests/test_cam_vz_late.c $S/cam_vz_shadow.c $S/cam_vz_obs.c $S/vert_ctrl.c -lm" ""
run test_cam_vz_obs "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_cam_vz_obs.c $S/cam_vz_obs.c -lm" ""
run test_cam_vz_replay_ih "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_cam_vz_replay_ih.c $S/cam_vz_obs.c -lm" "$CAL"
run test_cam_vz_shadow "-std=c99 -O2 -Wall -Wextra -I$U -I$M tools/tests/test_cam_vz_shadow.c $S/cam_vz_shadow.c $S/cam_vz_obs.c -lm" "$CAL"
run test_cam_vz_t2 "-std=c99 -O2 -Wall -Wextra -I$U -I$M tools/tests/test_cam_vz_t2.c $S/cam_vz_shadow.c $S/cam_vz_obs.c $S/vert_ctrl.c -lm" ""
run test_camera_hold "-std=c99 -Wall -Wextra -I$U tools/tests/test_camera_hold.c $S/camera_hold.c -lm" ""
run test_collision_replay "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_collision_replay.c $S/auto_fsm.c $S/auto_motion.c -lm" "$CAL"
run test_controller_pid_ihold "-std=gnu99 -O1 -Wall -Wno-absolute-value -DCONFIG_TARGET_ESP32_S2_DRONE_V1_2 -Itools/tests/controller_pid_host/stub -I$M -I$U -I$H tools/tests/test_controller_pid_ihold.c $MS/controller_pid.c $MS/attitude_pid_controller.c $MS/pid.c $S/torque_trim.c $S/num.c $S/filter.c -lm" ""
run test_drift_thermal "-std=c99 -O2 -Wall -Wextra -I$M -I$U tools/tests/test_drift_thermal.c $S/camera_hold.c -lm" ""
run test_flight_recorder_host "-std=gnu99 -O2 -Wall -Wextra -Itools/tests/flight_recorder_host/stub -I$M -Itools/tests/autonomy_glue_host/stub -I$H -I$U -Icomponents/config/include tools/tests/test_flight_recorder_host.c $MS/flight_recorder.c -lm" ""
run test_flight_summary "-std=c99 -Wall -Wextra -I$U tools/tests/test_flight_summary.c $S/flight_summary.c" ""
run test_ "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_ $S/optical_flow.c -lm" ""
run test_ "-std=c99 -O2 -Wall -Wextra -DOF_TEST_COUNT_RESTARTS -I$U tools/tests/test_ $S/optical_flow.c -lm" ""
run test_flow_scenes "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_flow_scenes.c $S/optical_flow.c $S/camera_hold.c -lm" ""
run test_ground_cool "-std=c99 -Wall -Wextra -I$U tools/tests/test_ground_cool.c $S/ground_cool.c -lm" ""
run test_gyro_tempco "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_gyro_tempco.c $S/gyro_tempco.c $S/imu_level_cal.c -lm" "$CAL"
run test_hold_limit_replay "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_hold_limit_replay.c $S/camera_hold.c -lm" "$CAL"
run test_hold_replay "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_hold_replay.c $S/camera_hold.c -lm" "$CAL"
run test_imu_level_cal "-std=c99 -Wall -Wextra -I$U tools/tests/test_imu_level_cal.c $S/imu_level_cal.c $S/torque_trim.c -lm" ""
run test_interactive_sim "-std=c99 -O2 -Wall -Wextra -DAUTO_MV_HOST_VH_OVERRIDE -I$U -I$M -I$H tools/tests/test_interactive_sim.c $S/auto_fsm.c $S/auto_motion.c $S/vert_ctrl.c $S/camera_hold.c $S/cam_vz_obs.c -lm" ""
run test_mahony_replay "-std=c99 -O2 -Wall -Wextra -I$M -I$U tools/tests/test_mahony_replay.c -lm" "$CAL"
run test_motion_replay "-std=c99 -O2 -Wall -Wextra -DAUTO_MV_HOST_VH_OVERRIDE -I$U tools/tests/test_motion_replay.c $S/auto_motion.c -lm" "$CAL"
run test_motor_guard "-std=c99 -Wall -Wextra -I$M tools/tests/test_motor_guard.c $S/motor_guard_core.c" ""
run test_motor_pwm_place "-std=c99 -Wall -Wextra -I$U tools/tests/test_motor_pwm_place.c $S/motor_pwm_place.c -lm" ""
run test_optical_flow "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_optical_flow.c $S/optical_flow.c -lm" ""
run test_sensfusion6_ratecorr "-std=c99 -O2 -Wall -I$M -I$U tools/tests/test_sensfusion6_ratecorr.c $S/cam_vz_shadow.c $S/cam_vz_obs.c -lm" ""
run test_speed300_replay "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_speed300_replay.c $S/auto_motion.c $S/camera_hold.c -lm" "$CAL"
run test_thrust_softstart "-std=c99 -Wall -Wextra -I$U tools/tests/test_thrust_softstart.c $S/thrust_softstart.c" ""
run test_vert_ctrl "-std=c99 -O2 -Wall -Wextra -I$U tools/tests/test_vert_ctrl.c $S/vert_ctrl.c -lm" ""
run test_vert_replay "-std=c99 -Wall -Wextra -I$U tools/tests/test_vert_replay.c $S/vert_ctrl.c -lm" "$CAL"
echo "SUMMARY: $pass passed, $fail failed:$failed"
